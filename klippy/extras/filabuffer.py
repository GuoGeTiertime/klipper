# Automatic filament buffer with multi feed units
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

# Buffer sensor bits (pin order: jam, low, full)
BUFF_JAM = 0x01
BUFF_LOW = 0x02
BUFF_FULL = 0x04

MODE_DISABLED = 'disabled'
MODE_INIT_FILA = 'init_fila'
MODE_INIT_WORK = 'init_work'
MODE_WORK = 'work'
MODE_ERROR = 'error'

UNIT_STOP = 'stop'
UNIT_READY = 'ready'
UNIT_RUNNING = 'running'

INIT_SUB_FEED = 'feed'
INIT_SUB_RETRACT = 'retract'

STARTUP_IGNORE_TIME = 2.0
# Min print_time gap between MCU digital_out/pwm events (avoid "Timer too close")
MCU_PIN_EVENT_GAP = 0.025 #stop after 25ms

# Per-unit options use suffix _0, _1, _2. Shared mechanics (microstep,
# rotate_distance, gear_ratio, max_speed, full_steps_per_rotation) fall back
# from _0 then unsuffixed name when not set on higher indexes.
UNIT_PIN_OPTIONS = ('inlet_pin', 'buffer_pin', 'step_pin', 'dir_pin',
                    'enable_pin')
UNIT_SHARED_OPTIONS = ('microstep', 'full_steps_per_rotation',
                       'rotate_distance', 'gear_ratio', 'max_speed')


def _option_keys(option, idx):
    keys = ['%s_%d' % (option, idx)]
    if idx > 0:
        keys.append('%s_0' % (option,))
    keys.append(option)
    return keys


def _has_unit_index(config, idx):
    """True only when step_pin is defined for this unit index (no _0 fallback)."""
    section = config.get_name()
    if config.fileconfig.has_option(section, 'step_pin_%d' % (idx,)):
        return True
    return idx == 0 and config.fileconfig.has_option(section, 'step_pin')


def _get_unit_option(config, option, idx, default=None):
    section = config.get_name()
    for key in _option_keys(option, idx):
        if config.fileconfig.has_option(section, key):
            return config.get(key)
    return default


def _get_unit_int(config, option, idx, default, **kwargs):
    section = config.get_name()
    for key in _option_keys(option, idx):
        if config.fileconfig.has_option(section, key):
            return config.getint(key, **kwargs)
    return default


def _get_unit_float(config, option, idx, default, **kwargs):
    section = config.get_name()
    for key in _option_keys(option, idx):
        if config.fileconfig.has_option(section, key):
            return config.getfloat(key, **kwargs)
    return default


def _parse_gear_ratio_key(config, key):
    ratios = config.getlists(key, (), seps=(':', ','), count=2,
                             parser=float)
    result = 1.
    for g1, g2 in ratios:
        result *= g1 / g2
    return result


def _get_unit_gear_ratio(config, idx):
    section = config.get_name()
    for key in _option_keys('gear_ratio', idx):
        if config.fileconfig.has_option(section, key):
            return _parse_gear_ratio_key(config, key)
    return 1.


def _consume_unit_config_options(config):
    """Mark per-unit options read so check_unused_options passes."""
    section = config.get_name()
    fc = config.fileconfig
    for key in config.get_prefix_options('gear_ratio'):
        if fc.has_option(section, key):
            _parse_gear_ratio_key(config, key)
    for option in UNIT_SHARED_OPTIONS:
        if option == 'gear_ratio':
            continue
        for key in config.get_prefix_options(option):
            if not fc.has_option(section, key):
                continue
            if option in ('microstep', 'full_steps_per_rotation'):
                config.getint(key)
            else:
                config.getfloat(key)
    for option in UNIT_PIN_OPTIONS + ('unit_name',):
        for key in config.get_prefix_options(option):
            if fc.has_option(section, key):
                config.get(key)


class GcodeQueue:
    def __init__(self, printer):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self._queue = []
        self._busy = False

    def enqueue(self, template, prefix=""):
        if template is None:
            return
        self._queue.append((template, prefix))
        if not self._busy:
            self.reactor.register_callback(self._process_queue)

    def _process_queue(self, eventtime):
        if not self._queue:
            return
        self._busy = True
        template, prefix = self._queue.pop(0)
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception:
            logging.exception("filabuffer gcode queue error")
        self._busy = False
        if self._queue:
            self.reactor.register_callback(self._process_queue)


class FeedMotor:
    def __init__(self, fb, unit_name, unit_index, config):
        self.fb = fb
        self.name = unit_name
        self.unit_index = unit_index
        self.reactor = fb.reactor
        self.pinout_delay = fb.pinout_delay
        ppins = config.get_printer().lookup_object('pins')
        step_pin = _get_unit_option(config, 'step_pin', unit_index)
        if step_pin is None:
            raise config.error(
                "filabuffer %s unit %d: step_pin_%d is required"
                % (fb.name, unit_index, unit_index))
        self.step = ppins.setup_pin('pwm', step_pin)
        self.step.setup_cycle_time(0.0002)
        self.dir = None
        dir_pin = _get_unit_option(config, 'dir_pin', unit_index)
        if dir_pin is not None:
            self.dir = ppins.setup_pin('digital_out', dir_pin)
            self.dir.setup_max_duration(0.)
        self.stepenable = None
        enable_pin = _get_unit_option(config, 'enable_pin', unit_index)
        if enable_pin is not None:
            self.stepenable = ppins.setup_pin('digital_out', enable_pin)
            self.stepenable.setup_max_duration(0.)
        self.gearing = _get_unit_gear_ratio(config, unit_index)
        self.microstep = _get_unit_int(
            config, 'microstep', unit_index, 16, minval=1, maxval=256)
        full_steps = _get_unit_int(
            config, 'full_steps_per_rotation', unit_index, 200, minval=1)
        self.rotate_distance = _get_unit_float(
            config, 'rotate_distance', unit_index, 31.4, above=0.1)
        self.scale_speed2freq = (
            self.microstep * full_steps * self.gearing / self.rotate_distance)
        self.max_speed = _get_unit_float(
            config, 'max_speed', unit_index, 100., above=0., maxval=1000.)
        self.cur_cycle_time = 0.1
        self.bfeeder_on = False
        self.is_feeding = False
        self.cur_feed_len = 0.
        self.last_feed_speed = 0.
        self.last_feed_time = 0.
        self.next_feed_time = 0.
        self.last_pulse_time = 0.
        self.last_enable_time = 0.
        self.last_dir_time = 0.
        self._withdraw = False

    def _sched_print_time(self, curtime, min_gap=0.):
        mcu = self.step.get_mcu()
        pt = mcu.estimated_print_time(curtime) + self.pinout_delay
        min_pt = max(self.last_pulse_time, self.last_enable_time,
                     self.last_dir_time) + min_gap
        return max(pt, min_pt)

    def _cal_step_cycle_time(self, speed):
        freq = abs(speed) * self.scale_speed2freq
        return 1.0 / freq if freq > 0 else 0.1

    def _set_step_cycle_time(self, cycle_time):
        if self.cur_cycle_time == cycle_time:
            return
        self.cur_cycle_time = cycle_time
        mcu = self.step.get_mcu()
        cycle_ticks = mcu.seconds_to_clock(cycle_time)
        mcu._serial.send("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d"
                         % (self.step._oid, cycle_ticks))

    def set_dir(self, print_time, forward):
        if self.dir is None:
            return
        self.dir.set_digital(print_time, 1 if forward else 0)
        self.last_dir_time = print_time

    def enable_stepper(self, on, curtime=None):
        self.bfeeder_on = not not on
        if self.stepenable is None:
            return
        if curtime is None:
            curtime = self.reactor.monotonic()
        pt = self._sched_print_time(curtime, 0.001)
        self.stepenable.set_digital(pt, 1 if on else 0)
        self.last_enable_time = pt

    def set_pulse(self, print_time, value, cycle_time):
        self._set_step_cycle_time(cycle_time)
        self.step.set_pwm(print_time, value)
        self.last_pulse_time = print_time

    def _update_feed_len(self, print_time):
        if self.last_feed_time > 0. and self.last_feed_speed > 0.:
            self.cur_feed_len += (
                (print_time - self.last_feed_time) * self.last_feed_speed)

    def feed_chunk(self, speed, length, curtime=None):
        if curtime is None:
            curtime = self.reactor.monotonic()
        if not self.bfeeder_on and not self.is_feeding:
            return 0.
        pt = self._sched_print_time(curtime, self.cur_cycle_time)
        if pt < self.next_feed_time:
            return 0.
        self._update_feed_len(pt)
        if length <= 0.:
            self.is_feeding = False
            self.set_pulse(pt, 0, 1.0)
            self.last_feed_speed = 0.
            self.last_feed_time = pt
            self.next_feed_time = pt + 0.05
            return 0.
        forward = length > 0
        if self._withdraw:
            forward = not forward
        speed = max(0.01, min(abs(speed), self.max_speed))
        cycle_time = self._cal_step_cycle_time(speed)
        feed_time = abs(length) / speed
        self.set_dir(pt, forward)
        self.is_feeding = True
        self.set_pulse(pt, 0.5, cycle_time)
        self.last_feed_speed = speed
        self.last_feed_time = pt
        self.next_feed_time = pt + feed_time
        return abs(length)

    def start_continuous(self, speed, max_len, curtime=None):
        if curtime is None:
            curtime = self.reactor.monotonic()
        self._withdraw = speed < 0
        self.enable_stepper(True, curtime)
        self.feed_chunk(abs(speed), min(max_len, 50.), curtime)

    def stop_immediate(self, curtime=None, slot=0):
        if curtime is None:
            curtime = self.reactor.monotonic()
        slot_gap = 0 # slot * MCU_PIN_EVENT_GAP * 3
        pt = self._sched_print_time(curtime, 0.001 + slot_gap)
        self._update_feed_len(pt)
        pt_pwm = pt + MCU_PIN_EVENT_GAP
        pt_pwm = max(pt_pwm, self.last_pulse_time + MCU_PIN_EVENT_GAP)
        self._set_step_cycle_time(1.0)
        self.step.set_pwm(pt_pwm, 0)
        self.last_pulse_time = pt_pwm
        if self.stepenable is not None:
            pt_en = pt_pwm + MCU_PIN_EVENT_GAP
            pt_en = max(pt_en, self.last_enable_time + MCU_PIN_EVENT_GAP)
            self.stepenable.set_digital(pt_en, 0)
            self.last_enable_time = pt_en
        self.is_feeding = False
        self.bfeeder_on = False
        self.last_feed_speed = 0.
        self.cur_feed_len = 0.
        self.next_feed_time = pt_pwm + 0.05

    def maybe_extend_feed(self, max_len, curtime):
        if not self.is_feeding or not self.bfeeder_on:
            return
        if curtime + 0.05 < self.next_feed_time:
            return
        remain = max_len - self.cur_feed_len
        if remain > 0.:
            self.feed_chunk(self.last_feed_speed, min(remain, 50.), curtime)


class FeedUnit:
    def __init__(self, fb, unit_name, unit_index, config):
        self.fb = fb
        self.name = unit_name
        self.unit_index = unit_index
        self.reactor = fb.reactor
        if self.name in fb.units:
            raise config.error(
                "Duplicate unit '%s' on filabuffer '%s'"
                % (self.name, fb.name))
        self.unit_state = UNIT_STOP
        self.init_substate = None
        self._inlet_present = False
        self._buffer_present = False
        self.motor = FeedMotor(fb, unit_name, unit_index, config)
        self._gpio_roles = []
        gpio_pin_list = []
        inlet_pin = _get_unit_option(config, 'inlet_pin', unit_index)
        buffer_pin = _get_unit_option(config, 'buffer_pin', unit_index)
        if inlet_pin is not None:
            gpio_pin_list.append(inlet_pin)
            self._gpio_roles.append('inlet')
        if buffer_pin is not None:
            gpio_pin_list.append(buffer_pin)
            self._gpio_roles.append('buffer')
        if gpio_pin_list:
            buttons = config.get_printer().load_object(config, 'buttons')
            buttons.register_buttons(gpio_pin_list, self._gpio_handler)
        fb.units[self.name] = self

    @property
    def inlet_present(self):
        return self._inlet_present

    @property
    def buffer_present(self):
        return self._buffer_present

    def update_sensor(self, role, eventtime, present):
        old_inlet = self._inlet_present
        old_buffer = self._buffer_present
        if role == 'inlet':
            self._inlet_present = bool(present)
        elif role == 'buffer':
            self._buffer_present = bool(present)
        else:
            return
        if (old_inlet == self._inlet_present
                and old_buffer == self._buffer_present):
            return
        if eventtime < self.fb.min_event_time:
            return
        self.fb.note_unit_change(
            self, eventtime, old_inlet, old_buffer)

    def _gpio_handler(self, eventtime, state):
        for i, role in enumerate(self._gpio_roles):
            self.update_sensor(role, eventtime, bool(state & (1 << i)))

    def get_status(self):
        return {
            'filabuffer': self.fb.name,
            'state': self.unit_state,
            'inlet': self._inlet_present,
            'buffer': self._buffer_present,
            'init_substate': self.init_substate,
            'cur_feed_len': self.motor.cur_feed_len,
            'is_feeding': self.motor.is_feeding,
            'gearing': self.motor.gearing,
        }


class BufferSensors:
    def __init__(self, config, fb):
        self.fb = fb
        self.state = 0
        buttons = config.get_printer().load_object(config, 'buttons')
        buttons.register_buttons(
            [config.get('jam_pin'), config.get('low_pin'),
             config.get('full_pin')], self._button_handler)

    def _button_handler(self, eventtime, state):
        old = self.state
        self.state = state & (BUFF_JAM | BUFF_LOW | BUFF_FULL)
        if eventtime < self.fb.min_event_time:
            return
        self.fb.note_buffer_change(eventtime, old, self.state)

    def get_status(self):
        return {
            'jam': bool(self.state & BUFF_JAM),
            'low': bool(self.state & BUFF_LOW),
            'full': bool(self.state & BUFF_FULL),
        }


class FilaBuffer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.units = {}
        self.active_unit = None
        self.mode = MODE_DISABLED
        self.error_msg = None
        self.feed_session_start = 0.
        self.init_target_unit = None
        self.min_event_time = 0.
        self.pinout_delay = config.getfloat(
            'pinout_delay', 0.025, minval=0.010, maxval=0.050)
        self.watchdog_time = config.getfloat('watchdog_time', 0.25, above=0.05)
        self.max_feed_time = config.getfloat('max_feed_time', 30., above=0.5)
        self.max_feed_len = config.getfloat('max_feed_len', 500., above=1.)
        self.init_max_feed_time = config.getfloat(
            'init_max_feed_time', 60., above=0.5)
        self.init_max_feed_len = config.getfloat(
            'init_max_feed_len', 1000., above=1.)
        self.retract_len = config.getfloat('retract_len', 2., above=0.)
        self.retract_speed = config.getfloat('retract_speed', 5., above=0.)
        self.feed_speed = config.getfloat('feed_speed', 10., above=0.)
        self.feed_speed_init = config.getfloat(
            'feed_speed_init', self.feed_speed * 2., above=0.)
        self.min_buffer_travel_mm = config.getfloat(
            'min_buffer_travel_mm', 6., above=0.)
        self.button_latency = config.getfloat('button_latency', 0.010, above=0.)
        self.hw_latency = config.getfloat('hw_latency', 0.002, above=0.)
        self.full_stop_mode = config.getchoice(
            'full_stop_mode',
            {'host': 'host', 'hardware_enable': 'hardware_enable'})
        self.pause_on_error = config.getboolean('pause_on_error', True)
        if self.pause_on_error:
            self.printer.load_object(config, 'pause_resume')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.jam_gcode = gcode_macro.load_template(config, 'jam_gcode', '')
        self.low_timeout_gcode = gcode_macro.load_template(
            config, 'low_timeout_gcode', '')
        self.break_gcode = gcode_macro.load_template(config, 'break_gcode', '')
        self.runout_gcode = gcode_macro.load_template(
            config, 'runout_gcode', '')
        self.gcode_queue = GcodeQueue(self.printer)
        self.sensors = BufferSensors(config, self)
        self._watchdog_timer = self.reactor.register_timer(
            self._watchdog_event)
        self.gcode.register_mux_command(
            'FILA_BUFFER_SELECT', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_SELECT,
            desc=self.cmd_FILA_BUFFER_SELECT_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_START', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_START,
            desc=self.cmd_FILA_BUFFER_START_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_STOP', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_STOP,
            desc=self.cmd_FILA_BUFFER_STOP_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_INIT_FILAMENT', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_INIT_FILAMENT,
            desc=self.cmd_FILA_BUFFER_INIT_FILAMENT_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_RESET', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_RESET,
            desc=self.cmd_FILA_BUFFER_RESET_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_STATUS', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_STATUS,
            desc=self.cmd_FILA_BUFFER_STATUS_help)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self._load_units(config)
        _consume_unit_config_options(config)

    def _load_units(self, config):
        idx = 0
        while _has_unit_index(config, idx):
            unit_name = _get_unit_option(config, 'unit_name', idx,
                                         default='unit%d' % (idx,))
            if unit_name in self.units:
                raise config.error(
                    "filabuffer %s: duplicate unit_name '%s'"
                    % (self.name, unit_name))
            FeedUnit(self, unit_name, idx, config)
            idx += 1
        if not self.units:
            raise config.error(
                "filabuffer %s: no feed units (define step_pin_0, etc.)"
                % (self.name,))

    def _handle_ready(self):
        self.min_event_time = self.reactor.monotonic() + STARTUP_IGNORE_TIME
        speed = max(self.feed_speed, self.feed_speed_init)
        if self.full_stop_mode == 'hardware_enable':
            delay = self.hw_latency + 0.005
        else:
            delay = self.button_latency + self.pinout_delay + 0.005
        need = speed * delay
        if self.min_buffer_travel_mm < need:
            raise self.printer.config_error(
                "filabuffer: min_buffer_travel_mm %.2f too small "
                "(need >= %.2f for speed %.1f)" % (
                    self.min_buffer_travel_mm, need, speed))
        self.reactor.update_timer(self._watchdog_timer, self.reactor.NOW)

    def _get_unit(self, name):
        if name not in self.units:
            raise self.gcode.error("Unknown fila_buffer_unit '%s'" % (name,))
        return self.units[name]

    def _active(self):
        if self.active_unit is None:
            return None
        return self.units.get(self.active_unit)

    def _stop_all_motors(self, curtime=None):
        if curtime is None:
            curtime = self.reactor.monotonic()
        for slot, u in enumerate(self.units.values()):
            u.motor.stop_immediate(curtime, slot=slot)
            if u.unit_state == UNIT_RUNNING:
                u.unit_state = UNIT_STOP

    def _check_buffer_exclusive(self):
        if self.sensors.state & BUFF_FULL:
            if self.sensors.state & (BUFF_JAM | BUFF_LOW):
                return False
        return True

    def _count_buffer_filament(self):
        return sum(1 for u in self.units.values() if u.buffer_present)

    def _check_unit_mutex(self):
        if self._count_buffer_filament() > 1:
            self._enter_error("multi_buffer_filament")
            return False
        return True

    def _pause_prefix(self):
        return "PAUSE\n" if self.pause_on_error else ""

    def _enter_error(self, msg, enqueue_break=False, curtime=None):
        self.mode = MODE_ERROR
        self.error_msg = msg
        self._stop_all_motors(curtime)
        if msg == "jam":
            self.gcode_queue.enqueue(self.jam_gcode, self._pause_prefix())
        elif enqueue_break:
            self.gcode_queue.enqueue(self.break_gcode, self._pause_prefix())
        elif msg == "feed_timeout":
            self.gcode_queue.enqueue(
                self.low_timeout_gcode, self._pause_prefix())
        logging.error("filabuffer %s error: %s", self.name, msg)

    def note_buffer_change(self, eventtime, old, new):
        if not self._check_buffer_exclusive():
            self._enter_error("full_exclusive_violation")
            return
        if (new & BUFF_JAM) and not (old & BUFF_JAM):
            self._on_jam(eventtime)
            return
        if self.mode == MODE_WORK:
            self._sync_work_feed(eventtime)
        elif self.mode == MODE_INIT_WORK:
            if (new & BUFF_FULL) and not (old & BUFF_FULL):
                self._on_init_work_full(eventtime)

    def note_unit_change(self, unit, eventtime, old_inlet, old_buffer):
        if not self._check_unit_mutex():
            return
        if self.mode == MODE_INIT_FILA:
            if (self.init_target_unit is None
                    or unit.name == self.init_target_unit):
                if unit.init_substate is not None:
                    self._handle_init_fila_edges(
                        unit, eventtime, old_inlet, old_buffer)
                elif unit.inlet_present and not old_inlet:
                    self._start_init_fila_unit(unit, eventtime)
            return
        unit_a = self._active()
        if unit is not unit_a:
            return
        if old_inlet and not unit.inlet_present:
            self._on_runout(eventtime)
        elif old_buffer and not unit.buffer_present:
            if self.mode in (MODE_WORK, MODE_INIT_WORK):
                self._on_break(eventtime)

    def _on_jam(self, eventtime):
        self._enter_error("jam", curtime=eventtime)

    def _sync_work_feed(self, eventtime):
        """Level-triggered work feed: LOW and not FULL -> run; else stop."""
        if self.mode != MODE_WORK:
            return
        unit = self._active()
        if unit is None or unit.unit_state != UNIT_READY:
            return
        state = self.sensors.state
        if state & BUFF_FULL or not (state & BUFF_LOW):
            if unit.motor.is_feeding:
                self._on_feed_stop(eventtime)
            return
        if not unit.motor.is_feeding:
            self._start_unit_feed(unit, self.feed_speed, self.max_feed_len,
                                  eventtime)

    def _on_low_start(self, eventtime):
        self._sync_work_feed(eventtime)

    def _on_feed_stop(self, eventtime):
        unit = self._active()
        if unit is None:
            return
        if self.full_stop_mode == 'host' or unit.motor.is_feeding:
            unit.motor.stop_immediate(eventtime)
        if unit.unit_state == UNIT_RUNNING:
            unit.unit_state = UNIT_READY
        self.feed_session_start = 0.

    def _on_init_work_full(self, eventtime):
        unit = self._active()
        if unit is None:
            return
        if self.full_stop_mode == 'host' or unit.motor.is_feeding:
            unit.motor.stop_immediate(eventtime)
        unit.unit_state = UNIT_READY
        self.mode = MODE_WORK
        self.feed_session_start = 0.
        logging.info("filabuffer %s init_work done -> work", self.name)

    def _on_runout(self, eventtime):
        unit = self._active()
        if unit is None:
            return
        unit.motor.stop_immediate(eventtime)
        unit.unit_state = UNIT_STOP
        prefix = self._pause_prefix() if self.is_printing() else ""
        self.gcode_queue.enqueue(self.runout_gcode, prefix)
        if self.is_printing():
            self._enter_error("runout")
        else:
            self.mode = MODE_ERROR
            self.error_msg = "runout"

    def _on_break(self, eventtime):
        unit = self._active()
        if unit is None:
            return
        unit.motor.stop_immediate(eventtime)
        self._enter_error("break", enqueue_break=True)

    def _start_unit_feed(self, unit, speed, max_len, eventtime):
        unit.unit_state = UNIT_RUNNING
        unit.motor.start_continuous(speed, max_len, eventtime)
        if self.feed_session_start <= 0.:
            self.feed_session_start = eventtime

    def _start_init_fila_unit(self, unit, eventtime):
        if not unit.inlet_present:
            return
        unit.init_substate = INIT_SUB_FEED
        unit.unit_state = UNIT_RUNNING
        unit.motor._withdraw = False
        unit.motor.start_continuous(
            self.feed_speed_init, self.init_max_feed_len, eventtime)
        self.feed_session_start = eventtime
        logging.info("filabuffer %s init_fila %s feed start",
                     self.name, unit.name)

    def _handle_init_fila_edges(self, unit, eventtime, old_inlet, old_buffer):
        if unit.init_substate == INIT_SUB_FEED:
            if unit.buffer_present and not old_buffer:
                unit.motor.stop_immediate(eventtime)
                unit.init_substate = INIT_SUB_RETRACT
                unit.motor.start_continuous(
                    -self.retract_speed, self.retract_len, eventtime)
                logging.info("filabuffer %s init_fila %s retract",
                             self.name, unit.name)
            elif not unit.inlet_present and old_inlet:
                unit.motor.stop_immediate(eventtime)
                unit.init_substate = None
                unit.unit_state = UNIT_STOP
                self._enter_error("init_runout")
        elif unit.init_substate == INIT_SUB_RETRACT:
            if not unit.buffer_present and old_buffer:
                unit.motor.stop_immediate(eventtime)
                unit.motor._withdraw = False
                unit.init_substate = None
                unit.unit_state = UNIT_READY
                logging.info("filabuffer %s init_fila %s done",
                             self.name, unit.name)

    def _verify_active_for_start(self, unit):
        if not unit.inlet_present:
            raise self.gcode.error(
                "Unit %s inlet has no filament" % (unit.name,))
        n = self._count_buffer_filament()
        if n > 1:
            raise self.gcode.error("Multiple units have buffer filament")
        if n == 1 and not unit.buffer_present:
            raise self.gcode.error(
                "Buffer filament on another unit, not %s" % (unit.name,))

    def is_printing(self):
        idle = self.printer.lookup_object('idle_timeout')
        return idle.get_status(self.reactor.monotonic())['state'] == 'Printing'

    def _watchdog_event(self, eventtime):
        if self.mode in (MODE_ERROR, MODE_DISABLED):
            return eventtime + self.watchdog_time
        if self.mode == MODE_WORK:
            self._sync_work_feed(eventtime)
        max_len = (self.init_max_feed_len if self.mode != MODE_WORK
                   else self.max_feed_len)
        max_time = (self.init_max_feed_time if self.mode != MODE_WORK
                    else self.max_feed_time)
        for unit in self.units.values():
            if unit.unit_state == UNIT_RUNNING:
                unit.motor.maybe_extend_feed(max_len, eventtime)
                if unit.motor.cur_feed_len > max_len:
                    self._feed_timeout(eventtime)
                    return eventtime + self.watchdog_time
        if self.feed_session_start > 0.:
            if eventtime - self.feed_session_start > max_time:
                self._feed_timeout(eventtime)
        return eventtime + self.watchdog_time

    def _feed_timeout(self, eventtime):
        self._enter_error("feed_timeout", curtime=eventtime)

    cmd_FILA_BUFFER_SELECT_help = "Select active feed unit on a buffer"
    def cmd_FILA_BUFFER_SELECT(self, gcmd):
        unit = self._get_unit(gcmd.get('UNIT'))
        self.active_unit = unit.name
        unit.unit_state = UNIT_READY
        gcmd.respond_info("filabuffer %s active unit: %s"
                          % (self.name, unit.name))

    cmd_FILA_BUFFER_START_help = "Start filabuffer mode"
    def cmd_FILA_BUFFER_START(self, gcmd):
        mode = gcmd.get('MODE', MODE_DISABLED).lower()
        if mode not in (MODE_DISABLED, MODE_INIT_FILA, MODE_INIT_WORK,
                        MODE_WORK):
            raise gcmd.error("Invalid MODE")
        if mode in (MODE_INIT_WORK, MODE_WORK):
            unit = self._active()
            if unit is None:
                raise gcmd.error("FILA_BUFFER_SELECT required")
            self._verify_active_for_start(unit)
            unit.unit_state = UNIT_READY
        if mode == MODE_INIT_WORK:
            self._start_unit_feed(
                self._active(), self.feed_speed_init,
                self.init_max_feed_len, self.reactor.monotonic())
        self.mode = mode
        self.error_msg = None
        if mode == MODE_WORK:
            self._sync_work_feed(self.reactor.monotonic())
        if mode == MODE_DISABLED:
            self._stop_all_motors()
            self.feed_session_start = 0.
            for u in self.units.values():
                u.unit_state = UNIT_STOP
                u.init_substate = None
        gcmd.respond_info("filabuffer %s mode: %s" % (self.name, mode))

    cmd_FILA_BUFFER_STOP_help = "Stop filabuffer"
    def cmd_FILA_BUFFER_STOP(self, gcmd):
        self.mode = MODE_DISABLED
        self.error_msg = None
        self.feed_session_start = 0.
        self._stop_all_motors()
        for u in self.units.values():
            u.unit_state = UNIT_STOP
            u.init_substate = None
        gcmd.respond_info("filabuffer %s stopped" % (self.name,))

    cmd_FILA_BUFFER_INIT_FILAMENT_help = "Init filament on one unit"
    def cmd_FILA_BUFFER_INIT_FILAMENT(self, gcmd):
        unit = self._get_unit(gcmd.get('UNIT'))
        self.mode = MODE_INIT_FILA
        self.init_target_unit = unit.name
        if unit.inlet_present:
            self._start_init_fila_unit(unit, self.reactor.monotonic())
        else:
            gcmd.respond_info("Waiting insert on %s" % (unit.name,))

    cmd_FILA_BUFFER_RESET_help = "Clear error and disable"
    def cmd_FILA_BUFFER_RESET(self, gcmd):
        self.error_msg = None
        self.mode = MODE_DISABLED
        self._stop_all_motors()
        for u in self.units.values():
            u.unit_state = UNIT_STOP
            u.init_substate = None
        gcmd.respond_info("filabuffer %s reset" % (self.name,))

    cmd_FILA_BUFFER_STATUS_help = "Report filabuffer status"
    def cmd_FILA_BUFFER_STATUS(self, gcmd):
        bs = self.sensors.get_status()
        msg = ("filabuffer %s: mode=%s active=%s error=%s full_stop=%s "
               "jam=%d low=%d full=%d" % (
                   self.name, self.mode, self.active_unit, self.error_msg,
                   self.full_stop_mode, bs['jam'], bs['low'], bs['full']))
        for name, unit in sorted(self.units.items()):
            st = unit.get_status()
            msg += ("\n %s: %s inlet=%d buf=%d feed=%d len=%.1f sub=%s" % (
                name, st['state'], st['inlet'], st['buffer'],
                st['is_feeding'], st['cur_feed_len'], st['init_substate']))
        gcmd.respond_info(msg)

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'mode': self.mode,
            'active_unit': self.active_unit,
            'error': self.error_msg,
            'full_stop_mode': self.full_stop_mode,
            'buffer_state': self.sensors.get_status(),
            'units': {n: u.get_status() for n, u in self.units.items()},
        }


class FilaBufferManager:
    def __init__(self, printer):
        self.printer = printer
        self.buffers = {}

    def register_buffer(self, config):
        name = config.get_name().split()[-1]
        if name in self.buffers:
            raise config.error("Duplicate filabuffer '%s'" % (name,))
        fb = FilaBuffer(config)
        self.buffers[name] = fb
        self.printer.add_object('filabuffer ' + name, fb)
        return fb

    def lookup_buffer(self, name):
        if name not in self.buffers:
            raise self.printer.config_error(
                "Unknown filabuffer '%s'" % (name,))
        return self.buffers[name]

    def note_sensor_change(self, buffer_name, unit_name, role, eventtime,
                           present):
        fb = self.buffers.get(buffer_name)
        if fb is None:
            logging.warning(
                "filabuffer: sensor event for unknown buffer '%s'",
                buffer_name)
            return
        unit = fb.units.get(unit_name)
        if unit is None:
            logging.warning(
                "filabuffer: sensor event for unknown unit '%s' on '%s'",
                unit_name, buffer_name)
            return
        unit.update_sensor(role, eventtime, present)

    def get_status(self, eventtime):
        return {n: b.get_status(eventtime) for n, b in self.buffers.items()}


def get_filabuffer_manager(printer):
    if 'filabuffer' not in printer.objects:
        printer.add_object('filabuffer', FilaBufferManager(printer))
    return printer.lookup_object('filabuffer')


def load_config_prefix(config):
    return get_filabuffer_manager(
        config.get_printer()).register_buffer(config)
