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
MODE_WORK = 'work'
MODE_ERROR = 'error'

UNIT_EMPTY = 'empty'
UNIT_READY = 'ready'
UNIT_BUFFERED = 'buffered'
UNIT_ACTIVE = 'active'
UNIT_ERROR = 'error'
UNIT_INIT = 'init'
UNIT_FEED = 'feed'

UNIT_RUN_STATES = (UNIT_INIT, UNIT_FEED)

INIT_PHASE_FORWARD = 'forward'
INIT_PHASE_RETRACT = 'retract'
# Init 回撤：走满 retract_len（建议 50~100mm）@ retract_speed（默认同 feed_speed）；
# 结束后 buffer false->ready，true->init_retract_fail。回撤中 buffer 变 false 不停止。


def stable_state_from_sensors(inlet, buffer, is_selected):
    if not inlet and not buffer:
        return UNIT_EMPTY
    if inlet and not buffer:
        return UNIT_READY
    if inlet and buffer:
        return UNIT_ACTIVE if is_selected else UNIT_BUFFERED
    return UNIT_ERROR

STARTUP_IGNORE_TIME = 2.0
# Min print_time gap between MCU digital_out/pwm events (avoid "Timer too close")
MCU_PIN_EVENT_DELAY = 0.025
# Renew software-PWM queue this many seconds before chunk ends (print_time)
FEED_RENEW_MARGIN = 0.15
# FilaMotor fixed timing (not from config)
MOTOR_MAX_CHUNK_TIME = 4.0


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



class FilaMotor:
    """PWM step/dir/enable driver. start/stop only; on_stop_cb from __init__."""

    def __init__(self, reactor, config, unit_index, mm_per_pulse,
                 on_stop_cb=None, name=''):
        self.reactor = reactor
        self.name = name or ('motor_%d' % unit_index)
        self.mm_per_pulse = mm_per_pulse
        self._on_stop_cb = on_stop_cb
        self.total_mm = 0.
        self._gen = 0
        self._timer = None
        self._active = False
        self._fired = True
        self._remain = 0.
        self._move_mm = 0.
        self._chunk_mm = 0.
        self._chunk_start_pt = 0.
        self._speed = 0.
        self._forward = True
        self._act_type = None
        self.cycle_time = 0.1
        self.last_pt = 0.
        self.chunk_end_pt = 0.
        fb_name = config.get_name().split()[-1]
        ppins = config.get_printer().lookup_object('pins')
        step_pin = _get_unit_option(config, 'step_pin', unit_index)
        if step_pin is None:
            raise config.error(
                "filabuffer %s unit %d: step_pin_%d is required"
                % (fb_name, unit_index, unit_index))
        self.step = ppins.setup_pin('pwm', step_pin)
        self.step.setup_cycle_time(0.0002)
        self.step.setup_max_duration(0.)
        self.dir = None
        dir_pin = _get_unit_option(config, 'dir_pin', unit_index)
        if dir_pin is not None:
            self.dir = ppins.setup_pin('digital_out', dir_pin)
            self.dir.setup_max_duration(0.)
        self.enable = None
        enable_pin = _get_unit_option(config, 'enable_pin', unit_index)
        if enable_pin is not None:
            self.enable = ppins.setup_pin('digital_out', enable_pin)
            self.enable.setup_max_duration(0.)

    def is_moving(self):
        return self._active

    def start(self, distance, speed, act_type):
        if distance == 0.:
            return
        if self._active:
            self._gen += 1
            self._cancel_timer()
            self._halt_pwm()
            self.total_mm += self._move_mm
        self._gen += 1
        move_gen = self._gen
        self._active = True
        self._fired = False
        self._move_mm = 0.
        self._remain = abs(distance)
        self._forward = distance > 0
        self._speed = max(0.01, abs(speed))
        self._act_type = act_type
        self.chunk_end_pt = 0.
        curtime = self.reactor.monotonic()
        if self.enable is not None:
            pt = self._sched_print_time(curtime, 0.001)
            self.enable.set_digital(pt, 1)
            self.last_pt = pt
        self._run_chunk(curtime, move_gen)

    def stop(self, act_type, call_stop_cb=False):
        if not self._active:
            return
        self._gen += 1
        self._end_move(act_type, 'stopped', call_stop_cb)

    def _end_move(self, act_type, reason, call_stop_cb=True):
        self._cancel_timer()
        self._halt_pwm()
        if self._fired:
            return
        self._fired = True
        self._active = False
        self._remain = 0.
        self.total_mm += self._move_mm
        if call_stop_cb and (self._on_stop_cb is not None):
            self._on_stop_cb(act_type, self._move_mm, reason)

    def _cancel_timer(self):
        if self._timer is not None:
            self.reactor.unregister_timer(self._timer)
            self._timer = None

    def _flush_current_chunk(self, halt_pt=None):
        """Add current chunk to _move_mm; halt_pt=None means chunk finished."""
        if self._chunk_mm <= 0.:
            return
        if halt_pt is None:
            self._move_mm += self._chunk_mm
        else:
            end_pt = self._chunk_start_pt + self._chunk_mm / self._speed
            if halt_pt <= self._chunk_start_pt:
                partial = 0.
            elif halt_pt >= end_pt:
                partial = self._chunk_mm
            else:
                partial = (halt_pt - self._chunk_start_pt) * self._speed
            self._move_mm += partial
        self._chunk_mm = 0.

    def _halt_pwm(self):
        curtime = self.reactor.monotonic()
        pt = self._sched_print_time(curtime, 0.001)
        pt_pwm = max(pt + MCU_PIN_EVENT_DELAY, self.last_pt + MCU_PIN_EVENT_DELAY)
        self._flush_current_chunk(halt_pt_pwm)
        self.step.set_pwm(pt_pwm, 0)
        self.last_pt = pt_pwm
        self.chunk_end_pt = pt_pwm + 0.05

    def _sched_print_time(self, curtime, gap=0.):
        mcu = self.step.get_mcu()
        return max(mcu.estimated_print_time(curtime) + MCU_PIN_EVENT_DELAY,
                   self.last_pt + gap)

    def _run_chunk(self, curtime, move_gen):
        if not self._active or move_gen != self._gen:
            return
        if self._remain <= 0.:
            self._flush_current_chunk()
            self._end_move(self._act_type, 'complete')
            return
        length = min(self._remain, self._speed * MOTOR_MAX_CHUNK_TIME)
        pt = max(self._sched_print_time(curtime, 0.001), self.chunk_end_pt)
        ct = 1.0 / (self._speed / self.mm_per_pulse)
        if self.cycle_time != ct:
            self.cycle_time = ct
            mcu = self.step.get_mcu()
            ticks = mcu.seconds_to_clock(ct)
            self.step._pwm_max = float(ticks)
            mcu._serial.send(
                "set_digital_out_pwm_cycle oid=%d cycle_ticks=%d"
                % (self.step._oid, ticks))
        if self.dir is not None:
            self.dir.set_digital(pt, 1 if self._forward else 0)
        self.step.set_pwm(pt, 0.333)
        self.last_pt = pt
        self.chunk_end_pt = pt + length / self._speed
        self._chunk_start_pt = pt
        self._chunk_mm = length
        self._remain -= length
        if move_gen != self._gen:
            return
        mcu = self.step.get_mcu()
        margin = FEED_RENEW_MARGIN if self._remain > 0. else 0.05
        delay = max(0.001, self.chunk_end_pt - margin
                    - mcu.estimated_print_time(curtime))
        self._chunk_gen = move_gen
        self._cancel_timer()
        self._timer = self.reactor.register_timer(
            self._timer_event, curtime + delay)

    def _timer_event(self, eventtime):
        self._timer = None
        if not self._active or self._chunk_gen != self._gen:
            return self.reactor.NEVER
        mcu = self.step.get_mcu()
        pt = mcu.estimated_print_time(eventtime)
        if self._remain > 0.:
            if pt + FEED_RENEW_MARGIN < self.chunk_end_pt:
                return eventtime + 0.01
            self._flush_current_chunk()
            self._run_chunk(eventtime, self._chunk_gen)
            return self.reactor.NEVER
        if pt < self.chunk_end_pt - 0.05:
            return eventtime + 0.01
        self._flush_current_chunk()
        self._end_move(self._act_type, 'complete')
        return self.reactor.NEVER

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
        self.step.setup_max_duration(0.)
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
        self.scheduled_len = 0.
        self.last_feed_speed = 0.
        self.last_feed_time = 0.
        self.next_feed_time = 0.
        self.last_pulse_time = 0.
        self.last_enable_time = 0.
        self.last_dir_time = 0.
        self._withdraw = False
        self._retract_not_before = 0.
        self._after_stop_timer = None
        self._pending_after_stop = None

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
        self.step._pwm_max = float(cycle_ticks)
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

    def set_pulse(self, print_time, value, cycle_time, update_cycle=True):
        # MCU rejects set_digital_out_pwm_cycle while queue_digital_out pending
        if update_cycle:
            self._set_step_cycle_time(cycle_time)
        self.step.set_pwm(print_time, value)
        self.last_pulse_time = print_time

    def feed_chunk(self, speed, length, curtime=None):
        if curtime is None:
            curtime = self.reactor.monotonic()
        if not self.bfeeder_on and not self.is_feeding:
            return 0.
        was_feeding = self.is_feeding
        if was_feeding:
            pt = max(self._sched_print_time(curtime, 0.001),
                     self.next_feed_time)
        else:
            pt = self._sched_print_time(curtime, self.cur_cycle_time)
            if pt < self.next_feed_time:
                return 0.
        if length <= 0.:
            self.is_feeding = False
            self.set_pulse(pt, 0, self.cur_cycle_time, update_cycle=False)
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
        self.set_pulse(pt, 0.333, cycle_time, update_cycle=not was_feeding)
        self.last_feed_speed = speed
        self.last_feed_time = pt
        self.next_feed_time = pt + feed_time
        self.scheduled_len += abs(length)
        self.cur_feed_len = self.scheduled_len
        return abs(length)

    def start_continuous(self, speed, max_len, curtime=None):
        if curtime is None:
            curtime = self.reactor.monotonic()
        self._withdraw = speed < 0
        self.scheduled_len = 0.
        self.cur_feed_len = 0.
        # stop_immediate() sets next_feed_time in the future; must clear so
        # the first feed_chunk after stop (e.g. init retract) is not skipped.
        self.next_feed_time = 0.
        self.enable_stepper(True, curtime)
        self.feed_chunk(abs(speed), min(max_len, 50.), curtime)

    def start_after_stop(self, speed, max_len, curtime=None):
        """Start feed after stop_immediate(); delay until MCU pwm queue is idle."""
        if curtime is None:
            curtime = self.reactor.monotonic()
        mcu = self.step.get_mcu()
        pt_now = mcu.estimated_print_time(curtime)
        delay = max(0.12, self.last_enable_time - pt_now + 0.05)
        self._retract_not_before = curtime + delay
        self._pending_after_stop = (speed, max_len)
        if self._after_stop_timer is not None:
            self.reactor.unregister_timer(self._after_stop_timer)
        self._after_stop_timer = self.reactor.register_timer(
            self._start_after_stop_timer, self._retract_not_before)

    def _start_after_stop_timer(self, eventtime):
        self._after_stop_timer = None
        speed, max_len = self._pending_after_stop
        self._pending_after_stop = None
        self.start_continuous(speed, max_len, eventtime)
        return self.reactor.NEVER

    def stop_immediate(self, curtime=None, slot=0):
        if curtime is None:
            curtime = self.reactor.monotonic()
        if self._after_stop_timer is not None:
            self.reactor.unregister_timer(self._after_stop_timer)
            self._after_stop_timer = None
            self._pending_after_stop = None
        slot_gap = 0 # slot * MCU_PIN_EVENT_DELAY * 3
        pt = self._sched_print_time(curtime, 0.001 + slot_gap)
        pt_pwm = pt + MCU_PIN_EVENT_DELAY
        pt_pwm = max(pt_pwm, self.last_pulse_time + MCU_PIN_EVENT_DELAY)
        self.step.set_pwm(pt_pwm, 0)
        self.last_pulse_time = pt_pwm
        if self.stepenable is not None:
            pt_en = pt_pwm + MCU_PIN_EVENT_DELAY
            pt_en = max(pt_en, self.last_enable_time + MCU_PIN_EVENT_DELAY)
            self.stepenable.set_digital(pt_en, 0)
            self.last_enable_time = pt_en
        self.is_feeding = False
        self.bfeeder_on = False
        self.last_feed_speed = 0.
        self.cur_feed_len = 0.
        self.scheduled_len = 0.
        self.next_feed_time = pt_pwm + 0.05

    def maybe_extend_feed(self, max_len, curtime):
        if not self.is_feeding or not self.bfeeder_on:
            return
        print_time = self.step.get_mcu().estimated_print_time(curtime)
        if print_time + FEED_RENEW_MARGIN < self.next_feed_time:
            return
        remain = max_len - self.scheduled_len
        if remain <= 0.:
            if print_time >= self.next_feed_time - 0.05:
                self.is_feeding = False
            return
        scheduled_ahead = max(
            0., (self.next_feed_time - print_time) * self.last_feed_speed)
        chunk_len = remain - scheduled_ahead
        if chunk_len < 0.01:
            if print_time >= self.next_feed_time - 0.05:
                self.is_feeding = False
            return
        self.feed_chunk(self.last_feed_speed, min(chunk_len, 50.), curtime)

# unit class for filabuffer, include one motor and two signals for inlet and buffer.
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
        self.unit_state = UNIT_EMPTY
        self._init_phase = None
        self._cmd_move_max = None
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

    def is_running(self):
        if self.unit_state not in UNIT_RUN_STATES:
            return False
        if self.motor._after_stop_timer is not None:
            return True
        return self.motor.is_feeding or self.motor.bfeeder_on

    def is_selected(self):
        return self.fb.active_unit == self.name

    def sync_stable_state(self, force=False):
        if self.is_running():
            return
        if self.unit_state == UNIT_ERROR and not force:
            if not self._inlet_present and not self._buffer_present:
                self.unit_state = UNIT_EMPTY
            return
        self.unit_state = stable_state_from_sensors(
            self._inlet_present, self._buffer_present, self.is_selected())

    def set_run_state(self, state, init_phase=None):
        self.unit_state = state
        self._init_phase = init_phase

    def set_error_state(self):
        self.unit_state = UNIT_ERROR
        self._init_phase = None

    def clear_run_state(self):
        self._init_phase = None
        self.sync_stable_state()

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
        msg = ("filabuffer %s unit %s sensor %s: inlet %d->%d buffer %d->%d "
               "state=%s"
               % (self.fb.name, self.name, role,
                  int(old_inlet), int(self._inlet_present),
                  int(old_buffer), int(self._buffer_present),
                  self.unit_state))
        self.fb.log_sensor_msg(msg)
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
            'init_phase': self._init_phase,
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
        self._cmd_move_max_time = None
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
        self.feed_speed = config.getfloat('feed_speed', 10., above=0.)
        self.feed_speed_init = config.getfloat(
            'feed_speed_init', self.feed_speed * 2., above=0.)
        self.retract_len = config.getfloat('retract_len', 75., above=0.)
        self.retract_speed = config.getfloat(
            'retract_speed', self.feed_speed, above=0.)
        self.min_buffer_travel_mm = config.getfloat(
            'min_buffer_travel_mm', 6., above=0.)
        self.button_latency = config.getfloat('button_latency', 0.010, above=0.)
        self.hw_latency = config.getfloat('hw_latency', 0.002, above=0.)
        self.full_stop_mode = config.getchoice(
            'full_stop_mode',
            {'host': 'host', 'hardware_enable': 'hardware_enable'})
        self.sensor_log = config.getboolean('sensor_log', False)
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
            'FILA_BUFFER_STATUS', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_STATUS,
            desc=self.cmd_FILA_BUFFER_STATUS_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_SELECT_UNIT', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_SELECT_UNIT,
            desc=self.cmd_FILA_BUFFER_SELECT_UNIT_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_SYNC_SENSORS', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_SYNC_SENSORS,
            desc=self.cmd_FILA_BUFFER_SYNC_SENSORS_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_UNIT_MOVE', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_UNIT_MOVE,
            desc=self.cmd_FILA_BUFFER_UNIT_MOVE_help)
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
        self.reactor.register_timer(
            self._startup_sync_event,
            self.reactor.monotonic() + STARTUP_IGNORE_TIME)

    def _startup_sync_event(self, eventtime):
        self._sync_all_from_linked_sensors(force=True, clear_error=False)
        return self.reactor.NEVER

    def _read_linked_sensor_present(self, obj):
        if hasattr(obj, 'bPresent'):
            return bool(obj.bPresent)
        runout = getattr(obj, 'runout_helper', None)
        if runout is not None:
            return bool(runout.filament_present)
        return None

    def _pull_linked_sensor_states(self):
        for obj in self.printer.objects.values():
            link = getattr(obj, 'filabuffer_link', None)
            if link is None or link[0] != self.name:
                continue
            _, unit_name, role = link
            unit = self.units.get(unit_name)
            if unit is None:
                continue
            present = self._read_linked_sensor_present(obj)
            if present is None:
                continue
            if role == 'inlet':
                unit._inlet_present = present
            elif role == 'buffer':
                unit._buffer_present = present

    def _sync_all_from_linked_sensors(self, force=False, clear_error=True):
        self._pull_linked_sensor_states()
        self._clear_cmd_moves()
        self._stop_all_motors()
        self.feed_session_start = 0.
        for u in self.units.values():
            u._init_phase = None
            u.sync_stable_state(force=force)
        if clear_error:
            self.error_msg = None
            if self.mode == MODE_ERROR:
                self.mode = MODE_DISABLED
        if self.mode == MODE_DISABLED:
            self._deactivate_all_units()

    def _verify_other_units_state(self, unit):
        """Other units must be empty or ready before SELECT_UNIT."""
        allowed = (UNIT_EMPTY, UNIT_READY)
        for name, u in self.units.items():
            if u is unit:
                continue
            if u.unit_state not in allowed:
                raise self.gcode.error(
                    "Unit %s state=%s, other units must be empty or ready"
                    % (name, u.unit_state))

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

    def _check_buffer_exclusive(self):
        if self.sensors.state & BUFF_FULL:
            if self.sensors.state & (BUFF_JAM | BUFF_LOW):
                return False
        return True

    def _count_buffer_filament(self, exclude_init=False):
        """Count units with buffer_present. Init unit may touch buffer briefly."""
        n = 0
        for u in self.units.values():
            if exclude_init and u.unit_state == UNIT_INIT:
                continue
            if u.buffer_present:
                n += 1
        return n

    def _check_unit_mutex(self):
        if self._count_buffer_filament(exclude_init=True) > 1:
            self._enter_error("multi_buffer_filament")
            return False
        return True

    def _pause_prefix(self):
        return "PAUSE\n" if self.pause_on_error else ""

    def _clear_cmd_moves(self):
        for u in self.units.values():
            u._cmd_move_max = None
        self._cmd_move_max_time = None

    def _deactivate_all_units(self):
        """Clear active_unit so no unit is UNIT_ACTIVE (is_selected=False)."""
        self.active_unit = None
        for u in self.units.values():
            if u.is_running():
                continue
            u.sync_stable_state(force=True)

    def _enter_error(self, msg, enqueue_break=False, curtime=None):
        self.mode = MODE_ERROR
        self.error_msg = msg
        self._deactivate_all_units()
        self._clear_cmd_moves()
        running = [u for u in self.units.values() if u.is_running()]
        self._stop_all_motors(curtime)
        for u in running:
            u.set_error_state()
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

    def log_sensor_msg(self, msg):
        if not self.sensor_log:
            return
        logging.info(msg)
        self.gcode.respond_info(msg)

    def note_unit_change(self, unit, eventtime, old_inlet, old_buffer):
        old_state = unit.unit_state
        msg = ("note_unit_change() : filabuffer %s unit %s change: inlet %d->%d buffer %d->%d "
               "old_state=%s mode=%s"
               % (self.name, unit.name,
                  int(old_inlet), int(unit.inlet_present),
                  int(old_buffer), int(unit.buffer_present),
                  old_state, self.mode))
        self.log_sensor_msg(msg)
        if not self._check_unit_mutex():
            self.log_sensor_msg(
                "filabuffer %s unit %s: ignored (unit mutex)"
                % (self.name, unit.name))
            return
        if unit.unit_state == UNIT_INIT:
            self._handle_init_edges(unit, eventtime, old_inlet, old_buffer)
            if unit.unit_state != old_state:
                self._log_unit_state_change(unit, old_state)
            return
        if unit.unit_state == UNIT_FEED:
            self._handle_feed_edges(unit, eventtime, old_inlet, old_buffer)
            if unit.unit_state != old_state:
                self._log_unit_state_change(unit, old_state)
            return
        unit.sync_stable_state()
        self.log_sensor_msg("unit %s state %s -> %s" % (unit.name, old_state, unit.unit_state))
        # unit from empty to ready, start init.
        if (self.mode == MODE_WORK
                and old_state == UNIT_EMPTY
                and unit.unit_state == UNIT_READY): 
            self.log_sensor_msg("unit %s from empty to ready, start init." % (unit.name))
            self._start_unit_init(unit, eventtime)
            self._log_unit_state_change(unit, old_state)
            return
        unit_a = self._active()
        if unit is unit_a and unit.unit_state == UNIT_ACTIVE:
            if old_inlet and not unit.inlet_present:
                self._on_runout(eventtime)
            elif old_buffer and not unit.buffer_present:
                if self.mode == MODE_WORK:
                    self._on_break(eventtime)
        if unit.unit_state != old_state:
            self._log_unit_state_change(unit, old_state)

    def _log_unit_state_change(self, unit, old_state):
        self.log_sensor_msg(
            "filabuffer %s unit %s state %s -> %s"
            % (self.name, unit.name, old_state, unit.unit_state))

    def _on_jam(self, eventtime):
        self._enter_error("jam", curtime=eventtime)

    def _sync_work_feed(self, eventtime):
        """Level-triggered work feed: LOW and not FULL -> run; else stop."""
        if self.mode != MODE_WORK:
            return
        unit = self._active()
        if unit is None or unit.unit_state not in (
                UNIT_READY, UNIT_ACTIVE, UNIT_FEED):
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
        if unit.unit_state == UNIT_FEED:
            unit.clear_run_state()
        self.feed_session_start = 0.

    def _on_runout(self, eventtime):
        unit = self._active()
        if unit is None:
            return
        unit.motor.stop_immediate(eventtime)
        unit.sync_stable_state()
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
        unit.set_error_state()
        self._enter_error("break", enqueue_break=True)

    def _start_unit_feed(self, unit, speed, max_len, eventtime):
        unit.set_run_state(UNIT_FEED)
        unit.motor.start_continuous(speed, max_len, eventtime)
        if self.feed_session_start <= 0.:
            self.feed_session_start = eventtime

    def _start_unit_init(self, unit, eventtime):
        if not unit.inlet_present:
            return
        unit.set_run_state(UNIT_INIT, INIT_PHASE_FORWARD)
        unit.motor._withdraw = False
        unit.motor.start_continuous(
            self.feed_speed_init, self.init_max_feed_len, eventtime)
        self.feed_session_start = eventtime
        logging.info("filabuffer %s %s init forward start",
                     self.name, unit.name)

    def _start_select_unit_feed(self, unit, eventtime):
        unit.set_run_state(UNIT_FEED)
        unit.motor._withdraw = False
        unit.motor.start_continuous(
            self.feed_speed_init, self.init_max_feed_len, eventtime)
        self.feed_session_start = eventtime
        logging.info("filabuffer %s %s select_unit feed start",
                     self.name, unit.name)

    def _handle_init_edges(self, unit, eventtime, old_inlet, old_buffer):
        if unit._init_phase == INIT_PHASE_FORWARD:
            if unit.buffer_present and not old_buffer:
                unit.motor.stop_immediate(eventtime)
                unit.set_run_state(UNIT_INIT, INIT_PHASE_RETRACT)
                self.feed_session_start = eventtime
                unit.motor.start_after_stop(
                    -self.retract_speed, self.retract_len, eventtime)
                logging.info("filabuffer %s %s init retract scheduled",
                             self.name, unit.name)
            elif not unit.inlet_present and old_inlet:
                unit.motor.stop_immediate(eventtime)
                unit.set_error_state()
                self._enter_error("init_runout", curtime=eventtime)
        elif unit._init_phase == INIT_PHASE_RETRACT:
            # 回撤走满 retract_len 后再判 buffer；回撤中 buffer 变 false 不停止
            if not unit.inlet_present and old_inlet:
                unit.motor.stop_immediate(eventtime)
                unit.set_error_state()
                self._enter_error("init_runout", curtime=eventtime)

    def _init_retract_distance_done(self, unit, eventtime):
        """True when scheduled_len >= retract_len and last PWM chunk ended."""
        if unit.motor._after_stop_timer is not None:
            return False
        if unit.motor.scheduled_len < self.retract_len - 0.05:
            return False
        mcu = unit.motor.step.get_mcu()
        print_time = mcu.estimated_print_time(eventtime)
        return print_time >= unit.motor.next_feed_time - 0.05

    def _complete_init_retract_check(self, unit, eventtime):
        """After full retract distance: buffer false -> ready, true -> fail."""
        unit.motor.stop_immediate(eventtime)
        unit.motor._withdraw = False
        self._pull_linked_sensor_states()
        if unit.buffer_present:
            unit.set_error_state()
            self._enter_error("init_retract_fail", curtime=eventtime)
            logging.error(
                "filabuffer %s %s init retract fail: buffer still present",
                self.name, unit.name)
            return
        unit.clear_run_state()
        self.feed_session_start = 0.
        logging.info("filabuffer %s %s init done -> %s",
                     self.name, unit.name, unit.unit_state)

    def _handle_feed_edges(self, unit, eventtime, old_inlet, old_buffer):
        if unit.buffer_present and not old_buffer:
            unit.motor.stop_immediate(eventtime)
            unit.clear_run_state()
            logging.info("filabuffer %s %s feed done -> active",
                         self.name, unit.name)
            return
        if not unit.is_selected():
            return
        if old_inlet and not unit.inlet_present:
            self._on_runout(eventtime)
        elif old_buffer and not unit.buffer_present:
            self._on_break(eventtime)

    def _on_init_retract_fail(self, eventtime, unit):
        unit.motor.stop_immediate(eventtime)
        unit.set_error_state()
        self._enter_error("init_retract_fail", curtime=eventtime)

    def _require_work_mode(self):
        if self.mode != MODE_WORK:
            raise self.gcode.error(
                "filabuffer %s must be in work mode (FILA_BUFFER_START)"
                % (self.name,))

    def _verify_can_enter_work(self):
        if self.mode == MODE_ERROR:
            raise self.gcode.error(
                "filabuffer %s in error: %s"
                % (self.name, self.error_msg or "unknown"))
        if self.sensors.state & BUFF_JAM:
            raise self.gcode.error("filabuffer %s buffer jam" % (self.name,))
        if not self._check_buffer_exclusive():
            raise self.gcode.error(
                "filabuffer %s buffer full exclusive violation"
                % (self.name,))
        allowed = (UNIT_EMPTY, UNIT_READY, UNIT_BUFFERED)
        for name, u in self.units.items():
            u.sync_stable_state()
            if u.is_running():
                raise self.gcode.error(
                    "Unit %s is busy (state=%s)" % (name, u.unit_state))
            if u.unit_state not in allowed:
                raise self.gcode.error(
                    "Unit %s state=%s, all units must be empty or ready"
                    % (name, u.unit_state))
        if self._count_buffer_filament() > 1: #最多一个缓冲区有料.
            raise self.gcode.error(
                "Over one buffer filament detected, buffer maybe collision")

    def is_printing(self):
        idle = self.printer.lookup_object('idle_timeout')
        return idle.get_status(self.reactor.monotonic())['state'] == 'Printing'

    def _watchdog_cmd_moves(self, eventtime):
        """UNIT_MOVE: always run (even when mode is disabled/error)."""
        for unit in self.units.values():
            move_max = unit._cmd_move_max
            if move_max is None:
                continue
            if unit.motor.is_feeding or unit.motor.bfeeder_on:
                unit.motor.maybe_extend_feed(move_max, eventtime)
                print_time = unit.motor.step.get_mcu().estimated_print_time(
                    eventtime)
                if (unit.motor.scheduled_len >= move_max
                        and print_time >= unit.motor.next_feed_time - 0.05):
                    unit.motor.stop_immediate(eventtime)
                    unit._cmd_move_max = None
                    self.feed_session_start = 0.
                    self._cmd_move_max_time = None
            else:
                unit._cmd_move_max = None
        if self.feed_session_start > 0. and self._cmd_move_max_time is not None:
            if eventtime - self.feed_session_start > self._cmd_move_max_time:
                for u in self.units.values():
                    if u._cmd_move_max is not None:
                        u.motor.stop_immediate(eventtime)
                self._clear_cmd_moves()
                self.feed_session_start = 0.
                logging.error("filabuffer %s unit_move timeout", self.name)

    def _watchdog_event(self, eventtime):
        self._watchdog_cmd_moves(eventtime)
        if self.mode in (MODE_ERROR, MODE_DISABLED):
            return eventtime + self.watchdog_time
        if self.mode == MODE_WORK:
            self._sync_work_feed(eventtime)
        max_len = self.max_feed_len
        max_time = self.max_feed_time
        for unit in self.units.values():
            if (unit.unit_state == UNIT_INIT
                    and unit._init_phase == INIT_PHASE_RETRACT
                    and eventtime >= unit.motor._retract_not_before
                    and not unit.motor.is_feeding
                    and unit.motor._after_stop_timer is None
                    and unit.motor.scheduled_len < self.retract_len - 0.05
                    and eventtime - unit.motor._retract_not_before > 2.0):
                self._on_init_retract_fail(eventtime, unit)
                return eventtime + self.watchdog_time
            if unit.is_running():
                umax_len = max_len
                if unit.unit_state == UNIT_INIT:
                    if unit._init_phase == INIT_PHASE_RETRACT:
                        umax_len = self.retract_len
                        if (unit.motor.scheduled_len < umax_len - 0.01
                                and not self._init_retract_distance_done(
                                    unit, eventtime)):
                            unit.motor.maybe_extend_feed(umax_len, eventtime)
                        if self._init_retract_distance_done(unit, eventtime):
                            self._complete_init_retract_check(
                                unit, eventtime)
                            return eventtime + self.watchdog_time
                    else:
                        umax_len = min(self.init_max_feed_len, 50.)
                        unit.motor.maybe_extend_feed(umax_len, eventtime)
                else:
                    unit.motor.maybe_extend_feed(umax_len, eventtime)
                if unit.motor.scheduled_len > umax_len + 0.5:
                    self._feed_timeout(eventtime)
                    return eventtime + self.watchdog_time
        sess_max_time = max_time
        for u in self.units.values():
            if u.unit_state == UNIT_INIT:
                if u._init_phase == INIT_PHASE_RETRACT:
                    sess_max_time = (self.retract_len
                                     / max(0.01, self.retract_speed) + 5.)
                else:
                    sess_max_time = self.init_max_feed_time
                break
        if (self.feed_session_start > 0.
                and self._cmd_move_max_time is None
                and eventtime - self.feed_session_start > sess_max_time):
            self._feed_timeout(eventtime)
            self.feed_session_start = 0.
        return eventtime + self.watchdog_time

    def _feed_timeout(self, eventtime):
        self._enter_error("feed_timeout", curtime=eventtime)

    cmd_FILA_BUFFER_SELECT_help = "Set default active feed unit name"
    def cmd_FILA_BUFFER_SELECT(self, gcmd):
        unit = self._get_unit(gcmd.get('UNIT'))
        self.active_unit = unit.name
        unit.sync_stable_state()
        gcmd.respond_info("filabuffer %s active unit: %s state=%s"
                          % (self.name, unit.name, unit.unit_state))

    cmd_FILA_BUFFER_SELECT_UNIT_help = (
        "Feed ready unit until buffer sensor, become active")
    def cmd_FILA_BUFFER_SELECT_UNIT(self, gcmd):
        self._require_work_mode()
        unit = self._get_unit(gcmd.get('UNIT'))
        self._pull_linked_sensor_states()
        for u in self.units.values():
            u.sync_stable_state()
        self._verify_other_units_state(unit) # must be empty or ready.
        if unit.unit_state not in (UNIT_READY, UNIT_BUFFERED):
            raise gcmd.error(
                "Unit %s must be ready (inlet=1 buffer=0) or buffered (inlet=1 buffer=1), state=%s"
                % (unit.name, unit.unit_state))
        self.active_unit = unit.name
        self._start_select_unit_feed(
            unit, self.reactor.monotonic())
        gcmd.respond_info("filabuffer %s feeding %s to active"
                          % (self.name, unit.name))

    cmd_FILA_BUFFER_UNIT_MOVE_help = (
        "Move unit motor: SPEED mm/s, LENGTH mm (negative=reverse), eg: FILA_BUFFER_UNIT_MOVE BUFFER=buffer0 UNIT=unit0 SPEED=5.0 LENGTH=10.0")
    def cmd_FILA_BUFFER_UNIT_MOVE(self, gcmd):
        unit = self._get_unit(gcmd.get('UNIT'))
        speed = gcmd.get_float('SPEED', 5., above=0.)
        length = gcmd.get_float('LENGTH')
        if length == 0.:
            raise gcmd.error("LENGTH must be non-zero")
        dist = abs(length)
        move_len_cap = max(self.init_max_feed_len, self.max_feed_len)
        if dist > move_len_cap:
            raise gcmd.error(
                "LENGTH %.2f exceeds max move length %.2f"
                % (dist, move_len_cap))
        if unit.is_running():
            raise gcmd.error("Unit %s is busy (state=%s)"
                             % (unit.name, unit.unit_state))
        for u in self.units.values():
            if u._cmd_move_max is not None:
                raise gcmd.error("Unit %s manual move in progress"
                                 % (u.name,))
        eventtime = self.reactor.monotonic()
        unit._cmd_move_max = dist
        if length < 0.:
            unit.motor.start_continuous(-speed, dist, eventtime)
        else:
            unit.motor._withdraw = False
            unit.motor.start_continuous(speed, dist, eventtime)
        move_time = dist / speed + 5.
        self.feed_session_start = eventtime
        self._cmd_move_max_time = move_time
        gcmd.respond_info(
            "filabuffer %s %s move %s %.2f mm @ %.2f mm/s"
            % (self.name, unit.name,
               "reverse" if length < 0. else "forward", dist, speed))

    cmd_FILA_BUFFER_START_help = (
        "Start filabuffer: MODE=work (default) or disabled")
    def cmd_FILA_BUFFER_START(self, gcmd):
        mode = gcmd.get('MODE', MODE_WORK).lower()
        if mode not in (MODE_DISABLED, MODE_WORK):
            raise gcmd.error("Invalid MODE (use work or disabled)")
        if mode == MODE_WORK:
            self._pull_linked_sensor_states()
            for u in self.units.values():
                u.sync_stable_state()
            self._verify_can_enter_work()
            self._deactivate_all_units()
            self.mode = MODE_WORK
            self.error_msg = None
            self._sync_work_feed(self.reactor.monotonic())
        else:
            self.mode = MODE_DISABLED
            self.error_msg = None
            self._clear_cmd_moves()
            self._stop_all_motors()
            self.feed_session_start = 0.
            for u in self.units.values():
                u._init_phase = None
            self._deactivate_all_units()
        gcmd.respond_info("filabuffer %s mode: %s" % (self.name, self.mode))

    cmd_FILA_BUFFER_STOP_help = "Stop filabuffer"
    def cmd_FILA_BUFFER_STOP(self, gcmd):
        self.mode = MODE_DISABLED
        self.error_msg = None
        self._clear_cmd_moves()
        self.feed_session_start = 0.
        self._stop_all_motors()
        for u in self.units.values():
            u._init_phase = None
        self._deactivate_all_units()
        gcmd.respond_info("filabuffer %s stopped" % (self.name,))

    cmd_FILA_BUFFER_INIT_FILAMENT_help = (
        "Init filament on one unit (optional; auto-init on insert)")
    def cmd_FILA_BUFFER_INIT_FILAMENT(self, gcmd):
        self._require_work_mode()
        unit = self._get_unit(gcmd.get('UNIT'))
        self._pull_linked_sensor_states()
        unit.sync_stable_state()
        if unit.unit_state == UNIT_EMPTY and unit.inlet_present:
            self._start_unit_init(unit, self.reactor.monotonic())
        elif unit.unit_state == UNIT_EMPTY:
            gcmd.respond_info("Waiting insert on %s" % (unit.name,))
        else:
            gcmd.respond_info("Unit %s state=%s" % (unit.name, unit.unit_state))

    cmd_FILA_BUFFER_SYNC_SENSORS_help = (
        "Read linked filament sensors and sync unit states. CLEAR_ERROR=1 clears error and sets disabled (default 1)")
    def cmd_FILA_BUFFER_SYNC_SENSORS(self, gcmd):
        clear_error = gcmd.get_int('CLEAR_ERROR', 1, minval=0, maxval=1)
        self._sync_all_from_linked_sensors(
            force=True, clear_error=bool(clear_error))
        gcmd.respond_info("filabuffer %s sensors synced" % (self.name,))

    cmd_FILA_BUFFER_STATUS_help = "Report filabuffer status"
    def cmd_FILA_BUFFER_STATUS(self, gcmd):
        bs = self.sensors.get_status()
        msg = ("filabuffer %s: mode=%s active=%s error=%s full_stop=%s "
               "jam=%d low=%d full=%d" % (
                   self.name, self.mode, self.active_unit, self.error_msg,
                   self.full_stop_mode, bs['jam'], bs['low'], bs['full']))
        for name, unit in sorted(self.units.items()):
            st = unit.get_status()
            msg += ("\n %s: %s inlet=%d buf=%d feed=%d len=%.1f phase=%s" % (
                name, st['state'], st['inlet'], st['buffer'],
                st['is_feeding'], st['cur_feed_len'], st['init_phase']))
        gcmd.respond_info(msg)

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'mode': self.mode,
            'active_unit': self.active_unit,
            'error': self.error_msg,
            'sensor_log': self.sensor_log,
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
