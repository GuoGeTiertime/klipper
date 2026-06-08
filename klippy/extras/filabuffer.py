# Automatic filament buffer with multi feeders
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import pwm_cycle_time

# Buffer sensor bits (pin order: jam, low, full)
BUFF_JAM = 0x01
BUFF_LOW = 0x02
BUFF_FULL = 0x04

MODE_DISABLED = 'disabled'
MODE_WORK = 'work'
MODE_ERROR = 'error'

FEEDER_EMPTY = 'empty'
FEEDER_INSERT = 'insert'
FEEDER_READY = 'ready'
FEEDER_BUFFERED = 'buffered'
FEEDER_ACTIVE = 'active'
FEEDER_ERROR = 'error'
FEEDER_INIT = 'init'
FEEDER_RUNOUT = 'runout'
FEEDER_RETRACT = 'retract'

FEEDER_RUN_STATES = (FEEDER_INIT, FEEDER_RETRACT)

INIT_PHASE_FORWARD = 'forward'
INIT_PHASE_RETRACT = 'retract'
# Init 回撤：走满 retract_len（建议 50~100mm）@ retract_speed（默认同 feed_speed）；
# 结束后 buffer false->ready，true->init_retract_fail。回撤中 buffer 变 false 不停止。

ERROR_JAM = "jam"
ERROR_RUNOUT = "runout"
ERROR_BREAK = "break"
ERROR_FULL_EXCLUSIVE_VIOLATION = "full_exclusive_violation"
ERROR_FEED_TIMEOUT = "feed_timeout"
ERROR_INIT_FEED_FAIL = "init_feed_fail"
ERROR_INIT_RETRACT_FAIL = "init_retract_fail"
ERROR_INIT_RUNOUT = "init_runout"
ERROR_MULTI_BUFFER_FILAMENT = "multi_buffer_filament"
ERROR_INIT_SENSOR_INVALID = "init_sensor_invalid"
ERROR_RETRACT_FAIL = "retract_fail"
ERROR_FEED_EXTRUDER_JAM = "extruder_jam"
ERROR_FEED_SLIP = "feed_slip"


# Min print_time gap between MCU digital_out/pwm events: [filabuffer] pinout_delay
# Renew software-PWM queue this many seconds before chunk ends (print_time)
FEED_RENEW_MARGIN = 0.15
# FilaMotor fixed timing (not from config)
MOTOR_MAX_CHUNK_TIME = 4.0
MOTOR_ACCEL_SEGMENT_TIME = 0.100
MOTOR_ACCEL_FRACTION = 0.25
MOTOR_STEP_DUTY = 0.333
MOTOR_DISTANCE_EPS = 0.000001


# Per-feeder options use suffix _0, _1, _2. Shared mechanics (microstep,
# rotate_distance, gear_ratio, max_speed, full_steps_per_rotation) fall back
# from _0 then unsuffixed name when not set on higher indexes.
FEEDER_PIN_OPTIONS = ('inlet_pin', 'buffer_pin', 'step_pin', 'dir_pin',
                    'enable_pin')
FEEDER_SHARED_OPTIONS = ('microstep', 'full_steps_per_rotation',
                       'rotate_distance', 'gear_ratio')


def _option_keys(option, idx):
    keys = ['%s_%d' % (option, idx)]
    if idx > 0:
        keys.append('%s_0' % (option,))
    keys.append(option)
    return keys


def _has_feeder_index(config, idx):
    """True only when step_pin is defined for this feeder index (no _0 fallback)."""
    section = config.get_name()
    if config.fileconfig.has_option(section, 'step_pin_%d' % (idx,)):
        return True
    return idx == 0 and config.fileconfig.has_option(section, 'step_pin')


def _get_feeder_option(config, option, idx, default=None):
    section = config.get_name()
    for key in _option_keys(option, idx):
        if config.fileconfig.has_option(section, key):
            return config.get(key)
    return default


def _get_feeder_int(config, option, idx, default, **kwargs):
    section = config.get_name()
    for key in _option_keys(option, idx):
        if config.fileconfig.has_option(section, key):
            return config.getint(key, **kwargs)
    return default


def _get_feeder_float(config, option, idx, default, **kwargs):
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


def _get_feeder_gear_ratio(config, idx):
    section = config.get_name()
    for key in _option_keys('gear_ratio', idx):
        if config.fileconfig.has_option(section, key):
            return _parse_gear_ratio_key(config, key)
    return 1.


def _consume_feeder_config_options(config):
    """Mark per-feeder options read so check_unused_options passes."""
    section = config.get_name()
    fc = config.fileconfig
    for key in config.get_prefix_options('gear_ratio'):
        if fc.has_option(section, key):
            _parse_gear_ratio_key(config, key)
    for option in FEEDER_SHARED_OPTIONS:
        if option == 'gear_ratio':
            continue
        for key in config.get_prefix_options(option):
            if not fc.has_option(section, key):
                continue
            if option in ('microstep', 'full_steps_per_rotation'):
                config.getint(key)
            else:
                config.getfloat(key)
    for option in FEEDER_PIN_OPTIONS + ('feeder_name',):
        for key in config.get_prefix_options(option):
            if fc.has_option(section, key):
                config.get(key)


def _get_config_feeder_name(config, idx):
    name = _get_feeder_option(config, 'feeder_name', idx, default=None)
    if name is None:
        name = 'feeder%d' % (idx,)
    return name


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


# motor move act_type
ACT_TYPE_CMD_MOVE = 'cmd_move'
ACT_TYPE_INIT_FORWARD = 'init_forward'
ACT_TYPE_INIT_RETRACT = 'init_retract'
ACT_TYPE_FEED = 'feed'
ACT_TYPE_RETRACT = 'retract'

class FilaMotor:
    """PWM step/dir/enable driver. start/stop only; on_stop_cb from __init__."""

    def __init__(self, reactor, config, feeder_index, mm_per_pulse,
                 pinout_delay, on_stop_cb=None, name=''):
        self.reactor = reactor
        self.pinout_delay = pinout_delay
        self.name = name or ('motor_%d' % feeder_index)
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
        self._chunk_speed = 0.
        self._speed = 0.
        self._target_speed = 0.
        self._forward = True
        self._act_type = None
        self.cycle_time = 0.1
        self.last_pt = 0.
        self.chunk_end_pt = 0.
        self._chunk_renew_margin = 0.
        self._chunk_gen = 0
        fb_name = config.get_name().split()[-1]
        ppins = config.get_printer().lookup_object('pins')
        step_pin = _get_feeder_option(config, 'step_pin', feeder_index)
        if step_pin is None:
            raise config.error(
                "filabuffer %s feeder %d: step_pin_%d is required"
                % (fb_name, feeder_index, feeder_index))
        pin_params = ppins.lookup_pin(step_pin, can_invert=True)
        self.mcu = pin_params['chip']
        self.step = pwm_cycle_time.MCU_pwm_cycle(
            pin_params, self.cycle_time, 0., 0.)
        self.dir = None
        dir_pin = _get_feeder_option(config, 'dir_pin', feeder_index)
        if dir_pin is not None:
            self.dir = ppins.setup_pin('digital_out', dir_pin)
            self.dir.setup_max_duration(0.)
        self.enable = None
        enable_pin = _get_feeder_option(config, 'enable_pin', feeder_index)
        if enable_pin is not None:
            self.enable = ppins.setup_pin('digital_out', enable_pin)
            self.enable.setup_max_duration(0.)

    def is_moving(self):
        return self._active

    def get_move_mm(self):
        return self._move_mm

    def get_total_mm(self):
        return self.total_mm

    def get_position_mm(self):
        """累计净位置 mm，含当前进行中行程（含未 flush 的 chunk）"""
        pos = self.total_mm + self._move_mm
        if not self._active or self._chunk_mm <= 0.:
            return pos
        curtime = self.reactor.monotonic()
        pt = self.mcu.estimated_print_time(curtime)
        end_pt = self._chunk_start_pt + self._chunk_mm / self._chunk_speed
        if pt <= self._chunk_start_pt:
            partial = 0.
        elif pt >= end_pt:
            partial = self._chunk_mm
        else:
            partial = (pt - self._chunk_start_pt) * self._chunk_speed
        if self._forward:
            return pos + partial
        return pos - partial

    def get_act_type(self):
        if not self._active:
            return None
        return self._act_type

    @staticmethod
    def get_renew_margin(duration, has_more):
        base_margin = FEED_RENEW_MARGIN if has_more else 0.05
        return min(base_margin, duration * 0.25)

    @staticmethod
    def build_speed_segments(distance, target_speed,
                             segment_time=MOTOR_ACCEL_SEGMENT_TIME,
                             accel_fraction=MOTOR_ACCEL_FRACTION):
        remain = abs(distance)
        target_speed = max(0.01, abs(target_speed))
        speed = 0.
        segments = []
        while remain > MOTOR_DISTANCE_EPS:
            speed = (min(target_speed, speed + target_speed * accel_fraction)
                     if speed < target_speed else target_speed)
            duration = min(segment_time, remain / speed)
            segments.append((duration, speed))
            remain -= speed * duration
        return segments

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
        self._target_speed = max(0.01, abs(speed))
        self._speed = 0.
        self._act_type = act_type
        curtime = self.reactor.monotonic()
        if self.enable is not None:
            pt = self._sched_print_time(curtime, self.pinout_delay)
            self.enable.set_digital(pt, 1)
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
        self._move_mm = 0.
        if call_stop_cb and (self._on_stop_cb is not None):
            self._on_stop_cb(act_type, reason)

    def _cancel_timer(self):
        if self._timer is not None:
            self.reactor.unregister_timer(self._timer)
            self._timer = None

    def _add_move_mm(self, delta):
        if delta <= 0.:
            return
        if self._forward:
            self._move_mm += delta
        else:
            self._move_mm -= delta

    def _flush_current_chunk(self, halt_pt=None):
        """Add current chunk to _move_mm; halt_pt=None means chunk finished."""
        if self._chunk_mm <= 0.:
            return
        if halt_pt is None:
            self._add_move_mm(self._chunk_mm)
        else:
            end_pt = self._chunk_start_pt + self._chunk_mm / self._chunk_speed
            if halt_pt <= self._chunk_start_pt:
                partial = 0.
            elif halt_pt >= end_pt:
                partial = self._chunk_mm
            else:
                partial = (halt_pt - self._chunk_start_pt) * self._chunk_speed
            self._add_move_mm(partial)
        self._chunk_mm = 0.
        self._chunk_speed = 0.

    def _halt_pwm(self):
        curtime = self.reactor.monotonic()
        pt = self._sched_print_time(curtime, self.pinout_delay)
        pt_pwm = max(pt, self.last_pt + self.pinout_delay)
        self._flush_current_chunk(pt_pwm)
        # Keep enable asserted after stopping pulses so the motor holds torque.
        self.step.set_pwm_cycle(pt_pwm, 0., self.cycle_time)
        self.last_pt = pt_pwm
        self.chunk_end_pt = pt_pwm + 0.05
        self._chunk_renew_margin = 0.

    def _sched_print_time(self, curtime, gap=0.):
        return max(self.mcu.estimated_print_time(curtime) + self.pinout_delay,
                   self.last_pt + gap)

    def _get_next_chunk(self):
        if self._speed < self._target_speed:
            speed = min(self._target_speed,
                        self._speed + self._target_speed
                        * MOTOR_ACCEL_FRACTION)
            max_time = MOTOR_ACCEL_SEGMENT_TIME
        else:
            speed = self._target_speed
            max_time = MOTOR_MAX_CHUNK_TIME
        duration = min(max_time, self._remain / speed)
        return speed * duration, speed

    def _run_chunk(self, curtime, move_gen):
        if not self._active or move_gen != self._gen:
            return
        if self._remain <= MOTOR_DISTANCE_EPS:
            self._flush_current_chunk()
            self._end_move(self._act_type, 'complete')
            return
        length, speed = self._get_next_chunk()
        pt = max(self._sched_print_time(curtime, self.pinout_delay),
                 self.chunk_end_pt)
        ct = self.mm_per_pulse / speed
        self.cycle_time = ct
        if self.dir is not None:
            self.dir.set_digital(pt, 1 if self._forward else 0)
        self.step.set_pwm_cycle(pt, MOTOR_STEP_DUTY, self.cycle_time)
        self.last_pt = pt
        self.chunk_end_pt = pt + length / speed
        duration = self.chunk_end_pt - pt
        self._chunk_start_pt = pt
        self._chunk_mm = length
        self._chunk_speed = speed
        self._remain -= length
        self._speed = speed
        if move_gen != self._gen:
            return
        mcu = self.mcu
        margin = self.get_renew_margin(duration, self._remain > 0.)
        self._chunk_renew_margin = margin
        delay = max(0.01, self.chunk_end_pt - margin
                    - mcu.estimated_print_time(curtime))
        self._chunk_gen = move_gen
        self._cancel_timer()
        self._timer = self.reactor.register_timer(
            self._timer_event, curtime + delay)

    def _timer_event(self, eventtime):
        self._timer = None
        if not self._active or self._chunk_gen != self._gen:
            return self.reactor.NEVER
        pt = self.mcu.estimated_print_time(eventtime)
        if self._remain > 0.:
            if pt + self._chunk_renew_margin < self.chunk_end_pt:
                return eventtime + 0.01
            self._flush_current_chunk()
            # self._run_chunk(eventtime, self._chunk_gen)
            self._run_chunk(self.reactor.monotonic(), self._chunk_gen)
            return self.reactor.NEVER
        if pt < self.chunk_end_pt - 0.05:
            return eventtime + 0.01
        self._flush_current_chunk()
        self._end_move(self._act_type, 'complete')
        return self.reactor.NEVER


# FilaFeeder: one feeder with motor and inlet/buffer sensors.
class FilaFeeder:
    def __init__(self, fb, feeder_name, feeder_index, config):
        self.fb = fb
        self.name = feeder_name
        self.feeder_index = feeder_index
        self.reactor = fb.reactor
        if self.name in fb.feeders:
            raise config.error(
                "Duplicate feeder '%s' on filabuffer '%s'"
                % (self.name, fb.name))
        self.feeder_state = FEEDER_EMPTY
        self._init_phase = None
        self._inlet_present = False
        self._buffer_present = False
        self._error_msg = None
        self.feed_match_feeder_base = 0.
        self.feed_match_extruder_base = 0.
        self.feed_match_skip = True
        gearing = _get_feeder_gear_ratio(config, feeder_index)
        microstep = _get_feeder_int(
            config, 'microstep', feeder_index, 16, minval=1, maxval=256)
        full_steps = _get_feeder_int(
            config, 'full_steps_per_rotation', feeder_index, 200, minval=1)
        rotate_distance = _get_feeder_float(
            config, 'rotate_distance', feeder_index, 31.4, above=0.1)
        mm_per_pulse = rotate_distance / (microstep * full_steps * gearing)
        self.gearing = gearing
        self.motor = FilaMotor(
            fb.reactor, config, feeder_index, mm_per_pulse, fb.pinout_delay,
            on_stop_cb=self._on_motor_stop, name=feeder_name)
        self._gpio_roles = []
        gpio_pin_list = []
        inlet_pin = _get_feeder_option(config, 'inlet_pin', feeder_index)
        buffer_pin = _get_feeder_option(config, 'buffer_pin', feeder_index)
        if inlet_pin is not None:
            gpio_pin_list.append(inlet_pin)
            self._gpio_roles.append('inlet')
        if buffer_pin is not None:
            gpio_pin_list.append(buffer_pin)
            self._gpio_roles.append('buffer')
        if gpio_pin_list:
            buttons = config.get_printer().load_object(config, 'buttons')
            buttons.register_buttons(gpio_pin_list, self._gpio_handler)
        fb.feeders[self.name] = self

    @property
    def inlet_present(self):
        return self._inlet_present

    @property
    def buffer_present(self):
        return self._buffer_present

    def is_running(self):
        if self.feeder_state in FEEDER_RUN_STATES:
            return True
        return self.motor.is_moving()

    def motor_halt(self):
        self.motor.stop('halt', call_stop_cb=False)

    def note_feed_match_select(self, extruder_mm):
        self.feed_match_feeder_base = self._feed_match_motor_mm()
        self.feed_match_extruder_base = extruder_mm
        self.feed_match_skip = not self.buffer_present

    def note_feed_match_full(self, extruder_mm):
        self.feed_match_feeder_base = self._feed_match_motor_mm()
        self.feed_match_extruder_base = extruder_mm
        self.feed_match_skip = False

    def check_feed_match(self, extruder_mm, tolerance):
        if self.feed_match_skip or tolerance <= 0.:
            return None
        if not self.fb.is_printing():
            self.feed_match_skip = True
            return None
        feeder_mm = self._feed_match_motor_mm()
        diff = ((feeder_mm - self.feed_match_feeder_base)
                - (extruder_mm - self.feed_match_extruder_base))
        if diff > tolerance:  # feed more filament than extrude, feeder slip
            self.fb.log_sensor_msg("feed match slip, diff: %f (feeder_mm: %f, feeder_base: %f, extruder_mm: %f, extruder_base: %f)" %  
            (diff, feeder_mm, self.feed_match_feeder_base, extruder_mm, self.feed_match_extruder_base))
            return ERROR_FEED_SLIP
        elif diff < -tolerance:  # feed less filament than expected, extruder jam. 
            self.fb.log_sensor_msg("feed match extruder jam, diff: %f (feeder_mm: %f, feeder_base: %f, extruder_mm: %f, extruder_base: %f)" %  
            (diff, feeder_mm, self.feed_match_feeder_base, extruder_mm, self.feed_match_extruder_base))
            return ERROR_FEED_EXTRUDER_JAM
        return None

    def _feed_match_motor_mm(self):
        return self.motor.get_position_mm()

    # callback function when motor stop at max length.
    def _on_motor_stop(self, act_type, reason):
        self.fb.on_feeder_motor_stop(self, act_type, reason)

    def is_selected(self):
        return self.fb.active_feeder == self.name

    def _stable_state_from_sensors(self):
        if self.feeder_state == FEEDER_INIT: # init phase is not stable state, NOT CHANG STATUS
            return FEEDER_INIT  #keep init state
        if self.feeder_state == FEEDER_RETRACT:
            return FEEDER_RETRACT
        if not self._inlet_present and not self._buffer_present:
            return FEEDER_EMPTY #empty can cover all other states
        if self.feeder_state == FEEDER_ERROR:
            return FEEDER_ERROR # keep error state
        if self._inlet_present and not self._buffer_present:
            return FEEDER_READY if self.feeder_state == FEEDER_READY else FEEDER_INSERT
        if self._inlet_present and self._buffer_present:
            if self.is_selected():
                return FEEDER_ACTIVE
            return FEEDER_BUFFERED
        if not self._inlet_present and self._buffer_present:
            if self.is_selected():
                return FEEDER_RUNOUT
        return FEEDER_ERROR

    def sync_stable_state(self, force=False):
        self.feeder_state = self._stable_state_from_sensors()

    def set_run_state(self, state, init_phase=None):
        self.feeder_state = state
        self._error_msg = None
        self._init_phase = init_phase

    def set_error_state(self, errmsg):
        self.feeder_state = FEEDER_ERROR
        self._error_msg = errmsg
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
        msg = ("filabuffer %s feeder %s sensor %s: inlet %d->%d buffer %d->%d "
               "state=%s"
               % (self.fb.name, self.name, role,
                  int(old_inlet), int(self._inlet_present),
                  int(old_buffer), int(self._buffer_present),
                  self.feeder_state))
        self.fb.log_sensor_msg(msg)
        self.fb.note_feeder_change(
            self, eventtime, old_inlet, old_buffer)

    def _gpio_handler(self, eventtime, state):
        for i, role in enumerate(self._gpio_roles):
            self.update_sensor(role, eventtime, bool(state & (1 << i)))

    def get_status(self):
        return {
            'filabuffer': self.fb.name,
            'state': self.feeder_state,
            'errmsg': self._error_msg,
            'inlet': self._inlet_present,
            'buffer': self._buffer_present,
            'init_phase': self._init_phase,
            'cur_feed_len': self.motor.get_move_mm(),
            'total_mm': self.motor.get_total_mm(),
            'is_feeding': self.motor.is_moving(),
            'act_type': self.motor.get_act_type(),
            'gearing': self.gearing,
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
        self.feeders = {}
        self.active_feeder = None
        self.mode = MODE_DISABLED
        self.error_msg = None
        self.pinout_delay = config.getfloat('pinout_delay', 0.025, minval=0.010, maxval=0.050)
        self.watchdog_time = config.getfloat('watchdog_time', 0.5, above=0.05)
        self.feed_idle_time = config.getfloat('feed_idle_time', 10., above=0.)
        self._not_full_idle_since = 0.
        self.feed_match_tolerance = config.getfloat('feed_match_tolerance', 30., minval=0.)
        self.print_stats = None
        self.idle = None

        #define length of fila tube, 2 segments, 1. inlet to buffer, 2. buffer to extruder.
        self.len2buffer = config.getfloat('len2buffer', 1000., above=10.) # inlet to buffer. 
        self.len2extruder = config.getfloat('len2extruder', 2000., above=100.) # buffer to extruder.
        self.feed_len = config.getfloat('feed_len', 100.0, above=50.) # feed length when feeder is active.
        self.init_retract_len = config.getfloat('init_retract_len', 50., above=0.) # init retract length.
        # speed for feed, init, withdraw
        self.feed_speed = config.getfloat('feed_speed', 30., above=0.)
        self.init_speed = config.getfloat('init_speed', self.feed_speed, above=0.)
        self.retract_speed = config.getfloat('retract_speed', self.feed_speed, above=0.)

        # self.button_latency = config.getfloat('button_latency', 0.010, above=0.)
        # self.hw_latency = config.getfloat('hw_latency', 0.002, above=0.)
        self.sensor_log = config.getboolean('sensor_log', False)
        self.pause_on_error = config.getboolean('pause_on_error', False)
        if self.pause_on_error:
            self.printer.load_object(config, 'pause_resume')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.jam_gcode = gcode_macro.load_template(config, 'jam_gcode', '')
        self.low_timeout_gcode = gcode_macro.load_template(config, 'low_timeout_gcode', '')
        self.break_gcode = gcode_macro.load_template(config, 'break_gcode', '')
        self.runout_gcode = gcode_macro.load_template(config, 'runout_gcode', '')
        self.extruder_jam_gcode = gcode_macro.load_template(config, 'extruder_jam_gcode', '')
        self.feed_slip_gcode = gcode_macro.load_template(config, 'feed_slip_gcode', '')
        self.gcode_queue = GcodeQueue(self.printer)
        self.sensors = BufferSensors(config, self)
        self._watchdog_timer = self.reactor.register_timer(
            self._watchdog_event)
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
            'FILA_BUFFER_SELECT_FEEDER', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_SELECT_FEEDER,
            desc=self.cmd_FILA_BUFFER_SELECT_FEEDER_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_SYNC_SENSORS', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_SYNC_SENSORS,
            desc=self.cmd_FILA_BUFFER_SYNC_SENSORS_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_RESET_FEEDER', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_RESET_FEEDER,
            desc=self.cmd_FILA_BUFFER_RESET_FEEDER_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_FEEDER_MOVE', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_FEEDER_MOVE,
            desc=self.cmd_FILA_BUFFER_FEEDER_MOVE_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_RETRACT_FILAMENT', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_RETRACT_FILAMENT,
            desc=self.cmd_FILA_BUFFER_RETRACT_FILAMENT_help)
        self.gcode.register_mux_command(
            'FILA_BUFFER_SET_TOLERANCE', 'BUFFER', self.name,
            self.cmd_FILA_BUFFER_SET_TOLERANCE,
            desc=self.cmd_FILA_BUFFER_SET_TOLERANCE_help)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self._load_feeders(config)
        _consume_feeder_config_options(config)

    def log_sensor_msg(self, msg):
        if not self.sensor_log:
            return
        logging.info(msg)
        self.gcode.respond_info(msg)

    def _load_feeders(self, config):
        idx = 0
        while _has_feeder_index(config, idx):
            feeder_name = _get_config_feeder_name(config, idx)
            if feeder_name in self.feeders:
                raise config.error(
                    "filabuffer %s: duplicate feeder_name '%s'"
                    % (self.name, feeder_name))
            FilaFeeder(self, feeder_name, idx, config)
            idx += 1
        if not self.feeders:
            raise config.error(
                "filabuffer %s: no feeders (define step_pin_0, etc.)"
                % (self.name,))

    def _handle_ready(self):
        self.print_stats = self.printer.lookup_object('print_stats', None)
        self.idle = self.printer.lookup_object('idle_timeout', None)
        self.reactor.update_timer(self._watchdog_timer, self.reactor.NOW)
        self.reactor.register_timer(
            self._startup_sync_event,
            self.reactor.monotonic() + 2.0)

    def _get_extruded_mm(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        return self.print_stats.get_status(eventtime)['filament_used']
        # not updated in real time, so use filament_used directly.
        # return self.print_stats.filament_used

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
            if link is None or link.buffer_name != self.name:
                continue
            feeder = self.feeders.get(link.feeder_name)
            if feeder is None:
                continue
            present = self._read_linked_sensor_present(obj)
            if present is None:
                continue
            if link.role == 'inlet':
                feeder._inlet_present = present
            elif link.role == 'buffer':
                feeder._buffer_present = present

    def _sync_all_from_linked_sensors(self, force=False, clear_error=True):
        self._pull_linked_sensor_states()
        for u in self.feeders.values():
            u._init_phase = None
            u.sync_stable_state(force=force)
        if clear_error:
            self.error_msg = None
            if self.mode == MODE_ERROR:
                self.mode = MODE_DISABLED
        if self.mode == MODE_DISABLED:
            self._deactivate_all_feeders()

    # def _verify_other_feeders_state(self, feeder):
    #     """Other feeders must be empty or ready before SELECT_FEEDER."""
    #     allowed = (FEEDER_EMPTY, FEEDER_INSERT, FEEDER_READY)
    #     for name, u in self.feeders.items():
    #         if u is feeder:
    #             continue
    #         if u.feeder_state not in allowed:
    #             raise self.gcode.error(
    #                 "Feeder %s state=%s, other feeders must be empty or ready"
    #                 % (name, u.feeder_state))

    def _get_feeder(self, name):
        if name not in self.feeders:
            raise self.gcode.error("Unknown fila_buffer_feeder '%s'" % (name,))
        return self.feeders[name]

    def _get_gcmd_feeder(self, gcmd):
        if gcmd is None:
            return None
        name = gcmd.get('FEEDER', None)
        if name is None:
            raise gcmd.error("FEEDER parameter is required")
        return self._get_feeder(name)

    def _derive_feeder_state_from_sensors(self, feeder, gcmd):
        if not feeder.inlet_present and feeder.buffer_present:
            raise gcmd.error(
                "Feeder %s invalid sensors inlet=0 buf=1, retract first"
                % (feeder.name,))
        if not feeder.inlet_present and not feeder.buffer_present:
            return FEEDER_EMPTY
        if feeder.inlet_present and not feeder.buffer_present:
            return FEEDER_INSERT
        for u in self.feeders.values():
            if u is feeder:
                continue
            if u.buffer_present and u.feeder_state != FEEDER_INIT:
                raise gcmd.error(
                    "Feeder %s buf=1 conflicts with %s"
                    % (feeder.name, u.name))
        return FEEDER_BUFFERED

    def _reset_feeder(self, feeder, gcmd):
        self._pull_linked_sensor_states()
        if feeder.motor.is_moving():
            raise gcmd.error("Feeder %s is busy" % (feeder.name,))
        if (feeder.feeder_state == FEEDER_INIT
                and feeder._init_phase is not None):
            raise gcmd.error("Feeder %s init in progress" % (feeder.name,))
        feeder.motor_halt()
        feeder._init_phase = None
        if self.active_feeder == feeder.name:
            self.active_feeder = None
        state = self._derive_feeder_state_from_sensors(feeder, gcmd)
        feeder.set_run_state(state)
        return state

    def _active(self):
        if self.active_feeder is None:
            return None
        return self.feeders.get(self.active_feeder)

    def _stop_all_motors(self):
        for u in self.feeders.values():
            u.motor_halt()

    def _check_buffer_exclusive(self):
        if self.sensors.state & BUFF_FULL:
            if self.sensors.state & (BUFF_JAM | BUFF_LOW):
                return False
        return True

    def _count_buffer_filament(self, exclude_init=False):
        """Count feeders with buffer_present. Init feeder may touch buffer briefly."""
        n = 0
        for u in self.feeders.values():
            if exclude_init and u.feeder_state == FEEDER_INIT:
                continue
            if u.buffer_present:
                n += 1
        return n

    def _enforce_single_buffer_filament(self):
        """Shared buffer: stable state allows at most one feeder buffer_present."""
        if self._count_buffer_filament(exclude_init=True) > 1:
            self._enter_error(ERROR_MULTI_BUFFER_FILAMENT)
            return False
        return True

    def _pause_prefix(self):
        return "PAUSE\n" if self.pause_on_error else ""

    def _deactivate_all_feeders(self):
        """Clear active_feeder so no feeder is FEEDER_ACTIVE (is_selected=False)."""
        self.active_feeder = None
        for u in self.feeders.values():
            if u.is_running():
                continue
            u.sync_stable_state(force=True)

    def _enter_error(self, msg):
        self.mode = MODE_ERROR
        self.error_msg = msg
        self._deactivate_all_feeders()
        self._stop_all_motors()
        if msg == ERROR_JAM:
            self.gcode_queue.enqueue(self.jam_gcode, self._pause_prefix())
        elif msg == ERROR_FEED_TIMEOUT:
            self.gcode_queue.enqueue(
                self.low_timeout_gcode, self._pause_prefix())
        elif msg == ERROR_RUNOUT:
            self.gcode_queue.enqueue(self.runout_gcode, self._pause_prefix())
        elif msg == ERROR_BREAK:
            self.gcode_queue.enqueue(self.break_gcode, self._pause_prefix())
        elif msg == ERROR_FEED_EXTRUDER_JAM:
            self.gcode_queue.enqueue(self.extruder_jam_gcode, self._pause_prefix())
        elif msg == ERROR_FEED_SLIP:
            self.gcode_queue.enqueue(self.feed_slip_gcode, self._pause_prefix())
        self.log_sensor_msg("filabuffer %s error: %s" % (self.name, msg))

    def on_feeder_motor_stop(self, feeder, act_type, reason):
        if reason != 'complete':
            return
        # 下面是motor走完整个行程的回调函数, 实际上大部分是异常了.
        if act_type == ACT_TYPE_CMD_MOVE:
            return
        if act_type == ACT_TYPE_INIT_FORWARD:
            feeder.set_error_state(ERROR_INIT_FEED_FAIL)
        elif act_type == ACT_TYPE_INIT_RETRACT:
            self._complete_init_retract_check(feeder)
        elif (act_type == ACT_TYPE_RETRACT):
            if feeder.buffer_present:
                feeder.set_error_state(ERROR_RETRACT_FAIL)
            elif feeder.inlet_present:
                feeder.feeder_state = FEEDER_INSERT
            else:
                feeder.feeder_state = FEEDER_EMPTY
        elif act_type == ACT_TYPE_FEED: # not trigger FULL after max feed length
            if feeder.feeder_state == FEEDER_RUNOUT:
                return
            self.log_sensor_msg("feeder %s feed timeout" % (feeder.name))
            self._enter_error(ERROR_FEED_TIMEOUT)

    def note_buffer_change(self, eventtime, old, new):
        if not self._check_buffer_exclusive():
            self._enter_error(ERROR_FULL_EXCLUSIVE_VIOLATION)
            return
        # work mode, sync work feed.
        if self.mode == MODE_WORK:
            # jump to jam state, handle jam error.
            if new & BUFF_JAM and not (old & BUFF_JAM):
                self._on_jam()
                return
            self._sync_work_feed()

    # 核心处理函数, 传感器数据变化处理,状态机更新,执行都在这里.
    def note_feeder_change(self, feeder, eventtime, old_inlet, old_buffer):
        old_state = feeder.feeder_state
        msg = ("note_feeder_change() : filabuffer %s feeder %s change: inlet %d->%d buffer %d->%d "
               "old_state=%s mode=%s"
               % (self.name, feeder.name,
                  int(old_inlet), int(feeder.inlet_present),
                  int(old_buffer), int(feeder.buffer_present),
                  old_state, self.mode))
        self.log_sensor_msg(msg)
        # 1. update feeder state from sensors.
        feeder.sync_stable_state()
        self.log_sensor_msg("feeder %s state %s -> %s" % (feeder.name, old_state, feeder.feeder_state))
        # 2. verify feeder state is valid for select.
        if feeder.is_selected() and feeder.feeder_state not in (FEEDER_ACTIVE, FEEDER_RUNOUT):
            self.log_sensor_msg("feeder %s state %s is not valid for select, clear active feeder" % (feeder.name, feeder.feeder_state))
            self.active_feeder = None
        # 3. handle init / retract edges.
        if feeder.feeder_state in (FEEDER_INIT, FEEDER_RETRACT):
            self._handle_init_edges(feeder, eventtime, old_inlet, old_buffer)
            return
        # 4. feeder from empty to ready, start init.
        if (self.mode == MODE_WORK
                and old_state == FEEDER_EMPTY
                and feeder.feeder_state == FEEDER_INSERT):
            self.log_sensor_msg("feeder %s from empty to ready, start init." % (feeder.name))
            self._start_feeder_init(feeder)
            self._log_feeder_state_change(feeder, old_state)
            return
        # 5. handle other feeder state changes.
        if feeder.is_selected():
            if feeder.feeder_state == FEEDER_RUNOUT and old_state == FEEDER_ACTIVE:
                self.log_sensor_msg("feeder %s runout from active to runout" % (feeder.name))
            elif feeder.feeder_state == FEEDER_EMPTY and old_state == FEEDER_RUNOUT:
                self.log_sensor_msg("feeder %s empty from runout to empty" % (feeder.name))
                feeder.motor_halt()
                self.active_feeder = None
            elif old_buffer and not feeder.buffer_present:
                self._on_break(feeder)

        # 6. log feeder state change.
        if feeder.feeder_state != old_state:
            self._log_feeder_state_change(feeder, old_state)

    def _log_feeder_state_change(self, feeder, old_state):
        self.log_sensor_msg(
            "filabuffer %s feeder %s state %s -> %s"
            % (self.name, feeder.name, old_state, feeder.feeder_state))

    def _on_jam(self):
        self._enter_error(ERROR_JAM)

    def _sync_work_feed(self):
        """Level-triggered work feed: LOW and not FULL begin feed until FULL or JAM trigger"""
        if self.mode != MODE_WORK:
            return
        state = self.sensors.state
        if state & BUFF_JAM:
            self._on_jam()
            return
        feeder = self._active()
        if feeder is None or feeder.feeder_state not in (
                FEEDER_READY, FEEDER_ACTIVE, FEEDER_RUNOUT):
            return
        
        if state & BUFF_FULL:
            self._not_full_idle_since = self.reactor.monotonic()
            if feeder.motor.is_moving():
                feeder.motor_halt()
                feeder.clear_run_state()
                feeder.note_feed_match_full(self._get_extruded_mm())
        elif state & BUFF_LOW:
            self._start_feeder_feed(feeder, self.feed_speed, self.feed_len)

    def _check_not_full_idle_feed(self, eventtime):
        if self.mode != MODE_WORK:
            self._not_full_idle_since = eventtime
        else:
            feeder = self._active()
            if feeder is not None and feeder.feeder_state == FEEDER_ACTIVE:
                state = self.sensors.state
                match_error = feeder.check_feed_match(
                            self._get_extruded_mm(eventtime),
                            self.feed_match_tolerance)
                if match_error is not None:
                    self._enter_error(match_error)
                    return
                if state & BUFF_FULL:
                    self._not_full_idle_since = eventtime
                elif feeder.motor.is_moving():
                    self._not_full_idle_since = eventtime
                elif eventtime - self._not_full_idle_since > self.feed_idle_time:
                    self._start_feeder_feed(feeder, self.feed_speed, self.feed_len)
                    self._not_full_idle_since = eventtime
                    self.log_sensor_msg(
                        "buffer %s force feed %s after max idle time"
                        % (self.name, feeder.name))
            else:
                self._not_full_idle_since = eventtime

    def _on_break(self, feeder):
        feeder.motor_halt()
        feeder.set_error_state(ERROR_BREAK)
        if feeder.is_selected():
            self.active_feeder = None  # clear active feeder
            if self.is_printing():
                self._enter_error(ERROR_BREAK)

    def _start_feeder_feed(self, feeder, speed, length):
        if feeder.motor.is_moving():
            return
        # Shared buffer: do not feed into buffer while another feeder holds it
        if (self._count_buffer_filament(exclude_init=True) > 0
                and not feeder.buffer_present):
            self.log_sensor_msg(
                "filabuffer %s: defer feed %s, shared buffer occupied" % (self.name, feeder.name))
            return
        feeder.motor.start(length, speed, ACT_TYPE_FEED)

    def _start_feeder_init(self, feeder):
        if not feeder.inlet_present or feeder.buffer_present:
            return
        feeder.set_run_state(FEEDER_INIT, INIT_PHASE_FORWARD)
        feeder.motor.start(self.len2buffer, self.init_speed, ACT_TYPE_INIT_FORWARD)
        logging.info("filabuffer %s %s init forward start" % (self.name, feeder.name))

    def _handle_init_edges(self, feeder, eventtime, old_inlet, old_buffer):
        if feeder.feeder_state == FEEDER_RETRACT:
            if old_buffer and not feeder.buffer_present:
                feeder.motor.start(-self.len2buffer, self.retract_speed, ACT_TYPE_RETRACT)
        elif feeder.feeder_state == FEEDER_INIT:
            if feeder._init_phase == INIT_PHASE_FORWARD:
                if feeder.buffer_present and not old_buffer:
                    feeder.motor_halt()
                    feeder.set_run_state(FEEDER_INIT, INIT_PHASE_RETRACT)
                    feeder.motor.start(-self.init_retract_len, self.retract_speed, ACT_TYPE_INIT_RETRACT)
                    logging.info("filabuffer %s %s init retract start",self.name, feeder.name)
                elif not feeder.inlet_present and old_inlet:
                    feeder.motor_halt()
                    feeder.set_error_state(ERROR_INIT_RUNOUT)
            elif feeder._init_phase == INIT_PHASE_RETRACT:
                if not feeder.inlet_present and old_inlet:
                    feeder.motor_halt()
                    feeder.set_error_state(ERROR_INIT_RUNOUT)

    def _complete_init_retract_check(self, feeder):
        """After full retract distance: buffer false -> ready, true -> fail."""
        self._pull_linked_sensor_states()
        if feeder.inlet_present and not feeder.buffer_present:
            feeder.set_run_state(FEEDER_READY)
            logging.info("filabuffer %s %s init done" % (self.name, feeder.name ))
        else:
            feeder.set_error_state(ERROR_INIT_RETRACT_FAIL)
            logging.info("filabuffer %s %s init retract fail, inlet=%d buffer=%d" % 
                (self.name, feeder.name, int(feeder.inlet_present), int(feeder.buffer_present)))

    def _require_work_mode(self):
        if self.mode != MODE_WORK:
            raise self.gcode.error(
                "filabuffer %s must be in work mode (FILA_BUFFER_START)"
                % (self.name,))
        if self.sensors.state & BUFF_JAM:
            raise self.gcode.error("filabuffer %s buffer jam" % (self.name,))

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
        allowed = (FEEDER_EMPTY, FEEDER_INSERT, FEEDER_READY, FEEDER_BUFFERED)
        for name, u in self.feeders.items():
            u.sync_stable_state()
            if u.is_running():
                raise self.gcode.error(
                    "Feeder %s is busy (state=%s)" % (name, u.feeder_state))
            if u.feeder_state not in allowed:
                raise self.gcode.error(
                    "Feeder %s state=%s, all feeders must be empty or ready"
                    % (name, u.feeder_state))
        if self._count_buffer_filament() > 1: #最多一个缓冲区有料.
            raise self.gcode.error(
                "Over one buffer filament detected, buffer maybe collision")

    def is_printing(self):
        return self.idle.get_status(self.reactor.monotonic())['state'] == 'Printing'

    def _watchdog_event(self, eventtime):
        if self.mode == MODE_WORK:
            if not self._enforce_single_buffer_filament():
                return eventtime + self.watchdog_time
            self._sync_work_feed()
            self._check_not_full_idle_feed(eventtime)
        return eventtime + self.watchdog_time

    def _buffered_feeders(self):
        return [u for u in self.feeders.values() if u.buffer_present]

    # get the optional feeder to send filament.
    def _get_optional_feeder(self):
        #can't 2 feeders be buffered at the same time.
        buffered = self._buffered_feeders()
        if len(buffered) > 1:
            return None
        for u in self.feeders.values():
            if u.feeder_state == FEEDER_BUFFERED:
                return u
        for u in self.feeders.values():
            if u.feeder_state == FEEDER_READY:
                return u
        for u in self.feeders.values():
            if u.feeder_state == FEEDER_INSERT:
                return u
        return None

    def _select_feeder(self, feeder):
        if self.mode != MODE_WORK: 
            return False
        if feeder is None:
            feeder = self._get_optional_feeder()
            if feeder is None:
                return False
        self.active_feeder = feeder.name
        feeder.set_run_state(FEEDER_ACTIVE)
        feeder.note_feed_match_select(self._get_extruded_mm())
        if not (self.sensors.state & BUFF_FULL): # not full, start feed right now
            maxlen = self.len2extruder if feeder.buffer_present else self.len2buffer + self.len2extruder
            self._start_feeder_feed(feeder, self.feed_speed, maxlen)
        return True

    cmd_FILA_BUFFER_SELECT_FEEDER_help = (
        "Feed ready feeder until buffer sensor, become active")
    def cmd_FILA_BUFFER_SELECT_FEEDER(self, gcmd):
        self._require_work_mode()
        feeder = self._get_gcmd_feeder(gcmd)
        self._pull_linked_sensor_states()
        for u in self.feeders.values():
            u.sync_stable_state()
        buffered = self._buffered_feeders()
        if len(buffered) > 1:
            raise gcmd.error("Over one feeder buffer filament detected")
        if buffered and buffered[0] is not feeder:
            raise gcmd.error(
                "Feeder %s has buffer filament, must select it first"
                % (buffered[0].name,))
        # self._verify_other_feeders_state(feeder) # must be empty, insert or ready.
        if feeder.feeder_state not in (FEEDER_INSERT, FEEDER_READY, FEEDER_BUFFERED):
            raise gcmd.error(
                "Feeder %s must be insert or ready (inlet=1 buffer=0) or buffered (inlet=1 buffer=1), state=%s"
                % (feeder.name, feeder.feeder_state))
        self._select_feeder(feeder)


    cmd_FILA_BUFFER_RETRACT_FILAMENT_help = (
        "Retract until buffer sensor clears, then by recorded/default "
        "distance. Optional SPEED, MAX_LEN, BUF_LEN")
    def cmd_FILA_BUFFER_RETRACT_FILAMENT(self, gcmd):
        feeder = self._get_gcmd_feeder(gcmd)
        if feeder.is_running():
            raise gcmd.error("Feeder %s is busy" % (feeder.name,))
        self._pull_linked_sensor_states()
        if not feeder.inlet_present and not feeder.buffer_present:
            gcmd.respond_info("filabuffer %s %s already empty"
                              % (self.name, feeder.name))
            return
        speed = gcmd.get_float('SPEED', self.retract_speed, above=0.)
        max_len = gcmd.get_float('MAX_LEN', self.len2extruder + self.len2buffer, above=1.)
        buf_len = gcmd.get_float('BUF_LEN', self.len2buffer, above=0.)
        length = max_len if feeder.buffer_present else buf_len
        feeder.feeder_state = FEEDER_RETRACT
        feeder.motor.start(-length, speed, ACT_TYPE_RETRACT)

    cmd_FILA_BUFFER_FEEDER_MOVE_help = (
        "Move feeder motor: SPEED mm/s, LENGTH mm (negative=reverse), eg: FILA_BUFFER_FEEDER_MOVE BUFFER=buffer0 FEEDER=feeder0 SPEED=5.0 LENGTH=10.0")
    def cmd_FILA_BUFFER_FEEDER_MOVE(self, gcmd):
        feeder = self._get_gcmd_feeder(gcmd)
        speed = gcmd.get_float('SPEED', 20., above=0.)
        length = gcmd.get_float('LENGTH', 10)
        if length == 0:
            raise gcmd.error("LENGTH can't be 0")
        if feeder.is_running():
            raise gcmd.error("Feeder %s is busy" % (feeder.name,))
        feeder.motor.start(length, speed, ACT_TYPE_CMD_MOVE)
        gcmd.respond_info("filabuffer %s %s move %.2f mm @ %.2f mm/s" % (self.name, feeder.name, length, speed))

    cmd_FILA_BUFFER_START_help = "Start filabuffer work mode"
    def cmd_FILA_BUFFER_START(self, gcmd):
        if self.mode == MODE_WORK:
            gcmd.respond_info("filabuffer %s already in work mode" % (self.name,))
            feeder = self._active()
            if feeder is None:
                self._select_feeder(None)
            return
        self._pull_linked_sensor_states()
        for u in self.feeders.values():
            u.sync_stable_state()
        self._verify_can_enter_work()
        self._deactivate_all_feeders()
        self.mode = MODE_WORK
        self.error_msg = None
        self._sync_work_feed()
        self._select_feeder(None) #自动选择一个feeder
        gcmd.respond_info("filabuffer %s mode: %s" % (self.name, self.mode))

    cmd_FILA_BUFFER_STOP_help = "Stop filabuffer"
    def cmd_FILA_BUFFER_STOP(self, gcmd):
        self.mode = MODE_DISABLED
        self.error_msg = None
        self._stop_all_motors()
        for u in self.feeders.values():
            u._init_phase = None
        self._deactivate_all_feeders()
        gcmd.respond_info("filabuffer %s stopped" % (self.name,))

    cmd_FILA_BUFFER_INIT_FILAMENT_help = (
        "Init filament on one feeder (optional; auto-init on insert)")
    def cmd_FILA_BUFFER_INIT_FILAMENT(self, gcmd):
        self._require_work_mode()
        feeder = self._get_gcmd_feeder(gcmd)
        self._pull_linked_sensor_states()
        feeder.sync_stable_state()
        if feeder.feeder_state == FEEDER_INSERT:
            self._start_feeder_init(feeder)
        elif feeder.feeder_state == FEEDER_EMPTY:
            gcmd.respond_info("Waiting insert on %s" % (feeder.name,))
        else:
            gcmd.respond_info("Feeder %s state=%s" % (feeder.name, feeder.feeder_state))

    cmd_FILA_BUFFER_SYNC_SENSORS_help = (
        "Read linked filament sensors and sync feeder states. CLEAR_ERROR=1 clears error and sets disabled (default 1)")
    def cmd_FILA_BUFFER_SYNC_SENSORS(self, gcmd):
        clear_error = gcmd.get_int('CLEAR_ERROR', 1, minval=0, maxval=1)
        self._sync_all_from_linked_sensors(
            force=True, clear_error=bool(clear_error))
        gcmd.respond_info("filabuffer %s sensors synced" % (self.name,))

    cmd_FILA_BUFFER_RESET_FEEDER_help = (
        "Clear feeder error/init lock and set state from sensors")
    def cmd_FILA_BUFFER_RESET_FEEDER(self, gcmd):
        feeder = self._get_gcmd_feeder(gcmd)
        state = self._reset_feeder(feeder, gcmd)
        gcmd.respond_info(
            "filabuffer %s %s reset -> %s (inlet=%d buffer=%d)"
            % (self.name, feeder.name, state, int(feeder.inlet_present), int(feeder.buffer_present))
        )

    cmd_FILA_BUFFER_STATUS_help = "Report filabuffer status"
    def cmd_FILA_BUFFER_STATUS(self, gcmd):
        bs = self.sensors.get_status()
        msg = ("filabuffer %s: mode=%s active=%s error=%s "
               "jam=%d low=%d full=%d" % (
                   self.name, self.mode, self.active_feeder, self.error_msg,
                   bs['jam'], bs['low'], bs['full']))
        for name, feeder in sorted(self.feeders.items()):
            st = feeder.get_status()
            msg += ("\n %s: %s inlet=%d buf=%d feed=%d len=%.1f total=%.1f act=%s phase=%s" % (
                name, st['state'], st['inlet'], st['buffer'],
                st['is_feeding'], st['cur_feed_len'], st['total_mm'],
                st['act_type'] or '-', st['init_phase']))
        gcmd.respond_info(msg)

    cmd_FILA_BUFFER_SET_TOLERANCE_help = "Set feed match tolerance"
    def cmd_FILA_BUFFER_SET_TOLERANCE(self, gcmd):
        tolerance = gcmd.get_float('TOLERANCE', self.feed_match_tolerance, minval=0.)
        self.feed_match_tolerance = tolerance
        gcmd.respond_info("filabuffer %s feed match tolerance set to %.2f" % (self.name, tolerance))

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'mode': self.mode,
            'active_feeder': self.active_feeder,
            'error': self.error_msg,
            'sensor_log': self.sensor_log,
            'buffer_state': self.sensors.get_status(),
            'feeders': {n: u.get_status() for n, u in self.feeders.items()},
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

    def get_status(self, eventtime):
        return {n: b.get_status(eventtime) for n, b in self.buffers.items()}


class FilaBufferSensorLink:
    def __init__(self, printer, buffer_name, feeder_name, role):
        self.printer = printer
        self.buffer_name = buffer_name
        self.feeder_name = feeder_name
        self.role = role
        self._feeder = None
        self._buffer = None

    def _resolve_feeder(self):
        if self._feeder is None:
            fb = self._resolve_buffer()
            self._feeder = fb.feeders.get(self.feeder_name)
            if self._feeder is None:
                gcode = self.printer.lookup_object('gcode')
                raise gcode.error("Unknown feeder: '%s' on buffer: '%s'" % (self.feeder_name, self.buffer_name))
        return self._feeder

    def _resolve_buffer(self):
        if self._buffer is None:
            manager = get_filabuffer_manager(self.printer)
            self._buffer = manager.buffers.get(self.buffer_name)
            if self._buffer is None:
                gcode = self.printer.lookup_object('gcode')
                raise gcode.error("Unknown filabuffer '%s'" % (self.buffer_name,))
        return self._buffer

    def notify(self, eventtime, present):
        try:
            if self.role == 'runout':
                if not present: # filament runout. 
                    logging.info("filabuffer: runout sensor %s present, selecting feeder to send filament", self.buffer_name)
                    fb = self._resolve_buffer()
                    if fb.mode == MODE_WORK:
                        bSelected = fb._select_feeder(None)
                        if not bSelected:
                            logging.error("filabuffer: failed to select feeder to refill filament")
                            fb._enter_error(ERROR_RUNOUT)
            else:
                self._resolve_feeder().update_sensor(self.role, eventtime, bool(present))
        except Exception:
            logging.exception("filabuffer notify from sensor failed")


def get_filabuffer_manager(printer):
    if 'filabuffer' not in printer.objects:
        printer.add_object('filabuffer', FilaBufferManager(printer))
    return printer.lookup_object('filabuffer')


def load_sensor_link(config):
    buffer_name = config.get('filabuffer', None)
    if buffer_name is None:
        return None
    feeder_name = config.get('filabuffer_feeder', None)
    role = config.get('filabuffer_role', None)
    if role is None or (feeder_name is None and role != 'runout'):
        raise config.error(
            "filabuffer link on %s requires filabuffer_feeder and "
            "filabuffer_role" % (config.get_name(),))
    role = role.lower()
    if role not in ('inlet', 'buffer', 'runout'):
        raise config.error(
            "filabuffer_role on %s must be 'inlet', 'buffer' or 'runout'"
            % (config.get_name(),))
    get_filabuffer_manager(config.get_printer())
    return FilaBufferSensorLink(
        config.get_printer(), buffer_name, feeder_name, role)


def load_config_prefix(config):
    return get_filabuffer_manager(
        config.get_printer()).register_buffer(config)
