# Feed filament by step motor, control by switch or holl senser.
#
# Copyright (C) 2024 guoge
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, threading


######################################################################
# Heater
######################################################################

# KELVIN_TO_CELSIUS = -273.15
# MAX_HEAT_TIME = 5.0
# AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.

MIN_DIRPULSE_TIME = 0.05
PINOUT_DELAY = 0.0 # 10ms, avoid Timer too close or Missed scheduling of next digital out event, 0.0 is ok after test.

class Feeder:  # Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()  # add by guoge 20231130.
        self.name = config.get_name().split()[-1]
        self.sensor = sensor
        self.lock = threading.Lock()
        self.bfeeder_on = False    # feeder on/off
        self.bInited = False    # feeder inited, has send the filament to the nozzle.

        switch_pin = config.get('switch_pin')
        if switch_pin is not None:
            logging.info("Add feeder %s with switch:%s", self.name, switch_pin)
            buttons = self.printer.load_object(config, 'buttons')
            buttons.register_buttons([switch_pin], self._switch_handler)
            self._switch_state = False # released.
            self.switch_feed_len = config.getfloat('switch_feed_len', 5, minval=0.1) # feed length when switch is pressed.
            self.feed_delay = config.getfloat('feed_delay', 0.5, minval=0.1) # check switch per delay time
            self._switch_update_timer = self.reactor.register_timer(self._switch_update_event)
            self.printer.register_event_handler("klippy:connect", self.handle_connect_switch)
        elif sensor is not None:  # Setup distance sensor
            logging.info("Add feeder %s with sensor:%s", self.name, sensor)
            self.hold_dis = config.getfloat('hold_dis', minval=-1000)
            self.min_dis = config.getfloat('min_dis', minval=-1000)
            self.max_dis = config.getfloat('max_dis', above=self.min_dis)
            self.sensor.setup_minmax(self.min_dis, self.max_dis)
            self.sensor.setup_callback(self.distance_callback)
            self.feed_delay = self.sensor.get_report_time_delta()
            # Setup temperature checks
            # self.min_extrude_temp = config.getfloat(
            #     'min_extrude_temp', 170.,
            #     minval=self.min_dis, maxval=self.max_dis)
            # is_fileoutput = (self.printer.get_start_args().get('debugoutput')
            #                  is not None)
            # self.can_extrude = self.min_extrude_temp <= 0. or is_fileoutput
            
            self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
            self.inv_smooth_time = 1. / self.smooth_time
            self.last_dis = self.smoothed_dis = self.target_dis = 0.
            self.last_dis_time = 0.
            # Setup control algorithm sub-class
            algos = {'watermark': ControlBangBang, 'pid': ControlPID}
            algo = config.getchoice('control', algos)
            self.control = algo(self, config)
            # Setup output feeder motor
            # self.feeder_motor = config.get('feeder_motor')
            # self.stepper = self.printer.lookup_object("manual_stepper " + self.feeder_motor)
            # stepinfo = str(self.stepper)
            # logging.info("stepper:%s, name:%s",  stepinfo, self.feeder_motor)

        # feed caching
        self.next_feed_time = 0.
        self.last_feed_len = 0.
        self.cur_feed_len = 0.          # total feed length for a feed without interruption
        self.total_feed_len = 0.0
        self.last_feed_time = 0.0
        self.last_feed_speed = 0.0
        self.is_feeding = False

        self.min_feed_len = config.getfloat('min_feed_len', 0, minval=0.1)  # min feed length. not feed if len < min_feed_len.
        self.max_feed_len = config.getfloat('max_feed_len', 10000, minval=0.1)  # max feed length. warning if curfeedlen > max_feed_len.
        self.max_speed = config.getfloat('max_speed', 1., above=0., maxval=1000.)

        self.cur_cycle_time = 0.1
        # Setup PWM output as stepper's pulse generator, and normal output as direct control.
        step_pin = config.get('step_pin')
        dir_pin = config.get('dir_pin')
        ppins = self.printer.lookup_object('pins')
        self.step = ppins.setup_pin('pwm', step_pin)
        self.dir = ppins.setup_pin('digital_out', dir_pin)
        self.dir.setup_max_duration(0.)
        self.last_dirtime = 0.0
        self.last_pulsetime = 0.0

        # setup stepper microstep and rotate distance.
        self.microstep = config.getint('microstep', 16, minval=1, maxval=256)
        self.rotate_distance = config.getfloat('rotate_distance', 31.4, above=0.1) #defaul: the diameter is 10mm, so the rotate distance is 31.4mm.
        self.scale_speed2freq = self.microstep * 200 / self.rotate_distance # 200 steps per round. default 16 microstep. the value freq for per mm/s.

        self.feed_speed = config.getfloat('feed_speed', 10.0, minval=1, maxval=100.0) # unit: mm/s, default 10mm/s.
        self.feed_spped_init = config.getfloat('feed_speed_init', self.feed_speed * 2.0, minval=1, maxval=200.0) 

        self.switch_invert = False # switch signal invert, for init feeding.

        # self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        # self.printer.load_object(config, "verify_feeder %s" % (self.name,))
        # self.printer.load_object(config, "pid_calibrate")
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command("SET_FEEDER_DISTANCE", "FEEDER",
                                   self.name, self.cmd_SET_FEEDER_DISTANCE,
                                   desc=self.cmd_SET_FEEDER_DISTANCE_help)
        self.gcode.register_mux_command("FEED_INIT", "FEEDER",
                                   self.name, self.cmd_FEED_INIT,
                                   desc=self.cmd_FEED_INIT_help)
        
        # test command: SET_FEEDER_DISTANCE FEEDER=feeder0 TARGET=1.23

    def _loginfo(self, msg):
        self.gcode.respond_raw(msg)

    def _cal_step_cycle_time(self, speed):
        freq = speed * self.scale_speed2freq
        return 1.0 / freq

    # feed filament len at speed
    def feed_filament(self, print_time, speed, len): # set feed len with speed. value is the length to feed.
        if not self.bfeeder_on:
            return
        if abs(len) < self.min_feed_len:
            len = 0.0
        # if print_time < self.next_feed_time: 
        if print_time < self.last_feed_time + MIN_DIRPULSE_TIME: # at least 0.1s interval.
            return

        # set dir pin
        self.set_dir(print_time, 1 if len > 0 else 0)

        # calculate the feed time and speed. send the pulse pwm cmd.
        self.is_feeding = len != 0
        if self.is_feeding:
            # calculate the cycle time of feed stepper's pulse and time to feed the len.
            speed = max(0.01, min(speed, self.max_speed)) # limit the speed between 0.01 and max_speed.
            cycle_time = self._cal_step_cycle_time(speed)
            feed_time = abs(len/speed)
            # set the feed stepper's pulse freq, 0.5 is the duty.
            self.set_pulse(print_time, 0.5, cycle_time)
        else:
            speed = 0.0
            feed_time = self.feed_delay
            self.set_pulse(print_time, 0, 1.0)

        # update feed len info
        prevlen = (print_time - self.last_feed_time) * self.last_feed_speed
        self.last_feed_len = prevlen
        self.cur_feed_len += prevlen
        self.total_feed_len += prevlen
        # update feed time info
        self.last_feed_speed = speed
        self.last_feed_time = print_time
        self.next_feed_time = print_time + feed_time  # set the next feed time as now + feed time.

        # verify max feed len, if over, stop feed.
        if self.cur_feed_len > self.max_feed_len:
            self._loginfo("feeder %s feed len:%.3fmm over max:%.3fmm" % (self.name, self.cur_feed_len, self.max_feed_len))
            raise self.printer.command_error("Feeder %s reach the max feed lenght. feed len:%.3fmm over max:%.3fmm" % (self.name, self.cur_feed_len, self.max_feed_len))

        # log info for debug
        self._loginfo("feeder %s feed_filament: %.3fmm @ %.3fmm/s, cycle time:%.6f, feed time:%.3f" % (self.name, len, speed, cycle_time, feed_time))
            
    def set_dir(self, print_time, value):
        print_time = max(print_time + PINOUT_DELAY, self.last_dirtime + MIN_DIRPULSE_TIME)
        self.last_dirtime = print_time
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self.dir.set_digital(print_time, value))
        
    # debug, not used. show set_pwm()'s call times and value
    # def _set_pulse(self, print_time, value, cycle_time):
    #     self.step.set_pwm(print_time, value, cycle_time)
    #     # self._loginfo("feeder %s set cycle time: %.6fs @ %.3f" % (self.name, cycle_time, print_time))
        
    def set_pulse(self, print_time, value, cycle_time):
        print_time = max(print_time + PINOUT_DELAY, self.last_pulsetime + MIN_DIRPULSE_TIME)
        self.last_pulsetime = print_time
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda print_time: self.step.set_pwm(print_time, value, cycle_time))
            # lambda print_time: self._set_pulse(print_time, value, cycle_time))

    def handle_connect_switch(self):
        self.reactor.update_timer(self._switch_update_timer, self.reactor.NOW)
        return
       
    # update feeder when the feeder's switch is pressed or released.
    def _switch_handler(self, eventtime, state):
        self._switch_state = state
        if state ^ self.switch_invert: # switch on, pressed
            speed = self.feed_speed if self.bInited else self.feed_spped_init
            self.feed_filament(eventtime, speed, self.switch_feed_len)
            self._loginfo("feeder %s switch pressed" % self.name)
        else: # switch off, released
            # self.feed_filament(eventtime, 0.0, 0.0)
            # self.cur_feed_len = 0.0
            self._loginfo("feeder %s switch released" % self.name)

    def _switch_update_event(self, eventtime):
        if self._switch_state ^ self.switch_invert: # switch pressed, continue feed filament.
            speed = self.feed_speed if self.bInited else self.feed_spped_init
            self.feed_filament(eventtime, speed, self.switch_feed_len)
        elif self.is_feeding: # switch released, stop feed filament.
            self.feed_filament(eventtime, 0.0, 0.0)
            if not self.bInited:
                self.bInited = True
        next_time = min(eventtime + self.feed_delay, self.next_feed_time)
        next_time = max(next_time, eventtime + 0.1) # at least 0.1s.
        return next_time

    def distance_callback(self, read_time, distance):
        with self.lock:
            curtime = self.reactor.monotonic()
            time_diff = read_time - self.last_feed_time
            if abs(self.last_dis - distance) > 1:
                logging.info(" feeder distance changed: %.3f -> %.3f", self.last_dis, distance)
            self.last_dis = distance
            self.last_feed_time = read_time
            self.control.distance_update(read_time, distance, self.target_dis)
            dis_diff = distance - self.smoothed_dis
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_dis += dis_diff * adj_time
            if self.smoothed_dis > 20:
                logging.info(" *** distance callbacke Error @ %.3f, feeder: %s distance:%.3f/%.3f @ read time:%.3f - %.3f, timeDiff:%.2f, distance diff:%.3f, too high", 
                             curtime, self.name, self.smoothed_dis, distance, read_time, self.last_feed_time, time_diff, dis_diff)
                self.smoothed_dis = 20
        # logging.debug("feeder %s @ %.3f - distance: %f", self.name, read_time, distance)
    # External commands

    def get_feed_delay(self):
        return self.feed_delay

    def get_max_speed(self):
        return self.max_speed

    def get_smooth_time(self):
        return self.smooth_time

    def set_distance(self, distance):
        if distance and (distance < self.min_dis or distance > self.max_dis):
            raise self.printer.command_error(
                "Requested distance (%.1f) out of range (%.1f:%.1f)"
                % (distance, self.min_dis, self.max_dis))
        with self.lock:
            self.target_dis = distance

    def get_distance(self, eventtime):
        # print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            # if self.last_feed_time < print_time:
            #     return 0., self.target_dis
            return self.smoothed_dis, self.target_dis

    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_dis, self.target_dis)

    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_dis = 0.
        return old_control

    def alter_target(self, target_dis):
        if target_dis:
            target_dis = max(self.min_dis, min(self.max_dis, target_dis))
        self.target_dis = target_dis

    def get_status(self, eventtime):
        return {
            'total_feed_len': self.total_feed_len,
            'cur_feed_len': self.cur_feed_len,
            'last_feed_time': self.last_feed_time}

    cmd_SET_FEEDER_DISTANCE_help = "Sets a feeder hold distance"
    def cmd_SET_FEEDER_DISTANCE(self, gcmd):
        distance = gcmd.get_float('TARGET', 0.)
        gcode = self.printer.lookup_object("gcode")
        ok_msg = "cmd: %s, current distance:%.2f/%.2f(smoothed)" % (gcmd.get_commandline(), self.last_dis, self.smoothed_dis)
        gcode.respond_raw(ok_msg)
        pfeeders = self.printer.lookup_object('filafeeders')
        pfeeders.set_distance(self, distance)

    # eg: FEED_INIT FEEDER=feeder1 SPEED=20.0 MAX_LEN=1000.0 INVERT=0 FEED_LEN=5.0 INIT=0 
    cmd_FEED_INIT_help = "init feed, send the filament to the nozzle."
    def cmd_FEED_INIT(self, gcmd):
        self.max_feed_len = gcmd.get_float('MAX_LEN', 1000.)
        self.switch_invert = gcmd.get_int('INVERT', 0)
        self.feed_speed = gcmd.get_float('SPEED', self.feed_speed)
        self.switch_feed_len = gcmd.get_float('FEED_LEN', self.switch_feed_len)
        init = gcmd.get_int('INIT', 0)
        self.bInited = not init
        self.bfeeder_on = True


######################################################################
# Bang-bang control algo
######################################################################
# send filament with max speed when the distance is below target_dis-max_delta.
# feed length is min_feed_len.
class ControlBangBang:
    def __init__(self, feeder, config):
        self.feeder = feeder
        self.feed_len = feeder.min_feed_len
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)  # stop feeding when the distance over target + max_delta.
        self.feeding = False

    def distance_update(self, read_time, dis, target_dis):
        if self.feeding and dis >= target_dis+self.max_delta:
            self.feeding = False
        elif not self.feeding and dis <= target_dis-self.max_delta:
            self.feeding = True
        if self.feeding:  # feed filament with max speed. time is the sensor report time.
            len = max(self.feed_len, target_dis - dis)
            self.feeder.feed_filament(read_time, self.feeder.feed_speed, len)
        else:
            self.feeder.feed_filament(read_time, self.feeder.feed_speed, 0.)

    def check_busy(self, eventtime, smoothed_dis, target_dis):
        return smoothed_dis < target_dis-self.max_delta


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1


class ControlPID:
    def __init__(self, feeder, config):
        self.feeder = feeder
        self.feed_max_len = feeder.get_max_speed() * feeder.get_feed_delay(),
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = feeder.get_smooth_time()
        self.dis_integ_max = 0.
        if self.Ki:
            self.dis_integ_max = self.feed_max_len / self.Ki
        self.prev_dis = 0
        self.prev_dis_time = 0.
        self.prev_dis_deriv = 0.
        self.prev_dis_integ = 0.

    def distance_update(self, read_time, dis, target_dis):
        time_diff = read_time - self.prev_dis_time
        # Calculate change of distance
        dis_diff = dis - self.prev_dis
        if time_diff >= self.min_deriv_time:
            dis_deriv = dis_diff / time_diff
        else:
            dis_deriv = (self.prev_dis_deriv * (self.min_deriv_time-time_diff)
                         + dis_diff) / self.min_deriv_time
        # Calculate accumulated distance "error"
        dis_err = target_dis - dis
        dis_integ = self.prev_dis_integ + dis_err * time_diff
        dis_integ = max(0., min(self.dis_integ_max, dis_integ))
        # Calculate output
        co = self.Kp*dis_err + self.Ki*dis_integ - self.Kd*dis_deriv
        # logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    dis, read_time, dis_diff, dis_deriv, dis_err, dis_integ, co)
        bounded_co = max(0., min(self.feed_max_len, co))
        self.feeder.set_speed(read_time, bounded_co)
        # Store state for next measurement
        self.prev_dis = dis
        self.prev_dis_time = read_time
        self.prev_dis_deriv = dis_deriv
        if co == bounded_co:
            self.prev_dis_integ = dis_integ

    def check_busy(self, eventtime, smoothed_dis, target_dis):
        dis_diff = target_dis - smoothed_dis
        return abs(dis_diff) > PID_SETTLE_DELTA
                # or abs(self.prev_dis_deriv) > PID_SETTLE_SLOPE)


######################################################################
# Sensor and feeder lookup
######################################################################

class RunoutHelper:
    def __init__(self, config):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # Read config
        self.runout_pause = config.getboolean('pause_on_runout', True)
        if self.runout_pause:
            self.printer.load_object(config, 'pause_resume')
        self.runout_gcode = self.insert_gcode = None
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        if self.runout_pause or config.get('runout_gcode', None) is not None:
            self.runout_gcode = gcode_macro.load_template(
                config, 'runout_gcode', '')
        if config.get('insert_gcode', None) is not None:
            self.insert_gcode = gcode_macro.load_template(
                config, 'insert_gcode')
        self.pause_delay = config.getfloat('pause_delay', .5, above=.0)
        self.event_delay = config.getfloat('event_delay', 3., above=0.)
        # Internal state
        self.min_event_systime = self.reactor.NEVER
        self.filament_present = False
        self.sensor_enabled = True
        # Register commands and event handlers
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_mux_command(
            "QUERY_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_QUERY_FILAMENT_SENSOR,
            desc=self.cmd_QUERY_FILAMENT_SENSOR_help)
        self.gcode.register_mux_command(
            "SET_FILAMENT_SENSOR", "SENSOR", self.name,
            self.cmd_SET_FILAMENT_SENSOR,
            desc=self.cmd_SET_FILAMENT_SENSOR_help)
    def _handle_ready(self):
        self.min_event_systime = self.reactor.monotonic() + 2.
    def _runout_event_handler(self, eventtime):
        # Pausing from inside an event requires that the pause portion
        # of pause_resume execute immediately.
        pause_prefix = ""
        if self.runout_pause:
            pause_resume = self.printer.lookup_object('pause_resume')
            pause_resume.send_pause_command()
            pause_prefix = "PAUSE\n"
            self.printer.get_reactor().pause(eventtime + self.pause_delay)
        self._exec_gcode(pause_prefix, self.runout_gcode)
    def _insert_event_handler(self, eventtime):
        self._exec_gcode("", self.insert_gcode)
    def _exec_gcode(self, prefix, template):
        try:
            self.gcode.run_script(prefix + template.render() + "\nM400")
        except Exception:
            logging.exception("Script running error")
        self.min_event_systime = self.reactor.monotonic() + self.event_delay
    def note_filament_present(self, is_filament_present):
        if is_filament_present == self.filament_present:
            return
        self.filament_present = is_filament_present
        eventtime = self.reactor.monotonic()
        if eventtime < self.min_event_systime or not self.sensor_enabled:
            # do not process during the initialization time, duplicates,
            # during the event delay time, while an event is running, or
            # when the sensor is disabled
            return
        # Determine "printing" status
        idle_timeout = self.printer.lookup_object("idle_timeout")
        is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        # Perform filament action associated with status change (if any)
        if is_filament_present:
            if not is_printing and self.insert_gcode is not None:
                # insert detected
                self.min_event_systime = self.reactor.NEVER
                logging.info(
                    "Filament Sensor %s: insert event detected, Time %.2f" %
                    (self.name, eventtime))
                self.reactor.register_callback(self._insert_event_handler)
        elif is_printing and self.runout_gcode is not None:
            # runout detected
            self.min_event_systime = self.reactor.NEVER
            logging.info(
                "Filament Sensor %s: runout event detected, Time %.2f" %
                (self.name, eventtime))
            self.reactor.register_callback(self._runout_event_handler)
    def get_status(self, eventtime):
        return {
            "filament_detected": bool(self.filament_present),
            "enabled": bool(self.sensor_enabled)}
    cmd_QUERY_FILAMENT_SENSOR_help = "Query the status of the Filament Sensor"
    def cmd_QUERY_FILAMENT_SENSOR(self, gcmd):
        if self.filament_present:
            msg = "Filament Sensor %s: filament detected" % (self.name)
        else:
            msg = "Filament Sensor %s: filament not detected" % (self.name)
        gcmd.respond_info(msg)
    cmd_SET_FILAMENT_SENSOR_help = "Sets the filament sensor on/off"
    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        self.sensor_enabled = gcmd.get_int("ENABLE", 1)


class FilaFeeders:  # PrinterHeaters:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.feeders = {}
        self.gcode_id_to_sensor = {}
        self.available_feeders = []
        self.available_sensors = []
        self.available_monitors = []
        self.has_started = self.have_load_sensors = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("gcode:request_restart",
                                            self.turn_off_all_feeders)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_FEEDERS", self.cmd_TURN_OFF_FEEDERS,
                               desc=self.cmd_TURN_OFF_FEEDERS_help)
        gcode.register_command("M135", self.cmd_M135, when_not_ready=True)
        gcode.register_command("DISTANCE_WAIT", self.cmd_DISTANCE_WAIT,
                               desc=self.cmd_DISTANCE_WAIT_help)

    def load_config(self, config):
        self.have_load_sensors = True
        # Load default feeder sensors(distance sensor or switch)
        pconfig = self.printer.lookup_object('configfile')
        dir_name = os.path.dirname(__file__)
        filename = os.path.join(dir_name, 'temperature_sensors.cfg')
        try:
            dconfig = pconfig.read_config(filename)
        except Exception:
            raise config.config_error("Cannot load config '%s'" % (filename,))
        for c in dconfig.get_prefix_sections(''):
            self.printer.load_object(dconfig, c.get_name())

    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory

    def setup_feeder(self, config, gcode_id=None):
        feeder_name = config.get_name().split()[-1]
        if feeder_name in self.feeders:
            raise config.error("Feeder %s already registered" % (feeder_name,))
        
        # Setup distance sensor/feeder
        sensor = self.setup_sensor(config)
        # Create feeder
        self.feeders[feeder_name] = feeder = Feeder(config, sensor)
        self.register_sensor(config, feeder, gcode_id)
        self.available_feeders.append(config.get_name())
        return feeder

    def get_all_feeders(self):
        return self.available_feeders

    def lookup_feeder(self, feeder_name):
        if feeder_name not in self.feeders:
            raise self.printer.config_error(
                "Unknown feeder '%s'" % (feeder_name,))
        return self.feeders[feeder_name]

    def setup_sensor(self, config):
        if not self.have_load_sensors:
            self.load_config(config)
        sensor_type = config.get('sensor_type')
        pheaters = self.printer.lookup_object("heaters")
        if sensor_type in pheaters.sensor_factories:  # use the sensor registered in heaters.
            return pheaters.sensor_factories[sensor_type](config)
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown feeder distance sensor '%s'" % (sensor_type,))
        if sensor_type == 'NTC 100K beta 3950':
            config.deprecate('sensor_type', 'NTC 100K beta 3950')
        return self.sensor_factories[sensor_type](config)

    def register_sensor(self, config, psensor, gcode_id=None):
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor

    def register_monitor(self, config):
        self.available_monitors.append(config.get_name())

    def get_status(self, eventtime):
        return {'available_feeders': self.available_feeders,
                'available_sensors': self.available_sensors,
                'available_monitors': self.available_monitors}

    def turn_off_all_feeders(self, print_time=0.):
        for feeder in self.feeders.values():
            feeder.set_distance(0.)

    cmd_TURN_OFF_FEEDERS_help = "Turn off all feeders"
    def cmd_TURN_OFF_FEEDERS(self, gcmd):
        self.turn_off_all_feeders()

    def _handle_ready(self):
        self.has_started = True

    def _get_dis(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_distance(eventtime)  # 用温度来表示距离
                out.append("Feeder %s cur dis:%.3fmm target:%.3fmm" % (gcode_id, cur, target))
        if not out:
            return "T:0"
        return " ".join(out)

    # G-Code M135 feeder distance reporting
    def cmd_M135(self, gcmd):
        # Get feeders's sensor distance
        reactor = self.printer.get_reactor()
        msg = self._get_dis(reactor.monotonic())
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)

    def _wait_for_distance(self, feeder):
        # Helper to wait on feeder.check_busy() and report M135 Feeder distance
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and feeder.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_dis(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

    def set_distance(self, feeder, dis, wait=False):
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.register_lookahead_callback((lambda pt: None))
        feeder.set_distance(dis)
        if wait and dis:
            self._wait_for_distance(feeder)

    cmd_DISTANCE_WAIT_help = "Wait for a distance on a sensor"
    def cmd_DISTANCE_WAIT(self, gcmd):  # sensor is equal to feeder
        sensor_name = gcmd.get('SENSOR')
        if sensor_name not in self.available_sensors:
            raise gcmd.error("Unknown sensor '%s'" % (sensor_name,))
        min_dis = gcmd.get_float('MINIMUM', float('-inf'))
        max_dis = gcmd.get_float('MAXIMUM', float('inf'), above=min_dis)
        if min_dis == float('-inf') and max_dis == float('inf'):
            raise gcmd.error(
                "Error on 'DISTANCE_WAIT': missing MINIMUM or MAXIMUM.")
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        if sensor_name in self.feeders:
            sensor = self.feeders[sensor_name]
        else:
            sensor = self.printer.lookup_object(sensor_name)
        toolhead = self.printer.lookup_object("toolhead")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown():
            dis, target = sensor.get_distance(eventtime)
            if dis >= min_dis and dis <= max_dis:
                return
            print_time = toolhead.get_last_move_time()
            gcmd.respond_raw(self._get_dis(eventtime))
            eventtime = reactor.pause(eventtime + 1.)


def load_config(config):
    return FilaFeeders(config)  # PrinterHeaters(config)