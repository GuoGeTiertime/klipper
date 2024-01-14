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


class Feeder:  # Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()  # add by guoge 20231130.
        self.name = config.get_name().split()[-1]
        # Setup distance sensor
        self.sensor = sensor
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
        self.min_feed_len = config.getfloat('min_feed_len', minval=0.1)  # min feed length. not feed if len < min_feed_len.
        self.max_speed = config.getfloat('max_speed', 1., above=0., maxval=1000.)
        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.lock = threading.Lock()
        self.last_dis = self.smoothed_dis = self.target_dis = 0.
        self.last_dis_time = 0.
        # feed caching
        self.next_feed_time = 0.
        self.last_feed_len = 0.
        self.last_feed_speed = 1.
        self.total_feed_len = 0.0
        
        self.last_feed_time = 0.0
        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        # Setup output feeder motor
        self.feeder_motor = config.get('feeder_motor')
        # self.stepper = self.printer.lookup_object(self.feeder_motor)
        # ppins = self.printer.lookup_object('pins')
        # self.mcu_pwm = ppins.setup_pin('pwm', feeder_pin)
        # pwm_cycle_time = config.getfloat('pwm_cycle_time', 0.100, above=0.,
        #                                  maxval=self.feed_delay)
        # self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        # self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        # self.printer.load_object(config, "verify_feeder %s" % (self.name,))
        # self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_FEEDER_DISTANCE", "FEEDER",
                                   self.name, self.cmd_SET_FEEDER_DISTANCE,
                                   desc=self.cmd_SET_FEEDER_DISTANCE_help)
        
        # test command: SET_FEEDER_DISTANCE FEEDER=feeder0 TARGET=1.23

    def set_feed(self, read_time, value): # set feed len with speed. value is the length to feed.
        if self.target_dis <= 0.:
            value = 0.
        if (read_time < self.next_feed_time or value < self.min_feed_len): 
            # not reach the next feed time or the feed len is too short.
            return
        self.next_feed_time = read_time + self.feed_delay  # set the next feed time as now + feed_delay(sensor report time).
        self.last_feed_len = value
        speed = self.last_feed_len / self.feed_delay  # calculate the speed.
        if speed > self.max_speed:
            speed = self.max_speed
            self.last_feed_len = speed * self.feed_delay
        self.last_feed_speed = speed
        logging.info("Feeder:%s feed:%.3fmm spped:%.3fmm/s, @time:%.3f", self.name, self.last_feed_len, self.last_feed_speed, read_time)
        # execute gcode command to feed filament.
        gcode = self.printer.lookup_object("gcode")
        speed = speed * 60.0  # convert to mm/min.
        # pos = self.stepper.get_position() + self.last_feed_len
        # cmd = "MANUAL_STEPPER STEPPER=%s ENABLE=1 SPEED=%d ACCEL=1000 MOVE=%.3f " % (self.feeder_motor, speed, pos)
        pos = self.last_feed_len
        self.total_feed_len = pos
        cmd = "MANUAL_STEPPER STEPPER=%s ENABLE=1 SET_POSITION=0 SPEED=%d ACCEL=1000 MOVE=%.3f" % (self.feeder_motor, speed, pos)
        logging.info("run filament feed gcode: %s, total feed len:%.2f", cmd, self.total_feed_len)
        gcode.run_script(cmd)

    def distance_callback(self, read_time, distance):
        with self.lock:
            curtime = self.reactor.monotonic()
            time_diff = read_time - self.last_feed_time
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

    def stats(self, eventtime):
        with self.lock:
            target_dis = self.target_dis
            last_dis = self.last_dis
            last_feed_len = self.last_feed_len
        is_active = target_dis or last_dis > 1.0
        return is_active, '%s: target=%.0f distance=%.1f feed len=%.3f' % (
            self.name, target_dis, last_dis, last_feed_len)

    def get_status(self, eventtime):
        with self.lock:
            target_dis = self.target_dis
            smoothed_dis = self.smoothed_dis
            last_feed_len = self.last_feed_len
        return {'distance': round(smoothed_dis, 2), 'target': target_dis,
                'feedlen': last_feed_len}

    cmd_SET_FEEDER_DISTANCE_help = "Sets a feeder hold distance"

    def cmd_SET_FEEDER_DISTANCE(self, gcmd):
        distance = gcmd.get_float('TARGET', 0.)
        gcode = self.printer.lookup_object("gcode")
        ok_msg = "cmd: %s, current distance:%.2f/%.2f(smoothed)" % (gcmd.get_commandline(), self.last_dis, self.smoothed_dis)
        gcode.respond_raw(ok_msg)
        pfeeders = self.printer.lookup_object('filafeeders')
        pfeeders.set_distance(self, distance)


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
            self.feeder.set_feed(read_time, len)
        else:
            self.feeder.set_feed(read_time, 0.)

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
        # Setup sensor
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
            feeder.set_dis(0.)

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
