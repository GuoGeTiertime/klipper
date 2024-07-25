# Feed filament by step motor, control by switch or holl senser.
#
# Copyright (C) 2024 guoge
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, threading
import mcu
from . import filament_switch_sensor


######################################################################
# Heater
######################################################################

# KELVIN_TO_CELSIUS = -273.15
# MAX_HEAT_TIME = 5.0
# AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.

MIN_DIRPULSE_TIME = 0.1
PINOUT_DELAY = 0.05 # 50ms, avoid Timer too close or Missed scheduling of next digital out event

class Feeder:  # Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()  # add by guoge 20231130.
        self.name = config.get_name().split()[-1]
        self.sensor = sensor
        self.lock = threading.Lock()
        self.bfeeder_on = False    # feeder on/off
        self.bInited = False    # feeder inited, has send the filament to the nozzle.

        # runout helper, use the filament_switch_sensor.py's RunoutHelper class.
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)

        # fila length runout check, need extruder object and runout_length
        self.extruder_name = config.get('extruder', None)

        self.runout_length = config.getfloat('runout_length', 100, above=1)
        self.runout_pos = 0.0   # runout position, the extruder's position when the runout happened.    

        buttons = self.printer.load_object(config, 'buttons')
        fila_pin = config.get('fila_pin')
        buttons.register_buttons([fila_pin], self._check_filament)
        self._fila_state = True # fila is inserted.

        switch_pin = config.get('switch_pin')
        if switch_pin is not None:
            logging.info("Add feeder %s with switch:%s", self.name, switch_pin)
            buttons.register_buttons([switch_pin], self._switch_handler)
            self._switch_state = False # released. feed switch is pressed or not, True is feeding. False is need to feed.
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
        self.max_feed_len = config.getfloat('max_feed_len', 100, minval=0.1)  # max continue feed length. warning if curfeedlen > max_feed_len.
        self.init_feed_len = config.getfloat('init_feed_len', 2000.0, minval=0.1)  # max feed length of init feed.
        self.max_speed = config.getfloat('max_speed', 1., above=0., maxval=1000.)

        self.cur_cycle_time = 0.1
        # Setup PWM output as stepper's pulse generator, and normal output as direct control.
        step_pin = config.get('step_pin')
        dir_pin = config.get('dir_pin')
        enable_pin = config.get('enable_pin')
        ppins = self.printer.lookup_object('pins')
        self.step = ppins.setup_pin('pwm', step_pin)
        self.step.setup_cycle_time(0.0002)
        self.dir = ppins.setup_pin('digital_out', dir_pin)
        self.dir.setup_max_duration(0.)
        self.stepenable = ppins.setup_pin('digital_out', enable_pin)
        self.stepenable.setup_max_duration(0.)
        self.last_dirtime = 0.0
        self.last_pulsetime = 0.0

        # setup stepper microstep and rotate distance.
        self.microstep = config.getint('microstep', 16, minval=1, maxval=256)
        full_steps = config.getint('full_steps_per_rotation', 200, minval=1) # default 200 steps per round.
        self.rotate_distance = config.getfloat('rotate_distance', 31.4, above=0.1) #defaul: the diameter is 10mm, so the rotate distance is 31.4mm.
        self.scale_speed2freq = self.microstep * full_steps / self.rotate_distance # the value freq for per mm/s.

        self.feed_speed = config.getfloat('feed_speed', 10.0, minval=1, maxval=100.0) # unit: mm/s, default 10mm/s.
        self.feed_speed_init = config.getfloat('feed_speed_init', self.feed_speed * 2.0, minval=1, maxval=200.0) 

        self.switch_invert = False # switch signal invert, for init feeding.

        self.isloginfo = 0  # 0: no log, 1:gcode response, 2: write log file, 3: response and write log file 
        
        # load runout/fail/insert gcode
        # insert: insert fila into feeder
        # runout:no fila in feeder
        # jam_break: fila break or nozzle jam, not feed when the extruder is working.
        # slip: can't feed the fila to the nozzle, feeder slip or tube broken.
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.insert_gcode = gcode_macro.load_template(config, 'insert_gcode', '')
        self.runout_gcode = gcode_macro.load_template(config, 'runout_gcode', '')
        self.jam_break_gcode = gcode_macro.load_template(config, 'jam_break_gcode', '')
        self.slip_gcode = gcode_macro.load_template(config, 'slip_gcode', '')

        # self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        # self.printer.load_object(config, "verify_feeder %s" % (self.name,))
        # self.printer.load_object(config, "pid_calibrate")
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command("SET_FEEDER_DISTANCE", "FEEDER",
                                   self.name, self.cmd_SET_FEEDER_DISTANCE,
                                   desc=self.cmd_SET_FEEDER_DISTANCE_help)
        self.gcode.register_mux_command("FEED_IN", "FEEDER",
                                   self.name, self.cmd_FEED_IN,
                                   desc=self.cmd_FEED_IN_help)
        self.gcode.register_mux_command("FEED_FILA", "FEEDER",
                            self.name, self.cmd_FEED_FILA,
                            desc=self.cmd_FEED_FILA_help)
        self.gcode.register_mux_command("FEED_STATUS", "FEEDER",
                            self.name, self.cmd_FEED_STATUS,
                            desc=self.cmd_FEED_STATUS_help)

        self.printer.register_event_handler('klippy:ready', self._handle_ready)

        # test command: SET_FEEDER_DISTANCE FEEDER=feeder0 TARGET=1.23

    def _handle_ready(self):
        self.extruder = self.printer.lookup_object(self.extruder_name, None)
        # self.estimated_print_time = (
        #         self.printer.lookup_object('mcu').estimated_print_time)
        # self._update_filament_runout_pos()
        # self._extruder_pos_update_timer = self.reactor.register_timer(
        #         self._extruder_pos_update_event)

    def _loginfo(self, msg, logflag=None):
        if logflag is None:
            logflag = self.isloginfo
        if logflag == 1:
            self.gcode.respond_info(msg, False) # only respond to gcode.
        elif logflag == 2:
           logging.info(msg) # only write log file.
        elif logflag == 3:
            self.gcode.respond_info(msg, True) # respond to gcode and write log file.
    
    def _get_extruder_pos(self, eventtime=None):
        if self.extruder is None:
            return 0.0
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.step.get_mcu().estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)
    def _update_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.runout_pos = self._get_extruder_pos(eventtime) + self.runout_length

    def _cal_step_cycle_time(self, speed):
        freq = speed * self.scale_speed2freq
        return 1.0 / freq if freq > 0 else 0.1
    
    def _set_step_cycle_time(self, cycle_time):
        if self.cur_cycle_time == cycle_time: # no need to change.
            return
        self.cur_cycle_time = cycle_time # update the cycle time.
        mcu = self.step.get_mcu()
        cycle_ticks = mcu.seconds_to_clock(cycle_time)
        cmd = "set_digital_out_pwm_cycle oid=%d cycle_ticks=%d" % (self.step._oid, cycle_ticks)
        mcu._serial.send(cmd)  # set the cycle time of step pin.
        self._loginfo("send command:" + cmd)

    # feed filament len at speed
    def feed_filament(self, print_time, speed, len): # set feed len with speed. value is the length to feed.
        if not self.bfeeder_on and not self.is_feeding:
            return 0.0

        # estimate the print time for output dir and pulse.
        curtime = self.reactor.monotonic()
        print_time = self.step.get_mcu().estimated_print_time(curtime) + PINOUT_DELAY

        if print_time < self.next_feed_time: 
        # if print_time < self.last_feed_time + MIN_DIRPULSE_TIME: # at least 0.1s interval.
            return 0.0

        # verify max feed len, if over, stop feed after set pwm with value 0.
        max_len = self.max_feed_len if self.bInited else self.init_feed_len
        if self.cur_feed_len > max_len:
            self._loginfo("feeder %s cur feed len:%.3f over max len:%.1f(inited:%s), stoped!" % (self.name, self.cur_feed_len, max_len, str(self.bInited)), 3)
            # raise self.printer.command_error("Feeder %s reach the max feed lenght. feed len:%.3fmm over max:%.3fmm" % (self.name, self.cur_feed_len, self.max_feed_len))
            len = 0.0
            self.enable_stepper(False)
            if self.bInited :
               if self.isprinting(): # exec slip gcode when printing.
                   self._exec_gcode(self.slip_gcode)
            else:
                self._loginfo("Can't feed filament to nozzle, fila feeder initialize failed, please check the feeder, filament and extruder.", 3)

        if not self.bfeeder_on or abs(len) < self.min_feed_len:
            len = 0.0

        # set dir pin
        self.set_dir(print_time, 1 if len > 0 else 0)

        # calculate the feed time and speed. send the pulse pwm cmd.
        self.is_feeding = len != 0
        cycle_time = self._cal_step_cycle_time(speed)
        if self.is_feeding:
            # calculate the cycle time of feed stepper's pulse and time to feed the len.
            speed = max(0.01, min(speed, self.max_speed)) # limit the speed between 0.01 and max_speed.
            feed_time = abs(len/speed)
            # set the feed stepper's pulse freq, 0.5 is the duty.
            self.set_pulse(print_time, 0.5, cycle_time)
            # update the runout pos at every feed.
            self._update_runout_pos(curtime)
        else:
            speed = 0.0
            cycle_time = 1.0
            feed_time = self.feed_delay
            self.cur_feed_len = 0.0 # reset the current feed len.
            self.set_pulse(print_time, 0, cycle_time)
            self._loginfo("feeder stop, set pwm with value=0 @ time:%.3f" % print_time)

        # update feed len info
        prevlen = (print_time - self.last_feed_time) * self.last_feed_speed
        self.last_feed_len = prevlen
        self.cur_feed_len += prevlen
        self.total_feed_len += prevlen
        # update feed time info
        self.last_feed_speed = speed
        self.last_feed_time = print_time
        self.next_feed_time = print_time + feed_time  # set the next feed time as now + feed time.


        # log info for debug
        self._loginfo("feeder %s feed_filament: %.3fmm @ %.3fmm/s, freq:%.3fkHz, period:%.3fms, feed time:%.3f, curlen:%.2f, total lenght:%.2f" % 
                      (self.name, len, speed, 0.001/cycle_time, cycle_time*1000, feed_time, self.cur_feed_len, self.total_feed_len))
        return len

    def set_dir(self, print_time, value):
        print_time = max(print_time, self.last_dirtime + MIN_DIRPULSE_TIME)
        self.last_dirtime = print_time
        self.dir.set_digital(print_time, value)
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.register_lookahead_callback(
        #     lambda print_time: self.dir.set_digital(print_time, value))

    def enable_stepper(self, bOn):
        self.bfeeder_on = not not bOn
        curtime = self.reactor.monotonic()
        print_time = self.step.get_mcu().estimated_print_time(curtime) + PINOUT_DELAY
        self.stepenable.set_digital(print_time, 1 if bOn else 0)
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.register_lookahead_callback(
        #     lambda print_time: self.stepenable.set_digital(print_time, 1 if bOn else 0) )
        self._loginfo("feeder %s is %s" % (self.name, "enabled" if bOn else "disabled") )
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.register_lookahead_callback(
        #     lambda print_time: self.stepenable.set_digital(print_time, value))

    # debug, not used. show set_pwm()'s call times and value
    # def _set_pulse(self, print_time, value, cycle_time):
    #     self.step.set_pwm(print_time, value, cycle_time)
    #     # self._loginfo("feeder %s set cycle time: %.6fs @ %.3f" % (self.name, cycle_time, print_time))
        
    def set_pulse(self, print_time, value, cycle_time):
        print_time = max(print_time, self.last_pulsetime + MIN_DIRPULSE_TIME)
        self.last_pulsetime = print_time
        self._set_step_cycle_time(cycle_time)
        self.step.set_pwm(print_time, value) #, cycle_time)
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.register_lookahead_callback(
        #     lambda print_time: self.step.set_pwm(print_time, value, cycle_time))

    # Determine "printing" status
    def isprinting(self):
        eventtime = self.reactor.monotonic()
        idle_timeout = self.printer.lookup_object("idle_timeout")
        return idle_timeout.get_status(eventtime)["state"] == "Printing"

    def handle_connect_switch(self):
        self.reactor.update_timer(self._switch_update_timer, self.reactor.NOW)
        return
    
    def _exec_gcode(self, script_template):        
        try:
            self.gcode.run_script( script_template.render() )
        except Exception:
            self._loginfo("Feeder script running error", 3)
        # script_template.run_gcode_from_command()

    # checked if fila is in the feeder
    def _check_filament(self, eventtime, state):
        if self._fila_state == state:
            return

        self._fila_state = state
        if self._fila_state: # fila is inserted into the feeder
            self._loginfo("feeder: %s fila inserted, begin sending, reset inited flag to False" % self.name)
            # self._exec_gcode(self.insert_gcode) 
            self._switch_handler(eventtime, self._switch_state)
            self.enable_stepper(True)
            self.bInited = False    # reset the inited flag forcelly.
        else:
            self.enable_stepper(False)
            self._loginfo("feeder: %s fila is removed, stop feeding" % self.name)
            if self.isprinting(): # exec runout gcode when printing.
                self._exec_gcode(self.runout_gcode) 

    # update feeder when the feeder's switch is pressed or released.
    def _switch_handler(self, eventtime, state):
        self._switch_state = state
        if state ^ self.switch_invert: # switch on, pressed
            speed = self.feed_speed if self.bInited else self.feed_speed_init
            self.feed_filament(eventtime, speed, self.switch_feed_len)
            self._loginfo("feeder %s switch pressed" % self.name)
        else: # switch off, released
            # self.feed_filament(eventtime, 0.0, 0.0)
            # self.cur_feed_len = 0.0
            self._loginfo("feeder %s switch released" % self.name)

    def _switch_update_event(self, eventtime):
        if self._switch_state ^ self.switch_invert: # switch pressed, continue feed filament.
            speed = self.feed_speed if self.bInited else self.feed_speed_init
            self.feed_filament(eventtime, speed, self.switch_feed_len)
        elif self.is_feeding: # switch released, stop feed filament.
            self.feed_filament(eventtime, 1.0, 0.0) #send 0 length to stop feed.
            if not self.bInited:
                self.bInited = True
                self._loginfo("feeder %s init feed finished, set bInited:%d" % (self.name, self.bInited), 3)

        next_time = min(eventtime + self.feed_delay, self.next_feed_time)
        next_time = max(next_time, eventtime + 0.1) # at least 0.1s.

        # check nozzle jam or feed failed when the extruder is working.
        if self.isprinting() and self.bInited:
            extruderpos = self._get_extruder_pos(eventtime)
            if extruderpos > self.runout_pos:
                self._exec_gcode(self.jam_break_gcode)

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
            'last_feed_time': self.last_feed_time,
            'enable': self.bfeeder_on,
            'inited': self.bInited,
            'fila_state': self._fila_state,
            'switch_state': self._switch_state,}

    cmd_SET_FEEDER_DISTANCE_help = "Sets a feeder hold distance"
    def cmd_SET_FEEDER_DISTANCE(self, gcmd):
        distance = gcmd.get_float('TARGET', 0.)
        gcode = self.printer.lookup_object("gcode")
        ok_msg = "cmd: %s, current distance:%.2f/%.2f(smoothed)" % (gcmd.get_commandline(), self.last_dis, self.smoothed_dis)
        self._loginfo(ok_msg)
        pfeeders = self.printer.lookup_object('filafeeders')
        pfeeders.set_distance(self, distance)

    # eg: FEED_INIT FEEDER=feeder1 SPEED=20.0 MAX_LEN=1000.0 INVERT=0 FEED_LEN=5.0 INIT=0 
    cmd_FEED_IN_help = "Begin feed filament, send to the nozzle."
    def cmd_FEED_IN(self, gcmd):
        self.max_feed_len = gcmd.get_float('MAX_LEN', self.max_feed_len, minval=1.0)
        self.switch_invert = gcmd.get_int('INVERT', 0)
        self.feed_speed = gcmd.get_float('SPEED', self.feed_speed, minval=1.0)
        self.switch_feed_len = gcmd.get_float('FEED_LEN', self.switch_feed_len)
        init = gcmd.get_int('INIT', 0)
        enable = gcmd.get_int('ENABLE', 1)
        self.bInited = not init
        if not self._fila_state:
            enable = 0  
            self._loginfo("feeder %s fila not inserted, can't feed filament" % (self.name,), logflag=3)
        self.enable_stepper(enable)

    # eg: FEED_FILA FEEDER=feeder1 SPEED=20.0 LEN=5.0
    cmd_FEED_FILA_help = "Feed LEN mm filament at SPEED."
    def cmd_FEED_FILA(self, gcmd):
        speed = gcmd.get_float('SPEED', self.feed_speed, minval=1.0)
        len = gcmd.get_float('LEN', self.switch_feed_len)
        len_feeded = self.feed_filament(self.reactor.monotonic(), speed, len)
        self._loginfo("feeder %s feed fila: %.3fmm(target:%.3f) @ %.3fmm/s" % (self.name, len_feeded, len, speed))
        self.isloginfo = gcmd.get_int('LOG', self.isloginfo)
        msg = "Set log flag:%d" % (self.isloginfo,)
        self._loginfo(msg, 1) #only response at command line.

    cmd_FEED_STATUS_help = "Show current status of the feeder."
    def cmd_FEED_STATUS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        msg = "feeder %s status: %s" % (self.name, status)
        self._loginfo(msg, 1) #only response at command line.

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
        
        # test mcu is exist.
        fila_pin = config.get('fila_pin')
        # Parse pins
        ppins = self.printer.lookup_object('pins')
        try:
            pin_params = ppins.parse_pin(fila_pin)
        except:
            logging.error("fila_pin %s not found, maybe the mcu is disconnected", fila_pin)
            # 遍历所有options, avoid the unused options error
            for option in config.get_prefix_options(''):
                option = option.lower()
                config.get(option)
            return None

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
