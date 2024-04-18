# HX710/HX711 weighing sensor support
#
# Copyright (C) 2023 guoge <guoge@tiertime.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mcu, chelper

MIN_TRIGGER_DELAY_TIME = 0.1  #min time after begin home move. avoid trigger too early before movement.
MIN_REPORT_TIME = 0.01

######################################################################
# Compatible Sensors:
#       HX710A / HX711 / HX712 
######################################################################

class HX71X_endstop:
    RETRY_QUERY = 1.000
    def __init__(self, hx71x, pin_params):
        self._hx71x = hx71x
        self._mcu = hx71x.mcu
        self._reactor = hx71x.reactor

        self._oid = self._mcu.create_oid()
        self._home_cmd = self._query_cmd = None
        self._trigger_completion = None
        self._rest_ticks = 0
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [mcu.MCU_trsync(self._mcu, self._trdispatch)]

        self.bHoming = False
        self.bTouched = False
        self.trigger_time = -1.0
        
        self.deformation = 0   # error of deformation of platform with nozzle.

        self.activetime = 0.0

    def get_mcu(self):
        return self._mcu
    def add_stepper(self, stepper):
        self._stepper = stepper
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = mcu.MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith('stepper_'):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self._mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")
    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # logging.info("start home in hx71x virtual stopend object")
        # logging.info("params: %.6f, %.6f, %d, rest time:%.6f, %d", 
        #              print_time, sample_time, sample_count, rest_time, triggered)
        #print_time 打印机时间,单位为秒.
        #sample_time 第一次触发后的采样间隔, 做sample_count递减时需要的间隔时间.防止误报
        # sample_count 采样计数, 需要有count数量的采样才认为可以触发.防止误报.
        # rest_time, 采样间隔.
        clock = self._mcu.print_time_to_clock(print_time)
        self._startclock = clock
        rest_ticks = self._mcu.print_time_to_clock(print_time+rest_time) - clock
        self._rest_ticks = rest_ticks
        # reactor = self._mcu.get_printer().get_reactor()
        self._trigger_completion = self._reactor.completion()
        expire_timeout = 0.05 #TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = 0.25 #TRSYNC_SINGLE_MCU_TIMEOUT
        for trsync in self._trsyncs:
            trsync.start(print_time, self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        # self._home_cmd.send(
        #     [self._oid, clock, self._mcu.seconds_to_clock(sample_time),
        #      sample_count, rest_ticks, triggered ^ self._invert,
        #      etrsync.get_oid(), etrsync.REASON_ENDSTOP_HIT], reqclock=clock)
        
        # 设定complete(1)可以模拟IO端口触发
        # self._trigger_completion.complete(1)

        self.activetime = print_time
        self.bHoming = True
        self.bTouched = False
        self.trigger_time = -1.0
        self._hx71x.updateNow()  # update sensor weight NOW!

        # # 采样定时器,更新位置/限位状态        
        # self._sample_timer = self._reactor.register_timer(self._sample_weight)
        # #启动采样定时器.更新重量和
        # self._reactor.update_timer(self._sample_timer, self._reactor.NOW)

        return self._trigger_completion

    def home_wait(self, home_end_time, homespeed): # the endstop has been triggered or movement is done.
        logging.info("wait home in hx71x virtual stopend object")

        # # 注销定时器,停止更新限位状态
        # self._reactor.unregister_timer(self._sample_timer)
        self.bHoming = False
        self._hx71x.updateNow()  # update sensor weight NOW! stop endstop mode.

        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()

        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        res[0] = etrsync.REASON_ENDSTOP_HIT if self.bTouched else etrsync.REASON_COMMS_TIMEOUT
        # if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
        #     return -1.
        # if res[0] != etrsync.REASON_ENDSTOP_HIT:
        #     return 0.
        if not self.bTouched:
            return 0
        
        if self._mcu.is_fileoutput():
            return home_end_time
        # params = self._query_cmd.send([self._oid])
        # self.query_endstop(home_end_time)
        # /next_clock = self._mcu.clock32_to_clock64(params['next_clock'])
        # return self._mcu.clock_to_print_time(self._startclock - self._rest_ticks)
        
        # modify the trigger time by deformation(weight) of probe.
        if homespeed>0:
            self.trigger_time -= self.deformation / homespeed
            # msg = "hx71x endstop deformation:%.4f, speed:%.1f, time offset:%.4f" % (self.deformation, homespeed, self.deformation / homespeed)
            # self._hx71x._loginfo(msg)
        return self.trigger_time
    
    def trigger(self, eventime):
        if( eventime - self.activetime < MIN_TRIGGER_DELAY_TIME):
            logging.info("Error, hx71x virtual endstop is triggered too early @ %.4f, active: %.4f", eventime, self.activetime)
            return
        if self._trigger_completion is not None :
            # msg = "hx71x virtual endstop is triggered @ %.4f with weight:%.2f" % (eventime, self._hx71x.total_weight)
            # self._hx71x._loginfo(msg)
            self._trigger_completion.complete(1)
            if not self.bTouched:
                self.bTouched = True
                self.trigger_time = eventime
                
    def query_endstop(self, eventime):
        return self._hx71x.is_endstop_on()


class HX71X:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer, config.get('hx71x_mcu', 'mcu'))
        self.oids = []
        self._endstop = None

        # add sampel record variable for hx71x sensor, 
        # use dictionary store multi hx711 unit.
        self._sample_cnt = {}  # 0
        self._sample_cnt_total = {}  # 0
        self._sample_times = 1000000000
        self._sample_tare = {}  # 0.0
        self._error_cnt = {}  # 0

        self._cnt_in_cycle = 0

        # add simulate variable for temperature sensor.
        self.last_temp = 0.
        self.measured_min = 99999999.
        self.measured_max = 0.

        self.weight = {}  # 0.0
        self.read_time = {}  # 0.0
        self.weight_min = {}  # 0.0
        self.weight_max = {}  # 0.0
        self.total_weight = 0.0
        self.total_weight_min = 0.0
        self.total_weight_max = 0.0
        self.prev_weight = 0.0

        # Determine pin from config
        ppins = config.get_printer().lookup_object("pins")

        # register chip for add endstop by setup_pin
        ppins.register_chip(self.name, self)

        N = 10  # max hx71x unit number.
        for i in range(N):
            # Add pin for one hx711 unit.
            sck = config.get('hx71x_sck_pin_'+str(i), None)
            out = config.get('hx71x_dout_pin_'+str(i), None)
            if sck is None and i == 0:  # only one hx711 unit.
                sck = config.get('hx71x_sck_pin', None)
                out = config.get('hx71x_dout_pin', None)
                
            if sck is None or out is None:
                break

            sck_params = ppins.lookup_pin(sck)
            dout_params = ppins.lookup_pin(out)
            oid = self.mcu.create_oid()
            self.oids.append(oid)

            # Add config commands
            self.mcu.add_config_cmd(
                "config_hx71x oid=%d  sck_pin=%s dout_pin=%s"
                % (oid, sck_params['pin'], dout_params['pin']))

            self._sample_cnt[oid] = 0
            self._sample_cnt_total[oid] = 0
            self._sample_tare[oid] = 0.0
            self._error_cnt[oid] = 0

            self.weight[oid] = 0.0
            self.weight_min[oid] = 0.0
            self.weight_max[oid] = 0.0
            self.read_time[oid] = 0.0

        # update period
        self.report_time = config.getfloat('hx71x_report_time', 1, minval=MIN_REPORT_TIME)
        self.pulse_cnt = config.getint('hx71x_pluse_cnt', 25)

        # unit scale
        self.scale = config.getfloat('hx71x_scale', 0.001)

        # set base value and triger threshold for endstop
        self.endstop_base = config.getfloat('endstop_base', 0.0)
        self.endstop_threshold = config.getfloat('endstop_threshold', 100.0)
        self.endstop_max = config.getfloat('endstop_max', self.endstop_threshold * 20)
        self.endstop_report_time = config.getfloat('endstop_report_time', 0.05, minval=MIN_REPORT_TIME)
        self.endstop_trigger_delay = config.getfloat('endstop_trigger_deley', 0.01)
        self.endstop_deformation = config.getfloat("endstop_deformation", 600.0) # 600 gram / 0.1mm deformation

        # set collision warning value for endstop or z motor collision
        self.collision_err = config.getfloat('collision_err', 0.0)

        #test weight sensor is ok or stepper motor is ok
        self.test_min = config.getfloat('test_min', 100.0)
        self.test_max = config.getfloat('test_max', 1000.0)
        self.test_gcode = {}
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        for i in range(4):  # max 4 test gcode
            self.test_gcode[i] =  gcode_macro.load_template(config, 'test_gcode_%d' % i, "")

        # set gcode response time, default is 0. display the weight in gcode response.
        self.gcode_response_time = config.getfloat('gcode_response_time', 0.0)
        self.last_response_time = 0.0  # self.reactor.monotonic()

        # set response threshold for response, response when the weight change is bigger than it.
        self.gcode_response_threshold = config.getfloat('gcode_response_threshold', 1.0)
        self.last_response_weight = 0.0

        # self.sample_timer = self.reactor.register_timer(self._sample_hx71x)
        self.printer.add_object("hx71x " + self.name, self)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)

        # register a sensor type for HX71X
        pheaters = self.printer.load_object(config, 'heaters')
        pheaters.add_sensor_factory("HX71X", HX71X)
        # self.sensor = pheaters.setup_sensor(config)
        # self.min_temp = config.getfloat('min_temp', KELVIN_TO_CELSIUS,
        #                                 minval=KELVIN_TO_CELSIUS)
        # self.max_temp = config.getfloat('max_temp', 99999999.9,
        #                                 above=self.min_temp)
        # self.sensor.setup_minmax(self.min_temp, self.max_temp)
        # self.setup_callback(self.temperature_callback)
        # self._callback = self.temperature_callback
        # pheaters.register_sensor(config, self)

        # callback function, call the function to update heater's temperature data.
        self._callback = None
        
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("QUERY_WEIGHT", "SENSOR", self.name,
                                        self.cmd_QUERY_WEIGHT,
                                        desc=self.cmd_QUERY_WEIGHT_help)
        self.gcode.register_mux_command("TARE_WEIGHT", "SENSOR", self.name,
                                        self.cmd_TARE_WEIGHT,
                                        desc=self.cmd_TARE_WEIGHT_help)
        self.gcode.register_mux_command("RESPONSE_WEIGHT", "SENSOR", self.name,
                                        self.cmd_RESPONSE_WEIGHT,
                                        desc=self.cmd_RESPONSE_WEIGHT_help)
        self.gcode.register_mux_command("TEST_WEIGHT", "SENSOR", self.name,
                                        self.cmd_TEST_WEIGHT,
                                        desc=self.cmd_TEST_WEIGHT_help)

    cmd_QUERY_WEIGHT_help = "Report on the status of a group of hx71x sensors, QUERY_WEIGHT SENSOR=xxxxx"
    def cmd_QUERY_WEIGHT(self, gcmd):
        out = []
        out.append(" Total: %.3fg (%.3f~%.3f)" % (self.total_weight, self.total_weight_min, self.total_weight_max))
        for oid in self.oids:
            out.append("\n oid%d: %.3fg(%.3f~%.3f) @%.3fs  CNT:%d" % 
                       (oid, self.weight[oid], self.weight_min[oid], self.weight_max[oid], self.read_time[oid], self._sample_cnt[oid]))
        out = " ".join(out)
        gcmd.respond_info("Sensor: " + self.name + out)

    cmd_TARE_WEIGHT_help = "Tare the weight sensor, TARE_WEIGHT SENSOR=xxxxx"
    def cmd_TARE_WEIGHT(self, gcmd):
        for oid in self.oids:
            self._sample_tare[oid] += self.weight[oid]
            self.weight[oid] = 0.0
            self.weight_min[oid] = self.weight_max[oid] = 0.0

        self.total_weight = self.total_weight_min = self.total_weight_max = 0.0

    cmd_RESPONSE_WEIGHT_help = "Set the GCode respose time of the weight sensor, paramters: TIME, THRESHOLD, REPORT"
    def cmd_RESPONSE_WEIGHT(self, gcmd):
        self.gcode_response_time = gcmd.get_float('TIME', self.gcode_response_time, minval=0.0)
        self.gcode_response_threshold = gcmd.get_float('THRESHOLD', self.gcode_response_threshold, minval=0.0)
        self.report_time = gcmd.get_float('REPORT', self.report_time, minval=MIN_REPORT_TIME)
        self.updateNow()
        msg = "Set HX71X sensor response time: %.2f, report time:%.2f, threshold: %.2f" % (self.gcode_response_time, self.report_time, self.gcode_response_threshold)
        self._loginfo(msg)

    cmd_TEST_WEIGHT_help = "test the weight sensor with a threshold by run a gcode cmd, parameter: ID, MIN, MAX, COLLISION"
    def cmd_TEST_WEIGHT(self, gcmd):
        id = gcmd.get_int('ID', -1, minval=-1, maxval=3)
        thMin = gcmd.get_float('MIN', self.test_min, minval=0.0)
        thMax = gcmd.get_float('MAX', self.test_max, minval=thMin)
        self.collision_err = gcmd.get_float('COLLISION', self.collision_err, minval=0.0)

        curTime = self.mcu.estimated_print_time(self.reactor.monotonic())
        self._loginfo("Test HX71X sensor with a program(ID:%d, threshold:%.2f~%.2f, collision:%.2f) @ %.3fs" % (id, thMin, thMax,self.collision_err, curTime))


        if id>= 0: # run a gcode cmd to test the weight sensor.
            self.cmd_TARE_WEIGHT(" ") # tare the weight sensor before run the test.

            # run a gcode cmd to test the weight sensor. # etc: "FORCE_MOVE STEPPER=stepper_z1 DISTANCE=5 VELOCITY=1 ACCEL=50"
            template = self.test_gcode[id]
            cmdstr = template.render()
            self._loginfo("Run test gcode cmd: %s" % cmdstr)
            self.gcode.run_script_from_command(cmdstr)
            # self.gcode.run_script(cmdstr)

        # analyze the weight sensor data, get max diff and min diff.
        curTime = self.mcu.estimated_print_time(self.reactor.monotonic())
        minDiff = min(self.weight_min.values())
        maxDiff = max(self.weight_max.values())            
        maxErr = max(abs(minDiff), abs(maxDiff))

        if( maxErr < thMin or maxErr > thMax ):
            msg = "Error, Weight sensor(HX71x) test failed, min: %.3f, max: %.3f, (threshold: %.2f~%.2f)" % (minDiff, maxDiff, thMin, thMax)
            # self._loginfo(msg)
            raise gcmd.error(msg)
            # self.gcode.run_script_from_command("M112")  # emergency stop
        else:
            self._loginfo("Weight sensor(HX71x) test passed, min: %.3f, max: %.3f, (threshold: %.2f~%.2f)" % (minDiff, maxDiff, thMin, thMax) )

    def handle_connect(self):
        # self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        return
    
    def _loginfo(self, msg):
        logging.info(msg)
        self.gcode.respond_info(msg)

    def build_config(self):
        # self.read_hx71x_cmd = self.mcu.lookup_query_command(
        #     "read_hx71x oid=%c read_len=%u",
        #     "read_hx71x_response oid=%c response=%*s", oid=self.oid,
        #     cq=self.cmd_queue)
        ticks = self.mcu.seconds_to_clock(self.report_time)

        for oid in self.oids:
            self.mcu.add_config_cmd( "query_hx71x oid=%d ticks=%d times=%d pulse_cnt=%d" % 
                (oid, ticks, self._sample_times, self.pulse_cnt))
            self.mcu.register_response(self._handle_hx71x_state, "hx71x_state", oid)

    def _handle_hx71x_state(self, params):
        oid = params['oid']
        value = params['value']
        # self._sample_cnt += 1
        self._sample_cnt[oid] = params['cnt']
        # get hx71x sample time.
        next_clock = self.mcu.clock32_to_clock64(params['next_clock']) # next_clock is later than the real sample time.
        last_read_time = self.mcu.clock_to_print_time(next_clock)

        if value == 0 or abs(value-0x800000)<0x100:
            self._error_cnt[oid] += 1
            if (self._error_cnt[oid] < 100) or ((self._error_cnt[oid] % 16)==0):
                logging.info("  *** Error Senser:%s(oid:%d) can't read hx711 or data error @ %.3f, cnt:%d, value:%d(0x%X), errcnt:%d", 
                         self.name, oid, last_read_time, self._sample_cnt[oid], value, value, self._error_cnt[oid])
            return
                
        self.weight[oid] = value * self.scale  # weight scale
        self.read_time[oid] = last_read_time  # read time
        
        self._sample_cnt_total[oid] += 1

        # 头五次作去皮处理
        # if self._sample_cnt < 5 :
        #     self._sample_tare = self.weight
        if self._sample_cnt_total[oid] < 5:
            self._sample_tare[oid] = self.weight[oid]
            
        self.weight[oid] -= self._sample_tare[oid]
        self.weight_min[oid] = min(self.weight_min[oid], self.weight[oid])
        self.weight_max[oid] = max(self.weight_max[oid], self.weight[oid])

        # debug log, print hx711 read value every 256 times.
        if self._sample_cnt[oid] < 10 or (self._sample_cnt[oid] % 256) == 0:
            logging.info("Senser:%s(oid:%d) read hx711 @ %.3f , weight:%.2f, cnt:%d, tare:%.2f, value:%d", 
                         self.name, oid, last_read_time, self.weight[oid], self._sample_cnt[oid], self._sample_tare[oid], value)
            
        # collision warning test
        if self.collision_err > 0 and abs(self.weight[oid]) > self.collision_err:
            msg = "Weight senser:%s(oid:%d) collision warning, weight:%.2f(%d-%X). Shutdown the printer!" % (self.name, oid, self.weight[oid], value, value)
            self._loginfo(msg)
            self.gcode.run_script_from_command("M112")  # emergency stop

        # update total weight when all seners are read.
        self._cnt_in_cycle += 1
        if self._cnt_in_cycle < len(self.oids):
            return
        self._cnt_in_cycle = 0

        # update total weight
        self.prev_weight = self.total_weight
        self.total_weight = 0.0
        for oid in self.oids:
            self.total_weight += self.weight[oid]
        self.total_weight_min = min(self.total_weight_min, self.total_weight)
        self.total_weight_max = max(self.total_weight_max, self.total_weight)

        # use total weight as temperature.
        self.last_temp = self.total_weight
        self.measured_min = min(self.measured_min, self.last_temp)
        self.measured_max = max(self.measured_max, self.last_temp)

        # report weight periodically or the change of weight is bigger than threshold.
        bResponse = False
        if( self.gcode_response_time > 0 and (last_read_time - self.last_response_time) > self.gcode_response_time):
            if( abs(self.total_weight - self.last_response_weight) > self.gcode_response_threshold):
                bResponse = True
            elif( last_read_time - self.last_response_time > (100.0*self.gcode_response_time)): # force response every 100 times of response time.
                bResponse = True

        if bResponse:
            self.last_response_weight = self.total_weight
            self.last_response_time = last_read_time
            msg = "Read HX71X multi sensors: %s  total Weight: %.2f @ %.3f" % (self.name, self.total_weight, last_read_time)
            self._loginfo(msg)
            if( self._endstop is None):
                self._loginfo("Error, no endstop for HX71X sensor!")

        # call callback function to update the temperature of heaters.
        if self._callback is not None:
            self._callback(last_read_time, self.last_temp)  # callback to update the temperature of heaters.

        # debug log, print weight when over endstop threshold every 16 times.
        if (self._endstop is not None) and (self._sample_cnt_total[oid] % 16) == 0:
            if self.is_endstop_on():
                msg = "Weight:%.2f, over endstop threshold: %.2f @ %.3f" % (self.total_weight, self.endstop_threshold, last_read_time)
                self._loginfo(msg)

        # timer interval is short when homing
        if (self._endstop is not None) and self._endstop.bHoming:
            # call endstop trigger function.
            if self.is_endstop_on():
                self._endstop.deformation = 0.1 * self.total_weight / self.endstop_deformation
                self._endstop.trigger(last_read_time - self.endstop_trigger_delay)

    # compare the total weight with threshold, if total weight is bigger than it, return True.
    def is_endstop_on(self):
        if self.total_weight > self.endstop_threshold:
            # prev weight should less than threshold, or the weight is bigger than threshold2.
            if self.prev_weight<self.endstop_threshold or self.total_weight > self.endstop_max:
                return True
        return False

    # def read_hx71x(self, read_len):
    #     return self.read_hx71x_cmd.send([self.oid, read_len])

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        if (self._endstop is not None) and self._endstop.bHoming:
            return self.endstop_report_time
        else:
            return self.report_time

    def updateNow(self):
        # self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        ticks = self.mcu.seconds_to_clock(self.report_time)
        if (self._endstop is not None) and self._endstop.bHoming:  # 在回零期间.
            ticks = self.mcu.seconds_to_clock(self.endstop_report_time)
            msg = "Start update hx71x at endstop mode, ticks:%d, time:%.3f" % (ticks, self.endstop_report_time)
            self._loginfo(msg)

        # debug log, update ticks and time.
        curTime = self.mcu.estimated_print_time(self.reactor.monotonic())
        msg = "Reset hx71x update ticks to %d @ %.3f" % (ticks, curTime)
        self._loginfo(msg)

        # 发送配置命令. 不能用add_config_cmd
        for oid in self.oids:
            self.mcu._serial.send("query_hx71x oid=%d ticks=%d times=%d pulse_cnt=%d" % (oid, ticks, self._sample_times, self.pulse_cnt))
        return

    # def _sample_hx71x(self, eventtime):
    #     params = self.read_hx71x(4)

    #     response = bytearray(params['response'])
    #     w = int.from_bytes(response, byteorder='little', signed=True)
    #     self.weight = w * self.scale # weight scale

    #     self._sample_cnt += 1
        
    #     if self._sample_cnt < 5 :
    #         self._sample_tare = self.weight
    #     else:
    #         self.weight -= self._sample_tare

    #     # logging.info("Senser:%s,  read hx711 @ %.3f , weight:%.2f", self.name, eventtime, self.weight)

    #     # use weight as temperature.
    #     self.last_temp = self.weight
    #     if self._callback is not None:
    #         self._callback(eventtime, self.last_temp) #callback to update the temperature of heaters.

    #     # timer interval is short when homing
    #     if (self._endstop is not None) and self._endstop.bHoming :
    #         if self.weight > (self.endstop_base + self.endstop_threshold) :
    #             self._endstop.trigger(eventtime)
    #         return eventtime + self.endstop_report_time
    #     else :
    #         return eventtime + self.report_time

    def get_status(self, eventtime):
        state = {
            'weight': round(self.total_weight, 2),
            'temperature': round(self.last_temp, 2),
            'measured_min_temp': round(self.measured_min, 2),
            'measured_max_temp': round(self.measured_max, 2)
        }

        idx = 0
        for oid in self.oids:
            idx += 1
            state['weight%d' % idx] = round(self.weight[oid], 2)
            state['weight%d_min' % idx] = round(self.weight_min[oid], 2)
            state['weight%d_max' % idx] = round(self.weight_max[oid], 2)
        
        # analyze the weight sensor data, get max diff and min diff.
        minDiff = min(self.weight_min.values())
        maxDiff = max(self.weight_max.values())            
        maxErr = max(abs(minDiff), abs(maxDiff))
        state['min_diff'] = round(minDiff, 2)
        state['max_diff'] = round(maxDiff, 2)
        state['max_err'] = round(maxErr, 2)

        return state

    def setup_pin(self, pin_type, pin_params):
        msg = "add a hx71x endstop, type:%s, pin: %s", pin_type, pin_params['pin']
        logging.info(msg)
        # self._loginfo(msg)

        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise self.error("HX71X virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise self.error("Can not pullup/invert HX71X virtual endstop")
        
        self._endstop = HX71X_endstop(self, pin_params)
        return self._endstop

    def get_temp(self, eventtime):
        logging.info("call HX71X.get_temp() of %s ,eventtime: %.2f ", self.name, eventtime)
        return self.last_temp, 0.
    
    def stats(self, eventtime):
        # logging.info("call HX71X.stats() of %s, eventtime: %.2f, temp:%.2f ", self.name, eventtime, self.last_temp)
        return False, '%s: temp=%.1f' % (self.name, self.last_temp)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
        return


def load_config(config):
    return HX71X(config)


def load_config_prefix(config):
    return HX71X(config)
