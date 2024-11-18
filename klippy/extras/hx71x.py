# HX710/HX711 weighing sensor support
#
# Copyright (C) 2023 guoge <guoge@tiertime.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mcu, chelper
import threading

MIN_TRIGGER_DELAY_TIME = 0.1  #min time after begin home move. avoid trigger too early before movement.
MIN_REPORT_TIME = 0.001 #max HX71X frequency is 320Hz(HX717),

class weight_sensor:
    def __init__(self, scale):
        self.prevValue = 0      #前一次读数
        self.curValue = 0       #当前读数
        self.scale = scale      #单位换算系数
        self.tareValue = 0      #去皮值
        self.weight = 0         #重量
        self.minWeight = 0      #最小重量
        self.maxWeight = 0      #最大重量

    def _tare(self):
        self.tareValue = self.curValue
        self.weight = self.minWeight = self.maxWeight = 0

    def update(self, value):
        self.prevValue = self.curValue
        self.curValue = value
        self.weight = (self.curValue - self.tareValue) * self.scale
        self.minWeight = min(self.minWeight, self.weight)
        self.maxWeight = max(self.maxWeight, self.weight)

    def strCurPrev(self):
        return "cur:%d(0x%X) prev:%d(0x%X)" % (self.curValue, self.curValue, self.prevValue, self.prevValue)

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
        self.activecurtime = 0.0

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
            report_offset = 0.1 # float(i) / len(self._trsyncs)
            trsync.start(print_time, report_offset, self._trigger_completion, expire_timeout)
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
        self.activecurtime = self._hx71x.reactor.monotonic()
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
        curtime = self._hx71x.reactor.monotonic()
        logging.info("Call HX71X_endstop.home_wait() @ curtime: %.4f, home end time:%.4f, diff:%.4f", curtime, home_end_time, home_end_time-curtime)

        # # 注销定时器,停止更新限位状态
        # self._reactor.unregister_timer(self._sample_timer)
        self.bHoming = False
        # self._hx71x.updateNow()  # update sensor weight NOW! stop endstop mode.

        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        curtime = self._hx71x.reactor.monotonic()
        logging.info("wait trigger completion @ curtime: %.4f", curtime)

        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        res[0] = etrsync.REASON_ENDSTOP_HIT if self.bTouched else etrsync.REASON_COMMS_TIMEOUT
        # if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
        #     return -1.
        # if res[0] != etrsync.REASON_ENDSTOP_HIT:
        #     return 0.
        if self._hx71x.isCommErr:
            raise self._hx71x.printer.command_error( "HX71X communication error during homing")
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
        curtime = self._hx71x.reactor.monotonic()
        logging.info("exit HX71X_endstop.home_wait() @ curtime: %.4f trigger time:%.4f", curtime, self.trigger_time)
        return self.trigger_time
    
    def trigger(self, eventime):
        if( eventime - self.activetime < MIN_TRIGGER_DELAY_TIME):
            # curtime = self._hx71x.reactor.monotonic()
            # logging.info("Error, hx71x virtual endstop is triggered too early @ %.4f, active: %.4f, active:%.4f/curtime:%.4f", eventime, self.activetime, self.activecurtime, curtime)
            return
        if self._trigger_completion is not None :
            # msg = "hx71x virtual endstop is triggered @ %.4f with weight:%.2f" % (eventime, self._hx71x.all_sensor.weight)
            # self._hx71x._loginfo(msg)
            self._trigger_completion.complete(1)
            if not self.bTouched:
                self.bTouched = True
                self.trigger_time = eventime
                curtime = self._hx71x.reactor.monotonic()
                logging.info("trigger hx71x virtual endstop @ eventtime: %.4f, curtime: %.4f", eventime, curtime)

    def query_endstop(self, eventime):
        return self._hx71x.is_endstop_on()


class HX71X:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer, config.get('hx71x_mcu', 'mcu'))
        self.oid = self.mcu.create_oid()
        self._endstop = None
        self.lock = threading.Lock()
        self.isloginfo = 0  # 0: no log, 1:gcode response, 2: write log file, 3: response and write log file

        # add sampel record variable for hx71x sensor, 
        # use list store the value of multi hx711 unit.
        self._sample_cnt = 0
        self._sample_times = 1000000000
        self._error_cnt = 0
        self.collision_cnt = 0
        # unit scale
        self.scale = config.getfloat('hx71x_scale', 0.001)

        # record the weight of each hx711 unit and total weight.
        self.all_sensor = weight_sensor(self.scale)
        self.sensors = []

        # register chip for add endstop by setup_pin
        ppins = config.get_printer().lookup_object("pins")
        ppins.register_chip(self.name, self)

        # Determine pin from config
        configcmd = "config_hx71x oid=%d" % self.oid
        for i in range(2): # max 6 hx71x unit number.
            # Add pin for one hx711 unit.
            sck = config.get('hx71x_sck_pin_'+str(i), None)
            dout = config.get('hx71x_dout_pin_'+str(i), None)
            if sck is None and i == 0:  # only one hx711 unit.
                sck = config.get('hx71x_sck_pin', None)
                dout = config.get('hx71x_dout_pin', None)
            
            letteri = chr(ord('a') + i) # a, b, c, d, e, f
            if sck is None or dout is None:
                configcmd += " s%c=0 d%c=0" % (letteri, letteri)
            else:
                sck_params = ppins.lookup_pin(sck)
                dout_params = ppins.lookup_pin(dout)
                configcmd += " s%c=%s d%c=%s" % (letteri, sck_params['pin'], letteri, dout_params['pin'])
                self.sensors.append(weight_sensor(self.scale))

        # Add config commands
        logging.info("%s hx71x config command: %s", self.name, configcmd) #log for debug.
        self.mcu.add_config_cmd(configcmd)
      
        # update period, HX71X pulse tiems, delay loop times.
        self.report_time = config.getfloat('hx71x_report_time', 1, minval=MIN_REPORT_TIME)
        self.pulse_cnt = config.getint('hx71x_pluse_cnt', 25)
        self.delayloop = config.getint('hx71x_delayloop', 1)

        # set base value and triger threshold for endstop
        self.endstop_base = config.getfloat('endstop_base', 0.0)
        self.endstop_threshold = config.getfloat('endstop_threshold', 100.0)
        self.endstop_max = config.getfloat('endstop_max', self.endstop_threshold * 5)
        self.endstop_report_time = config.getfloat('endstop_report_time', 0.05, minval=MIN_REPORT_TIME)
        self.endstop_trigger_delay = config.getfloat('endstop_trigger_deley', 0.01)
        self.endstop_deformation = config.getfloat("endstop_deformation", 600.0) # 600 gram / 0.1mm deformation

        # set collision warning value for endstop or z motor collision
        self.collision_err = config.getfloat('collision_err', 1000000.0)
        self.collision_err_cnt = config.getfloat('collision_err_cnt', 10) #每次+3,实际3次左右.

        self.max_comm_err = config.getint('max_comm_err', 20) # 默认连续20次出错报警.

        # 采样滤波参数,采样次数
        self._filter_times = config.getint('filter_times', 10)
        self._filter_delay_times = config.getint('filter_delay_times', 20)
        self._filter_store_times = self._filter_times + self._filter_delay_times  #存储滤波数据的次数等于滤波次数+延迟次数
        self._filter_values = []
        self._filter_cur_Values = 0.0
        # self._filter_prev_Values = 0.0

        # 超过阈值判断,用于log数据,分析, 当总重量大于overload后,输出在log中.
        self._OverLoad = config.getfloat('overload', 10000.0)

        #paramters for collision and communication error.
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.collision_gcode = gcode_macro.load_template(config, 'collision_gcode', 'M118 Collision warning by weight sensor\nM112')
        self.comm_err_gcode = gcode_macro.load_template(config, 'comm_err_gcode', 'M118 There are too many error in the weight sensor communication')
        self.gcode_interval = config.getfloat('gcode_interval', 20.0, minval=1.0, maxval=600.0)
        self.last_collision_time = 0
        self.last_comm_err_time = -10000
        self.isCommErr = False

        # log weight in gcoce response time, threshold, and log flag.
        self.response_time = config.getfloat('log_time', 0.0)
        self.last_response_time = 0.0  # self.reactor.monotonic()

        # set response threshold for response, response when the weight change is bigger than it.
        self.response_threshold = config.getfloat('log_threshold', 1.0)
        self.last_response_weight = 0.0

        # self.sample_timer = self.reactor.register_timer(self._sample_hx71x)
        self.printer.add_object("hx71x " + self.name, self)

        # config callback function, start to read hx71x sensor.
        self.mcu.register_config_callback(self.build_config)

        # register a sensor type for HX71X
        # pheaters = self.printer.load_object(config, 'heaters')
        # pheaters.add_sensor_factory("HX71X", HX71X)

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
        out.append(" Total: %.3fg (%.3f~%.3f)" % (self.all_sensor.weight, self.all_sensor.minWeight, self.all_sensor.maxWeight))
        for i in range(len(self.sensors)):
            out.append("\n -- S:%d " + self.sensors[i].strCurPrev()) 
        out = " ".join(out)
        gcmd.respond_info("Sensor: " + self.name + out)

    cmd_TARE_WEIGHT_help = "Tare the weight sensor, TARE_WEIGHT SENSOR=xxxxx"
    def cmd_TARE_WEIGHT(self, gcmd):
        with self.lock:
            self.all_sensor._tare()
            self._filter_values.clear()
            for i in range(len(self.sensors)):
                self.sensors[i]._tare()

    cmd_RESPONSE_WEIGHT_help = "Set the GCode respose time of the weight sensor, paramters: TIME, THRESHOLD, REPORT LOG"
    def cmd_RESPONSE_WEIGHT(self, gcmd):
        self.response_time = gcmd.get_float('TIME', self.response_time, minval=0.0)
        self.response_threshold = gcmd.get_float('THRESHOLD', self.response_threshold, minval=0.0)
        self.report_time = gcmd.get_float('REPORT', self.report_time, minval=MIN_REPORT_TIME)
        self.isloginfo = gcmd.get_int('LOG', self.isloginfo)
        self.updateNow()
        msg = "Set HX71X sensor log time:%.2f, update time:%.4f, threshold:%.1f, log:%d" % (self.response_time, self.report_time, self.response_threshold, self.isloginfo)
        self._loginfo(msg)

    cmd_TEST_WEIGHT_help = "Descarded cmd"
    def cmd_TEST_WEIGHT(self, gcmd):
        raise gcmd.error("Error, HX71X sensor test is not supported now!")

    def _loginfo(self, msg, logflag=None):
        if logflag is None:
            logflag = self.isloginfo
        if logflag == 1:
            self.gcode.respond_info(msg, False) # only respond to gcode.
        elif logflag == 2:
           logging.info(msg) # only write log file.
        elif logflag == 3:
            self.gcode.respond_info(msg, True) # respond to gcode and write log file.

    # exec gcode template after collsion / communication error event.
    def _exec_gcode(self, script_template, now=False):
        if now:
            script_template.run_gcode_from_command()
        else:
            try:
                self.gcode.run_script( script_template.render() )
            except Exception:
                self._loginfo("hx71x %s script running error" % (self.name,), 3)

    def _collision_handler(self, eventtime):
        self._exec_gcode(self.collision_gcode, now=True)
    def _comm_err_handler(self, eventtime):
        self._exec_gcode(self.comm_err_gcode)

    def build_config(self):
        ticks = self.mcu.seconds_to_clock(self.report_time)
        self.mcu.add_config_cmd( "query_hx71x oid=%d ticks=%d times=%d pulse_cnt=%d delayloop=%d" 
                                % (self.oid, ticks, self._sample_times, self.pulse_cnt, self.delayloop) )
        self.mcu.register_response(self._handle_hx71x_state, "hx71x_state", self.oid)

    def _endstop_trigger(self, last_read_time):
        if (self._endstop is not None) and self._endstop.bHoming:
            # call endstop trigger function.
            if self.is_endstop_on():
                self._endstop.deformation = 0.1 * self.all_sensor.weight / self.endstop_deformation
                self._endstop.trigger(last_read_time - self.endstop_trigger_delay)

    def _handle_hx71x_state(self, params):
        # get hx71x sample time, cnt
        self._sample_cnt = params['cnt']
        next_clock = self.mcu.clock32_to_clock64(params['next_clock']) # next_clock is later than the real sample time.
        last_read_time = self.mcu.clock_to_print_time(next_clock)

        bErrorValue = False
        # errorValue = 0
        readvalue = [0] * len(self.sensors)
        with self.lock:
            for i in range(len(self.sensors)):
                readvalue[i] = params['v'+str(i)]
                # check the value is zero or wrong value.
                bZeroValue = readvalue[i] == 0
                bWrongValue = abs(readvalue[i]-0x800000)<0x10 and abs(readvalue[i] - self.prevValues[i]) > abs(100.0/self.scale)
                if bZeroValue or bWrongValue:
                    bErrorValue = True

        # if data error, skip the data, return
        if bErrorValue:
            self._error_cnt += 1
            errcnt = self._error_cnt
            if errcnt < 4 or (errcnt % 4)==0:
                out = []
                out.append(" **--**-- HX71x read or data error @ %.3f, cnt:%d\n -- read value -- " % (last_read_time, self._sample_cnt,))
                for i in range(len(self.sensors)):
                    out.append(" --i:%d value:%d(0x%X)" % (i, readvalue[i], readvalue[i]))
                logging.info(" ".join(out))

            if last_read_time < self.last_comm_err_time + self.gcode_interval :
                self._error_cnt = 0  # clear err cnt in interval time.
            elif errcnt == self.max_comm_err - 3:
                logging.info("hx71x %s communication errors(%d) is near max(%d)" % (self.name, errcnt, self.max_comm_err))
            elif errcnt == self.max_comm_err:
                logging.info("hx71x %s communication errors(%d) is reach max(%d), run communication error script" % (self.name, errcnt, self.max_comm_err))
                self.reactor.register_callback(self._comm_err_handler) # run script by callback function
                self.last_comm_err_time = last_read_time
                self.isCommErr = True
            if self.isCommErr:  # force set endstop on when communication error reach max.
                self._endstop_trigger(last_read_time)
            return

        # clear communication error cnt
        self._error_cnt = max(0, self._error_cnt-2)
        # rest err flag after interval time.
        if self.isCommErr and last_read_time > self.last_comm_err_time + self.gcode_interval:
            self.isCommErr = False

        # date is ok, update the weight value.
        total = 0
        with self.lock:
            for i in range(len(self.sensors)):
                total += readvalue[i]
                self.sensors[i].update(readvalue[i])
            self.all_sensor.update(total)
            self._push_filter_value(self.all_sensor.weight)

        # 头五次作去皮处理
        if self._sample_cnt < 5:
            self.cmd_TARE_WEIGHT(None)

        # add for debug very large weight.
        if abs(self.all_sensor.weight) > self._OverLoad:
            msg = "HX71X overload, total Weight: %.2f > %.1f, cnt: %d" % (self.all_sensor.weight, self._OverLoad, self._sample_cnt)
            self._loginfo(msg)

        # check for collision
        self._collision_check(last_read_time)

        self._logWeight(last_read_time)

        # call callback function to update system status.
        if self._callback is not None:
            self._callback(last_read_time, self.all_sensor.weight)  # callback to update the system status.

        # debug log, print weight when over endstop threshold every 16 times.
        # if (self._endstop is not None) and (self._sample_cnt_total[oid] % 16) == 0:
        #     if self.is_endstop_on():
        #         msg = "Weight:%.2f, over endstop threshold: %.2f @ %.3f" % (self.all_sensor.weight, self.endstop_threshold, last_read_time)
        #         self._loginfo(msg)

        # timer interval is short when homing
        self._endstop_trigger(last_read_time)

    def _collision_check(self, readtime):
        # use total weight to test collision, cnt > 10 (every time +3) ,then shutdown the printer.
        if readtime < self.last_collision_time + self.gcode_interval: # avoid run gcode too many times.
            self.collision_cnt = max(0, self.collision_cnt-1)
            return
        
        if abs(self.all_sensor.weight) > self.collision_err:
            self.collision_cnt += 3
            if self.collision_cnt > self.collision_err_cnt:
                self.collision_cnt= 0
                self.last_collision_time = readtime
                msg = "Weight senser:%s collision warning, weight:%.2f, collision count:%d. Shutdown the printer!" % (self.name, self.all_sensor.weight, self.collision_cnt)
                self._loginfo(msg, 3) #log info at command line and log file
                self.reactor.register_callback(self._collision_handler) # run script by callback function

    def _logWeight(self, readtime):
        if self.isloginfo == 0:
            return
        # report weight periodically or the change of weight is bigger than threshold.
        bResponse = False
        if( self.response_time > 0 and (readtime - self.last_response_time) > self.response_time):
            if( abs(self.all_sensor.weight - self.last_response_weight) > self.response_threshold):
                bResponse = True
            elif( readtime - self.last_response_time > (100.0*self.response_time)): # force response every 100 times of response time.
                bResponse = True
        # if self._endstop.bHoming: #add by guoge 20240424, 检测probe时,重力传感器的响应速度和变化幅度.
        #     bResponse = True
        if bResponse:
            self.last_response_weight = self.all_sensor.weight
            self.last_response_time = readtime
            out = []
            out.append("Read HX71X sensors:%s total Weight: %.2f @ %.3f\n" % (self.name, self.all_sensor.weight, readtime))
            for i in range(len(self.sensors)):
                w = self.sensors[i].weight
                v = self.sensors[i].curValue
                out.append(" %d: %.2f(%d/0x%X) " % (i, w, v, v))
            self._loginfo(" ".join(out))

    def _push_filter_value(self, weight):
        if len(self._filter_values) >= self._filter_store_times:
            self._filter_values.pop(0)  # remove the first element.
        self._filter_values.append(weight)

    def _cal_filter_value(self):
        # calculate the average value of filter values.
        total = 0.0
        n = min(len(self._filter_values), self._filter_times)
        for i in range(n):
            total += self._filter_values[i]
        self._filter_cur_Values = total / n if n > 0 else 0.0

    # compare the total weight with threshold, if total weight is bigger than it, return True.
    def is_endstop_on(self):
        if self.isCommErr :  # force set endstop on.
            return True
        
        if self._filter_times > 0: # use filter value to compare.
            self._cal_filter_value()
            if (self.all_sensor.weight - self._filter_cur_Values) > self.endstop_threshold:
                return True
            if self.all_sensor.weight > self.endstop_max:
                self._loginfo("  ***** Error, weight is over endstop_max, weight:%.2f, max:%.2f" % (self.all_sensor.weight, self.endstop_max))
                return True
            return False
        else:   # use absolute weight to compare.
            if self.all_sensor.weight > self.endstop_threshold:
                # prev weight should less than threshold, or the weight is bigger than threshold2.
                if self.prev_weight<self.endstop_threshold:
                    return True
                if self.all_sensor.weight > self.endstop_max:
                    self._loginfo("  ***** Error, weight is over endstop_max, weight:%.2f, max:%.2f, bHoming:%d" % (self.all_sensor.weight, self.endstop_max, self._endstop.bHoming))
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
        duration = self.report_time
        if (self._endstop is not None) and self._endstop.bHoming:  # 在回零期间.
            duration = self.endstop_report_time
            self._loginfo(" -- Start update hx71x at endstop mode -- ")

        ticks = self.mcu.seconds_to_clock(duration)

        # debug log, update ticks and time.
        msg = "Reset hx71x update ticks to %d Period:%.3f" % (ticks, duration)
        self._loginfo(msg)

        # 发送配置命令. 不能用add_config_cmd
        self.mcu._serial.send("query_hx71x oid=%d ticks=%d times=%d pulse_cnt=%d delayloop=%d" 
                              % (self.oid, ticks, self._sample_times, self.pulse_cnt, self.delayloop))
        return

    def get_status(self, eventtime):
        state = {
            'weight': round(self.all_sensor.weight, 2),
            'weight_min': round(self.all_sensor.minWeight, 2),
            'weight_max': round(self.all_sensor.maxWeight, 2)
        }
        # add every sensor data for debug.
        minDiff = 0
        maxDiff = 0
        for i in range(len(self.sensors)):
            sensor = self.sensors[i]
            state['weight%d' % i] = round(sensor.weight, 2)
            state['weight%d_min' % i] = round(sensor.minWeight, 2)
            state['weight%d_max' % i] = round(sensor.maxWeight, 2)
            minDiff = min(minDiff, sensor.minWeight)
            maxDiff = max(maxDiff, sensor.maxWeight)
        
        # analyze the weight sensor data, get max diff and min diff.
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
    
    def stats(self, eventtime):
        # logging.info("call HX71X.stats() of %s, eventtime: %.2f, temp:%.2f ", self.name, eventtime, self.last_temp)
        return False, '%s: weight=%.1f' % (self.name, self.all_sensor.weight)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
        return


def load_config(config):
    return HX71X(config)


def load_config_prefix(config):
    return HX71X(config)
