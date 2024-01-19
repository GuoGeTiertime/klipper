# HX710/HX711 weighing sensor support
#
# Copyright (C) 2023 guoge <guoge@tiertime.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mcu, chelper

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
        logging.info("start home in hx71x virtual stopend object")
        logging.info("params: %.6f, %.6f, %d, rest time:%.6f, %d", 
                     print_time, sample_time, sample_count, rest_time, triggered)
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

        self.bHoming = True
        self.bTouched = False
        self.trigger_time = -1.0
        self._hx71x.updateNow()  # update sensor weight NOW!

        # # 采样定时器,更新位置/限位状态        
        # self._sample_timer = self._reactor.register_timer(self._sample_weight)
        # #启动采样定时器.更新重量和
        # self._reactor.update_timer(self._sample_timer, self._reactor.NOW)

        return self._trigger_completion

    def home_wait(self, home_end_time):
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
        return self.trigger_time
    
    def trigger(self, eventime):
        if self._trigger_completion is not None :
            logging.info("hx71x virtual endstop is triggered @ %.4f", eventime)
            self._trigger_completion.complete(1)
            if not self.bTouched:
                self.bTouched = True
                self.trigger_time = eventime
                
    def query_endstop(self, eventime):
        return self._hx71x.is_endstop_on(eventime)


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

        # add simulate variable for temperature sensor.
        self.last_temp = 0.
        self.measured_min = 99999999.
        self.measured_max = 0.

        self.weight = {}  # 0.0
        self.read_time = {}  # 0.0
        self.total_weight = 0.0

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
            self.read_time[oid] = 0.0

        # update period
        self.report_time = config.getfloat('hx71x_report_time', 1, minval=0.02)
        self.pulse_cnt = 25

        # unit scale
        self.scale = config.getfloat('hx71x_scale', 0.001)

        # set base value and triger threshold for endstop
        self.endstop_base = config.getfloat('endstop_base', 0.0)
        self.endstop_threshold = config.getfloat('endstop_threshold', 100.0)
        self.endstop_report_time = config.getfloat('endstop_report_time', 0.05, minval=0.02)

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

    cmd_QUERY_WEIGHT_help = "Report on the status of a group of hx71x sensors"
    def cmd_QUERY_WEIGHT(self, gcmd):
        out = []
        out.append(" Total: %.3fg " % self.total_weight)
        for oid in self.oids:
            out.append(" oid%d: %.3fg / %.3fs / %dcnt " % (oid, self.weight[oid], self.read_time[oid], self._sample_cnt[oid]))
        out = " ".join(out)
        gcmd.respond_info(self.name + ": " + out)

    cmd_TARE_WEIGHT_help = "Tare the weight sensor"
    def cmd_TARE_WEIGHT(self, gcmd):
        for oid in self.oids:
            self._sample_tare[oid] += self.weight[oid]
        self.total_weight = 0.0

    def handle_connect(self):
        # self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        return

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
        next_clock = self.mcu.clock32_to_clock64(params['next_clock'])
        last_read_time = self.mcu.clock_to_print_time(next_clock)

        if value == 0:
            self._error_cnt[oid] += 1
            if self._error_cnt[oid] > 100 and (self._error_cnt[oid] % 16):
                return
            logging.info("  *** Error Senser:%s(oid:%d) can't read hx711 @ %.3f, cnt:%d, value:%d, errcnt:%d", self.name, oid, last_read_time, self._sample_cnt[oid], value, self._error_cnt[oid])
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

        if self._sample_cnt[oid] < 1000 or (self._sample_cnt[oid] % 32) == 0:
            logging.info("Senser:%s(oid:%d) read hx711 @ %.3f , weight:%.2f, cnt:%d, tare:%.2f, value:%d", 
                         self.name, oid, last_read_time, self.weight[oid], self._sample_cnt[oid], self._sample_tare[oid], value)

        # update total weight
        self.total_weight = 0.0
        for oid in self.oids:
            self.total_weight += self.weight[oid]

        # use total weight as temperature.
        self.last_temp = self.total_weight
        self.measured_min = min(self.measured_min, self.last_temp)
        self.measured_max = max(self.measured_max, self.last_temp)

        # call callback function to update the temperature of heaters.
        if self._callback is not None:
            self._callback(last_read_time, self.last_temp)  # callback to update the temperature of heaters.

        # timer interval is short when homing
        if (self._endstop is not None) and self._endstop.bHoming:
            # call endstop trigger function.
            if self.is_endstop_on():
                self._endstop.trigger(last_read_time)

    # compare the total weight with endstop_base+threshold, if total weight is bigger than it, return True.
    def is_endstop_on(self, eventtime):
        if self.total_weight > (self.endstop_base + self.endstop_threshold):
            return True
        else:
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
            logging.info("Start update hx71x at endstop mode, ticks:%d, time:%.3f", ticks, self.endstop_report_time)

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
        return {
            'weight': round(self.total_weight, 2),
            'temperature': round(self.last_temp, 2),
            'measured_min_temp': round(self.measured_min, 2),
            'measured_max_temp': round(self.measured_max, 2)
        }

    def setup_pin(self, pin_type, pin_params):
        logging.info("add a hx71x endstop, type:%s, pin: %s", pin_type, pin_params['pin'])
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
