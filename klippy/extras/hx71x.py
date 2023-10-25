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
        self._hx71x.updateNow() #update sensor weight NOW!

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
        if not self.bTouched : 
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
            if not self.bTouched :
                self.bTouched = True
                self.trigger_time = eventime

class HX71X:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.mcu = mcu.get_printer_mcu(self.printer, config.get('hx71x_mcu', 'mcu'))
        self.oid = self.mcu.create_oid()
        self._endstop = None

        # Determine pin from config
        ppins = config.get_printer().lookup_object("pins")
        sck_params = ppins.lookup_pin(config.get('hx71x_sck_pin'))
        dout_params = ppins.lookup_pin(config.get('hx71x_dout_pin'))
        
        # register chip for add endstop by setup_pin
        ppins.register_chip(self.name, self)


        self.mcu.add_config_cmd(
            "config_hx71x oid=%d  sck_pin=%s dout_pin=%s"
            % (self.oid, sck_params['pin'], dout_params['pin']))

        #update period
        self.report_time = config.getfloat('hx71x_report_time', 1, minval=0.02)

        #unit scale
        self.scale = config.getfloat('hx71x_scale', 0.001)

        #set base value and triger threshold for endstop
        self.endstop_base = config.getfloat('endstop_base', 0.0)
        self.endstop_threshold = config.getfloat('endstop_threshold', 100.0)
        self.endstop_report_time = config.getfloat('endstop_report_time', 0.05, minval=0.02)

        self.weight = 0.0
        self.sample_timer = self.reactor.register_timer(self._sample_hx71x)
        self.printer.add_object("hx71x " + self.name, self)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        self.cmd_queue = self.mcu.alloc_command_queue()
        self.mcu.register_config_callback(self.build_config)



    def handle_connect(self):
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        return

    def build_config(self):
        self.read_hx71x_cmd = self.mcu.lookup_query_command(
            "read_hx71x oid=%c read_len=%u",
            "read_hx71x_response oid=%c response=%*s", oid=self.oid,
            cq=self.cmd_queue)

    def read_hx71x(self, read_len):
        return self.read_hx71x_cmd.send([self.oid, read_len])

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        if (self._endstop is not None) and self._endstop.bHoming :
            return self.endstop_report_time
        else :
            return self.report_time

    def updateNow(self):
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        return

    def _sample_hx71x(self, eventtime):
        params = self.read_hx71x(4)

        response = bytearray(params['response'])
        w = int.from_bytes(response, byteorder='little', signed=True)
        self.weight = w * self.scale # weight scale

        logging.info(" read hx711 @ %.3f , weight:%.2f", eventtime, self.weight)

        # timer interval is short when homing
        if (self._endstop is not None) and self._endstop.bHoming :
            if self.weight > (self.endstop_base + self.endstop_threshold) :
                self._endstop.trigger(eventtime)
            return eventtime + self.endstop_report_time
        else :
            return eventtime + self.report_time

    def get_status(self, eventtime):
        return {
            'weight': round(self.weight, 2)
        }

    def setup_pin(self, pin_type, pin_params):
        logging.info("add a hx71x endstop, type:%s, pin: %s", pin_type, pin_params['pin'])
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise self.error("HX71X virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise self.error("Can not pullup/invert HX71X virtual endstop")
        
        self._endstop = HX71X_endstop(self, pin_params)
        return self._endstop


def load_config(config):
    return HX71X(config)

def load_config_prefix(config):
    return HX71X(config)
