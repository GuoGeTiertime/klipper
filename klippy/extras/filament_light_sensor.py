# Filament Light Sensor Module (Voltage-based)
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor
from . import adc_light

DEFAULT_REPORT_TIME = 0.1  # 默认10Hz

# Dark-filament calib + sensitivity (S=1 dark … S=10 light)
RUNOUT_PULL = 0.20
PRESENT_PULL_S1 = 0.35
GAP_MIN = 2.0          # present↔runout，以及 runout↔cal_max
JUMP_MIN = 4.0
JUMP_RATIO_S1 = 0.35

class FilamentLightSensor:
    """光敏电阻丝材传感器：通过ADC读取电压，转换为电阻和光强，使用光强检测丝材状态"""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        
        # 初始化ADC转换器：从配置文件导入预定义传感器类型
        sensor_type = config.get('sensor_type')
        if not hasattr(adc_light.load_config_prefix, 'sensor_factories'):
            raise config.error("Sensor type '%s' not found. Make sure to define it with [adc_light %s] first" % (
                sensor_type, sensor_type))
        if sensor_type not in adc_light.load_config_prefix.sensor_factories:
            raise config.error("Unknown sensor_type '%s' in %s" % (sensor_type, config.get_name()))
        factory = adc_light.load_config_prefix.sensor_factories[sensor_type]
        self.adc_voltage = factory.create(config)
        
        # 设置ADC回调和采样频率
        self.adc_voltage.setup_callback(self.voltage_callback)
        self.report_time = config.getfloat('report_time', DEFAULT_REPORT_TIME, above=0.)
        self.adc_voltage.setup_adc_callback(self.report_time)
                
        # LED光源配置
        self.led_pin = None
        self.led_power = 0.0  # 当前设定功率
        self.led_enable = True
        led_pin = config.get('led_pin', None)
        if led_pin:
            ppins = self.printer.lookup_object('pins')
            self.led_pin = ppins.setup_pin('pwm', led_pin)
            self.led_pin.setup_max_duration(0.)
            led_cycle_time = config.getfloat('led_cycle_time', 0.0017, above=0.)
            hardware_pwm = config.getboolean('led_hardware_pwm', True)
            self.led_pin.setup_cycle_time(led_cycle_time, hardware_pwm)
            self.led_power = config.getfloat('led_power', 1.0, minval=0., maxval=1.)
            self.led_enable = config.getboolean('led_enable', True)
            self.led_invert = config.getboolean('led_invert', True)  # 默认True=低电平有效（需要反转）
            self.printer.register_event_handler("klippy:ready", self._setup_led)
        
        # 使用RunoutHelper处理丝材检测逻辑
        self.filabuffer_link = filament_switch_sensor.load_filabuffer_link(config)
        default_enable = not self.filabuffer_link
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.runout_helper.sensor_enabled = config.getboolean(
            'sensor_enable', default_enable)

        self.bInited = False # 是否已初始化
        self.bPresent = False # 当前是否存在丝材        
        # 无丝延迟触发配置
        self.runout_delay = config.getfloat('runout_delay', 3.0, minval=0.)  # 无丝延迟触发时间（秒），0=立即触发
        self.reactor = self.printer.get_reactor()
        self.runout_timer = None  # 无丝延迟触发定时器        
        # 丝材检测参数（使用光强阈值）
        self.lux_runout = config.getfloat('lux_runout', 30.0)  # 无材料光照度
        self.lux_present = config.getfloat('lux_present', 10.0)  # 有材料光照度
        self.alarm_count = config.getint('alarm_count', 1, minval=1)  # 报警次数
        # 内部状态
        self.last_voltage = 0.0
        self.last_resistance = 0.0  # kΩ
        self.last_lux = 0.0  # Lux
        self.alarm_trigger_count = 0  # 报警触发总次数统计, 可以删除
        
        # 光强统计（快 EMA 极值，供采样；标定锁定另存 cal_lux_*）
        self.lux_min = 999999
        self.lux_max = -999999
        self.cal_lux_min = None
        self.cal_lux_max = None
        self.sensitivity = config.getint('sensitivity', 1, minval=1, maxval=10)
        
        # 双EMA跳变检测配置
        self.ema_fast_time = config.getfloat('ema_fast_time', 2.0, above=0.)  # 快EMA时间常数（秒）
        self.ema_slow_time = config.getfloat('ema_slow_time', 30.0, above=0.)  # 慢EMA时间常数（秒）
        self.jump_threshold = config.getfloat('jump_threshold', 10.0, above=0.)  # 跳变阈值（Lux）
        self.jump_use_abs = config.getboolean('jump_use_abs', False)  # 单边/双边检测
        # 计算EMA权重（使用简化公式：α = Δt/τ，适用于 Δt << τ 的情况）
        self.alpha_fast = self.report_time / self.ema_fast_time
        self.alpha_slow = self.report_time / self.ema_slow_time
        # EMA状态变量
        self.lux_ema_fast = -1.0  # 快EMA值
        self.lux_ema_slow = -1.0  # 慢EMA值
        self.lux_ema_diff = 0.0 # EMA差值
        self.lux_ema_diff_max = 0.0  # 最大EMA差值
        
        # 注册G-code命令（使用mux支持多个传感器实例）
        self.printer.add_object('filament_light_sensor ' + self.name, self)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('QUERY_FILAMENT_LIGHT_SENSOR', 'SENSOR', self.name,
                                    self.cmd_QUERY_FILAMENT_LIGHT_SENSOR,
                                    desc=self.cmd_QUERY_FILAMENT_LIGHT_SENSOR_help)
        gcode.register_mux_command('SET_FILAMENT_LIGHT_SENSOR', 'SENSOR', self.name,
                                    self.cmd_SET_FILAMENT_LIGHT_SENSOR,
                                    desc=self.cmd_SET_FILAMENT_LIGHT_SENSOR_help)
        gcode.register_mux_command('AUTO_SET_FILAMENT_LIGHT_THRESHOLD', 'SENSOR', self.name,
                                    self.cmd_AUTO_SET_FILAMENT_LIGHT_THRESHOLD,
                                    desc=self.cmd_AUTO_SET_FILAMENT_LIGHT_THRESHOLD_help)
        gcode.register_mux_command('SAVE_FILAMENT_LIGHT_CALIB', 'SENSOR', self.name,
                                    self.cmd_SAVE_FILAMENT_LIGHT_CALIB,
                                    desc=self.cmd_SAVE_FILAMENT_LIGHT_CALIB_help)
        gcode.register_mux_command('LOAD_FILAMENT_LIGHT_CALIB', 'SENSOR', self.name,
                                    self.cmd_LOAD_FILAMENT_LIGHT_CALIB,
                                    desc=self.cmd_LOAD_FILAMENT_LIGHT_CALIB_help)
        self._calib_loaded = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready_load_calib)

    def _calib_var_prefix(self):
        return "fls_" + self.name.replace('-', '_').replace(' ', '_')

    def _sens_save_key(self):
        # One S per buffer; standalone sensor keeps per-name key
        if self.filabuffer_link is not None:
            buf = self.filabuffer_link.buffer_name.replace('-', '_').replace(' ', '_')
            return "fls_buf_%s_sensitivity" % (buf,)
        return "%s_sensitivity" % (self._calib_var_prefix(),)

    def _get_save_variables(self):
        return self.printer.lookup_object('save_variables', None)

    def _handle_ready_load_calib(self):
        try:
            self._load_calib_from_variables(respond=None)
        except Exception:
            logging.exception("[%s] load filament light calib failed" % self.name)

    def _load_calib_from_variables(self, respond=None):
        sv = self._get_save_variables()
        if sv is None:
            return False
        prefix = self._calib_var_prefix()
        v = sv.allVariables
        cal_min, cal_max = v.get("%s_cal_min" % prefix), v.get("%s_cal_max" % prefix)
        sens = v.get(self._sens_save_key())
        if sens is None:
            sens = v.get("%s_sensitivity" % prefix)  # legacy per-sensor
        runout, present, jump = (v.get("%s_runout" % prefix),
                                 v.get("%s_present" % prefix),
                                 v.get("%s_jump" % prefix))
        if all(x is None for x in (cal_min, cal_max, sens, runout, present, jump)):
            return False
        if sens is not None:
            self.sensitivity = max(1, min(10, int(float(sens))))
        if cal_min is not None and cal_max is not None:
            self.cal_lux_min, self.cal_lux_max = float(cal_min), float(cal_max)
            try:
                self._apply_sensitivity_thresholds()
            except Exception:
                if runout is not None:
                    self.lux_runout = float(runout)
                if present is not None:
                    self.lux_present = float(present)
                if jump is not None:
                    self.jump_threshold = float(jump)
        else:
            if runout is not None:
                self.lux_runout = float(runout)
            if present is not None:
                self.lux_present = float(present)
            if jump is not None:
                self.jump_threshold = float(jump)
        self._push_s_to_filabuffer()
        self._calib_loaded = True
        if respond is not None:
            respond("Loaded calib %s: present=%.2f runout=%.2f jump=%.2f S=%d"
                    % (self.name, self.lux_present, self.lux_runout,
                       self.jump_threshold, self.sensitivity))
        if self.bInited:
            self._state_init(self.last_lux)
        return True

    def _save_calib_to_variables(self, respond=None, save_jump=True, save_s=True):
        sv = self._get_save_variables()
        if sv is None:
            if respond is not None:
                respond("save_variables not configured")
            return False
        gcode = self.printer.lookup_object('gcode')
        p = self._calib_var_prefix()
        cmds = [
            "SAVE_VARIABLE VARIABLE=%s_runout VALUE=%.4f" % (p, self.lux_runout),
            "SAVE_VARIABLE VARIABLE=%s_present VALUE=%.4f" % (p, self.lux_present),
        ]
        if save_jump:
            cmds.append("SAVE_VARIABLE VARIABLE=%s_jump VALUE=%.4f"
                        % (p, self.jump_threshold))
        if self.cal_lux_min is not None and self.cal_lux_max is not None:
            cmds.append("SAVE_VARIABLE VARIABLE=%s_cal_min VALUE=%.4f"
                        % (p, self.cal_lux_min))
            cmds.append("SAVE_VARIABLE VARIABLE=%s_cal_max VALUE=%.4f"
                        % (p, self.cal_lux_max))
        if save_s:
            cmds.append("SAVE_VARIABLE VARIABLE=%s VALUE=%d"
                        % (self._sens_save_key(), self.sensitivity))
        for cmd in cmds:
            gcode.run_script_from_command(cmd)
        self._calib_loaded = True
        if respond is not None:
            respond("Saved calib %s: present=%.2f runout=%.2f jump=%.2f S=%d"
                    % (self.name, self.lux_present, self.lux_runout,
                       self.jump_threshold, self.sensitivity))
        return True

    def _push_s_to_filabuffer(self):
        if self.filabuffer_link is None:
            return
        mgr = self.printer.lookup_object('filabuffer', None)
        fb = None if mgr is None else mgr.buffers.get(self.filabuffer_link.buffer_name)
        if fb is not None:
            fb.light_sensitivity = self.sensitivity

    def apply_buffer_sensitivity(self, sensitivity, save=False, respond=None):
        """One S for all light sensors on this filabuffer; optional SAVE of thresholds + S."""
        sensitivity = max(1, min(10, int(sensitivity)))
        bname = None if self.filabuffer_link is None else self.filabuffer_link.buffer_name
        n = 0
        for _, obj in self.printer.lookup_objects('filament_light_sensor'):
            link = getattr(obj, 'filabuffer_link', None)
            if bname is None:
                if obj is not self:
                    continue
            elif link is None or link.buffer_name != bname:
                continue
            obj.sensitivity = sensitivity
            if obj.cal_lux_min is None or obj.cal_lux_max is None:
                continue
            try:
                obj._apply_sensitivity_thresholds()
            except ValueError:
                continue
            if obj.bInited:
                obj._state_init(obj.last_lux)
            n += 1
            if save:
                obj._save_calib_to_variables(save_s=False)
        self._push_s_to_filabuffer()
        if save:
            sv = self._get_save_variables()
            if sv is not None:
                self.printer.lookup_object('gcode').run_script_from_command(
                    "SAVE_VARIABLE VARIABLE=%s VALUE=%d"
                    % (self._sens_save_key(), sensitivity))
        if respond is not None:
            respond("light S=%d on %s (%d applied)" % (sensitivity, bname or self.name, n))
        return n

    def _apply_sensitivity_thresholds(self, set_jump=True):
        lo, hi = self.cal_lux_min, self.cal_lux_max
        if lo is None or hi is None:
            raise ValueError("No cal range for %s" % (self.name,))
        span = hi - lo
        if span <= 0.:
            raise ValueError("cal span must be > 0")
        r10 = hi - GAP_MIN
        r1 = min(hi - span * RUNOUT_PULL, r10)
        p10 = r10 - GAP_MIN
        p1 = min(lo + span * PRESENT_PULL_S1, r1 - GAP_MIN)
        if p1 >= r1 or p1 > p10:
            raise ValueError("span too small (cal=[%.2f-%.2f])" % (lo, hi))
        t = (self.sensitivity - 1) / 9.0
        self.lux_runout = r1 + t * (r10 - r1)
        self.lux_present = p1 + t * (p10 - p1)
        if set_jump:
            j1 = max(JUMP_MIN, span * JUMP_RATIO_S1)
            self.jump_threshold = j1 + t * (JUMP_MIN - j1)
        return span

    def _format_lux_minmax(self):
        return "[%.2f-%.2f]Lux" % (self.lux_min, self.lux_max)
    
    def _runout_timer_callback(self, eventtime):
        """无丝延迟定时器回调"""
        if not self.bPresent:
            self.runout_helper.note_filament_present(False)
            logging.info("[%s] callback:note_filament_present: False" % self.name)
        self.runout_timer = None
        return self.reactor.NEVER
        
    def _reset_ema(self, lux):
        self.lux_ema_fast = lux
        self.lux_ema_slow = lux
        self.lux_ema_diff = 0.0
        self.lux_ema_diff_max = 0.0
    
    def _update_ema(self, lux):
        # 首次调用时初始化
        if self.lux_ema_fast < 0:
            self._reset_ema(lux)
            return
        # 更新快EMA：v <- v + alpha_fast * (x - v)
        self.lux_ema_fast += self.alpha_fast * (lux - self.lux_ema_fast)  
        # 更新慢EMA：av <- av + alpha_slow * (x - av)
        self.lux_ema_slow += self.alpha_slow * (lux - self.lux_ema_slow)
    
    def _detect_jump(self, read_time, lux):
        diff = self.lux_ema_fast - self.lux_ema_slow
        # record max ema diff(absolute value)
        self.lux_ema_diff_max = max(self.lux_ema_diff_max, abs(diff))
        bJump = False
        if self.jump_use_abs: #仅检测跳变阈值，超过阈值，状态改变,且前一次的差值小于阈值
            if abs(diff) > self.jump_threshold and abs(self.lux_ema_diff) < self.jump_threshold:
                bJump = True
        else:
            if self.bPresent: # 有材料时，Lux向上跳变
                if diff > self.jump_threshold:
                    bJump = True
            else: # 无材料时，Lux向下跳变
                if diff < -self.jump_threshold:
                    bJump = True
        # filament state change!
        if bJump:
            logging.info("[%s] ----- Lux jump, state changed, bPresent: %s, lux: %.2f, ema fast:%.1f slow:%.1f diff: %.1f ----- " % 
                (self.name, self.bPresent, lux, self.lux_ema_fast, self.lux_ema_slow, diff) )
        self.lux_ema_diff = diff # record current ema diff        
        return bJump
    def _notify_filabuffer(self, eventtime):
        if not self.filabuffer_link:
            return
        if self.runout_helper.sensor_enabled:
            return
        self.filabuffer_link.notify(eventtime, self.bPresent)

    def _state_init(self, lux):
        self.bInited = True
        # init state by lux value, detect the nearest lux to the runout or present lux
        self.bPresent = abs(lux - self.lux_present) < abs(lux - self.lux_runout)
        self.runout_helper.note_filament_present(self.bPresent)
        self._notify_filabuffer(self.reactor.monotonic())
    def voltage_callback(self, read_time, voltage, resistance, lux):
        if not self.bInited:
            self._state_init(lux)
            return
        self.last_voltage = voltage
        self.last_resistance = resistance
        self.last_lux = lux
        self._update_ema(lux)
        # min/max from fast EMA (reject single-sample spikes)
        self.lux_min = min(self.lux_min, self.lux_ema_fast)
        self.lux_max = max(self.lux_max, self.lux_ema_fast)
        # detect jump and change filament state
        bJump = self._detect_jump(read_time, lux)

        #没有跳变，用绝对值进行校正，防止大幅度漂移
        if self.bPresent and self.lux_ema_fast > self.lux_runout:
            bJump = True
            logging.info("[%s] ---- Force set filament runout by light fast lux: %.2f" % (self.name, self.lux_ema_fast) )
        elif (not self.bPresent) and self.lux_ema_fast < self.lux_present:
            bJump = True
            logging.info("[%s] ---- Force set filament present by light fast lux: %.2f" % (self.name, self.lux_ema_fast) )

        if bJump:
            self.bPresent = not self.bPresent
            self._reset_ema(lux) #force reset ema to current lux value after state change
            logging.info("[%s] Filament light sensor jump, filament: %s, time: %.3f" % 
                (self.name, "Present" if self.bPresent else "Runout", read_time))
            self._notify_filabuffer(self.reactor.monotonic())
            # 处理状态变化
            if self.runout_timer is not None:
                self.reactor.unregister_timer(self.runout_timer)
                logging.info("[%s] unregister_timer" % self.name)
                self.runout_timer = None
            if self.bPresent:  # 有丝，立即触发
                self.runout_helper.note_filament_present(True)
                logging.info("[%s] note_filament_present: True" % self.name)
            else: # 无丝，延迟触发
                waketime = self.reactor.monotonic() + self.runout_delay
                self.runout_timer = self.reactor.register_timer(
                    self._runout_timer_callback, waketime)
                logging.info("[%s] register_timer: %.1f" % (self.name, self.runout_delay))
    def _get_pwm_value(self):
        power = self.led_power if self.led_enable else 0.0
        pwm_value = (1.0 - power) if self.led_invert else power
        return pwm_value
    
    def _setup_led(self, eventtime=None):
        # init led after klipper ready
        try:
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            pwm_value = self._get_pwm_value()
            self.led_pin.set_pwm(print_time, pwm_value)
        except Exception as e:
            logging.debug("Failed to setup filament light sensor's LED: %s" % str(e))
    
    def set_led_power(self, power, enable):
        if self.led_pin is None:
            return
        self.led_power = max(0.0, min(1.0, power))
        self.led_enable = enable
        self._setup_led()
    
    def get_status(self, eventtime):
        status = self.runout_helper.get_status(eventtime)
        status['voltage'] = round(self.last_voltage, 2)
        status['resistance'] = round(self.last_resistance, 2)  # kΩ
        status['lux'] = round(self.last_lux, 2)  # Lux
        status['lux_runout'] = round(self.lux_runout, 1)
        status['lux_present'] = round(self.lux_present, 1)
        status['lux_min'] = (None if self.lux_min >= 999999
                             else round(self.lux_min, 2))
        status['lux_max'] = (None if self.lux_max <= -999999
                             else round(self.lux_max, 2))
        status['lux_ema_fast'] = round(self.lux_ema_fast, 2)
        status['lux_ema_slow'] = round(self.lux_ema_slow, 2)
        status['lux_ema_diff'] = round(self.lux_ema_diff, 2)
        status['lux_ema_diff_max'] = round(self.lux_ema_diff_max, 2)
        status['jump_threshold'] = round(self.jump_threshold, 2)
        status['jump_use_abs'] = self.jump_use_abs
        status['sensitivity'] = self.sensitivity
        status['cal_lux_min'] = (None if self.cal_lux_min is None
                                 else round(self.cal_lux_min, 2))
        status['cal_lux_max'] = (None if self.cal_lux_max is None
                                 else round(self.cal_lux_max, 2))
        status['runout_delay'] = round(self.runout_delay, 2)
        status['report_time'] = round(self.report_time, 3)
        status['alarm_count'] = self.alarm_count
        status['alarm_trigger_count'] = self.alarm_trigger_count  # 报警触发总次数
        if self.filabuffer_link is not None:
            status['filabuffer'] = self.filabuffer_link.buffer_name
            status['filabuffer_feeder'] = self.filabuffer_link.feeder_name
            status['filabuffer_role'] = self.filabuffer_link.role
        if self.led_pin is not None:
            status['led_enable'] = self.led_enable
            status['led_power'] = self.led_power
        status['calib_loaded'] = self._calib_loaded
        status['calib_var_prefix'] = self._calib_var_prefix()
        return status
    
    cmd_QUERY_FILAMENT_LIGHT_SENSOR_help = "Query filament light sensor value. Usage: QUERY_FILAMENT_LIGHT_SENSOR SENSOR=<name>"
    def cmd_QUERY_FILAMENT_LIGHT_SENSOR(self, gcmd):
        msg = "Filament status: %s " % ( "Present" if self.bPresent else "Runout")
        msg += "Current volt=%.3fV, resis=%.3fkΩ, lux=%.2f" % (self.last_voltage, self.last_resistance, self.last_lux)
        msg += ",  Lux=%s" % self._format_lux_minmax()
        msg += ", EMA Fast=%.2f, Slow=%.2f, diff=%.2f, max Diff=%.2f" % (
            self.lux_ema_fast, self.lux_ema_slow, self.lux_ema_diff, self.lux_ema_diff_max)
        msg += ", S=%d present=%.1f runout=%.1f jump=%.1f" % (
            self.sensitivity, self.lux_present, self.lux_runout, self.jump_threshold)
        if self.led_pin is not None:
            msg += ", LED Power=%.2f, Enable=%s" % (self.led_power, self.led_enable)
        gcmd.respond_info(msg)
    
    def _show_set_help(self, gcmd):
        """显示SET命令的帮助信息"""
        gcmd.respond_info("Usage: SET_FILAMENT_LIGHT_SENSOR SENSOR=<name> [parameters...]")
        gcmd.respond_info("Parameters:")
        gcmd.respond_info("  LED_POWER=<0.0-1.0>  - LED power (0.0=off, 1.0=max)" )
        gcmd.respond_info("  LED_ENABLE=<0|1>    - LED enable (0=off, 1=on)" )
        gcmd.respond_info("  SENSOR_ENABLE=<0|1>  - Sensor enable (0=off, 1=on)" )
        gcmd.respond_info("  LUX_RUNOUT=<lux>     - Filament runout threshold (Lux)" )
        gcmd.respond_info("  LUX_PRESENT=<lux>    - Filament present threshold (Lux)" )
        gcmd.respond_info("  ALARM_COUNT=<count>  - Alarm count ")
        gcmd.respond_info("  REPORT_TIME=<sec>    - Report time in seconds " )
        gcmd.respond_info("  EMA_FAST_TIME=<sec>  - Fast EMA time constant " )
        gcmd.respond_info("  EMA_SLOW_TIME=<sec>  - Slow EMA time constant " )
        gcmd.respond_info("  JUMP_THRESHOLD=<lux> - jump detection threshold " )
        gcmd.respond_info("  JUMP_USE_ABS=<0|1>  - jump detection mode (0=single-sided, 1=both-sided)" )
        gcmd.respond_info("  SENSITIVITY=<1-10>  - 1=dark 10=light; linear raise present+runout, lower jump" )
        gcmd.respond_info("  RUNOUT_DELAY=<sec>  - Runout delay time in seconds (0=immediate, current: %.1f)" % self.runout_delay)
        gcmd.respond_info("  RESET_LUX=x     - Reset lux min/max statistics " )
        gcmd.respond_info("  RESET_EMA=x     - Reset EMA values and clear jump statistics " )
        gcmd.respond_info("  RESET_ALL=x      - Reset all statistics (min/max, EMA, jump)" )
        gcmd.respond_info("  SAVE=<0|1>      - Persist lux/jump thresholds to save_variables" )
        gcmd.respond_info("Example: SET_FILAMENT_LIGHT_SENSOR SENSOR=%s LED_POWER=0.8" % self.name)
        
    def _update_ema_weights(self):
        self.alpha_fast = self.report_time / self.ema_fast_time
        self.alpha_slow = self.report_time / self.ema_slow_time
    
    cmd_SET_FILAMENT_LIGHT_SENSOR_help = "Set filament light sensor parameters. Usage: SET_FILAMENT_LIGHT_SENSOR SENSOR=<name> [parameters...]"
    def cmd_SET_FILAMENT_LIGHT_SENSOR(self, gcmd):
        # 所有支持的参数列表
        all_params = ['LED_POWER', 'LED_ENABLE', 'SENSOR_ENABLE', 'LUX_RUNOUT', 'LUX_PRESENT',
                     'ALARM_COUNT', 'REPORT_TIME', 'EMA_FAST_TIME', 'EMA_SLOW_TIME', 'JUMP_THRESHOLD', 'JUMP_USE_ABS',
                     'SENSITIVITY', 'RUNOUT_DELAY', 'RESET_LUX', 'RESET_EMA', 'RESET_ALL', 'SAVE']
        # 检查是否有任何参数
        if not any(gcmd.get(param, None) is not None for param in all_params):
            self._show_set_help(gcmd)
            return
        thresh_changed = False
        # 传感器使能
        if gcmd.get('SENSOR_ENABLE', None) is not None:
            enable = gcmd.get_int('SENSOR_ENABLE', 1)
            self.runout_helper.sensor_enabled = enable
            gcmd.respond_info("Sensor enable set to %s" % enable)
        # LED设置（需要同时处理LED_POWER和LED_ENABLE）
        if self.led_pin is not None and (gcmd.get('LED_POWER', None) is not None or gcmd.get('LED_ENABLE', None) is not None):
            power = gcmd.get_float('LED_POWER', self.led_power)
            led_enable = gcmd.get_int('LED_ENABLE', 1 if self.led_enable else 0) != 0
            self.set_led_power(power, led_enable)
            gcmd.respond_info("LED: power=%.2f, enable=%s" % (self.led_power, self.led_enable))
        # 简单参数设置（直接赋值）
        if gcmd.get('LUX_RUNOUT', None) is not None:
            self.lux_runout = gcmd.get_float('LUX_RUNOUT', above=0.)
            thresh_changed = True
            gcmd.respond_info("LUX_RUNOUT set to %.2fLux" % self.lux_runout)
        if gcmd.get('LUX_PRESENT', None) is not None:
            self.lux_present = gcmd.get_float('LUX_PRESENT', above=0.)
            thresh_changed = True
            gcmd.respond_info("LUX_PRESENT set to %.2fLux" % self.lux_present)
        if gcmd.get('ALARM_COUNT', None) is not None:
            self.alarm_count = gcmd.get_int('ALARM_COUNT', minval=1)
            gcmd.respond_info("Alarm count set to %d" % self.alarm_count)
        if gcmd.get('JUMP_THRESHOLD', None) is not None:
            self.jump_threshold = gcmd.get_float('JUMP_THRESHOLD', above=0.)
            thresh_changed = True
            gcmd.respond_info("JUMP_THRESHOLD set to %.2fLux" % self.jump_threshold)
        if gcmd.get('JUMP_USE_ABS', None) is not None:
            self.jump_use_abs = gcmd.get_int('JUMP_USE_ABS', 0) != 0
            mode_str = "both-sided for transparent filament" if self.jump_use_abs else "single-sided"
            gcmd.respond_info("JUMP_USE_ABS set to %s" % mode_str)
        if gcmd.get('SENSITIVITY', None) is not None:
            self.apply_buffer_sensitivity(
                gcmd.get_int('SENSITIVITY', minval=1, maxval=10),
                save=bool(gcmd.get_int('SAVE', 0)),
                respond=gcmd.respond_info)
            thresh_changed = True
            saved_s = True
        else:
            saved_s = False
        if gcmd.get('RUNOUT_DELAY', None) is not None:
            self.runout_delay = gcmd.get_float('RUNOUT_DELAY', minval=0.)
            gcmd.respond_info("Runout delay set to %.1f seconds (0=immediate)" % self.runout_delay)
        # REPORT_TIME（需要更新ADC回调和EMA权重）
        if gcmd.get('REPORT_TIME', None) is not None:
            self.report_time = gcmd.get_float('REPORT_TIME', above=0.)
            self._update_ema_weights()
            self.adc_voltage.setup_adc_callback(self.report_time)
            gcmd.respond_info("Report time set to %.3f seconds " % self.report_time)
        # EMA时间常数（需要重新计算权重）
        if gcmd.get('EMA_FAST_TIME', None) is not None:
            self.ema_fast_time = gcmd.get_float('EMA_FAST_TIME', above=0.)
            self._update_ema_weights()
            gcmd.respond_info("EMA fast time set to %.2f seconds " % self.ema_fast_time)
        if gcmd.get('EMA_SLOW_TIME', None) is not None:
            self.ema_slow_time = gcmd.get_float('EMA_SLOW_TIME', above=0.)
            self._update_ema_weights()
            gcmd.respond_info("EMA slow time set to %.2f seconds " % self.ema_slow_time)
        # 重置Lux,ema统计信息
        if gcmd.get('RESET_LUX', None) is not None:
            self.lux_min = 999999
            self.lux_max = -999999
            gcmd.respond_info("Lux statistics reset (min/max cleared)")
        if gcmd.get('RESET_EMA', None) is not None:
            self._reset_ema(self.last_lux)
            gcmd.respond_info("Lux EMA reset")
        if gcmd.get('RESET_ALL', None) is not None:
            self.lux_min = 999999
            self.lux_max = -999999
            self._reset_ema(self.last_lux)
            gcmd.respond_info("All statistics reset: min/max cleared, EMA reset")
        if gcmd.get_int('SAVE', 0) and not saved_s:
            if not thresh_changed:
                gcmd.respond_info("SAVE=1 with no lux/jump change; saving current thresholds")
            self._save_calib_to_variables(respond=gcmd.respond_info)

    cmd_AUTO_SET_FILAMENT_LIGHT_THRESHOLD_help = (
        "Lock cal min/max from sample, apply S to buffer. "
        "AUTO_SET_FILAMENT_LIGHT_THRESHOLD SENSOR=<name> [SENSITIVITY=1] [SAVE=1]")
    def cmd_AUTO_SET_FILAMENT_LIGHT_THRESHOLD(self, gcmd):
        if self.lux_min >= 999999 or self.lux_max <= -999999:
            raise gcmd.error(
                "No lux min/max yet for %s; RESET_ALL then sample empty and dark filament"
                % (self.name,))
        if self.lux_max <= self.lux_min:
            raise gcmd.error("Invalid lux range for %s" % (self.name,))
        self.cal_lux_min, self.cal_lux_max = self.lux_min, self.lux_max
        s = gcmd.get_int('SENSITIVITY', 1, minval=1, maxval=10)
        self.sensitivity = s
        try:
            self._apply_sensitivity_thresholds()
        except ValueError as e:
            raise gcmd.error(str(e))
        self.apply_buffer_sensitivity(
            s, save=bool(gcmd.get_int('SAVE', 1)), respond=gcmd.respond_info)
        self._state_init(self.last_lux)
        gcmd.respond_info(
            "Auto set %s cal=[%.2f-%.2f] present=%.2f runout=%.2f jump=%.2f S=%d"
            % (self.name, self.cal_lux_min, self.cal_lux_max,
               self.lux_present, self.lux_runout, self.jump_threshold, s))

    cmd_SAVE_FILAMENT_LIGHT_CALIB_help = (
        "Save lux/jump thresholds to save_variables. "
        "Usage: SAVE_FILAMENT_LIGHT_CALIB SENSOR=<name>")
    def cmd_SAVE_FILAMENT_LIGHT_CALIB(self, gcmd):
        ok = self._save_calib_to_variables(respond=gcmd.respond_info)
        if not ok:
            raise gcmd.error("Failed to save calib for %s" % (self.name,))

    cmd_LOAD_FILAMENT_LIGHT_CALIB_help = (
        "Load lux/jump thresholds from save_variables. "
        "Usage: LOAD_FILAMENT_LIGHT_CALIB SENSOR=<name>")
    def cmd_LOAD_FILAMENT_LIGHT_CALIB(self, gcmd):
        ok = self._load_calib_from_variables(respond=gcmd.respond_info)
        if not ok:
            raise gcmd.error("No saved calib (or save_variables missing) for %s" % (self.name,))

def load_config_prefix(config):
    return FilamentLightSensor(config)
