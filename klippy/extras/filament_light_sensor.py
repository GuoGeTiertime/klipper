# Filament Light Sensor Module (Voltage-based)
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import filament_switch_sensor
from . import adc_light

DEFAULT_REPORT_TIME = 0.1  # 默认10Hz

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
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.runout_helper.sensor_enabled = config.getboolean('sensor_enable', True)

        self.bInited = False # 是否已初始化
        self.bPresent = False # 当前是否存在丝材        
        # 丝材检测参数（使用光强阈值）
        self.lux_runout = config.getfloat('lux_runout', 30.0)  # 无材料光照度
        self.lux_present = config.getfloat('lux_present', 10.0)  # 有材料光照度
        self.alarm_count = config.getint('alarm_count', 1, minval=1)  # 报警次数
        # 内部状态
        self.last_voltage = 0.0
        self.last_resistance = 0.0  # kΩ
        self.last_lux = 0.0  # Lux
        self.alarm_trigger_count = 0  # 报警触发总次数统计, 可以删除
        
        # 光强统计（用于监测过程中的最大值和最小值）
        self.lux_min = 999999
        self.lux_max = -999999
        
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
        
    def _format_lux_minmax(self):
        return "[%.2f-%.2f]Lux" % (self.lux_min, self.lux_max)
        
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
            logging.info(" ----- Lux jump, state changed, bPresent: %s, lux: %.2f, ema fast:%.1f slow:%.1f diff: %.1f ----- " % 
                (self.bPresent, lux, self.lux_ema_fast, self.lux_ema_slow, diff) )
        self.lux_ema_diff = diff # record current ema diff        
        return bJump
    def _state_init(self, lux):
        self.bInited = True
        # init state by lux value, detect the nearest lux to the runout or present lux
        self.bPresent = abs(lux - self.lux_present) < abs(lux - self.lux_runout)
        self.runout_helper.note_filament_present(self.bPresent)
    def voltage_callback(self, read_time, voltage, resistance, lux):
        if not self.bInited:
            self._state_init(lux)
            return
        self.last_voltage = voltage
        self.last_resistance = resistance
        self.last_lux = lux        
        # 更新光强最大值和最小值
        self.lux_min = min(self.lux_min, lux)
        self.lux_max = max(self.lux_max, lux)
        self._update_ema(lux)
        # detect jump and change filament state
        bJump = self._detect_jump(read_time, lux)

        #没有跳变，用绝对值进行校正，防止大幅度漂移
        if self.bPresent and self.lux_ema_fast > self.lux_runout:
            bJump = True
            logging.info(" ---- Force set filament runout by light fast lux: %.2f" % self.lux_ema_fast )
        elif (not self.bPresent) and self.lux_ema_fast < self.lux_present:
            bJump = True
            logging.info(" ---- Force set filament present by light fast lux: %.2f" % self.lux_ema_fast )

        if bJump:
            self.bPresent = not self.bPresent
            self.runout_helper.note_filament_present(self.bPresent)
            self._reset_ema(lux) #force reset ema to current lux value after state change
            logging.info( "Filament light sensor jump, filament: %s, time: %.3f ", "Present" if self.bPresent else "Runout", read_time)

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
        status['alarm_count'] = self.alarm_count
        status['alarm_trigger_count'] = self.alarm_trigger_count  # 报警触发总次数
        if self.led_pin is not None:
            status['led_enable'] = self.led_enable
            status['led_power'] = self.led_power
        return status
    
    cmd_QUERY_FILAMENT_LIGHT_SENSOR_help = "Query filament light sensor value. Usage: QUERY_FILAMENT_LIGHT_SENSOR SENSOR=<name>"
    def cmd_QUERY_FILAMENT_LIGHT_SENSOR(self, gcmd):
        msg = "Filament status: %s " % ( "Present" if self.bPresent else "Runout")
        msg += "Current volt=%.3fV, resis=%.3fkΩ, lux=%.2f" % (self.last_voltage, self.last_resistance, self.last_lux)
        msg += ",  Lux=%s" % self._format_lux_minmax()
        msg += ", EMA Fast=%.2f, Slow=%.2f, diff=%.2f, max Diff=%.2f" % (
            self.lux_ema_fast, self.lux_ema_slow, self.lux_ema_diff, self.lux_ema_diff_max)
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
        gcmd.respond_info("  RESET_LUX=x     - Reset lux min/max statistics " )
        gcmd.respond_info("  RESET_EMA=x     - Reset EMA values and clear jump statistics " )
        gcmd.respond_info("  RESET_ALL=x      - Reset all statistics (min/max, EMA, jump)" )
        gcmd.respond_info("Example: SET_FILAMENT_LIGHT_SENSOR SENSOR=%s LED_POWER=0.8" % self.name)
        
    def _update_ema_weights(self):
        self.alpha_fast = self.report_time / self.ema_fast_time
        self.alpha_slow = self.report_time / self.ema_slow_time
    
    cmd_SET_FILAMENT_LIGHT_SENSOR_help = "Set filament light sensor parameters. Usage: SET_FILAMENT_LIGHT_SENSOR SENSOR=<name> [parameters...]"
    def cmd_SET_FILAMENT_LIGHT_SENSOR(self, gcmd):
        # 所有支持的参数列表
        all_params = ['LED_POWER', 'LED_ENABLE', 'SENSOR_ENABLE', 'LUX_RUNOUT', 'LUX_PRESENT', 
                     'ALARM_COUNT', 'REPORT_TIME', 'EMA_FAST_TIME', 'EMA_SLOW_TIME', 'JUMP_THRESHOLD', 'JUMP_USE_ABS',
                     'RESET_LUX', 'RESET_EMA', 'RESET_ALL']
        # 检查是否有任何参数
        if not any(gcmd.get(param, None) is not None for param in all_params):
            self._show_set_help(gcmd)
            return
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
            gcmd.respond_info("LUX_RUNOUT set to %.2fLux" % self.lux_runout)        
        if gcmd.get('LUX_PRESENT', None) is not None:
            self.lux_present = gcmd.get_float('LUX_PRESENT', above=0.)
            gcmd.respond_info("LUX_PRESENT set to %.2fLux" % self.lux_present)
        if gcmd.get('ALARM_COUNT', None) is not None:
            self.alarm_count = gcmd.get_int('ALARM_COUNT', minval=1)
            gcmd.respond_info("Alarm count set to %d" % self.alarm_count)
        if gcmd.get('JUMP_THRESHOLD', None) is not None:
            self.jump_threshold = gcmd.get_float('JUMP_THRESHOLD', above=0.)
            gcmd.respond_info("JUMP_THRESHOLD set to %.2fLux" % self.jump_threshold)
        if gcmd.get('JUMP_USE_ABS', None) is not None:
            self.jump_use_abs = gcmd.get_int('JUMP_USE_ABS', 0) != 0
            mode_str = "both-sided for transparent filament" if self.jump_use_abs else "single-sided"
            gcmd.respond_info("JUMP_USE_ABS set to %s" % mode_str)
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

    cmd_AUTO_SET_FILAMENT_LIGHT_THRESHOLD_help = "Auto set filament light runout present lux by max and min lux. Usage: AUTO_SET_FILAMENT_LIGHT_THRESHOLD SENSOR=<name> PERCENT=<0-40>"
    def cmd_AUTO_SET_FILAMENT_LIGHT_THRESHOLD(self, gcmd):
        percent = gcmd.get_float('PERCENT', 20., minval=0., maxval=40.)
        delta = (self.lux_max - self.lux_min) * percent / 100.
        self.lux_runout = self.lux_max - delta
        self.lux_present = self.lux_min + delta
        gcmd.respond_info("Filament light threshold set to %.2fLux (runout) and %.2fLux (present)" % (self.lux_runout, self.lux_present))
        gcmd.respond_info("Percent: %.2f%%, delta: %.2fLux" % (percent, delta))
        gcmd.respond_info("Max: %.2fLux, Min: %.2fLux" % (self.lux_max, self.lux_min))
        self._state_init(self.last_lux)
        gcmd.respond_info("Reinit filament state to: %s" % ("Present" if self.bPresent else "Runout"))

def load_config_prefix(config):
    return FilamentLightSensor(config)
