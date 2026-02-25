# Voltage to Resistance conversion using linear interpolation
#
# Copyright (C) 2025
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import adc_temperature

######################################################################
# Interface between MCU adc and voltage/resistance callbacks
######################################################################

# Interface between ADC and voltage/resistance callbacks
class PrinterADCtoVoltage:
    def __init__(self, config, voltage_resistance_table, voltage_lux_table=None):
        self.voltage_resistance_table = voltage_resistance_table
        self.voltage_lux_table = voltage_lux_table
        ppins = config.get_printer().lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', config.get('sensor_pin'))
        
        # 设置ADC采样参数
        sample_time = config.getfloat('sample_time', 0.1, above=0.)
        sample_count = config.getint('sample_count', 8, minval=1)
        range_check_count = config.getint('range_check_count', 0, minval=0)
        self.mcu_adc.setup_minmax(sample_time, sample_count, minval=0., maxval=1., 
                                  range_check_count=range_check_count)
        
        # ADC电压参数
        self.adc_voltage = config.getfloat('adc_voltage', 3.3, above=0.)
        self.voltage_offset = config.getfloat('voltage_offset', 0.0)
        
        # 电压回调函数（由外部设置）
        self.voltage_callback = None
        
        # 注册到query_adc
        query_adc = config.get_printer().load_object(config, 'query_adc')
        query_adc.register_adc(config.get_name(), self.mcu_adc)
    
    def setup_callback(self, voltage_callback):
        """设置电压值回调函数"""
        self.voltage_callback = voltage_callback
    
    def setup_adc_callback(self, report_time):
        """设置ADC回调"""
        self.mcu_adc.setup_adc_callback(report_time, self.adc_callback)
    
    def adc_callback(self, read_time, read_value):
        """ADC数据回调：转换为电压、电阻和照度值"""
        # 计算电压值
        voltage = read_value * self.adc_voltage + self.voltage_offset
        
        # 从查找表获取电阻值（kΩ）
        resistance = 0.0
        if self.voltage_resistance_table:
            resistance = self.voltage_resistance_table.interpolate(voltage)
        
        # 从查找表获取照度值（Lux）
        lux = 0.0
        if self.voltage_lux_table:
            lux = self.voltage_lux_table.interpolate(voltage)
        
        # 调用电压回调函数
        if self.voltage_callback:
            self.voltage_callback(read_time, voltage, resistance, lux)


######################################################################
# Custom sensor type (for sensor_type support)
######################################################################

class CustomLinearVoltageResistance:
    """自定义电压到电阻传感器类型（用于sensor_type支持）"""
    def __init__(self, config):
        self.name = " ".join(config.get_name().split()[1:])
        samples = []
        lux_samples = []
        for i in range(1, 1000):
            voltage = config.getfloat("voltage%d" % (i,), None)
            if voltage is None:
                break
            resistance = config.getfloat("resistance%d" % (i,))
            samples.append((voltage, resistance))
            lux = config.getfloat("lux%d" % (i,))
            lux_samples.append((voltage, lux))
        if len(samples) < 2:
            raise config.error("adc_light %s: need at least 2 calibration points" % (self.name,))
        # 创建 LinearInterpolate 对象存储电压到电阻的映射
        try:
            self.li = adc_temperature.LinearInterpolate(samples)
        except ValueError as e:
            raise config.error("adc_light %s: %s" % (self.name, str(e)))
        # 创建电压到lux的查找表（如果提供了lux值）
        self.lux_table = None
        if len(lux_samples) >= 2:
            try:
                self.lux_table = adc_temperature.LinearInterpolate(lux_samples)
            except ValueError as e:
                raise config.error("adc_light %s: lux table error: %s" % (self.name, str(e)))
    
    def create(self, config):
        """创建传感器对象"""
        return PrinterADCtoVoltage(config, self.li, self.lux_table)


######################################################################
# Configuration loading
######################################################################

def load_config_prefix(config):
    """处理 [adc_light xxx] 配置段（只注册，不创建对象）"""
    custom_sensor = CustomLinearVoltageResistance(config)
    if not hasattr(load_config_prefix, 'sensor_factories'):
        load_config_prefix.sensor_factories = {}
    load_config_prefix.sensor_factories[custom_sensor.name] = custom_sensor
