# Support generic temperature sensors
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

KELVIN_TO_CELSIUS = -273.15

class PrinterSensorGeneric:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.min_temp = config.getfloat('min_temp', KELVIN_TO_CELSIUS,
                                        minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', 99999999.9,
                                        above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)
        self.last_temp = 0.
        self.measured_min = 99999999.
        self.measured_max = 0.
    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        if temp:
            self.measured_min = min(self.measured_min, temp)
            self.measured_max = max(self.measured_max, temp)
    def get_temp(self, eventtime):
        return self.last_temp, 0.
    def stats(self, eventtime):
        return False, '%s: temp=%.1f' % (self.name, self.last_temp)
    def get_status(self, eventtime):
        status = {
            'temperature': round(self.last_temp, 2),
            'measured_min_temp': round(self.measured_min, 2),
            'measured_max_temp': round(self.measured_max, 2)
        }
        # Pass-through extra environmental fields from underlying sensor
        # (for example humidity/pressure/gas on AHT/BME/HTU/SHT sensors).
        try:
            sensor_status = self.sensor.get_status(eventtime)
            if isinstance(sensor_status, dict):
                for key in ('humidity', 'pressure', 'gas'):
                    if key in sensor_status:
                        status[key] = sensor_status[key]
        except Exception:
            # Keep generic temperature_sensor status robust even if a specific
            # sensor implementation raises an exception in get_status().
            pass
        return status

def load_config_prefix(config):
    return PrinterSensorGeneric(config)
