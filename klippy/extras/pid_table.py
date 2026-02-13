# PID 参数按温度查表插值，供多个 heater 复用
#
# 配置段 [pid_table] 定义温度->Kp/Ki/Kd 表，heater 通过 pid_table: pid_table 引用。
# This file may be distributed under the terms of the GNU GPLv3 license.

from .adc_temperature import LinearInterpolate


class PidTable:
    """从 config 读取多组 (温度, Kp, Ki, Kd)，对给定温度做线性插值得到 PID 参数。可被多个 heater 复用。"""
    def __init__(self, config):
        # 配置段为 [pid_table]，键为 temperature1, Kp1, Ki1, Kd1, temperature2, ...
        samples_kp = []
        samples_ki = []
        samples_kd = []
        for i in range(1, 1000):
            t = config.getfloat("temperature%d" % (i,), None)
            if t is None:
                break
            kp = config.getfloat("Kp%d" % (i,), minval=0.)
            ki = config.getfloat("Ki%d" % (i,), minval=0.)
            kd = config.getfloat("Kd%d" % (i,), minval=0.)
            samples_kp.append((t, kp))
            samples_ki.append((t, ki))
            samples_kd.append((t, kd))
        if len(samples_kp) < 2:
            raise config.error("pid_table needs at least two points "
                               "(temperature1/Kp1/Ki1/Kd1, temperature2/...)")
        try:
            self._interp_kp = LinearInterpolate(samples_kp)
            self._interp_ki = LinearInterpolate(samples_ki)
            self._interp_kd = LinearInterpolate(samples_kd)
        except ValueError as e:
            raise config.error("pid_table %s: %s" % (config.get_name(), str(e)))

    def get_pid(self, temp):
        """按温度插值得到 (Kp, Ki, Kd)，与 config 中 pid_Kp/pid_Ki/pid_Kd 同尺度。"""
        return (self._interp_kp.interpolate(temp),
                self._interp_ki.interpolate(temp),
                self._interp_kd.interpolate(temp))


def load_config(config):
    return PidTable(config)

# 动态 PID：引用共享的 pid_table 段（多个 heater 可复用同一表）
def load_config_prefix(config):
    return PidTable(config)
