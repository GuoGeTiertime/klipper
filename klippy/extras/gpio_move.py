# GPIO controlled movement module for Klipper
import logging
import mcu, chelper

        # 轴名称到索引的映射
AXIS_NAME_TO_INDEX = {'x': 0, 'y': 1, 'z': 2}

class GPIOMove:
    def __init__(self, config):
        # 基础初始化
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split(' ')[-1]

        #配置按钮
        self.last_check_state = 0
        self.last_switch_state = 0
        self.position = 0
        self._init_move = False
        self.moving_lock = self.reactor.mutex()  # 状态锁

        self.cur_len = 0

        # 配置输入端口
        self.switch_pin = config.get('switch_pin')
        self.check_pin = config.get('check_pin')
        buttons = self.printer.load_object(config, "buttons")
        buttons.register_buttons([self.switch_pin], self.switch_button_callback)
        buttons.register_buttons([self.check_pin], self.check_button_callback)

        # 获取配置参数
        axis_name = config.get('axis', 'x').lower()  # 获取轴名称并转为小写
        if axis_name not in AXIS_NAME_TO_INDEX:
            raise config.error("Unsupported axis '%s'" % (axis_name,))
        self.axis = AXIS_NAME_TO_INDEX[axis_name]
        self.is_enabled = config.getboolean('enable', False)
        # 添加速度和距离相关参数
        self.trigger_distance = config.getfloat('trigger_distance', 20., minval=0.)  # 延时触发距离，单位mm
        self.move_speed = config.getfloat('move_speed', 10., above=0.)  # 默认移动速度 10mm/s
        self.max_move_distance = config.getfloat('max_move_distance', 50., above=0.)  # 最大移动距离，默认1000mm
        self.init_move_speed = config.getfloat('init_move_speed', 10., above=0.)  # 初始移动速度，默认10mm/s
        self.init_move_distance = config.getfloat('init_move_distance', 500., above=0.)  # 初始最大移动距离，默认1000mm

        self._trigger_completion = None

        self.isloginfo = 0  # 0: no log, 1:gcode response, 2: write log file, 3: response and write log file 

        self._delayed_trigger_off_timer = self.reactor.register_timer(self._delayed_trigger_off)

        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.trigger_on_gcode = gcode_macro.load_template(config, 'trigger_on_gcode')
        self.slip_gcode = gcode_macro.load_template(config, 'slip_gcode')
        self.insert_gcode = gcode_macro.load_template(config, 'insert_gcode')
        self.runout_gcode = gcode_macro.load_template(config, 'runout_gcode')

        # 注册G代码命令
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("GPIO_MOVE_SET", "IN", self.name,
                                        self.cmd_GPIO_MOVE_SET,
                                        desc=self.cmd_GPIO_MOVE_SET_help)
        self.gcode.register_mux_command('GPIO_MOVE_START', 'IN', self.name,
                                      self.cmd_GPIO_MOVE_START,
                                      desc=self.cmd_GPIO_MOVE_START_help)
        
        # 注册系统ready处理器
        self.printer.register_event_handler('klippy:ready', self._handle_ready)

    def _loginfo(self, msg, logflag=None):
        if logflag is None:
            logflag = self.isloginfo
        if logflag == 1:
            self.gcode.respond_info(msg, False) # only respond to gcode.
        elif logflag == 2:
           logging.info(msg) # only write log file.
        elif logflag == 3:
            self.gcode.respond_info(msg, True) # respond to gcode and write log file.

    def _handle_ready(self):
        # 获取toolhead对象
        self.toolhead = self.printer.lookup_object('toolhead')

    def _exec_gcode(self, script_template, now=False):
        if script_template is None:
            return
        try:
            if now:
                script_template.run_gcode_from_command()
            else:
                self.gcode.run_script(script_template.render())
        except Exception as e:
            self._loginfo("GPIOMove %s script running error: %s" % (self.name, str(e)), 3)
    # switch pin on, trigger move
    def _trigger_on_handler(self, eventtime):
        self._exec_gcode(self.trigger_on_gcode)

    # switch pin on, move to endstop
    def _slip_handler(self, eventtime):
        self._exec_gcode(self.slip_gcode)
    
    # check pin on, fila insert
    def _insert_handler(self, eventtime):
        self._exec_gcode(self.insert_gcode)
    
    # check pin off, fila runout, send runout signal by gcode
    def _runout_handler(self, eventtime):
        self._exec_gcode(self.runout_gcode)
    
    def check_button_callback(self, eventtime, state):
        self._loginfo("update gpiomove %s check state:%d" % (self.name, state))
        if self.last_check_state == state:  # 状态没有变化, 直接返回
            return
        self.last_check_state = state

        # check pin off, stop move by set trigger_completion to 1 if motor is moving(trigger_completion is not None )
        if not state: # 低电平, 停止运动
            if self._trigger_completion:
                self._trigger_completion.complete(1)
                self._loginfo("GPIOMove %s force set trigger_completion to 1 by checkpin turn off" % (self.name,))
        # 这里可以添加 checkpin 触发后的具体逻辑
        if state:
            self.reactor.register_callback(self._insert_handler)
        else:
            self.reactor.register_callback(self._runout_handler)


    def switch_button_callback(self, eventtime, state):
        self._loginfo("update gpiomove %s switch state:%d" % (self.name, state))
        if self.last_switch_state == state: # 状态没有变化, 直接返回
            return
        self.last_switch_state = state

        if state:  # 高电平, 启动电机运动
            if not self.is_enabled or self.moving_lock.test():  # 未使能 或 锁已被占用(正在移动
                self._loginfo("%s is disable or already moving, ignore start signal" % (self.name,))
                return
            self._loginfo("%s start move by IO turn on" % (self.name,))
            self.reactor.register_callback(self._trigger_on_handler)
        else:  # 低电平.发送endstop信号, 有延时
            if self._trigger_completion:
                waketime = self.reactor.NOW if self.trigger_distance == 0. else self.reactor.monotonic() + self.trigger_distance / self.move_speed
                self.reactor.update_timer(self._delayed_trigger_off_timer, waketime)
                self._loginfo("Schedule delayed trigger at %.3f seconds (distance: %.3fmm, speed: %.3fmm/s)" % 
                        (waketime, self.trigger_distance, self.move_speed))

    def _delayed_trigger_off(self, eventtime):
        if self._trigger_completion and not self.last_switch_state:
            self._trigger_completion.complete(1)
            self._loginfo("GPIOMove %s delayed triggered @ %.4f" % (self.name, eventtime))
        return self.reactor.NEVER

    def _start_move_handler(self, eventtime):
        if self.moving_lock.test(): # 返回True表示锁已被占用，正在移动
            return
        try:
            with self.moving_lock:
                self.__drip_move()
        except Exception as e:
            self._loginfo("Error in drip move: %s" % str(e), 3)
            self._trigger_completion = None  # 确保清理
    
    def __drip_move(self):
        # 如果switch或check状态为0，则不执行运动
        if not self.last_switch_state or not self.last_check_state:
            self._loginfo("GPIOMove %s can't start move when IO turn off, switch:%d, check:%d" % (self.name, self.last_switch_state, self.last_check_state))
            return

        self._trigger_completion = self.reactor.completion()

        # 获取记录运动开始时的位置,设定终止位置和运动速度
        self.toolhead.flush_step_generation()
        axis = self.axis
        pos = self.toolhead.get_position()
        prevx = pos[axis]
        dis = self.init_move_distance if self._init_move else self.max_move_distance
        speed = self.init_move_speed if self._init_move else self.move_speed
        pos[axis] += dis
        self._init_move = False # 初始移动完成,只有一次.
        # self._loginfo("GPIOMove __drip_move() @ %.4f, prevx:%.4f, targetx:%.4f, diff:%.4f" % 
        #               (self.reactor.monotonic(), prevx, pos[axis], dis))
        # 获取运动学对象
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        start_pos = {s.get_name():s.get_mcu_position()
                     for s in kin.get_steppers()}

        self.toolhead.dwell(0.01)
        # Issue move
        error = None
        try:
            self.toolhead.drip_move(pos, speed, self._trigger_completion)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)
            self._loginfo(error, 3)

        # 获取stop实际位置
        self.toolhead.flush_step_generation()
        stop_pos = {s.get_name():s.get_mcu_position()
                    for s in kin.get_steppers()}
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += (stop_pos[sname] - start_pos[sname]) * stepper.get_step_dist()

        curpos = list(kin.calc_position(kin_spos))
        thpos = self.toolhead.get_position()
        curpos = curpos[:3] + thpos[3:]
        self.cur_len = curpos[axis] - prevx
        # 更新工具头位置
        self._loginfo("GPIOMove %s Move completed. Stepper positions: %s" % (self.name, curpos,))
        self.toolhead.set_position(curpos)
        self.position = curpos[axis]

        # 没有触发trigger, 异常情况
        if not self._trigger_completion.test(): #trigger没有触发.
            self.reactor.register_callback(self._slip_handler)
            self._loginfo("GPIOMove %s Not finish after max len" % (self.name,), 3)

        self._trigger_completion = None


    cmd_GPIO_MOVE_SET_help = "GPIO_MOVE_SET IN=xxxxx EN=1 SPEED=10 DISTANCE=1000 TRIGGER_DISTANCE=5"
    def cmd_GPIO_MOVE_SET(self, gcmd):
        # 允许通过G代码动态修改速度、最大距离和触发延时距离
        self.move_speed = gcmd.get_float('SPEED', self.move_speed)
        self.max_move_distance = gcmd.get_float('DISTANCE', self.max_move_distance)
        self.trigger_distance = gcmd.get_float('TRIGGER_DISTANCE', self.trigger_distance)
        self.init_move_speed = gcmd.get_float('INIT_SPEED', self.init_move_speed)   
        self.init_move_distance = gcmd.get_float('INIT_DISTANCE', self.init_move_distance)
        self._loginfo("GPIOMove %s enable: %d, speed: %.3f, max_distance: %.3f, trigger_distance: %.3f, init_speed: %.3f, init_distance: %.3f @ %.4f" % 
                      (self.name, self.is_enabled, self.move_speed, self.max_move_distance, 
                       self.trigger_distance, self.init_move_speed, self.init_move_distance, self.reactor.monotonic()))

    cmd_GPIO_MOVE_START_help = "Force start/stop[EN=0] GPIO_MOVE motion"
    def cmd_GPIO_MOVE_START(self, gcmd):
        self.is_enabled = gcmd.get_int('EN', self.is_enabled)
        self._init_move = gcmd.get_int('INIT', 0)
        self.cur_len = 0
        if self.is_enabled:
            self._loginfo("GPIOMove %s enabled, start move by gcode" % (self.name,))
            # self.reactor.register_callback(self._start_move_handler)
            self._start_move_handler(self.reactor.monotonic())
        else:
            if self._trigger_completion: # 如果正在移动，强制trigger完成, 停止移动
                self._trigger_completion.complete(1)
                self._loginfo("GPIOMove %s is force stop by set trigger_completion to 1" % (self.name,))


    def get_status(self, eventtime=None):
        return {
            'enabled': bool(self.is_enabled),
            'moving': bool(self.moving_lock.test()),
            'last_switch_state': bool(self.last_switch_state),
            'last_check_state': bool(self.last_check_state),
            'position': self.position,
            'axis': self.axis,
            'speed': self.move_speed,
            'max_distance': self.max_move_distance,
            'trigger_distance': self.trigger_distance,
            'init_speed': self.init_move_speed,
            'init_distance': self.init_move_distance,
            'cur_len': self.cur_len
        }

def load_config(config):
    return GPIOMove(config)

def load_config_prefix(config):
    return GPIOMove(config)
