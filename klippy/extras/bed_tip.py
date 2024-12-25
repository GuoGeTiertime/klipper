 # 床调平提示助手
class BedTip:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        # 配置螺距(mm)
        self.screw_pitch = config.getfloat('screw_pitch', 0.5)
        # 状态变量
        self.tips = ""
        self.corners = {}
        self.adjustments = {}
        
        # 注册G-code命令
        self.gcode.register_command(
            'BED_CAL_TIP', self.cmd_BED_CAL_TIP,
            desc=self.cmd_BED_CAL_TIP_help)
    
    def get_status(self, eventtime):
        return {
            'tips': self.tips,
            'corners': self.corners,
            'adjustments': self.adjustments
        }
        
    def get_mesh_corners(self):
        bed_mesh = self.printer.lookup_object('bed_mesh')
        if bed_mesh.z_mesh is None:
            raise self.gcode.error("No bed mesh has been loaded")
            
        matrix = bed_mesh.z_mesh.get_mesh_matrix()
        if not matrix or not matrix[0]:
            raise self.gcode.error("Bed mesh matrix is empty")
            
        # 获取四个角的值
        self.corners = {
            'left_back': matrix[0][0],      # 第一行第一个(左后)
            'left_front': matrix[0][-1],    # 第一行最后一个(左前)
            'right_back': matrix[-1][0],    # 最后一行第一个(右后)
            'right_front': matrix[-1][-1]   # 最后一行最后一个(右前)
        }
        return self.corners
        
    def analyze_corners(self):
        # 找出最高点(值最大的角)
        max_height = max(self.corners.values())
        
        # 计算每个角需要调整的圈数
        self.adjustments = {}
        for corner, height in self.corners.items():
            diff = max_height - height
            turns = diff / self.screw_pitch
            self.adjustments[corner] = turns
            
        # 生成调整提示
        tips = ["Bed leveling tips:"]
        tips.append("Reference height: %.3f" % max_height)
        tips.append("\nCorner measurements:")
        for corner, height in self.corners.items():
            tips.append("%s: %.3f" % (corner, height))
            
        tips.append("\nSuggested adjustments (clockwise turns):")
        for corner, turns in self.adjustments.items():
            if turns > 0:
                tips.append("%s: %.2f turns" % (corner, turns))
                
        self.tips = "\n".join(tips)
        return self.tips

    cmd_BED_CAL_TIP_help = "Analyze bed mesh and provide leveling suggestions"
    def cmd_BED_CAL_TIP(self, gcmd):
        try:
            # 获取网格角点数据
            self.get_mesh_corners()
            # 分析并生成提示
            tips = self.analyze_corners()
            # 输出提示
            self.gcode.respond_info(tips)
        except Exception as e:
            raise self.gcode.error(str(e))

def load_config(config):
    return BedTip(config)