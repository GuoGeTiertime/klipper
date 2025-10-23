# Support for i2c based temperature sensors
#
# Copyright (C) 2020  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import struct
from . import bus
import time

BL24C16F_CHIP_ADDR = [0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57]
PAGE_SIZE = 16

class EEPROMCommandHelper:
    def __init__(self, config, chip):
        self.printer = config.get_printer()
        self.chip = chip
        name_parts = config.get_name().split()
        self.base_name = name_parts[0]
        self.name = name_parts[-1]
        self.register_commands(self.name)
        if len(name_parts) == 1:
            if self.name == "bl24c16f" or not config.has_section("bl24c16f"):
                self.register_commands(None)

    def register_commands(self, name):
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("EEPROM_DEBUG_READ", "CHIP", name,
                                   self.cmd_EEPROM_DEBUG_READ,
                                   desc=self.cmd_EEPROM_DEBUG_READ_help)
        gcode.register_mux_command("EEPROM_DEBUG_WRITE_BYTE", "CHIP", name,
                                   self.cmd_EEPROM_DEBUG_WRITE_BYTE,
                                   desc=self.cmd_EEPROM_DEBUG_WRITE_BYTE_help)
        gcode.register_mux_command("EEPROM_DEBUG_WRITE_INT", "CHIP", name,
                                   self.cmd_EEPROM_DEBUG_WRITE_INT,
                                   desc=self.cmd_EEPROM_DEBUG_WRITE_INT_help)
        gcode.register_mux_command("EEPROM_DEBUG_WRITE_FLOAT", "CHIP", name,
                                   self.cmd_EEPROM_DEBUG_WRITE_FLOAT,
                                   desc=self.cmd_EEPROM_DEBUG_WRITE_FLOAT_help)
        
        gcode.register_mux_command("EEPROM_READ", "CHIP", name,
                                   self.cmd_EEPROM_READ,
                                   desc=self.cmd_EEPROM_READ_help)
        gcode.register_mux_command("EEPROM_WRITE_BYTE", "CHIP", name,
                                   self.cmd_EEPROM_WRITE_BYTE,
                                   desc=self.cmd_EEPROM_WRITE_BYTE_help)
        gcode.register_mux_command("EEPROM_WRITE_INT", "CHIP", name,
                                   self.cmd_EEPROM_WRITE_INT,
                                   desc=self.cmd_EEPROM_WRITE_INT_help)
        gcode.register_mux_command("EEPROM_WRITE_FLOAT", "CHIP", name,
                                   self.cmd_EEPROM_WRITE_FLOAT,
                                   desc=self.cmd_EEPROM_WRITE_FLOAT_help)
        gcode.register_mux_command("EEPROM_WRITE_STRING", "CHIP", name,
                                   self.cmd_EEPROM_WRITE_STRING,
                                   desc=self.cmd_EEPROM_WRITE_STRING_help)
        gcode.register_mux_command("EEPROM_READ_STRING", "CHIP", name,
                                   self.cmd_EEPROM_READ_STRING,
                                   desc=self.cmd_EEPROM_READ_STRING_help)
        
        gcode.register_mux_command("EEPROM_IS_FIRST_USED", "CHIP", name,
                                   self.cmd_EEPROM_IS_FIRST_USED)
        gcode.register_mux_command("EEPROM_POS", "CHIP", name,
                                   self.cmd_EEPROM_POS)
        gcode.register_mux_command("EEPROM_PRINTER_INFO", "CHIP", name,
                                   self.cmd_EEPROM_PRINTER_INFO)
        gcode.register_mux_command("EEPROM_FACTORY_RESET", "CHIP", name,
                                   self.cmd_EEPROM_FACTORY_RESET,
                                   desc=self.cmd_EEPROM_FACTORY_RESET_help)

    def cmd_EEPROM_IS_FIRST_USED(self, gcmd):
        val = self.chip.read_reg(1, 1)
        state = False if int.from_bytes(val, 'little') != 255 else True
        gcmd.respond_info("EEPROM_IS_USED val:%s state:%s" % (int.from_bytes(val, 'little'), state))
        if int.from_bytes(val, 'little') != 255:
            return False
        else:
            return True
    
    def cmd_EEPROM_POS(self, gcmd):
        pos = self.chip.read_reg(0, 1)
        gcmd.respond_info("EEPROM_POS int_pos:%s, pos:%s" % (int.from_bytes(pos, 'little'), pos))
        
    def cmd_EEPROM_PRINTER_INFO(self, gcmd):
        pos = int.from_bytes(self.chip.read_reg(0, 1), 'little')
        file_position = self.chip.read_reg(pos*8, 4)
        base_position_e = self.chip.read_reg(pos*8+4, 4)
        ret =  {"file_position": int.from_bytes(file_position, 'little'), "base_position_e": struct.unpack('f', base_position_e)[0]}
        gcmd.respond_info("EEPROM_PRINTER_INFO ret:%s" % str(ret))

    cmd_EEPROM_DEBUG_READ_help = "Read data bytes from eeprom"
    def cmd_EEPROM_DEBUG_READ(self, gcmd):
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        size = gcmd.get("SIZE", minval=0, maxval=56, parser=lambda x: int(x, 0))
        vals = self.chip.read_reg(addr, size)
        gcmd.respond_info("EEPROM_DEBUG_READ size: 0x%x" % size)
        reg_vals = 'read vals: '
        for i in range(size):
            if i % 16 == 0:
                reg_vals += '\n'
            reg_vals += '0x%x ' % vals[i]

        gcmd.respond_info(reg_vals)

    cmd_EEPROM_DEBUG_WRITE_BYTE_help = "Write byte data to eeprom"
    def cmd_EEPROM_DEBUG_WRITE_BYTE(self, gcmd):
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        val = gcmd.get("VAL", minval=0, maxval=255, parser=lambda x: int(x, 0))
        gcmd.respond_info("EEPROM_DEBUG_WRITE_BYTE : ADDR[0x%x] = 0x%x" % (addr, val))
        self.chip.write_reg(addr, val)

    cmd_EEPROM_DEBUG_WRITE_INT_help = "Write int (4 byte) data to eeprom"
    def cmd_EEPROM_DEBUG_WRITE_INT(self, gcmd):
        pos = self.chip.read_reg(0, 1)
        gcmd.respond_info("EEPROM_POS int_pos:%s" % int.from_bytes(pos, 'little'))
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        val = gcmd.get("VAL", minval=0, maxval=4294967296, parser=lambda x: int(x, 0))
        gcmd.respond_info("EEPROM_DEBUG_WRITE_INT : val = %d" % val)
        vals = [val & 0xFF]
        vals += [   (val >> 8) & 0xFF,
                    (val >> 16) & 0xFF,
                    (val >> 24) & 0xFF,
                ]
        gcmd.respond_info("EEPROM_DEBUG_WRITE_INT : ADDR[0x%x] = 0x%02x 0x%02x 0x%02x 0x%02x"
                % (addr, vals[0], vals[1], vals[2], vals[3]))
        self.chip.write_reg(addr, vals)

    cmd_EEPROM_DEBUG_WRITE_FLOAT_help = "Write float (4 byte) data to eeprom"
    def cmd_EEPROM_DEBUG_WRITE_FLOAT(self, gcmd):
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        val = gcmd.get_float("VAL", 0.)
        gcmd.respond_info("EEPROM_DEBUG_WRITE_FLOAT : val = %f" % val)
        bs = struct.pack("f", val)
        data = int.from_bytes(bs, byteorder="little")

        vals = [data & 0xFF]
        vals += [   (data >> 8) & 0xFF, 
                    (data >> 16) & 0xFF,
                    (data >> 24) & 0xFF
                ]
        gcmd.respond_info("EEPROM_DEBUG_WRITE_FLOAT : ADDR[0x%x] = 0x%02x 0x%02x 0x%02x 0x%02x"
                        % (addr, vals[0], vals[1], vals[2], vals[3]))
        self.chip.write_reg(addr, vals)
    
    cmd_EEPROM_WRITE_STRING_help = "Write string data to eeprom (UTF-8 encoding, padded with null bytes)"
    def cmd_EEPROM_WRITE_STRING(self, gcmd):
        """写入字符串到EEPROM"""
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        string_val = gcmd.get("STRING", "")
        if not string_val:
            gcmd.respond_info("Error: STRING parameter is required")
            gcmd.respond_info("Usage: EEPROM_WRITE_STRING ADDR=address STRING=\"string\"")
            return
        # 根据字符串长度自动识别，最大15字节（留1字节给结束符）
        string_bytes = string_val.encode('utf-8')
        max_length = min(len(string_bytes), 15) 
        if len(string_bytes) > 15:
            gcmd.respond_info("Warning: String length exceeds 15 bytes, auto-truncated")
        # 添加结束符，不固定长度
        string_bytes = string_bytes[:max_length] + b'\x00'
        self.chip.write_reg(addr, [b for b in string_bytes])

    cmd_EEPROM_READ_STRING_help = "Read string from eeprom (UTF-8 decoding, stops at null terminator)"
    def cmd_EEPROM_READ_STRING(self, gcmd):
        """从EEPROM读取字符串"""
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        size = gcmd.get("SIZE", 16, minval=0, maxval=56, parser=lambda x: int(x, 0))
        # 读取数据
        try:
            raw_data = self.chip.read_reg(addr, size)
            # 查找结束符位置
            end_pos = -1
            for i, byte in enumerate(raw_data):
                if byte == 0:  # 找到结束符
                    end_pos = i
                    break
            if end_pos == -1:
                gcmd.respond_info("Warning: No string terminator found, returning full data")
                end_pos = len(raw_data)
            # 提取字符串数据
            string_data = raw_data[:end_pos]
            # 转换为字符串
            try:
                string_result = string_data.decode('utf-8', errors='ignore')
                self.chip.status["string_data"] = string_result
            except Exception as e:
                gcmd.respond_info("Error: String decoding failed - %s" % str(e))
                gcmd.respond_info("Raw bytes: %s" % ' '.join(['0x%02x' % b for b in string_data]))
        except Exception as e:
            gcmd.respond_info("Error: Failed to read EEPROM - %s" % str(e))
    
    cmd_EEPROM_READ_help = "Read data bytes from eeprom"
    def cmd_EEPROM_READ(self, gcmd):
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        count = gcmd.get("COUNT", minval=0, maxval=14, parser=lambda x: int(x, 0))
        data_type = gcmd.get("TYPE", "BYTE")
        data_type_upper = data_type.upper()
        size = count*4 if data_type_upper in ["INT", "FLOAT"] else count
        vals = self.chip.read_reg(addr, size)

        reg_vals = []
        if data_type_upper == "BYTE":            # 字节格式显示
            reg_vals = [vals[i] for i in range(size)]
            self.chip.status["byte_data"] = reg_vals
        elif data_type_upper == "INT":
            for i in range(0, size, 4):
                # 读取3字节数据 + 1字节CRC
                data_bytes = vals[i:i+3]
                received_crc = vals[i+3]
                # 校验CRC
                calculated_crc = self.crc8(data_bytes)
                if calculated_crc != received_crc:
                    gcmd.respond_info("CRC Error at 0x%04X" % (addr+i))
                    reg_vals.append(10000000000)
                    self.chip.status["int_data"] = reg_vals
                else:
                    # 转换为有符号整数（24位）
                    int_val = data_bytes[0] | (data_bytes[1] << 8) | (data_bytes[2] << 16)
                    if int_val & 0x800000:  # 检查符号位
                        int_val -= 0x1000000
                    reg_vals.append(int_val)
            # gcmd.respond_info("EEPROM_READ_INT : ADDR[0x%04X] = 0x%02X 0x%02X 0x%02X (CRC=0x%02X)" % (addr, vals[0], vals[1], vals[2], vals[3]))
            self.chip.status["int_data"] = reg_vals
        elif data_type_upper == "FLOAT":
            for i in range(0, size, 4):
                float_val = struct.unpack('<f', bytes(vals[i:i+4]))[0]  # 直接解包字节
                reg_vals.append(f"{float_val:.6f}")
            self.chip.status["float_data"] = reg_vals
        elif data_type_upper == "STRING":
            self.cmd_EEPROM_READ_STRING(gcmd)
        else:
            gcmd.respond_info("Error: Invalid TYPE parameter. Use BYTE, INT, FLOAT, or STRING")
    
    cmd_EEPROM_WRITE_BYTE_help = "Write byte data to eeprom"
    def cmd_EEPROM_WRITE_BYTE(self, gcmd):
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        val = gcmd.get("VAL", minval=0, maxval=255, parser=lambda x: int(x, 0))
        # gcmd.respond_info("EEPROM_WRITE_BYTE : ADDR[0x%x] = 0x%x" % (addr, val))
        self.chip.write_reg(addr, val)
    
    cmd_EEPROM_WRITE_INT_help = "Write int (4 byte) data to eeprom"
    def cmd_EEPROM_WRITE_INT(self, gcmd):
        # gcmd.respond_info("EEPROM_POS int_pos:%s" % int.from_bytes(pos, 'little'))
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        val = gcmd.get("VAL", minval=-8388608, maxval=8388607, parser=lambda x: int(x, 0))
        # 将32位整数转为24位有符号数（丢弃高8位）
        val_24bit = val & 0xFFFFFF
        if val < 0:
            val_24bit -= 0x1000000  # 补码转换（确保最高位为1）
        # 拆分为3字节数据
        data_bytes = [
            val_24bit & 0xFF,          # 字节0: LSB
            (val_24bit >> 8) & 0xFF,    # 字节1
            (val_24bit >> 16) & 0xFF    # 字节2（含符号位）
        ]
        crc = self.crc8(data_bytes)     # 计算前3字节的CRC
        vals = data_bytes + [crc]  # 组合成4字节
        # gcmd.respond_info("EEPROM_WRITE_INT : ADDR[0x%04X] = ""Data: 0x%02X 0x%02X 0x%02X | ""CRC: 0x%02X" % (addr, vals[0], vals[1], vals[2], vals[3]))
        self.chip.write_reg(addr, vals)

    cmd_EEPROM_WRITE_FLOAT_help = "Write float (4 byte) data to eeprom"
    def cmd_EEPROM_WRITE_FLOAT(self, gcmd):
        addr = gcmd.get("ADDR", minval=0, maxval=2047, parser=lambda x: int(x, 0))
        val = gcmd.get_float("VAL", 0.)
        # gcmd.respond_info("EEPROM_WRITE_FLOAT : val = %f" % val)
        bs = struct.pack("f", val)
        data = int.from_bytes(bs, byteorder="little")

        vals = [data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF]
        # gcmd.respond_info("EEPROM_WRITE_FLOAT : ADDR[0x%x] = 0x%02x 0x%02x 0x%02x 0x%02x"
                        # % (addr, vals[0], vals[1], vals[2], vals[3]))
        self.chip.write_reg(addr, vals)
    
    def crc8(self, data, poly=0x31, init=0x55, xor_out=0x00):
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) & 0xFF) ^ poly
                else:
                    crc = (crc << 1) & 0xFF
        return crc ^ xor_out
    
    cmd_EEPROM_FACTORY_RESET_help = "Reset EEPROM to factory defaults (requires CONFIRM=1)"
    def cmd_EEPROM_FACTORY_RESET(self, gcmd):
        """恢复EEPROM出厂默认设置"""
        # 需要确认参数，防止误操作
        confirm = gcmd.get_int("CONFIRM", 0)
        if confirm != 1:
            gcmd.respond_info("警告: 此操作将清除所有EEPROM数据!")
            gcmd.respond_info("如果确认要恢复出厂默认，请执行:")
            gcmd.respond_info("EEPROM_FACTORY_RESET CONFIRM=1")
            return
        
        # 获取清除范围，默认清除所有2048字节
        start_addr = gcmd.get_int("START", 0, minval=0, maxval=2047)
        end_addr = gcmd.get_int("END", 2047, minval=0, maxval=2047)
        
        if end_addr < start_addr:
            gcmd.respond_info("错误: END地址必须大于或等于START地址")
            return
        
        gcmd.respond_info("开始恢复出厂默认设置...")
        gcmd.respond_info("清除地址范围: 0x%04X - 0x%04X" % (start_addr, end_addr))
        
        # 批量写入0xFF（EEPROM擦除状态）
        batch_size = 16  # 每次写入16字节（一页）
        total_bytes = end_addr - start_addr + 1
        cleared_bytes = 0
        
        for addr in range(start_addr, end_addr + 1, batch_size):
            # 计算本次写入的字节数
            remaining = end_addr - addr + 1
            write_size = min(batch_size, remaining)
            
            # 写入0xFF
            data = [0xFF] * write_size
            self.chip.write_reg(addr, data)
            
            cleared_bytes += write_size
            
            # 每256字节显示一次进度
            if cleared_bytes % 256 == 0 or cleared_bytes == total_bytes:
                progress = (cleared_bytes * 100) // total_bytes
                gcmd.respond_info("进度: %d%% (%d/%d 字节)" % (progress, cleared_bytes, total_bytes))
        
        # 重置关键标志位
        if start_addr == 0 and end_addr >= 1:
            self.chip.write_reg(0, 0)    # 重置位置指针
            self.chip.write_reg(1, 255)  # 设置首次使用标志
            gcmd.respond_info("已重置位置指针和首次使用标志")
        
        gcmd.respond_info("恢复出厂默认设置完成!")
        gcmd.respond_info("总计清除: %d 字节" % cleared_bytes)  

class BL24C16F:
    def __init__(self, config):
        self.printer = config.get_printer()
        EEPROMCommandHelper(config, self)
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = [bus.MCU_I2C_from_config(
            config, default_addr=addr, default_speed=400000) for addr in BL24C16F_CHIP_ADDR]
        self.mcu = self.i2c[0].get_mcu()
        self.printer.add_object("bl24c16f " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.status = {
            "byte_data": [],
            "int_data": [],
            "float_data": [],
            "string_data": "",
            }

    def get_status(self, eventtime=None):
        return self.status

    def handle_connect(self):
        self._init_bl24c16f()

    def _init_bl24c16f(self):
        logging.info("bl24c16f init...")

    def read_reg(self, addr, read_len):
        index = addr // 256
        offset = addr % 256
        reg = [offset]
        params = self.i2c[index].i2c_read(reg, read_len)

        return bytearray(params['response'])

    def write_reg(self, addr, data):
        if type(data) is not list:
            data = [data]

        # 分页写入逻辑
        current_page_start = (addr // PAGE_SIZE) * PAGE_SIZE
        current_page_end = current_page_start + PAGE_SIZE  
        # 检查是否需要分页写入
        if addr + len(data) <= current_page_end:
            # 单页写入
            self._write_to_chip(addr, data)
        else:
            # 跨页写入，需要分页处理
            # 计算第一页的数据
            first_page_bytes = current_page_end - addr
            first_page_data = data[:first_page_bytes] 
            # 写入第一页
            self._write_to_chip(addr, first_page_data)
            # 计算第二页的数据
            second_page_data = data[first_page_bytes:]
            second_page_addr = current_page_end
            # 写入第二页
            self._write_to_chip(second_page_addr, second_page_data)

    def _write_to_chip(self, chip_addr, write_data):
        index = chip_addr // 256
        offset = chip_addr % 256
        data_to_write = [offset] + write_data
        self.i2c[index].i2c_write(data_to_write)
        time.sleep(0.005)

    def setEepromDisable(self):
        self.write_reg(1, 255)

    def checkEepromFirstEnable(self):
        val = self.read_reg(1, 1)
        if int.from_bytes(val, 'little') != 255:
            return False
        else:
            return True
        
    def eepromReadHeader(self):
        pos = self.read_reg(0, 1)
        return int.from_bytes(pos, 'little')
    
    def eepromReadBody(self, pos):
        file_position = self.read_reg(pos*8, 4)
        base_position_e = self.read_reg(pos*8+4, 4)
        return {"file_position": int.from_bytes(file_position, 'little'), "base_position_e": struct.unpack('f', base_position_e)[0]}  

def load_config(config):
    return BL24C16F(config)

def load_config_prefix(config):
    return BL24C16F(config)
