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
                int_val = vals[i] | (vals[i+1] << 8) | (vals[i+2] << 16) | (vals[i+3] << 24)
                reg_vals.append(int_val)
            self.chip.status["int_data"] = reg_vals
        elif data_type_upper == "FLOAT":
            for i in range(0, size, 4):
                # 将4字节转换为浮点数
                float_bytes = bytes([vals[i], vals[i+1], vals[i+2], vals[i+3]])
                float_val = struct.unpack('f', float_bytes)[0]
                reg_vals.append(float_val)
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
        val = gcmd.get("VAL", minval=0, maxval=4294967296, parser=lambda x: int(x, 0))
        # gcmd.respond_info("EEPROM_WRITE_INT : val = %d" % val)
        vals = [val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0xFF, (val >> 24) & 0xFF]
        # gcmd.respond_info("EEPROM_WRITE_INT : ADDR[0x%x] = 0x%02x 0x%02x 0x%02x 0x%02x"
                # % (addr, vals[0], vals[1], vals[2], vals[3]))
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

    def _write_to_chip(chip_addr, write_data):
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
