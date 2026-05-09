# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, sys, logging, io, re, json  # 添加json模块用于文件位置保存

VALID_GCODE_EXTS = ['gcode', 'g', 'gco']

DEFAULT_ERROR_GCODE = """
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
"""

class VirtualSD:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_shutdown)
        # sdcard state
        sd = config.get('path')
        self.sdcard_dirname = os.path.normpath(os.path.expanduser(sd))
        self.current_file = None
        self.file_position = self.file_size = 0
        # Print Stat Tracking
        self.print_stats = self.printer.load_object(config, 'print_stats')
        # Work timer
        self.reactor = self.printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.next_file_position = 0
        self.work_timer = None
        # Error handling
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.on_error_gcode = gcode_macro.load_template(
            config, 'on_error_gcode', DEFAULT_ERROR_GCODE)
        # 文件位置保存配置
        self.power_loss_file = "/home/tier/printer_data/logs/print_position.json"  # 保存文件路径（固定到日志目录）
        self.save_interval = 100  # 每100行保存一次
        self.save_count = 0  # 保存计数
        self.count_line = 0  # 行计数器，用于保存间隔判断
        self.load_file_path = None  # 加载的文件路径
        self.load_file_position = 0  # 加载的文件位置
        # 缓存常用对象，减少 lookup_object 开销
        self.gcode_move = None
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        for cmd in ['M20', 'M21', 'M23', 'M24', 'M25', 'M26', 'M27']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        for cmd in ['M28', 'M29', 'M30']:
            self.gcode.register_command(cmd, self.cmd_error)
        self.gcode.register_command(
            "SDCARD_RESET_FILE", self.cmd_SDCARD_RESET_FILE,
            desc=self.cmd_SDCARD_RESET_FILE_help)
        self.gcode.register_command(
            "SDCARD_PRINT_FILE", self.cmd_SDCARD_PRINT_FILE,
            desc=self.cmd_SDCARD_PRINT_FILE_help)
        self.gcode.register_command(
            "POWER_LOSS_RESUME", self.cmd_POWER_LOSS_RESUME,
            desc=self.cmd_POWER_LOSS_RESUME_help)
        self.gcode.register_command(
            "POWER_LOSS_WORK_HANDLER", self.cmd_POWER_LOSS_WORK_HANDLER,
            desc=self.cmd_POWER_LOSS_WORK_HANDLER_help)
        self.gcode.register_command(
            "QUERY_POWER_LOSS", self.cmd_QUERY_POWER_LOSS,
            desc=self.cmd_QUERY_POWER_LOSS_help)
        self.gcode.register_command(
            "STOP_PRINT_ERROR", self.cmd_STOP_PRINT_ERROR,
            desc=self.cmd_STOP_PRINT_ERROR_help)
        
    def _atomic_save_json(self, file_path, data):
        """原子性保存 JSON 数据到文件（通用方法）"""
        temp_file = file_path + ".tmp"
        try:
            # 写入临时文件
            with open(temp_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            
            # 验证临时文件不为空
            if os.path.getsize(temp_file) == 0:
                logging.error("%s 临时文件为空，保存失败" % file_path)
            else:
                os.rename(temp_file, file_path)
                return True
        except Exception as e:
            logging.exception("保存文件失败 %s: %s" % (file_path, e))
        return False

    def _write_current_state_to_disk(self):
        """将数据写入磁盘（使用原子性写入，防止断电时文件为空或不完整）"""
        # 获取 gcode_move 对象
        if self.gcode_move is None:
            self.gcode_move = self.printer.lookup_object('gcode_move', None)
            if self.gcode_move is None:
                logging.warning("gcode_move 对象不存在，无法保存状态")
                return
        
        # 获取当前位置和状态
        gcode_state = self.gcode_move.get_status()
        
        # 构建要保存的数据
        save_data = {
            "file_path": self.file_path(),
            "file_position": self.file_position,
            "gcode_state": gcode_state,
            "save_count": self.save_count
        }
        
        if self._atomic_save_json(self.power_loss_file, save_data):
            self.save_count += 1
        
    def handle_shutdown(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except:
                logging.exception("virtual_sdcard shutdown read")
                return
            logging.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                         readpos, repr(data[:readcount]),
                         self.file_position, repr(data[readcount:]))
    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        return True, "sd_pos=%d" % (self.file_position,)
    def get_file_list(self, check_subdirs=False):
        if check_subdirs:
            flist = []
            for root, dirs, files in os.walk(
                    self.sdcard_dirname, followlinks=True):
                for name in files:
                    ext = name[name.rfind('.')+1:]
                    if ext not in VALID_GCODE_EXTS:
                        continue
                    full_path = os.path.join(root, name)
                    r_path = full_path[len(self.sdcard_dirname) + 1:]
                    size = os.path.getsize(full_path)
                    flist.append((r_path, size))
            return sorted(flist, key=lambda f: f[0].lower())
        else:
            dname = self.sdcard_dirname
            try:
                filenames = os.listdir(self.sdcard_dirname)
                return [(fname, os.path.getsize(os.path.join(dname, fname)))
                        for fname in sorted(filenames, key=str.lower)
                        if not fname.startswith('.')
                        and os.path.isfile((os.path.join(dname, fname)))]
            except:
                logging.exception("virtual_sdcard get_file_list")
                raise self.gcode.error("Unable to get file list")
    def get_status(self, eventtime):
        return {
            'file_path': self.file_path(),
            'progress': self.progress(),
            'is_active': self.is_active(),
            'file_position': self.file_position,
            'file_size': self.file_size,
            'load_file_path': self.load_file_path,  # 加载的文件路径
        }
    def file_path(self):
        if self.current_file:
            return self.current_file.name
        return None
    def progress(self):
        if self.file_size:
            return float(self.file_position) / self.file_size
        else:
            return 0.
    def is_active(self):
        return self.work_timer is not None
    def do_pause(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            while self.work_timer is not None and not self.cmd_from_sd:
                self.reactor.pause(self.reactor.monotonic() + .001)
    def do_resume(self):
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
    def do_cancel(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
            self.print_stats.note_cancel()
        self.file_position = self.file_size = 0
    # def _cleanup_work_timer(self):
    #     if self.work_timer is not None:
    #         try:
    #             self.reactor.unregister_timer(self.work_timer)
    #         except:
    #             pass
    #         self.work_timer = None

    def _open_resume_file_absolute(self, gcmd, abs_path):
        """直接用 JSON 中的绝对路径打开 G-code（与 print_position.json 中 file_path 一致）。"""
        abs_path = os.path.normpath(abs_path)
        f = io.open(abs_path, 'r', newline='')
        f.seek(0, os.SEEK_END)
        fsize = f.tell()
        f.seek(0)
        bn = os.path.basename(abs_path)
        gcmd.respond_raw("File opened:%s Size:%d" % (bn, fsize))
        gcmd.respond_raw("File selected")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        self.print_stats.set_current_file(bn)
        if self.gcode_move is None:
            self.gcode_move = self.printer.lookup_object('gcode_move', None)

    cmd_POWER_LOSS_RESUME_help = "Resume printing after power loss"
    def cmd_POWER_LOSS_RESUME(self, gcmd):
        """Resume printing after power loss"""
        if self.is_active():
            raise gcmd.error("A print is already in progress; cannot resume")
        
        if not os.path.exists(self.power_loss_file):
            raise gcmd.error("No power-loss recovery data found")

        try:
            with open(self.power_loss_file, 'r', encoding='utf-8') as f:
                # 尝试读取并解析 JSON
                try:
                    saved_data = json.load(f)
                except json.JSONDecodeError as e:
                    # JSON 解析失败，可能是文件不完整
                    raise gcmd.error("Power-loss recovery data is corrupted (possibly incomplete due to power cut): %s" % str(e))
            
            file_path = str(saved_data.get('file_path', ''))
            file_position = saved_data.get('file_position')
            gcode_state = saved_data.get('gcode_state')

            # 保存XYZE位置到变量中
            # pos = gcode_state.get('position', [0.0, 0.0, 0.0, 0.0])
            pos = gcode_state.get('gcode_position', [0.0, 0.0, 0.0, 0.0])
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=power_loss_x VALUE=%f" % pos[0])
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=power_loss_y VALUE=%f" % pos[1])
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=power_loss_z VALUE=%f" % pos[2])
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=power_loss_e VALUE=%f" % pos[3])
            #保存坐标系设置
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=power_loss_coord VALUE=%d" % (1 if gcode_state.get('absolute_coordinates', False) else 0,))
            self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=power_loss_extruder VALUE=%d" % (1 if gcode_state.get('absolute_extrude', False) else 0,))

            self.load_file_position = file_position
            abs_saved = os.path.normpath(str(file_path).strip()) if file_path else ""
            if not abs_saved:
                raise gcmd.error("Invalid saved file_path in recovery data")
            if not os.path.isfile(abs_saved):
                raise gcmd.error(
                    "Saved G-code not found (only json file_path is used): %s"
                    % (abs_saved,))
            self.load_file_path = os.path.basename(abs_saved)
            gcmd.respond_info(
                "Starting power-loss resume: %s, pos: %d"
                % (abs_saved, self.load_file_position))
            try:
                self._open_resume_file_absolute(gcmd, abs_saved)
            except Exception as e:
                logging.exception(
                    "Open saved file_path failed: %s", abs_saved)
                raise gcmd.error("Unable to open file: %s" % (str(e),))

            self.file_position = self.load_file_position
            self.print_stats.note_start()
            
        except Exception as e:
            logging.exception("Failed to resume print: %s" % e)
            # 确保打印状态被取消
            try:
                if self.print_stats:
                    self.print_stats.note_cancel()
            except:
                pass
            raise gcmd.error("Failed to resume print: %s" % str(e))
    
    cmd_STOP_PRINT_ERROR_help = "Trigger an error to stop printing (keep state)"
    def cmd_STOP_PRINT_ERROR(self, gcmd):
        """Trigger an error to stop printing (keep state)"""
        if not self.is_active():
            gcmd.respond_info("No active print job")
            return
        
        error_msg = gcmd.get("MSG", "Print stopped by STOP_PRINT_ERROR command")
        logging.error("STOP_PRINT_ERROR: %s" % error_msg)
        
        # 记录 error 状态
        self.print_stats.note_error(error_msg)
        # 设置 must_pause_work 标志，让 work_handler 退出
        self.must_pause_work = True
        
        gcmd.respond_info("Triggered error stop: %s" % error_msg)

    cmd_POWER_LOSS_WORK_HANDLER_help = "Start power-loss recovery work handler"
    def cmd_POWER_LOSS_WORK_HANDLER(self, gcmd):
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.monotonic() + 0.1)

    cmd_QUERY_POWER_LOSS_help = "Query whether power-loss recovery data exists"
    def cmd_QUERY_POWER_LOSS(self, gcmd):
        """Query whether power-loss recovery data exists"""
        self.load_file_path = None
        # 仅当 has_power_loss == 1 时，才输出断电续打发现信息
        try:
            sv = self.printer.lookup_object('save_variables', None)
            variables = getattr(sv, 'allVariables', {}) if sv is not None else {}
            has_power_loss = 1 if int(variables.get('has_power_loss', 0)) == 1 else 0
        except Exception:
            has_power_loss = 0
        if has_power_loss != 1:
            gcmd.respond_info("No power-loss recovery data")
            return
        if os.path.exists(self.power_loss_file):
            try:
                with open(self.power_loss_file, 'r', encoding='utf-8') as f:
                    saved_data = json.load(f)
                file_path = saved_data.get('file_path', 'unknown')
                file_position = saved_data.get('file_position', 0)
                gcode_state = saved_data.get('gcode_state', {}) or {}
                pos = gcode_state.get('position', [0.0, 0.0, 0.0, 0.0])
                z_position = pos[2] if len(pos) > 2 else 0.0
                
                self.load_file_path = os.path.basename(file_path) if file_path else None
                if not self.load_file_path:
                    gcmd.respond_info("No power-loss recovery data found")
                    self.gcode.run_script_from_command("SAVE_VARIABLE VARIABLE=has_power_loss VALUE=0")
                    return

                gcmd.respond_info("Power-loss recovery data found:")
                gcmd.respond_info("  File: %s" % file_path)
                gcmd.respond_info("  Position: %d bytes" % (file_position,))
                gcmd.respond_info("  Z position: %.2f mm" % z_position)
            except Exception as e:
                gcmd.respond_info("Failed to read power-loss recovery data: %s" % str(e))
        else:
            gcmd.respond_info("No power-loss recovery data")
    
    # G-Code commands
    def cmd_error(self, gcmd):
        raise gcmd.error("SD write not supported")
    def _reset_file(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
        self.file_position = self.file_size = 0
        self.print_stats.reset()
        self.count_line = 0  # 重置行计数器
        self.save_count = 0  # 重置保存计数
        # 确保清理断电续打状态（无论文件是否存在）
        self.load_file_path = None
        self.printer.send_event("virtual_sdcard:reset_file")
    cmd_SDCARD_RESET_FILE_help = "Clears a loaded SD File. Stops the print "\
        "if necessary"
    def cmd_SDCARD_RESET_FILE(self, gcmd):
        if self.cmd_from_sd:
            raise gcmd.error(
                "SDCARD_RESET_FILE cannot be run from the sdcard")
        self._reset_file()
    cmd_SDCARD_PRINT_FILE_help = "Loads a SD file and starts the print.  May "\
        "include files in subdirectories."
    def cmd_SDCARD_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get("FILENAME")
        if filename[0] == '/':
            filename = filename[1:]
        self._load_file(gcmd, filename, check_subdirs=True)
        self.do_resume()
    def cmd_M20(self, gcmd):
        # List SD card
        files = self.get_file_list()
        gcmd.respond_raw("Begin file list")
        for fname, fsize in files:
            gcmd.respond_raw("%s %d" % (fname, fsize))
        gcmd.respond_raw("End file list")
    def cmd_M21(self, gcmd):
        # Initialize SD card
        gcmd.respond_raw("SD card ok")
    def cmd_M23(self, gcmd):
        # Select SD file
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get_raw_command_parameters().strip()
        if filename.startswith('/'):
            filename = filename[1:]
        self._load_file(gcmd, filename)
    def _load_file(self, gcmd, filename, check_subdirs=False):
        files = self.get_file_list(check_subdirs)
        flist = [f[0] for f in files]
        files_by_lower = { fname.lower(): fname for fname, fsize in files }
        fname = filename
        try:
            if fname not in flist:
                fname = files_by_lower[fname.lower()]
            fname = os.path.join(self.sdcard_dirname, fname)
            f = io.open(fname, 'r', newline='')
            f.seek(0, os.SEEK_END)
            fsize = f.tell()
            f.seek(0)
        except:
            logging.exception("virtual_sdcard file open")
            raise gcmd.error("Unable to open file")
        gcmd.respond_raw("File opened:%s Size:%d" % (filename, fsize))
        gcmd.respond_raw("File selected")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        self.print_stats.set_current_file(filename)
        # 用于获取 G-code 状态和位置信息
        if self.gcode_move is None:
            self.gcode_move = self.printer.lookup_object('gcode_move', None)

    def cmd_M24(self, gcmd):
        # Start/resume SD print
        self.do_resume()
    def cmd_M25(self, gcmd):
        # Pause SD print
        self.do_pause()
    def cmd_M26(self, gcmd):
        # Set SD position
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        pos = gcmd.get_int('S', minval=0)
        self.file_position = pos
    def cmd_M27(self, gcmd):
        # Report SD print status
        if self.current_file is None:
            gcmd.respond_raw("Not SD printing.")
            return
        gcmd.respond_raw("SD printing byte %d/%d"
                         % (self.file_position, self.file_size))
    def get_file_position(self):
        return self.next_file_position
    def set_file_position(self, pos):
        self.next_file_position = pos
    def is_cmd_from_sd(self):
        return self.cmd_from_sd
    # Background work timer
    def work_handler(self, eventtime):
        logging.info("Starting SD card print (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard seek")
            self.work_timer = None
            return self.reactor.NEVER
        self.print_stats.note_start()
        gcode_mutex = self.gcode.get_mutex()
        partial_input = ""
        lines = []
        error_message = None
        while not self.must_pause_work and not self.gcode.is_stopping():
            if not lines:
                # Read more data
                try:
                    data = self.current_file.read(8192)
                except:
                    logging.exception("virtual_sdcard read")
                    break
                if not data:
                    # End of file
                    self.current_file.close()
                    self.current_file = None
                    logging.info("Finished SD card print")
                    self.gcode.respond_raw("Done printing file")
                    break
                lines = data.split('\n')
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()
                lines.reverse()
                self.reactor.pause(self.reactor.NOW)
                continue
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue
            # Dispatch command
            self.cmd_from_sd = True
            line = lines.pop()
            if sys.version_info.major >= 3:
                next_file_position = self.file_position + len(line.encode()) + 1
            else:
                next_file_position = self.file_position + len(line) + 1
            self.next_file_position = next_file_position
            try:
                self.gcode.run_script(line)
            except self.gcode.error as e:
                error_message = str(e)
                try:
                    self.gcode.run_script(self.on_error_gcode.render())
                except:
                    logging.exception("virtual_sdcard on_error")
                break
            except:
                logging.exception("virtual_sdcard dispatch")
                break
            self.cmd_from_sd = False
            self.file_position = self.next_file_position
            # 行计数，用于间隔性保存断电续打位置
            self.count_line += 1
            # 定期保存当前打印位置到磁盘
            if self.count_line % self.save_interval == 0:
                self._write_current_state_to_disk()
            # Do we need to skip around?
            if self.next_file_position != next_file_position:
                try:
                    self.current_file.seek(self.file_position)
                except:
                    logging.exception("virtual_sdcard seek")
                    self.work_timer = None
                    return self.reactor.NEVER
                lines = []
                partial_input = ""
        logging.info("Exiting SD card print (position %d)", self.file_position)
        # 循环退出时，如果不是暂停且仍有打开文件，再保存一次位置
        if self.must_pause_work:
            self._write_current_state_to_disk()
        self.work_timer = None
        self.cmd_from_sd = False
        if error_message is not None:
            self.print_stats.note_error(error_message)
        elif self.current_file is not None:
            self.print_stats.note_pause()
        else:
            self.print_stats.note_complete()
            # print next job if autoprint is enabled
            obj = self.printer.lookup_object('autoprint', None)
            if obj is not None:
                self.gcode.run_script_from_command("AUTO_PREPARENEXT")
                self.gcode.run_script_from_command("AUTO_STARTNEXT FLAG=4")
        return self.reactor.NEVER

def load_config(config):
    return VirtualSD(config)
