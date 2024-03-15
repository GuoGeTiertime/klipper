# Save arbitrary variables so that values can be kept across restarts.
#
# Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser, shutil

class SaveVariables:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('filename'))
        base, ext = os.path.splitext(self.filename)
        self.bakfile = base + ".bak"
        logging.info("save variables backup file: %s" % self.bakfile)
        self.allVariables = {}
        try:
            # 检查原文件是否存在或长度是否小于10，如果是，则尝试从备份文件中恢复
            if not os.path.exists(self.filename) or os.path.getsize(self.filename) < 20:
                if os.path.exists(self.bakfile):
                    # 尝试从备份恢复
                    shutil.copy(self.bakfile, self.filename)
                    logging.info("Recovered variables from backup file")
                else:
                    logging.info("save variables is empty, and can't copy from backup file, write a emtpy file")
                    open(self.filename, "w").close()
            self.loadVariables()
        except self.printer.command_error as e:
            raise config.error(str(e))
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SAVE_VARIABLE', self.cmd_SAVE_VARIABLE,
                               desc=self.cmd_SAVE_VARIABLE_help)
        # 注册 BACKUP_VARIABLES 命令
        gcode.register_command('BACKUP_VARIABLES', self.cmd_BACKUP_VARIABLES, 
                               desc="Backup current variables to a backup file")
        # 注册 RESTORE_VARIABLES 命令
        gcode.register_command('RESTORE_VARIABLES', self.cmd_RESTORE_VARIABLES, 
                               desc="Restore variables from the backup file")

    def loadVariables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            varfile.read(self.filename)
            if varfile.has_section('Variables'):
                for name, val in varfile.items('Variables'):
                    allvars[name] = ast.literal_eval(val)
        except:
            msg = "Unable to parse existing variable file"
            logging.exception(msg)
            raise self.printer.command_error(msg)
        self.allVariables = allvars
    cmd_SAVE_VARIABLE_help = "Save arbitrary variables to disk"
    def cmd_SAVE_VARIABLE(self, gcmd):
        varname = gcmd.get('VARIABLE')
        value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section('Variables')
        for name, val in sorted(newvars.items()):
            varfile.set('Variables', name, repr(val))
        try:
            with open(self.filename, "w") as f:
                varfile.write(f)
        except:
            msg = "Unable to save variable"
            logging.exception(msg)
            raise gcmd.error(msg)
        self.loadVariables()
    
    def cmd_BACKUP_VARIABLES(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        # 检查原文件是否存在或长度是否小于10，如果太短或不存在,不能备份.
        if not os.path.exists(self.filename) or os.path.getsize(self.filename) < 20:
            gcode.respond_info("Variables file(%s) is not exist or too short, can't backup" % self.filename)
            return
        try:
            shutil.copy(self.filename, self.bakfile)
            gcode.respond_info("Variables backed up successfully.")
        except Exception as e:
            raise gcmd.error("Failed to backup variables: %s -> %s" % (self.filename, self.bakfile))

    def cmd_RESTORE_VARIABLES(self, gcmd):
        gcode = self.printer.lookup_object('gcode')
        if not os.path.exists(self.bakfile) or os.path.getsize(self.bakfile) < 20:
            gcode.respond_info("Backup file does not exist or too short.")
            return
        try:
            shutil.copy(self.bakfile, self.filename)
            self.loadVariables()  # 重新加载变量
            gcode.respond_info("Variables restored from backup successfully.")
            gcode.respond_info("Variables:\n %s" % self.allVariables)
        except Exception as e:
            raise gcmd.error("Failed to restore variables from backup file: %s" % self.bakfile)

    def get_status(self, eventtime):
        return {'variables': self.allVariables}

def load_config(config):
    return SaveVariables(config)
