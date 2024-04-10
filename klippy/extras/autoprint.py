# Save arbitrary variables so that values can be kept across restarts.
#
# Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser, datetime

class AutoPrint:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('filename'))
        self.maxJobs = config.getint('maxjobs', 20, minval=1, maxval=100)
        self.allJobs = []
        self.autoflag = 0
        self.pause = True #default pause the auto print process.
        try:
            if not os.path.exists(self.filename):
                open(self.filename, "w").close()
            self.loadJobs()
        except self.printer.command_error as e:
            raise config.error(str(e))
        
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.prepare_gcode = gcode_macro.load_template(config, "prepare_gcode", "AUTO_PREPARE_OK")

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('AUTO_ADDJOB', self.cmd_AUTO_ADDJOB,
                                desc=self.cmd_AUTO_ADDJOB_help)
        self.gcode.register_command('AUTO_DELJOB', self.cmd_AUTO_DELJOB,
                                desc=self.cmd_AUTO_DELJOB_help)
        self.gcode.register_command('AUTO_FINISHJOB', self.cmd_AUTO_FINISHJOB,
                                desc=self.cmd_AUTO_FINISHJOB_help)
        self.gcode.register_command('AUTO_PREPARENEXT', self.cmd_AUTO_PREPARENEXT,
                                desc=self.cmd_AUTO_PREPARENEXT_help)
        self.gcode.register_command('AUTO_PREPARE_OK', self.cmd_AUTO_PREPARE_OK,
                                desc=self.cmd_AUTO_PREPARE_OK_help)
        self.gcode.register_command('AUTO_STARTNEXT', self.cmd_AUTO_STARTNEXT,
                                desc=self.cmd_AUTO_STARTNEXT_help)
        self.gcode.register_command('AUTO_LIST', self.cmd_AUTO_LIST,
                                desc=self.cmd_AUTO_LIST_help)
        self.gcode.register_command('AUTO_MOVEJOB', self.cmd_AUTO_MOVEJOB,
                                desc=self.cmd_AUTO_MOVEJOB_help)
        self.gcode.register_command('AUTO_CLEAR', self.cmd_AUTO_CLEAR, 
                                desc=self.cmd_AUTO_CLEAR_help)
        self.gcode.register_command('AUTO_PAUSE', self.cmd_AUTO_PAUSE, 
                                desc=self.cmd_AUTO_PAUSE_help)
    def loadJobs(self):
        alljobs = []
        jobfile = configparser.ConfigParser()
        try:
            jobfile.read(self.filename)
            for i in range(self.maxJobs): # max jobs
                if jobfile.has_section('Job_%d' % i):
                    job = {}
                    for name, val in jobfile.items('Job_%d' % i):
                        job[name] = ast.literal_eval(val)
                    # job['date_add'] = datetime.datetime.strptime(job['date'], '%Y%m%d_%H:%M:%S')
                    # verify the job is valid
                    required_keys = ['filename', 'completed', 'times', 'date']
                    is_missing_any_key = not all(key in job for key in required_keys)
                    bValid = True
                    if is_missing_any_key: # if any key is missing, fix this job
                        if 'filename' not in job:
                            bValid = False
                        if 'completed' not in job:
                            job['completed'] = 0
                        if 'times' not in job:
                            job['times'] = max( job['completed'], 1)
                        if 'date' not in job:
                            job['date'] = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                    # add the job to the list if it is valid
                    if bValid:
                        alljobs.append(job)
                else:
                    break
        except:
            msg = "Unable to parse existing autoprint file: %s" % self.filename
            logging.exception(msg)
        self.allJobs = alljobs
    def saveJobs(self):
        # Write file
        jobfile = configparser.ConfigParser()
        for i, job in enumerate(self.allJobs):
            jobfile.add_section('Job_%d' % i)
            for name, val in sorted(job.items()):
                jobfile.set('Job_%d' % i, name, repr(val))
        try:
            f = open(self.filename, "w")
            jobfile.write(f)
            f.close()
        except:
            msg = "Unable to save autoprint file: %s" % self.filename
            logging.exception(msg)
            raise self.printer.command_error(msg)(msg)

    def _jobinfo(self, job):
        return "File:%s, printed %d/%d, date: %s" % (job['filename'], job['completed'], job['times'], job['date'])
    
    def _loginfo(self, head, job=None):
        msg = head
        if job is not None:
            msg += self._jobinfo(job)
        self.gcode.respond_info(msg)

    def _logalljobs(self):
        info = []
        for i, job in enumerate(self.allJobs):
            msg = "Job %d: %s" % (i, self._jobinfo(job))
            info.append(msg)
        self.gcode.respond_info("Jobs in auto print list:\n" + "\n".join(info))

    def _pauselist(self):
        self.autoflag = 0
        self.pause = True

    cmd_AUTO_ADDJOB_help = "add a print job to auto print job list"
    def cmd_AUTO_ADDJOB(self, gcmd):
        if len(self.allJobs) >= self.maxJobs:
            self.gcode.respond_info("Max jobs(%d) reached" % self.maxJobs)
            return
        
        # get new job's filename and print times, and current time as start time
        filename = gcmd.get('FILE')
        times = gcmd.get_int('TIMES', 1, minval=1, maxval=1000)
        # get current system time
        curTime = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        job = {'filename': filename, 'times': times, 'date': curTime, 'completed': 0}
        self.allJobs.append(job)
        # save the jobs to file
        self.saveJobs()
        self._loginfo("Add print job, ", job)

    cmd_AUTO_DELJOB_help = "Decrease print times of job or delete the job, negative CNT means increase print times of the job"
    def cmd_AUTO_DELJOB(self, gcmd):
        idx = gcmd.get_int('IDX', -1, minval=-1, maxval=len(self.allJobs)-1)
        if idx == -1:
            filename = gcmd.get('FILE', "")
            for i, job in enumerate(self.allJobs):
                if job['filename'] == filename:
                    idx = i
                    break
        if idx < 0 or idx >= len(self.allJobs):
            self.gcode.respond_info("No such job")
            return
        
        # cnt is the number of times to remove from the job, 0 means remove the job
        cnt = gcmd.get_int('CNT', 1000, minval=-1000, maxval=1000)
        job = self.allJobs[idx]
        job['times'] -= cnt

        if job['completed'] >= job['times']:
            self.allJobs.pop(idx)
            self._loginfo("Delete print job, ", job)
        
        # save the jobs to file
        self.saveJobs()

    # 添加帮助信息
    cmd_AUTO_MOVEJOB_help = "Move a job in the auto print job list by index and move distance"
    # 实现 AUTO_MOVEJOB 命令
    def cmd_AUTO_MOVEJOB(self, gcmd):
        idx = gcmd.get_int('IDX', minval=0)
        move = gcmd.get_int('MOVE', 1) # 默认向前移动一个位置
        # 确保索引在合法范围内
        if idx < 0 or idx >= len(self.allJobs):
            self.gcode.respond_info("Invalid job index")
            return
        new_idx = idx - move
        # 确保新索引在合法范围内
        if new_idx < 0 :
            new_idx = 0
        elif new_idx > len(self.allJobs)-1:
            new_idx = len(self.allJobs)-1

        # 移动作业
        job = self.allJobs.pop(idx)
        self.allJobs.insert(new_idx, job)
        # 保存更改到文件
        self.saveJobs()
        self._loginfo("Job moved to index:%d successfully. " % new_idx, job)

    # 添加帮助信息
    cmd_AUTO_CLEAR_help = "Clear all jobs in the auto print job list"
    # 实现 AUTO_CLEAR 命令
    def cmd_AUTO_CLEAR(self, gcmd):
        # 清空作业列表
        self.allJobs.clear()
        # 保存更改到文件
        self.saveJobs()
        # 响应命令执行成功
        self.gcode.respond_info("All jobs cleared successfully")

    cmd_AUTO_FINISHJOB_help = "finish a job and remove it from auto print job list if it has been printed enough times"
    def cmd_AUTO_FINISHJOB(self, gcmd):
        if self.autoflag != 1:   # only set to finish state when the job is printed from auto print list, and finised successfully!
            self._loginfo("Not in auto print mode, can't finish job, autoflag: %d" % self.autoflag)
            self._pauselist()
            return

        if len(self.allJobs) == 0:
            self._loginfo("No jobs to finish")
            return
        
        job = self.allJobs[0]
        job['completed'] += 1
        if job['completed'] >= job['times']:
            self._loginfo("job finished, delete from auto print list", job)
            self.allJobs.pop(0)
        else:
           self._loginfo("job printed, ", job)
        self.saveJobs()

        self.autoflag = 2   # set to finish state

    cmd_AUTO_PREPARENEXT_help = "Perpare for next job, change platform or other things if needed"
    def cmd_AUTO_PREPARENEXT(self, gcmd):
        if self.autoflag != 2:  # only set to prepare state when the job is finished successfully!
            self._loginfo("Print not finished succesufully, can't auto prepare next job, autoflag: %d" % self.autoflag)
            self._pauselist()
            return

        self.autoflag = 3 # set to begin prepare state

        # run a gcode cmd to prepare for next job, like change platform or other things
        cmdstr = self.prepare_gcode.render()
        self._loginfo("Run parpare next gcode cmd: %s" % cmdstr)
        self.gcode.run_script_from_command(cmdstr)

    cmd_AUTO_PREPARE_OK_help = "Perpare for next job, change platform or other things if needed"
    def cmd_AUTO_PREPARE_OK(self, gcmd):
        if self.autoflag != 3:  # only set to prepare state when the job is finished successfully!
            self._loginfo("Prepare for next job failed, autoflag: %d" % self.autoflag)
            self._pauselist()
            return
        
        self.autoflag = 4

    cmd_AUTO_STARTNEXT_help = "start next job in auto print job list"
    def cmd_AUTO_STARTNEXT(self, gcmd):
        if len(self.allJobs) == 0:
            self.gcode.respond_info("No jobs in the autoprint list")
            self._pauselist()
            return
        
        flag = gcmd.get_int('FLAG', default=-1, minval=-1, maxval=4)
        if( flag != -1 and flag != self.autoflag):
            self._loginfo("Not in the right state to start next job, autoflag: %d" % self.autoflag)
            self._pauselist()
            return
        
        if flag==-1: # force to resume the auto print list.
            self.pause = False

        if self.pause:
            self._loginfo("Auto print is paused, can't start next job")
            return

        job = self.allJobs[0]
        self._loginfo("Start next job, ", job)

        self.gcode.run_script_from_command( "SDCARD_PRINT_FILE FILENAME=%s" % job['filename'])
        self.autoflag = 1 # set to start state

    cmd_AUTO_LIST_help = "list all jobs info or Specified job info"
    def cmd_AUTO_LIST(self, gcmd):
        idx = gcmd.get_int('IDX', -1, minval=-1, maxval=len(self.allJobs)-1)
        if idx == -1:
            filename = gcmd.get('FILE', "")
            for i, job in enumerate(self.allJobs):
                if job['filename'] == filename:
                    idx = i
                    break
        
        if idx>=0 and idx<len(self.allJobs):
            self._loginfo("Job %d: " % idx, self.allJobs[idx])
        else:
            self._logalljobs()

    # 添加帮助信息
    cmd_AUTO_PAUSE_help = "Pause/resume the auto print process"
    # 实现 AUTO_PAUSE 命令
    def cmd_AUTO_PAUSE(self, gcmd):
        bPause = gcmd.get_int('PAUSE', 0 if self.pause else 1, minval=0, maxval=1)
        self.pause = not not bPause
        if self.pause:
            self._loginfo("Auto print paused")
        else:
            self._loginfo("Auto print resumed")

    def get_status(self, eventtime):
        return {'autoprint_jobs': self.allJobs}
    
def load_config(config):
    return AutoPrint(config)
