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
        self.maxJobs = config.getint('maxJobs', 20, minval=1, maxval=100)
        self.allJobs = []
        try:
            if not os.path.exists(self.filename):
                open(self.filename, "w").close()
            self.loadJobs()
        except self.printer.command_error as e:
            raise config.error(str(e))
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('AUTO_ADDJOB', self.cmd_AUTO_ADDJOB,
                               desc=self.cmd_AUTO_ADDJOB_help)
        gcode.register_command('AUTO_ADDJOB', self.cmd_AUTO_DELJOB,
                               desc=self.cmd_AUTO_DELJOB_help)
        gcode.register_command('AUTO_FINISHJOB', self.cmd_AUTO_FINISHJOB,
                               desc=self.cmd_AUTO_FINISHJOB_help)
        gcode.register_command('AUTO_STARTNEXT', self.cmd_AUTO_STARTNEXT,
                               desc=self.cmd_AUTO_STARTNEXT_help)
    def loadJobs(self):
        alljobs = []
        jobfile = configparser.ConfigParser()
        try:
            jobfile.read(self.filename)
            for i in range(self.maxJobs): # max jobs
                if jobfile.has_section('Job_%d' % i):
                    job = {}
                    for name, val in jobfile.items('Job%d' % i):
                        job[name] = ast.literal_eval(val)
                    # parse time from Date string
                    # job['Date'] = datetime.datetime.strptime(job['Date'], '%Y-%m-%d %H:%M:%S.%f')
                    alljobs[i] = job
                else:
                    break
        except:
            msg = "Unable to parse existing autoprint file: %s" % self.filename
            logging.exception(msg)
            raise self.printer.command_error(msg)
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

    cmd_AUTO_ADDJOB_help = "add a print job to auto print job list"
    def cmd_AUTO_ADDJOB(self, gcmd):
        if len(self.allJobs) >= self.maxJobs:
            gcode = self.printer.lookup_object('gcode')
            gcode.respond_error("Max jobs(%d) reached" % self.maxJobs)
            return
        
        # get new job's filename and print times, and current time as start time
        filename = gcmd.get('FILE')
        times = gcmd.get_int('TIMES', 1, minval=1, maxval=1000)
        # get current system time
        curTime = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        cnt = 0

        job = {'Filename': filename, 'Times': times, 'Date': curTime, 'Completed': cnt}
        self.allJobs.append(job)
        # save the jobs to file
        self.saveJobs()

    cmd_AUTO_DELJOB_help = "del a print job to auto print job list"
    def cmd_AUTO_DELJOB(self, gcmd):
        id = gcmd.get_int('ID', -1, minval=-1, maxval=len(self.allJobs)-1)
        if id == -1:
            filename = gcmd.get('FILE', "")
            for i, job in enumerate(self.allJobs):
                if job['Filename'] == filename:
                    id = i
                    break
        if id == -1:
            gcmd.respond_error("No such job")
            return
        
        cnt = gcmd.get_int('CNT', 1000, minval=0, maxval=1000)
        job = self.allJobs[id]
        job['Times'] -= cnt

        if job['completed'] >= job['Times']:
            self.allJobs.pop(id)
        
        # save the jobs to file
        self.saveJobs()


    cmd_AUTO_FINISHJOB_help = "finish a job and remove it from auto print job list"
    def cmd_AUTO_FINISHJOB(self, gcmd):

    cmd_AUTO_STARTNEXT_help = "start next job in auto print job list"
    def cmd_AUTO_STARTNEXT(self, gcmd):

    def get_status(self, eventtime):
        return {'autoprint_jobs': self.allJobs}

def load_config(config):
    return AutoPrint(config)
