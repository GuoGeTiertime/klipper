# send http request to remote klipper instance, and get the status of remote klipper instance.
#
# Copyright (C) 2024 guoge
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import urllib.request
# import requests
import json
import logging

class RequestURL:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        
        # 从配置中读取远程Klipper实例的URL
        self.remote_url = config.get('remote_url', None)
        if not self.remote_url:
            raise self.config_error("Missing 'remote_url' in configuration")
        logging.info(f"Remote Klipper URL set to: {self.remote_url}")
        self.repeat = config.getfloat('repeat', 0)  # 重复调用的时间，等于零就调用一次

        # 添加timer,循环执行request.
        self._request_timer = self.reactor.register_timer(self._call_request)

        self.isloginfo = 1  # 0: no log, 1:gcode response, 2: write log file, 3: response and write log file 

        # 设置 G 代码命令
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "HTTP_REQUEST", "URL", self.name,
            self.cmd_HTTP_REQUEST,
            desc=self.cmd_HTTP_REQUEST_help
        )
    
    def _loginfo(self, msg, logflag=None):
        if logflag is None:
            logflag = self.isloginfo
        if logflag == 1:
            self.gcode.respond_info(msg, False) # only respond to gcode.
        elif logflag == 2:
           logging.info(msg) # only write log file.
        elif logflag == 3:
            self.gcode.respond_info(msg, True) # respond to gcode and write log file.

    def _call_request(self, eventtime):
        res = self._request()
        if self.repeat > 0:
            return eventtime + self.repeat
        else:
            return self.reactor.NEVER

    # eg: HTTP_REQUEST url=192.168.3.XX repeat=1.0
    cmd_HTTP_REQUEST_help = "call http request to remote server"
    def cmd_HTTP_REQUEST(self, gcmd):
        self.repeat = gcmd.get_float('REPEAT', self.repeat)
        self.isloginfo = gcmd.get_int('LOG', self.isloginfo)
        self.reactor.update_timer(self._request_timer, self.reactor.NOW)

    def _request(self):
        try:
            # 发起GET请求以查询远程Klipper实例状态
            req = urllib.request.Request(self.remote_url)
            self._loginfo("request OK", 3)
            resp = urllib.request.urlopen(req)
            status_data = resp.read().decode('utf-8')
            self._loginfo("request response: " + status_data, 3)
            print(status_data)
            status_json = json.loads(status_data)
            self._loginfo(f"request response: {json.dumps(status_json)}")
            # return status_json
            # x = requests.get('https://www.runoob.com/')
            # # 返回网页内容
            # print(x.text)
            return None
        except:
            self._loginfo(f"Failed to call request: {self.remote_url}")
            return None

def load_config(config):
    return RequestURL(config)

def load_config_prefix(config):
    return RequestURL(config)

# 在配置文件中添加对组件的加载
# [remote_klipper_query]
# remote_url: http://<REMOTE_KLIPPER_IP>:<REMOTE_KLIPPER_PORT>

# 使用 M118 G代码命令触发状态检查
# M118 QUERY_REMOTE_STATUS
