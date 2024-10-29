# send http request to remote klipper instance, and get the status of remote klipper instance.
#
# Copyright (C) 2024 guoge
#
# This file may be distributed under the terms of the GNU GPLv3 license.
# import urllib.request
# import requests klipper没有此lib
import json
import logging
import http.client
import ast

class RequestURL:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        
        # 从配置中读取远程Klipper实例的URL
        self.host = config.get('host', 'localhost')
        self.port = config.getint('port', 7126)
        self.timeout = config.getfloat('timeout', 0.5)
        self.url = config.get('url', None)
        self.method = config.get('method', 'GET')
        self.body = config.get('body', None)
        self.headers = config.get('headers', None)

        self.repeat = config.getfloat('repeat', 0)  # 重复调用的时间，等于零就调用一次

        # 添加timer,循环执行request.
        self._request_timer = self.reactor.register_timer(self._call_request)

        self.isloginfo = 1  # 0: no log, 1:gcode response, 2: write log file, 3: response and write log file 

        # 设置 G 代码命令
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "HTTP_REQUEST", "TYPE", self.name,
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
        # repeat<0, 不执行request
        if self.repeat>=0 :
            self._request()
        if self.repeat > 0:
            return max(eventtime + self.repeat, self.reactor.monotonic())
        else:
            return self.reactor.NEVER

    # eg: HTTP_REQUEST url=192.168.3.XX repeat=1.0
    cmd_HTTP_REQUEST_help = "call http request to remote server"
    def cmd_HTTP_REQUEST(self, gcmd):
        self.repeat = gcmd.get_float('REPEAT', self.repeat)
        self.isloginfo = gcmd.get_int('LOG', self.isloginfo)
        self.reactor.update_timer(self._request_timer, self.reactor.NOW)

    def _request(self):
        # try:
            # 发起GET请求以查询远程Klipper实例状态
            conn = http.client.HTTPConnection(self.host, port=self.port, timeout=self.timeout)
            self._loginfo(f"Http connect to: {self.host, self.port, self.timeout}")
            body = json.dumps(self.body)
            # conn.request( method=self.method, url=self.url, body=self.body, headers=self.headers)
            bodystr = json.dumps({"script": "BEEP P=1000"})
            headstr = json.dumps({"Content-Type": "application/json"})
            # conn.request('POST', url=self.url, body=bodystr, headers=self.headers)
            # conn.request('POST', url=self.url, body=bodystr, headers=headstr)
            
            headers = { "Content-Type": "application/json" }
            tstr = str(type(headers))
            self._loginfo(f"request headers1: {headers}, type: {tstr}")
            self._loginfo("type of request headers1: %s" % (str(type(headers)),))
            self._loginfo(f"parameter of request headers: {self.headers}")
            # headers = json.loads(self.headers)
            # self.headers = '{ "Content-Type": "application/json" }'
            # self.headers = '{"Content-Type": "application/json"}'
            # self._loginfo(f"parameter of request headers replace by string: {self.headers}")
            headers = ast.literal_eval(self.headers)
            self._loginfo("type of SELF. headers: %s" % (str(type(self.headers)),))
            self._loginfo("type of request headers2: %s" % (str(type(headers)),))
            self._loginfo(f"request headers2: {headers}, type: {type(headers)}")
            # 构建请求体
            payload = json.dumps({"script": "BEEP P=1000"})
            conn.request("POST", self.url, body=payload, headers=headers)
            # conn.request("POST", self.url, body=payload, headers=self.headers)

            self._loginfo("request OK")
            if self.method == 'POST':
                return None
            
            response = conn.getresponse()
            self._loginfo(f"request response: {response.status}")
            
            # 读取响应数据并解析JSON
            data = response.read().decode("utf-8")
            json_data = json.loads(data)
            print(json_data)
            conn.close()
            self._loginfo(f"request response: {json.dumps(json_data)}")
            return json_data

        # except:
        #     self._loginfo(f"Failed to call request: {self.host, self.port, self.url}")
        #     return None

def load_config(config):
    return RequestURL(config)

def load_config_prefix(config):
    return RequestURL(config)

# 在配置文件中添加对组件的加载
# [remote_klipper_query]
# remote_url: http://<REMOTE_KLIPPER_IP>:<REMOTE_KLIPPER_PORT>

# 使用 M118 G代码命令触发状态检查
# M118 QUERY_REMOTE_STATUS
