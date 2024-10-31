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
from functools import reduce
import asyncio

class RequestURL:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        self.isloginfo = config.getint('log', 0)  # 0: no log, 1: log to gcode, 2: log to file, 3: log to gcode and file
        
        # 从配置中读取远程Klipper实例的URL
        self.host = config.get('host', 'localhost')
        self.port = config.getint('port', 7126)
        self.timeout = config.getfloat('timeout', 0.5)
        self.url = config.get('url', None)
        self.method = config.get('method', 'GET')
        self.body = config.get('body', None)
        self.headers = config.get('headers', None)

        #设定用读取数据更新本地对象的变量
        self.update_object_name = []
        self.update_object = []
        self.update_object_vars = []
        self.remote_vars = []
        idx = ['0', '1', '2','3','4','5','6','7','8','9']
        for i in idx:
            obj = config.get('update_object'+i, None)
            if obj is None:
                break
            self.update_object_name.append(obj) #本地对象名
            vars = config.get('update_object_vars'+i, None)    #本地变量名,逗号分隔, pathlen,pathwidth,bedtemp,hotendtemp
            self.update_object_vars.append(vars.split(','))
            vars = config.get('remote_vars'+i, None) #远程变量名 result.status.pathlen, result.status.pathwidth, result.status.bedtemp, result.status.hotendtemp
            self.remote_vars.append(vars.split(','))
            self.update_object.append(None)

        self.repeat = config.getfloat('repeat', 0)  # 重复调用的时间，等于零就调用一次

        # 添加timer,循环执行request.
        self._request_status_timer = self.reactor.register_timer(self._call_request_status)

        # 设置 G 代码命令
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "HTTP_UPDATE_STATUS", "TYPE", self.name,
            self.cmd_HTTP_UPDATE_STATUS,
            desc=self.cmd_HTTP_UPDATE_STATUS_help
        )

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        self._loginfo(f"RequestURL {self.name} ready")
        for i in range(len(self.update_object_name)):
            self.update_object[i] = self.printer.lookup_object(self.update_object_name[i])
            if self.update_object[i] is None:
                raise self.printer.config_error("update_object not found: %s" % self.update_object_name[i])
            self._loginfo( "update object:%s by Request %s" % (self.update_object_name[i], self.name ) )
        self._loginfo(f"RequestURL {self.name} ready, update object: {self.update_object_name}")

    def _loginfo(self, msg, logflag=None):
        if logflag is None:
            logflag = self.isloginfo
        if logflag == 1:
            self.gcode.respond_info(msg, False) # only respond to gcode.
        elif logflag == 2:
           logging.info(msg) # only write log file.
        elif logflag == 3:
            self.gcode.respond_info(msg, True) # respond to gcode and write log file.

    def _call_request_status(self, eventtime):
        # repeat<0, 不执行request
        if self.repeat>=0 :
            data = self.HTTP_UPDATE_STATUS()
            if len(self.update_object) > 0:
                self._update_object(data['result']['status'])
        if self.repeat > 0:
            return max(eventtime + self.repeat, self.reactor.monotonic())
        else:
            return self.reactor.NEVER

    # eg: HTTP_UPDATE_STATUS TYPE=name
    cmd_HTTP_UPDATE_STATUS_help = "call http request to remote server"
    def cmd_HTTP_UPDATE_STATUS(self, gcmd):
        self.repeat = gcmd.get_float('REPEAT', self.repeat)
        self.isloginfo = gcmd.get_int('LOG', self.isloginfo)
        self.body = gcmd.get("BODY", self.body)
        self.reactor.update_timer(self._request_status_timer, self.reactor.NOW)

    def HTTP_UPDATE_STATUS(self):
        try:
            # 发起GET请求以查询远程Klipper实例状态
            conn = http.client.HTTPConnection(self.host, port=self.port, timeout=self.timeout)
            self._loginfo(f"Http connect to: {self.host, self.port, self.timeout}")
            conn.request(self.method, self.url, body=self.body, headers=json.loads(self.headers))
            self._loginfo("request OK")

            response = conn.getresponse()
            self._loginfo(f"request response status: {response.status}")
            
            # 读取响应数据并解析JSON
            data = response.read().decode("utf-8")
            json_data = json.loads(data)
            self._loginfo(f"request response: {json.dumps(json_data)}")
            conn.close()
            # status_data = json_data['result']['status']
            # self._loginfo(f"status: {json.dumps(status_data)}")
            return json_data

        except:
            self._loginfo(f"Failed to call request: {self.host, self.port, self.url, self.body, self.headers}", 3)
            return None
    
    def get_nested_value(self, data, path):
        try:
            return reduce(lambda d, key: d[key], path.split("."), data)
        except (KeyError, TypeError):
            return None
    
    def _update_object(self, data):
        self._loginfo(f"being update object: {self.update_object_name}, with data: {json.dumps(data)}")
        for i in range(len(self.update_object)):
            for j in range(len(self.update_object_vars[i])):
                value = self.get_nested_value(data, self.remote_vars[i][j])
                if value is not None:
                    setattr(self.update_object[i], self.update_object_vars[i][j], value)
                    self._loginfo(f"update {self.update_object_name[i]}: {self.update_object_vars[i][j]}={value}")
                else:
                    self._loginfo(f"Can't find {self.remote_vars[i][j]} in data")

def load_config(config):
    return RequestURL(config)

def load_config_prefix(config):
    return RequestURL(config)

# 在配置文件中添加对组件的加载
# [remote_klipper_query]
# remote_url: http://<REMOTE_KLIPPER_IP>:<REMOTE_KLIPPER_PORT>

# 使用 M118 G代码命令触发状态检查
# M118 QUERY_REMOTE_STATUS
