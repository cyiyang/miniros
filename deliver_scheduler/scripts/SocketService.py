#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import json


class SocketServiceMaster(object):
    def __init__(self, slave_addr, slave_port):
        self.slave_addr = slave_addr
        self.slave_port = slave_port
        self.socket = None

    def wait_for_service(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.slave_addr, self.slave_port))

    def call(self, need=0):
        if not self.socket:
            raise RuntimeError("Service not connected. Call 'wait_for_service' first.")

        # 发送带有"NeedToChange: True"的JSON字符串
        request = {"NeedToChange": True, "need": need}
        request_str = json.dumps(request)

        self.socket.sendall(request_str)

        # 等待从节点返回响应
        response_str = self.socket.recv(4096)

        # 解析响应JSON字符串
        response = json.loads(response_str)

        # 检查是否成功
        if "Success" in response and response["Success"] == True:
            return True
        else:
            return False
