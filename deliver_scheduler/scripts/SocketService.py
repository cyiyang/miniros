#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import socket
import time


class SocketServiceMaster(object):
    def __init__(
        self,
        slave_addr,
        slave_port,
        slave_ready_port,
        use_full_socket=False,
        time_to_return=5,
    ):
        self.slave_addr = slave_addr

        # slave_port 目前没有作用
        self.slave_port = slave_port
        self.socket = None

        # slave 向 sch@master 汇报到达手写数字点的端口
        self.slave_ready_port = slave_ready_port

        # 如果不采用socket, wait_for_service 将立即返回, call 方法将等待 time_to_return 秒后返回
        self.use_full_socket = use_full_socket
        self.time_to_return = time_to_return

    def wait_for_service(self):
        if self.use_full_socket:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.slave_addr, self.slave_port))

        # 等待副车上线并到达手写数字点

        # 创建套接字对象
        slave_ready_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (self.slave_addr, self.slave_ready_port)
        print("等待连接到slave...")
        slave_ready_socket.connect(server_address)
        print("成功连接到slave!")


        # 接收数据
        print("正在接收数据...")
        received_data = slave_ready_socket.recv(1024)

        # 解析JSON数据
        json_data = received_data.decode("utf-8")
        data = json.loads(json_data)

        # 获取 slave_ready 的值
        self.slave_ready = data.get("slave_ready")

        # 打印 slave_ready 的值
        print("slave_ready:", self.slave_ready)

        # 关闭连接
        slave_ready_socket.close()

    def call(self, need=0):
        if not self.slave_ready:
            raise RuntimeError("Slave未到达手写数字点")

        if not self.use_full_socket:
            time.sleep(self.time_to_return)
            return True

        # 使用socket
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
