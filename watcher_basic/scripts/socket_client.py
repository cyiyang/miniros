#! /usr/bin/env python
# -*- coding: utf-8 -*-
#这是socket的客户端，应该放在主车上

import socket
import json
import time
# 创建套接字对象，AF_INET基于TCP/UDP通信，SOCK_STREAM以数据流的形式传输数据，这里就可以确定是TCP了
client = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
# 连接服务端,ip为服务器的ip，
client.connect(('192.168.137.172',12345))
count=0 
while True:
    file_msg_send = {"NeedToChange":True}  # 构建一个json文件
    file_msg_send = json.dumps(file_msg_send)
    client.send(file_msg_send) 
    print("Send done") 
    file_msg_rec = client.recv(1024)
    msg_data = json.loads(file_msg_rec)
    print(msg_data)
