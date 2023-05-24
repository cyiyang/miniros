#! /usr/bin/env python
# -*- coding: utf-8 -*-
#这是socket的服务端，应该放在从车（watcher）上
import rospy
import socket
import json
from watcher_basic.srv import VoiceJudgeMsg,VoiceJudgeMsgRequest,VoiceJudgeMsgResponse

#运行在从车上的socket_server
class bridge_socket(object):
    def __init__(self) :
        rospy.init_node("bridge")
        self.voicejudge_client = rospy.ServiceProxy("voicejudge", VoiceJudgeMsg)
        self.voicejudge_client.wait_for_service()
        rospy.loginfo("连上voicejudge 服务器了")
        self.socket_server = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
        self.socket_server.bind(('0.0.0.0',12345))
        self.socket_server.listen(10)
        conn,ip_addr = self.socket_server.accept()
        while not rospy.is_shutdown():
            file_msg = conn.recv(1024) #如果客户端没有发送，就阻塞在这里
            msg_data = json.loads(file_msg)
            if(msg_data.get("NeedToChange") ==True):
                rospy.loginfo("接收到Need2Change")
                self.voicejudge_respon=self.voicejudge_client.call(True)              
                if(self.voicejudge_respon.success): #成功
                    file_msg = {"Success":True}
                    conn.sendall(json.dumps(file_msg))  # 发送传输需要的数据
                    rospy.loginfo("[Watcher]已经发送了OK")
                else: #失败             
                    file_msg = {"Success":False}
                    conn.sendall(json.dumps(file_msg))  # 发送传输需要的数据
                    rospy.loginfo("[Watcher]已经发送了False")



if __name__ == "__main__":
    try:
       bridge_socket()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序意外退出")
        pass






