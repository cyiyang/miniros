#! /usr/bin/env python
# -*- coding: utf-8 -*-
#这是socket的服务端，应该放在从车（watcher）上
import rospy
import socket
import json
from watcher_basic.srv import VoiceJudgeMsg,VoiceJudgeMsgRequest,VoiceJudgeMsgResponse
import threading
from std_msgs.msg import Bool

def thread_arrived():
    stop_threads = False 
    while True:
        rospy.spin()#阻塞在这里了，无法回收



#运行在从车上的socket_server
class bridge_socket(object):
    def __init__(self) :
        rospy.init_node("bridge")
        self.arrived_sub = rospy.Subscriber("arrived",Bool,CarArrived,queue_size=10)
        self.arrived_flag = False #false没到，true到了
        add_thread = threading.Thread(target=thread_arrived)
        add_thread.start()
        rospy.loginfo("deal arrived thread OK")

        self.socket_server = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
        self.socket_server.bind(('0.0.0.0',12345))
        self.socket_server.listen(10)
        conn,ip_addr = self.socket_server.accept()
        while not rospy.is_shutdown():
            file_msg = conn.recv(1024) #如果客户端没有发送，就阻塞在这里
            msg_data = json.loads(file_msg)
            #在这里开始判断车到没到
            while not self.arrived_flag:
                rospy.loginfo("等待车辆到达")
                rospy.sleep(1)
                pass
            if(msg_data.get("NeedToChange") ==True):
                rospy.loginfo("接收到Need2Change") 
                file_msg = {"Success":True}
                conn.sendall(json.dumps(file_msg))  # 发送传输需要的数据
                rospy.loginfo("[Watcher]已经发送了OK")


        def CarArrived(pub_info):
            rospy.loginfo(pub_info)
            self.arrived_flag = True
            if (pub_info): #为真，车已经到了
                rospy.loginfo("bridge启动,车辆已到达")
                self.arrived_sub.unregister() #取消订阅
                rospy.loginfo("已经取消订阅")
                global stop_threads
                stop_threads=True
                rospy.loginfo("线程已回收")



if __name__ == "__main__":
    try:
       bridge_socket()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序意外退出")
        pass






