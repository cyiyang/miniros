#! /usr/bin/env python
# -*- coding: utf-8 -*- 

#用于模仿虚伪的语音识别器
#用于模仿mic的行为

import rospy
from watcher_basic.srv import VoiceJudgeMsg,VoiceJudgeMsgResponse
import random


def doPrint(request):
    
    response = VoiceJudgeMsgResponse()
    if (request.NeedToChange ==True):
        rospy.loginfo("接受到客户端的请求")
        response.success = random.randint(0,1)
        rospy.loginfo("睡一会模拟识别")
        rospy.sleep(5)
        if(response.success):
            rospy.loginfo("识别正确")
        else:
            rospy.loginfo("识别错误")
    else:
        rospy.loginfo("没有接受到客户端的请求")

    return response



if __name__ == "__main__":
    rospy.init_node("mic")  # 服务器节点起名为mic
    server = rospy.Service("voicejudge", VoiceJudgeMsg, doPrint)  # 话题名为voicejudge
    rospy.loginfo("服务器启动")
    rospy.spin()
    pass