#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import glob
import cv2
import numpy as np
import imutils
import cv_bridge
import random
import sys
from sensor_msgs.msg import Image
from char_recognizer.srv import (
    NeedToSeeMsg,
    NeedToSeeMsgResponse,
    DestinationMsg,
    PermissionMsg,
)


class ActualCharRecognizer:
    def __init__(self, tmpl_path):
        """
        :funciton: recognizerChar实例化
        :param tmpl_path: 字母库在系统中的绝对路径, 末尾不含斜杠
        """
        self.charTmplLib = []
        for char in ("A", "B", "C"):
            path = glob.glob("%s/%s/*.jpg" % (tmpl_path, char))
            pathAndChar = list(zip(path, [char] * len(path)))
            self.charTmplLib.extend(pathAndChar)

    def vertexFind(self, contours):
        """
        :function: vertexFind寻找顶点(输入近似矩形的轮廓, 返回它的四个顶点, 代替原cv2.approxPolyDP())
        :param contours: 轮廓
        :return: 四个顶点(list, array(4,1,2))
        """
        cnt_reshaped = contours.reshape(contours.shape[0], 2)
        vertexes = [0 for _ in range(4)]
        sumYX = cnt_reshaped.sum(axis=1)
        vertexes[0] = cnt_reshaped[sumYX.argmin()]  # 上左
        vertexes[2] = cnt_reshaped[sumYX.argmax()]  # 下右
        diffYX = np.diff(cnt_reshaped, axis=1)
        vertexes[3] = cnt_reshaped[diffYX.argmin()]  # 上右
        vertexes[1] = cnt_reshaped[diffYX.argmax()]  # 下左
        # 顶点顺序: 上左, 下左, 下右, 上右(逆时针)
        return np.array(vertexes).reshape(4, 1, 2)

    def perspTrans(self, image, approx, size):
        """
        :function: perspectiveTransform透视变换
        :param image: 待变换原图Error: perspTrans() takes 3 positional arguments but 4 were given\n']

        :param approx: 原图中边界坐标(array(4,1,2))
        :param size: 新图尺寸(list, [width, height])
        :return: 变换后的新图(BGR)
        """
        # 读入数据: pts: 原图中边界四点坐标, rect: 按顺序排列四点坐标
        pts = approx.reshape(4, 2)  # array(4,1,2) --> array(4,2)
        rect = np.zeros((4, 2), dtype="float32")
        # 四点坐标排序: rect[上左, 上右, 下右, 下左]
        sumYX = pts.sum(axis=1)
        rect[0] = pts[sumYX.argmin()]  # 上左
        rect[2] = pts[sumYX.argmax()]  # 下右
        diffYX = np.diff(pts, axis=1)
        rect[1] = pts[diffYX.argmin()]  # 上右
        rect[3] = pts[diffYX.argmax()]  # 下左
        # 确定变换后字母框的四点坐标
        dst = np.array(
            [[0, 0], [size[0] - 1, 0], [size[0] - 1, size[1] - 1], [0, size[1] - 1]],
            dtype="float32",
        )
        # 透视变换: ptMatrix: 变换矩阵, warped: 变换后图片
        ptMatrix = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, ptMatrix, (size[0], size[1]))
        return warped

    def cntsDetect(self, image):
        """
        :function: contoursDetect轮廓检测
        :param image: 待检测原图
        :return: 4个字母框的近似矩形轮廓在原图中的坐标(list, array(4,1,2), len=4)
        """
        # Step1: 边缘检测
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_gray = cv2.GaussianBlur(image_gray, (5, 5), 0)  # [参数!] 高斯模糊: 滤除噪声
        image_edged = cv2.Canny(image_gray, 80, 200)  # [参数!] 边缘检测(两个阈值越大, 保留的轮廓越少)
        # Step2: 轮廓提取(目标: 识别板内四个字母框的轮廓, 思路: 先提取面积最大轮廓, 若长宽比近似于1则认为面积前四即字母框, 否则认为是识别板外轮廓)
        found = 0
        image_forCnt = image_edged.copy()  # 用于提取轮廓的edged图
        self.image_forChar = image.copy()  # 用于抠出字母框作匹配的color图
        while not found:
            temp, cnts, hier = cv2.findContours(
                image_forCnt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)  # 取面积最大轮廓
            approx = self.vertexFind(cnts[0])
            aprx = approx.reshape(4, 2)
            widthT = np.sqrt(
                (aprx[0][0] - aprx[3][0]) ** 2 + (aprx[0][1] - aprx[3][1]) ** 2
            )
            widthB = np.sqrt(
                (aprx[1][0] - aprx[2][0]) ** 2 + (aprx[1][1] - aprx[2][1]) ** 2
            )
            heightL = np.sqrt(
                (aprx[0][0] - aprx[1][0]) ** 2 + (aprx[0][1] - aprx[1][1]) ** 2
            )
            heightR = np.sqrt(
                (aprx[2][0] - aprx[3][0]) ** 2 + (aprx[2][1] - aprx[3][1]) ** 2
            )
            width = max(widthT, widthB)
            height = max(heightL, heightR)
            if (
                width < height
            ):  # 理论上width > height, 但实际上由于四点顺序不同可能有width < height出现, 此时交换二者
                width, height = height, width
            r = width / height
            if r <= 1.2:
                found = 1
            else:
                image_forCnt = self.perspTrans(
                    image_forCnt, approx, [int(630 * r), 630]
                )
                self.image_forChar = self.perspTrans(
                    self.image_forChar, approx, [int(630 * r), 630]
                )  # 确保image_forCnt和image_forChar的内容保持同步(仅色彩不同)
        aprxes = []
        for i in range(4):
            approxI = self.vertexFind(cnts[i])
            aprxes.append(approxI)
        return aprxes

    def charDetect(self, aprxes):
        """
        :function: charDetect字母检测
        :param aprxes: 4个字母框的近似矩形轮廓在原图中的坐标(list, array(4,1,2), len=4)
        :return: 纠正后的4个字母框图片(list, len=4)
        """
        aprxes_tl = np.array([aprxes[i][0][0] for i in range(4)])
        aprxes_ordered = [0 for _ in range(4)]
        sumYX = aprxes_tl.sum(axis=1)
        aprxes_ordered[0] = aprxes[sumYX.argmin()]  # 上左(1)
        aprxes_ordered[3] = aprxes[sumYX.argmax()]  # 下右(4)
        diffYX = np.diff(aprxes_tl, axis=1)
        aprxes_ordered[1] = aprxes[diffYX.argmin()]  # 上右(2)
        aprxes_ordered[2] = aprxes[diffYX.argmax()]  # 下左(3)
        charImages = []
        for aprx in aprxes_ordered:
            charImage = self.perspTrans(self.image_forChar, aprx, [110, 110])
            charImages.append(charImage)
        return charImages

    def tmplMatch(self, charImage, threshold):
        """
        :function: templateMatch模板匹配
        :param charImage: 待识别字母图片(经过透视变换)
        :param threshold: 识别阈值(一旦与某个模板的匹配度超过该值, 即返回该模板对应字母)
        :return: 识别结果(字母)
        """
        image_gray = cv2.cvtColor(charImage, cv2.COLOR_BGR2GRAY)
        black_ratio = float((image_gray < 128).sum()) / image_gray.size
        if black_ratio < 0.17:
            return " "
        for tmpl_char in self.charTmplLib:
            tmpl = cv2.imread(tmpl_char[0])
            tmpl_gray = cv2.cvtColor(tmpl, cv2.COLOR_BGR2GRAY)
            # 放大待识别图片image
            for scale in np.linspace(1.0, 1.3, 30):
                image_resized = imutils.resize(
                    image_gray, width=int(image_gray.shape[1] * scale)
                )
                tmplRes = cv2.matchTemplate(
                    image_resized, tmpl_gray, cv2.TM_CCOEFF_NORMED
                )
                minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(tmplRes)
                if maxVal >= threshold:
                    return tmpl_char[1]
            # 缩小模板图片tmpl
            for scale in np.linspace(0.7, 1.0, 30):
                tmpl_resized = imutils.resize(
                    tmpl_gray, width=int(tmpl_gray.shape[1] * scale)
                )
                tmplRes = cv2.matchTemplate(
                    image_resized, tmpl_resized, cv2.TM_CCOEFF_NORMED
                )
                minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(tmplRes)
                if maxVal >= threshold:
                    return tmpl_char[1]
        return " "

    def recognize(self, image):
        """
        :function: recognize识别(主函数)
        :param image: 待识别目标板原图
        :return: 识别结果(字母)(list, [char1, char2, char3, char4])
        """
        charAprxes = self.cntsDetect(image)
        charImages = self.charDetect(charAprxes)
        charResult = ["", "", "", ""]
        for i in range(4):
            char_single = self.tmplMatch(charImages[i], 0.7)  # [参数!]
            charResult[i] = char_single
        return charResult


class CharRecognizer:
    def __init__(self, use_fake_img):
        self.use_fake_img = use_fake_img  # 是否使用假图的参数
        rospy.init_node("char_recognizer")
        self.bridge = cv_bridge.CvBridge()  # 创建CV桥
        self.actRecognizer = ActualCharRecognizer(
            "/home/EPRobot/robot_ws/src/char_recognizer/template"
        )  # 创建字母识别器(实际)
        self.scheduler_client = rospy.ServiceProxy(
            "mission", DestinationMsg
        )  # 创建调度器请求客户
        self.actuator_client = rospy.ServiceProxy(
            "permission", PermissionMsg
        )  # 创建执行器请求客户
        self.board_reminder_server = rospy.Service(
            "board_reminder_server", NeedToSeeMsg, self.ReminderHandler
        )  # 创建目标板提示服务器
        rospy.loginfo("[recognizer]recognizer_server正常启动")
        rospy.wait_for_service("mission")
        rospy.loginfo("[recognizer]连接schduler_server成功")
        rospy.wait_for_service("permission")
        rospy.loginfo("[recognizer]连接actuator_server成功")
        rospy.spin()

    def ReminderHandler(self, req):
        rospy.loginfo("[recognizer]收到req: %d" % req.need_to_see)
        if req.need_to_see:
            can_see = 0  # 默认“不能看”
            while not can_see:
                can_see = self.actuator_client.call(0).permission  # 向actuator请求“想看”
                rospy.loginfo("[recognizer]已向actuator请求“想看”")
                rospy.sleep(2)
            rospy.loginfo("[recognizer]收到actuator回复“能看”")
            self.RecognizeHandler()  # “能看”以后开始识别
        return NeedToSeeMsgResponse(True)

    def RecognizeHandler(self):
        rospy.loginfo("[recognizer]开始识别...")
        # Step1. 获取目标板照片
        if not self.use_fake_img:
            temp_sub = rospy.Subscriber("/camera/rgb/image_raw", Image)  # 创建临时的照片订阅
            rospy.sleep(2)  # 延时2s, 等待相机自动曝光
            board_image = rospy.wait_for_message(
                "/camera/rgb/image_raw", Image
            )  # 订阅一次照片
            temp_sub.unregister()  # 获取到照片后, 取消临时订阅
            board_image = self.bridge.imgmsg_to_cv2(board_image, "bgr8")
            image_name = (
                "/home/EPRobot/robot_ws/src/char_recognizer/board1_image/board1_%s.jpg"
                % str(random.randint(10000, 99999))
            )
            cv2.imwrite(image_name, board_image)
        else:
            board_image = cv2.imread(
                "/home/EPRobot/robot_ws/src/char_recognizer/board1_AAAA.jpg"
            )
        board_image = imutils.resize(board_image, width=1000)
        # Step2. 识别目标板字母
        charResult = self.actRecognizer.recognize(board_image)
        rospy.loginfo(
            "[recognizer]识别成功, 结果: 1[%c] 2[%c] 3[%c] 4[%c]"
            % (charResult[0], charResult[1], charResult[2], charResult[3])
        )
        self.actuator_client.call(1)  # 向actuator请求“看完了”
        rospy.loginfo("[recognizer]已向actuator请求“看完了”")
        # Step3. 向scheduler请求新的配送需求(作为client)
        drugTypeToInt = {"A": 0, "B": 1, "C": 2}
        for i in range(4):
            if charResult[i] != " ":
                self.scheduler_client.call(
                    0, 0, drugTypeToInt[charResult[i]], i + 1
                )  # (i+1): 目的地编号为1~4
                rospy.loginfo(
                    "[recognizer]已向scheduler请求新配送需求: 药品[%c] --> 窗口[%d]"
                    % (charResult[i], i + 1)
                )


if __name__ == "__main__":
    # 读入是否使用假图的参数
    myargv = rospy.myargv(argv=sys.argv)
    if "--use_fake_img" in myargv:
        use_fake_img = int(myargv.index("--use_fake_img") + 1)
    else:
        use_fake_img = 0
    charRecognizer = CharRecognizer(use_fake_img)
