#! /usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler

from math import pi

quaternions = list()
euler_angles = (
    pi / 2,
    pi / 2,
    pi / 2,
    0,
    -pi / 2,
    -pi / 2,
    -pi / 2,
    -pi / 2,
    0,
    pi,
    pi / 2,
    -pi / 2,
)
# 将上面的Euler angles转换成Quaternion的格式
for angle in euler_angles:
    q_angle = quaternion_from_euler(0, 0, angle, axes="sxyz")
    q = Quaternion(*q_angle)
    quaternions.append(q)
point_ABC_master = [
    Pose(Point(0.42, 2.30, 0), quaternions[0]),  
    Pose(Point(1.30, 2.75, 0), quaternions[1]), 
    Pose(Point(1.26, 1.82, 0), quaternions[2]),
]  # 3
point_1234_master = [
    Pose(Point(0, 0, 0), quaternions[3]),  # 无意义
    Pose(Point(-1.84, 2.26, 0), quaternions[4]),     
    Pose(Point(-1.03, 1.78, 0), quaternions[5]), 
    Pose(Point(-1.85, 1.30, 0), quaternions[6]),  
    Pose(Point(-1.03, 0.89, 0), quaternions[7]),
]  # D
point_special_master = [
    Pose(Point(0, 0, 0), quaternions[8]),  # 起点
    Pose(Point(-0.4, 3.7, 0), quaternions[9]),  # 手写数字
    Pose(Point(0.885, 2.54, 0), quaternions[10]),    # 运动1点
    Pose(Point(-1.44, 2.035, 0), quaternions[11]),
]  # 运动2点

point_ABC_slave = [
    Pose(Point(2.639, 2.21, 0), quaternions[0]), 
    Pose(Point(2.639, 2.21, 0), quaternions[1]),  
    Pose(Point(2.639, 2.21, 0), quaternions[2]),
]  # 3
point_1234_slave = [
    Pose(Point(0, 0, 0), quaternions[3]),  # 无意义
    Pose(Point(0.38, 1.77, 0), quaternions[4]),    
    Pose(Point(0.38, 1.77, 0), quaternions[5]),       
    Pose(Point(0.38, 1.77, 0), quaternions[6]),  
    Pose(Point(0.38, 1.77, 0), quaternions[7]),
]  # D
point_special_slave = [
    Pose(Point(0, 0, 0), quaternions[8]),       # 起点
    Pose(Point(2.08, 3.8, 0), quaternions[9]),  # 手写数字
    Pose(Point(2.616, 0.83, 0), quaternions[10]),  #运动1点
    Pose(Point(0.35, 3.36, 0), quaternions[11]),
]  # 运动2点


point_special_watcher = [
    Pose(Point(1.4, 2.3, 0), quaternions[10]),
    Pose(Point(-0.75, 3.92, 0), quaternions[9]),
]  # 运动2点


