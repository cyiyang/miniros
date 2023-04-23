## 第二十五届中国机器人及人工智能大赛智慧药房组
ROS 破车的代码仓库
# 起点
* 发车点车辆摆放遵循以下的要求
  * 上边沿与绿色线条对齐
![上边沿对齐](./picture/Start_Point1.JPG)
  * 左边沿与点的右竖杠对齐
![左边沿对齐](./picture/Start_Point2.JPG)


导航时，取消当前目标点或者暂停导航是一个常见的问题
经过查阅资料．发现仅用如下一条指令即可另导航停止，小车停止:

rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}

调参工具
rosrun rqt_reconfigure rqt_reconfigure

杀死所有节点 

解决gazebo服务死亡的问题
killall gzserver