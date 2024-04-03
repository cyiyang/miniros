# 第二十五届中国机器人及人工智能大赛智慧药房组

ROS 破车的代码仓库  

庞大的ROS网络的rqt_graph存放在picture文件夹，实现了三车在一个ROS_CORE下的**多机协同**。
## 常用命令

* 调参工具
`rosrun rqt_reconfigure rqt_reconfigure`
* 杀死所有节点
`rosnode kill -a`
* 打开rviz观察路径
`roslaunch robot_navigation navigation_rviz.launch`
* 车不动调参
修改`move_base.launch`文件，将 `cmd_vel` 话题重命名（打开文件一看就懂）
* 观察系统结构
`rosrun rqt_graph rqt_graph`

## 需要启动的功能包 (Master)
 * `roslaunch robot_navigation robot_navigation.launch`
 * `roslaunch astra_camera astra.launch`    #使用原始的摄像头运行厂家节点
 * `roslaunch actuator car_master.launch `
 * `roslaunch char_recognizer char_recognizer.launch`         # 真图
 * `roslaunch char_recognizer char_recognizer_fake.launch`    # 假图
 * `roslaunch deliver_scheduler scheduler.launch`
 * Master 可独立工作，完成**配送、识别标准数字**，并且可以自动修改周期为小哥周期 2，药品周期 3.
## 需要启动的功能包 (Slave)
 * `roslaunch robot_navigation robot_navigation.launch`
 * `roslaunch actuator car_slave.launch `
 * Slave 需依赖 Master 与 Watcher，在 EveryoneStatus 中监听到相关状态，方开始工作。仅负责**配送**。
## 需要启动的功能包 (Watcher)
 * `roslaunch robot_navigation robot_navigation.launch`
 * `./launch_watcher.bash (会依次启动下面的 watcher 的 launch 和 YoloX) `
 * Watcher 需依赖 Master，在 EveryoneStatus 中监听到相关状态，方开始工作，到达识别区后不移动，进行YoloX的识别任务。负责**识别手写数字**。

## map参数
* 以 $114 \times 147 \mathrm{pixel}$ 的 fake_map（人为制作，不是 slam 建图）为基础，以左上角为 $(0,0)$ 的原点右 $x$ 正方向，下 $y$ 正方向的图片坐标系。
* 以 $114 \times 147 \mathrm{pixel}$ 的fake_map按照 $5\mathrm{cm}/$ 转化为物理坐标系，那么生成 $570\mathrm{cm} \times 735\mathrm{cm}$ 的物理扩大地图（左上为原点，$x,y$ 与图片坐标系一致）。
* 真实地图在扩大地图居中。
* ROS 栅格地图坐标系为 $x$ 右 $y$ 上。

## 完美收官捏

以下为国赛最终成绩

![这是国赛截图](/picture/final.png "国赛截图捏")

击败哈工深（狗头），捧杯力

![这是国赛公示截图](/picture/first_prize.png "捏") 
