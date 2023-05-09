#!/bin/bash
# 从小车同步到虚拟机的git仓库
# rsync -av --delete /home/$(whoami)/robot_ws/src/actuator/ /home/$(whoami)/drug-deliverer/actuator/
# rsync -av --delete /home/$(whoami)/robot_ws/src/deliver_scheduler/ /home/$(whoami)/drug-deliverer/deliver_scheduler/
# rsync -av --delete /home/$(whoami)/robot_ws/src/board_reminder/ /home/$(whoami)/drug-deliverer/board_reminder/

# 从小车的robot_ws同步到虚拟机的git仓库
# 判断网络连通性
ping -c 1 EPRobot > /dev/null

if [ $? -eq 0 ]; then
    rsync -av --delete EPRobot@EPRobot:/home/EPRobot/robot_ws/src/actuator/ /home/$(whoami)/drug-deliverer/actuator/
    rsync -av --delete EPRobot@EPRobot:/home/EPRobot/robot_ws/src/deliver_scheduler/ /home/$(whoami)/drug-deliverer/deliver_scheduler/
    rsync -av --delete EPRobot@EPRobot:/home/EPRobot/robot_ws/src/board_reminder/ /home/$(whoami)/drug-deliverer/board_reminder/
    rsync -av --delete EPRobot@EPRobot:/home/EPRobot/robot_ws/src/char_recognizer/ /home/$(whoami)/drug-deliverer/char_recognizer/
    rsync -av --delete EPRobot@EPRobot:/home/EPRobot/robot_ws/src/robot_navigation/param/ /home/$(whoami)/drug-deliverer/param/myparam/
    echo "成功同步小车工作空间的修改到本地drug-deliverer仓库"
else
    echo "无法连接到小车"
fi
