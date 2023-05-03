#!/bin/bash
#启动move_base
gnome-terminal -t "move_base" -x bash -c "roslaunch robot_navigation robot_navigation.launch;exec bash;"
sleep 5s
#启动actuator
gnome-terminal -t "actuator" -x bash -c "roslaunch actuator car.launch;exec bash;"