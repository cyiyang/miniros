#!/bin/bash

# gnome-terminal -x bash -c "roscore; exec bash"
gnome-terminal --tab-with-profile=Default -- bash -c 'ssh EPRobot@EPRobot "roslaunch robot_navigation robot_navigation.launch; exec bash"'
gnome-terminal --tab-with-profile=Default -- bash -c 'ssh EPRobot@EPRobot "roslaunch actuator car_master.launch; exec bash"'
# gnome-terminal --tab -- ssh EPRobot@EPRobot 'roslaunch robot_navigation robot_navigation.launch'
# gnome-terminal --tab -- ssh EPRobot@EPRobot 'roslaunch actuator car_master.launch'
# gnome-terminal -x bash -c "ssh -t EPRobot@EPRobot 'roslaunch robot_navigation robot_navigation.launch;'; exec bash"
# gnome-terminal -x bash -c "ssh -t EPRobot@EPRobot 'roslaunch actuator car_master.launch;'; exec bash"
# gnome-terminal -x bash -c "ssh -t EPRobot@EPRobot 'roslaunch char_recognizer char_recognizer_fake.launch;'; exec bash"
# gnome-terminal -x bash -c "ssh -t EPRobot@EPRobot 'roslaunch deliver_scheduler scheduler.launch;'; exec bash"
