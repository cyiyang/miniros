
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
    uint8_t goal_number = 4;

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal[4];

    goal[0].target_pose.pose.position.x = -2.04284858704;
    goal[0].target_pose.pose.position.y = 2.20917868614;
    goal[0].target_pose.pose.orientation.z = 0.573673982816;
    goal[0].target_pose.pose.orientation.w = 0.819083732863;

    goal[1].target_pose.pose.position.x = 0.00959491729736;
    goal[1].target_pose.pose.position.y = 0.260794043541;
    goal[1].target_pose.pose.orientation.z = 0.0227149893426;
    goal[1].target_pose.pose.orientation.w = 0.999741981343;

    goal[2].target_pose.pose.position.x = 0.359997034073;
    goal[2].target_pose.pose.position.y = 2.77852630615;
    goal[2].target_pose.pose.orientation.z = 0.756688701887;
    goal[2].target_pose.pose.orientation.w = 0.653775350129;

    goal[3].target_pose.pose.position.x = -1.13329148293;
    goal[3].target_pose.pose.position.y = 1.19635868073;
    goal[3].target_pose.pose.orientation.z = -0.619395798101;
    goal[3].target_pose.pose.orientation.w = 0.78507887839;

    ROS_INFO(" Init success!!! ");
    while (goal_number) // total is 4 goals
    {
        switch ((4 - goal_number))
        {
        case 0:
            goal[4 - goal_number].target_pose.header.frame_id = "map";
            goal[4 - goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[4 - goal_number]);
            ROS_INFO("Send NO. %d Goal !!!", 4 - goal_number);
            break;
        case 1:
            goal[4 - goal_number].target_pose.header.frame_id = "map";
            goal[4 - goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[4 - goal_number]);
            ROS_INFO("Send NO. %d Goal !!!", 4 - goal_number);
            break;
        case 2:
            goal[4 - goal_number].target_pose.header.frame_id = "map";
            goal[4 - goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[4 - goal_number]);
            ROS_INFO("Send NO. %d Goal !!!", 4 - goal_number);
            break;
        case 3:
            goal[4 - goal_number].target_pose.header.frame_id = "map";
            goal[4 - goal_number].target_pose.header.stamp = ros::Time::now();
            ac.sendGoal(goal[4 - goal_number]);
            ROS_INFO("Send NO. %d Goal !!!", 4 - goal_number);
            break;
        default:
            break;
        }
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The NO. %d Goal achieved success !!!", 4 - goal_number);
            goal_number--;
        }
        else
        {
            ROS_WARN("The NO. %d Goal Planning Failed for some reason", 4 - goal_number);
        }
    }
    return 0;
}