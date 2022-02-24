#include <ros/ros.h>
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/Map.h"

#include "fs_msgs/ControlCommand.h"
#include "nav_msgs/Odometry.h"
#include "fs_msgs/Track.h"


void controlCommand(const fsd_common_msgs::ControlCommand& msg)
{
    ROS_INFO("control command triggered");
}

void velocityEstimate(const fsd_common_msgs::CarStateDt& msg)
{
    ROS_INFO("velocity estimate triggered");
}

void state(const fsd_common_msgs::CarState& msg)
{
    ROS_INFO("state triggered");
}

void map(const fsd_common_msgs::Map& msg)
{
    ROS_INFO("map triggered");
}

void controlCommandAS(const fs_msgs::ControlCommand& msg)
{
    ROS_INFO("control command AS triggered");
}

void odom(const nav_msgs::Odometry& msg)
{
    ROS_INFO("odom AS triggered");
}

void track(const fs_msgs::Track& msg)
{
    ROS_INFO("track AS triggered");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wrapper");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber1 = nodeHandle.subscribe("/control/pure_pursuit/control_command", 10, controlCommand);
//     ros::Subscriber subscriber2 = nodeHandle.subscribe("/estimation/velocity_estimation/velocity_estimate", 10, velocityEstimate);
//     ros::Subscriber subscriber3 = nodeHandle.subscribe("/estimation/slam/state", 10, state);
//     ros::Subscriber subscriber4 = nodeHandle.subscribe("/estimation/slam/map", 10, map);
    
//     ros::Subscriber subscriber5 = nodeHandle.subscribe("/fsds/control_command", 10, controlCommandAS);
    ros::Subscriber subscriber6 = nodeHandle.subscribe("/fsds/testing_only/odom", 10, odom);
//     ros::Subscriber subscriber7 = nodeHandle.subscribe("/fsds/testing_only/track", 10, track);
    
//     ros::Publisher publisher1 = nodeHandle.advertise<fs_msgs::ControlCommand>("/fsds/control_command", 10);
//     ros::Publisher publisher2 = nodeHandle.advertise<nav_msgs::Odometry>("/fsds/testing_only/odom", 10);
//     ros::Publisher publisher3 = nodeHandle.advertise<nav_msgs::Odometry>("/fsds/testing_only/odom", 10);
//     ros::Publisher publisher4 = nodeHandle.advertise<fs_msgs::Track>("/fsds/testing_only/track", 10);
//     publisher1.publish(message);
    ros::spin();

    return 0;
}
