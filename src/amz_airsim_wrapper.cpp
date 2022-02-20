#include <ros/ros.h>
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/Map.h"


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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wrapper");
    ros::NodeHandle nodeHandle;
    
    ros::Subscriber subscriber1 = nodeHandle.subscribe("/control/pure_pursuit/control_command", 10, controlCommand);
    ros::Subscriber subscriber2 = nodeHandle.subscribe("/estimation/velocity_estimation/velocity_estimate", 10, velocityEstimate);
    ros::Subscriber subscriber3 = nodeHandle.subscribe("/estimation/slam/state", 10, state);
    ros::Subscriber subscriber4 = nodeHandle.subscribe("/estimation/slam/map", 10, map);
    
    return 0;
}
