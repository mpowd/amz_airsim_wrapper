#include <ros/ros.h>
#include "fsd_common_msgs/ControlCommand"

void controlCommand(const fsd_common_msgs/ControlCommand& msg)
{
    ROS_INFO("control command triggered");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wrapper");
    ros::NodeHandle nodeHandle;
    
    ros::Subscriber subscriber = nodeHandle.subscribe("/control/pure_pursuit/control_command", 10, callback_function);
    
    return0;
}
