#include <ros/ros.h>
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/Map.h"

#include "fs_msgs/ControlCommand.h"
#include "nav_msgs/Odometry.h"
#include "fs_msgs/Track.h"
#include "std_msgs/Float32.h"

ros::Publisher controlCommandPublisher;
ros::Publisher velocityPublisher;
ros::Publisher statePublisher;
ros::Publisher mapPublisher;

void controlCommand(const fsd_common_msgs::ControlCommand& msg)
{
    fs_msgs::ControlCommand message;
    message.throttle = 5.0 * static_cast<double>(msg.throttle.data);
    message.steering = 1.745329 * msg.steering_angle.data;
    message.brake = 0;
    controlCommandPublisher.publish(message);
}

// void velocityEstimate(const fsd_common_msgs::CarStateDt& msg)
// {
//     ROS_INFO("velocity estimate triggered");
// }

// void state(const fsd_common_msgs::CarState& msg)
// {
//     ROS_INFO("state triggered");
// }

// void map(const fsd_common_msgs::Map& msg)
// {
//     ROS_INFO("map triggered");
// }

// void controlCommandAS(const fs_msgs::ControlCommand& msg)
// {
//     ROS_INFO("control command AS triggered");
// }

void odom(const nav_msgs::Odometry& msg)
{
    geometry_msgs::Pose2D position;
    position.x = msg.pose.pose.position.x;
    position.y = msg.pose.pose.position.y;
    position.theta = msg.pose.pose.orientation.z;

    geometry_msgs::Pose2D velocity;
    velocity.x = msg.twist.twist.linear.x;
    velocity.y = msg.twist.twist.linear.y;
    velocity.theta = msg.twist.twist.angular.z;

    fsd_common_msgs::CarStateDt velocityEstimate;
    velocityEstimate.car_state_dt = velocity;

    fsd_common_msgs::CarState state;
    state.car_state = position;
    state.car_state_dt = velocityEstimate;
    
    velocityPublisher.publish(velocityEstimate);
    statePublisher.publish(state);
}

void track(const fs_msgs::Track& msg)
{
    fsd_common_msgs::Map map;
    for(const fs_msgs::Cone cone : msg.track)
    {
        fsd_common_msgs::Cone newCone;
        newCone.position = cone.location;
        switch(cone.color)
        {
            case 0: // BLUE
                newCone.color.data = "BLUE";
                map.cone_blue.push_back(newCone);
            case 1: // YELLOW
                newCone.color.data = "YELLOW";
                map.cone_yellow.push_back(newCone);
            case 2: // ORANGE BIG
            case 3: // ORANGE SMALL
                newCone.color.data = "ORANGE";
                map.cone_orange.push_back(newCone);
            case 4: // UNKNOWN
                ROS_INFO("UNKNOWN CONE COLOR");
            default:
                ROS_INFO("INVALID CONE COLOR");
        }
    }
    mapPublisher.publish(map);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wrapper");
    ros::NodeHandle nodeHandle;
    ros::Subscriber controlCommandSubscriber = nodeHandle.subscribe("/control/pure_pursuit/control_command", 10, controlCommand);
//     ros::Subscriber subscriber2 = nodeHandle.subscribe("/estimation/velocity_estimation/velocity_estimate", 10, velocityEstimate);
//     ros::Subscriber subscriber3 = nodeHandle.subscribe("/estimation/slam/state", 10, state);
//     ros::Subscriber subscriber4 = nodeHandle.subscribe("/estimation/slam/map", 10, map);
    
//     ros::Subscriber subscriber5 = nodeHandle.subscribe("/fsds/control_command", 10, controlCommandAS);
    ros::Subscriber odomSubscriber = nodeHandle.subscribe("/fsds/testing_only/odom", 10, odom);
    ros::Subscriber trackSubscriber = nodeHandle.subscribe("/fsds/testing_only/track", 10, track);
    
    controlCommandPublisher = nodeHandle.advertise<fs_msgs::ControlCommand>("/fsds/control_command", 10);
    velocityPublisher = nodeHandle.advertise<fsd_common_msgs::CarStateDt>("/estimation/velocity_estimation/velocity_estimate", 10);
    statePublisher = nodeHandle.advertise<fsd_common_msgs::CarState>("/estimation/slam/state", 10);
    mapPublisher = nodeHandle.advertise<fsd_common_msgs::Map>("/estimation/slam/map", 10);
    
//     ros::Publisher publisher3 = nodeHandle.advertise<nav_msgs::Odometry>("/fsds/testing_only/odom", 10);
//     ros::Publisher publisher4 = nodeHandle.advertise<fs_msgs::Track>("/fsds/testing_only/track", 10);
//     publisher1.publish(message);
    ros::spin();

    return 0;
}
