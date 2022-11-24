#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

void huskyposeCallback(const nav_msgs::Odometry::ConstPtr& msg){
    ROS_INFO("Frame ID - %s \n", msg->header.frame_id.c_str());
    ROS_INFO("Linear x: %f \n", msg->twist.twist.linear.x);
    ROS_INFO("Linear y: %f \n", msg->twist.twist.linear.y);
    ROS_INFO("Linear z: %f \n", msg->twist.twist.linear.z);

    ROS_INFO("Angular x: %f \n", msg->twist.twist.angular.x);
    ROS_INFO("Angular y: %f \n", msg->twist.twist.angular.y);
    ROS_INFO("Angular z: %f \n", msg->twist.twist.angular.z);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "odom_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/odometry/filtered", 1000, huskyposeCallback);
    ros::spin();
    return 0;
}