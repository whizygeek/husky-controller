#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

ros::Publisher velocity_publisher;

void move (double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);

double degrees2radians(double angle_in_degrees);

const double PI = 3.14159265359;

using namespace std;

int main (int argc, char **argv)
{
    ros::init(argc, argv,"husky_mover");
    ros::NodeHandle n;
    
    double speed,distance;
    bool isForward;
    double angular_speed,angle;
    bool clockwise;

                                                        //name of the topic is here
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel",10);
    
    cout << "enter speed!";
    cin >> speed;
    cout << "enter distance";
    cin >> distance;
    cout << "forward?";
    cin >> isForward;
    move(speed,distance,isForward);

    cout<<"enter angular velocity (degrees/sec) - ";
    cin>>angular_speed;
    cout<<"enter angle (degrees) - ";
    cin>>angle;
    cout<<"clockwise? - ";
    cin>>clockwise;
    rotate(degrees2radians(angular_speed), degrees2radians(angle),clockwise);

    ros::spin();

    return 0;
}

//makes the robot to move with a certain linear velocity for a
//certain distance in a forward or a backward direction.

void move(double speed, double distance, bool isForward){
    geometry_msgs::Twist vel_msg; 

    // distance = speed * time

    //set linear velocity in x-axis
    if(isForward)
        vel_msg.linear.x = abs(speed);
    else
        vel_msg.linear.x = -abs(speed);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    //set angulat velocity
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    //t0: current time
    double t0 = ros::Time::now().toSec();
    double current_distance = 0;
    ros::Rate loop_rate(10);

    //loop
    do
    {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_distance < distance);
    
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}


void rotate(double angular_speed, double relative_angle, bool clockwise){
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x=0;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;

    vel_msg.angular.x=0;
    vel_msg.angular.y=0;

    if(clockwise)
        vel_msg.angular.z = -abs(angular_speed);
    else
        vel_msg.angular.z = abs(angular_speed);

    double current_angle = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);

    do{
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while (current_angle<relative_angle);   

    vel_msg.angular.z=0;
    velocity_publisher.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees){
    return angle_in_degrees*PI/180.0;
}




/*
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "stdlib.h"

int main (int argc, char **argv)
{
    ros::init(argc, argv,"husky_mover");
    ros::NodeHandle n;
    
                                                            //name of the topic is here
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel ",10);
 
    
    while(ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear.x=10.0;
    msg.linear.y=1.0;
    msg.angular.z=1.0;
    velocity_publisher.publish(msg);
    ros::Duration(1.0).sleep();
    }
}
*/