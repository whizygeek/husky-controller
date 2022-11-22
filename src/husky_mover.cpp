#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher velocity_publisher;

void move (double speed, double distance, bool isForward);

using namespace std;

int main (int argc, char **argv)
{
    ros::init(argc, argv,"robot_mover");
    ros::NodeHandle n;
    
    double speed;
    double distance;
    bool isForward;

                                                        //name of the topic is here
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel",10);
    
    cout << "enter speed!";
    cin >> speed;
    cout << "enter distance";
    cin >> distance;
    cout << "forward?";
    cin >> isForward;
    move(speed,distance,isForward);
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



