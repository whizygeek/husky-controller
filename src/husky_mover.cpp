#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
nav_msgs::Odometry husky_pose;


void move (double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);

double degrees2radians(double angle_in_degrees);
double setDesiredOrientation(double desired_angle_radians);

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message);

void moveGoal(nav_msgs::Odometry goal_pose, double distance_tolerence);

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

                                                        
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel",100);
    pose_subscriber = n.subscribe("/odometry/filtered",10,poseCallback);
    
    
    /*

    -------------------------You can use either (v1.0 + v1.2) or v1.3 --------------------

    UNCOMMENT THE BELOW COMMANDS TO TRY v1.0, AND v1.2. 
    
    --------------------------From here(for v1.0 and v1.2)---------------------------------

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

    --------------------Till here (for v1.0 and v1.2)---------------------------------------
    

    TO USE ONLY v1.3 UNCOMMENT (AND ALSO, COMMENT UPPER COMMENTS),
    --------------------------From here(for v1.3)-------------------------------------------

    setDesiredOrientation(degrees2radians(120));
    ros::Rate loop_rate(0.5);
    loop_rate.sleep();
    setDesiredOrientation(degrees2radians(-60));
    loop_rate.sleep();
    setDesiredOrientation(degrees2radians(0));

    --------------------Till here (for v1.3)------------------------------------------------
   
    */
    

    nav_msgs::Odometry goal_pose;

    goal_pose.twist.twist.linear.x = 10;
    goal_pose.twist.twist.linear.y = 1;
    goal_pose.twist.twist.angular.z = 0;
    moveGoal(goal_pose, 0.5);
    
    ros::Rate loop_rate(0.5);
    loop_rate.sleep();

    ros::spinOnce();

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

double setDesiredOrientation(double desired_angle_radians){
    double relative_angle_radians = desired_angle_radians - husky_pose.twist.twist.angular.z;
    bool clockwise = ((relative_angle_radians<0)?true:false);
    cout<<desired_angle_radians<<","<<husky_pose.twist.twist.angular.z<<","<<relative_angle_radians;
    rotate(abs(relative_angle_radians), abs(relative_angle_radians),clockwise);
    return 0;
} 

void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
    husky_pose.twist.twist.linear.x = pose_message->twist.twist.linear.x;
    husky_pose.twist.twist.linear.y = pose_message->twist.twist.linear.y;
    husky_pose.twist.twist.angular.z = pose_message->twist.twist.angular.z;
}

double getDistance(double x1,double x2,double y1,double y2){
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void moveGoal(nav_msgs::Odometry goal_pose, double distance_tolerence){
    geometry_msgs::Twist vel_msg;
    
    ros::Rate loop_rate(10);

    do
    {
        vel_msg.linear.x = 1.5*getDistance(husky_pose.twist.twist.linear.x, husky_pose.twist.twist.linear.y,goal_pose.twist.twist.linear.x,goal_pose.twist.twist.linear.y);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 4*(atan2(goal_pose.twist.twist.linear.y - husky_pose.twist.twist.linear.y, goal_pose.twist.twist.linear.x - husky_pose.twist.twist.linear.x) - husky_pose.twist.twist.angular.z);
        
        velocity_publisher.publish(vel_msg);
        
        ros::spinOnce();
        loop_rate.sleep();

    } while (getDistance(husky_pose.twist.twist.linear.x, husky_pose.twist.twist.linear.y,goal_pose.twist.twist.linear.x,goal_pose.twist.twist.linear.y)>distance_tolerence);

    cout<<"distance goal achieve"<<endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);    
}