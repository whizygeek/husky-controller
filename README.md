# husky-mover

## Installation guide
$ sudo apt install ros-noetic-gazebo-ros-* ros-noetic-lms1xx

$ cd ~/catkin_ws/src   

$ git clone https://github.com/husky/husky.git

$ cd ..

$ rosdep install --from-path src --ignore-src  

$ catkin build

$ source devel/setup.bash

## Run simulation
Roslaunch husky_gazebo husky_empty_world.launch  

## Odometry
$ rostopic echo /odometry/filtered


## NOTE:- I have also noted in the husky_mover.cpp to uncomment the sections to try (v1.0 + v1.2) OR (V1.3) version of this programm.

##Release v2.0 - Final version of the Publisher of /twist_marker_server/cmd_vel, and Subscriber of /odometry/filtered is launched. "roslaunch" is also introduced to run both publisher and subscriber file parallel.

Publisher file & command - husky_mover.cpp - rosrun husky_mover husky_mover
Subscriber file & command - odom_sub.cpp - rosrun husky_mover odom_sub


### Release v1.3 - program is trained to rotate angularly using relative angle. Moreover, subscriber is also introduced (specifically to get the relative angle with respect to the current angle of the husky)
All other settings/dependencies (of CMakeLists.txt and package.xml) will remain same as per previous v1.0.


### Release v1.2 - program will ask for the values of angular velocity (degree/secs), angle (degrees), and angle orientation (clockwise/anti-clockwise.)
All other settings/dependencies (of CMakeLists.txt and package.xml) will remain same as per previous v1.0.


### Release v1.0 - program will ask for speed, distance, and forward/backward direction values for the linear motion.
The settings/dependencies are made in CMakeLists.txt and package.xml, which can be directly used with the help of git pull of this repository.

