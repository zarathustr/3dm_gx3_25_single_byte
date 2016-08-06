# ROS Data Acquisition Driver for 3DM-GX3-25 IMU module (Single Byte Protocol)

This program is from Nathan Michael's original ROS driver for the Lord Corporation Microstrain 3DM GX3 25 IMU (https://github.com/KumarRobotics/imu_3dm_gx3) and is revised for attitude estimation using AQUA complementary filter (https://github.com/ccny-ros-pkg/imu_tools).

Adding and compiling:

1. cd ~/catkin_ws/src

2. git clone https://github.com/zarathustr/3dm_gx3_25_single_byte.git

3. rosdep install 3dm_gx3_25_single_byte

4. cd ..

5. catkin_make 


Usage:

rosrun 3dm_gx3_25_single_byte imu_3dm_gx3 _port:=/dev/ttyACM0


