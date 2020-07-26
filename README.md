# KIOS CAV
## Table of contents
* [Arduino setup](#arduino-setup)
* [VESC package testing](#vesc-package-testing)
* [Racecar package testing](#racecar-package-testing)
* [Racecar Controller package testing](#racecar-controller-package-testing)


## Arduino setup
This is about servo_and_ir_ros folder of this github repo.
servo_and_ir_ros.ino drives through Arduino Mega 2560 the steering of the servo motor of KIOS CAV and the measurements of all the six SparkFun ToF Range Finder Sensors -  VL6180. The code is based on the demo for one VL6180 and the rosserial_arduino ROS packages for communication between Arduino and ROS.

All the user has to do is copy the contents of servo_and_ir_ros folder of this repository in the Arduino directory of the already set up NVIDIA Jetson TX2. Then compile and upload the code to the board. Then to run this project, use the commands:
```
$ roscore
```
in another terminal:
```
$ rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM*
```
after the rosrun command has succesfuly been executed the user can check the measurements of the VL6180 sensors by executing:
```
$ rostopic echo /range*_data
```

## VESC package testing
Here we test the communication of VESC 6 Plus and the motor of KIOS CAV. Initially the user has to replace (already using the ROS package https://github.com/RacecarJ/vesc/tree/VESC6) by copying vesc_driver_node.launch and vesc_driver_nodelet.launch in the vesc folder of this repository. Then to run this project:
```
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch vesc vesc_driver vesc_driver_node.launch
$ rostopic pub 
```

## Racecar package testing
This is the basic pckage of using the KIOS CAV. Inside the racecar folder of this repository there are files:

### racecar/racecar/launch/teleop.launch
Running the racecar-v2-teleop.launch.launch.xml and the rosserial for the ROS-Arduino communication (driving servo steering and VL6180 proximity sensors)

### racecar/racecar/launch/includes/racecar-v2-teleop.launch.xml
Running the drivers about VESC 6 Plus, ZED Stereo caera, RPLIDAR A3, Sparkfun 9DoF IMU, ublox C94-M8P GPS

### racecar/racecar/launch/includes/racecar-v2/vesc.launch.xml
Running vesc.yaml (explained below) and all the vesc drivers about odometry and teleoperation

### racecar/racecar/launch/includes/common/sensors.launch.xml
Running the ROS packages of sensors like the imu and the laser scanner

### racecar/racecar/config/vesc.yaml
Configuring the VESC 6 Plus specifications like maximum and minimum speed the limits of servo steering etc.

After copying all those files to the already downloaded ROS package (https://github.com/RacecarJ/racecar/tree/VESC6) then ti run this project:
```
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch racecar teleop.launch
```
Then by using Logitech F710 Wireess Gamepad you can teleoperate KIOS CAV.


## Racecar Controllers package testing
This is the basic package of letting KIOS CAV executing autonomous navigation inside an unknown space. Dynamic potential fields are used based on the Laser scanner measurements and give the ability of autonomous navigation, avoiding obstacles.

After copying all those files to the already downloaded ROS package (https://github.com/RacecarJ/racecar-controllers) then ti run this project:
```
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch racecar-controllers
$
```
After this the car starts instantly to navigate inside its space. If the user pushes the trigger button of Logitech F710 Wireless Gamepad then KIOS CAV turns into teleoperation mode. Whenever the user leaves the dead-man button autonomous navigation starts right away.



