# KIOS CAV
## Table of contents
* [UDEV rules setup](#udev-rules-setup)
* [Arduino setup](#arduino-setup)
* [VESC package testing](#vesc-package-testing)
* [Racecar package testing](#racecar-package-testing)
* [Racecar Controller package testing](#racecar-controller-package-testing)


## UDEV rules setup
The 40-sensors-actuators.rules file is placed in the path: /etc/udev/rules.d/
It creates unique names for certain USB devices connected to the USB hub through symbolic links. It also gives read and write permissions to these devices. These devices are:
* VESC device -> ttyACM0 -> ttyVESC
* IMU device -> ttyACM1 -> ttyIMU
* Arduino device -> ttyACM2 -> ttyArduino
* ublox C94-M8P GPS -> ttyACM3 -> ttyGPS
* Logitech F710 Wireless Gamepad -> js0 (it only gives read and write permissions)


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
$ roslaunch vesc_driver vesc_driver_node.launch
$ rostopic pub /commands/motor/speed std_msgs/Float64 "data: 0.0"
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
Since the package is launched the user can teleoperate KIOS CAV using the Logitech F710 Wireless Gamepad. For the teleoperation to happen the dead-man (LB) button button has to be pushed all the times. When this happens then the car responds to the motion of the gamepad's joysticks.
When this ROS package is executed all the sensors are launched. ZED stereo camera will have to be launched separately after the teleop.launch regarding the overload of the memory bus. This happens using the following command on another terminal:
	
```
$ roslaunch zed_wrapper zed_no_tf.launch
```
During testing it has been observed that when all the sensors are launched and acquiring information the responsiveness of the car to the joystick's motions  is limited. If only the teleoperation package file is launched then the teleoperation range of KIOS CAV has been tested up to 20-25 meters from the user. If the zed file is additionally launched then the teleoperation range of the car limits to 8-10 meters distance from the user.

It also reported that even if the teleoperation file launches the ZED stereo camera and not the RPLIDAR A3 then again the joystick's range is limited to 8-10 meters from the user. Hence, it is concluded that the information provided from ZED stereo camera is heavy. The user can configure the quality of the information acquired from the camera to be lighter for the Jetson TX2 to process. This can happen tweaking the files presented below:
* catkin\_ws/src/zed-ros-wrapper/zed\_wrapper/cfg/Zed.cfg
* catkin\_ws/src/zed-ros-wrapper/zed\_wrapper/params/common.yaml
* catkin\_ws/src/zed-ros-wrapper/zed\_wrapper/params/zed.yaml

It's indicated that KIOS CAV's motor is capped on the 6.000 rpm, while it's full potential reaches 100.000 rpm. After the 10.000 rpm the user of the CAV must be really experienced because the car gets really aggressive. An inexperienced user might end up to big damages of the car.
WARNING: Sometimes and if the vehicles is aggressively teleoperated squeaking noise coming from the motor might appear. In that case the user should leave immediately the dead-man button and pus it again. In that case the noise should stop. If the sound persists kill the ROS launch files of the teleoperation package running. If this is not possible then unplug the LiPo battery from the KIOS CAV's motor. This noise indicates that the motor is stressed.


## Racecar Controllers package testing
This is the basic package of letting KIOS CAV executing autonomous navigation inside an unknown space. Dynamic potential fields are used based on the Laser scanner measurements and give the ability of autonomous navigation, avoiding obstacles.

### racecar-controllers/racecar_potential_field_controller/config/default.yaml
Configuring the potential field controller specifications like the force_scale, the force offset the speed_p_gain etc.


After copying all those files to the already downloaded ROS package (https://github.com/RacecarJ/racecar-controllers) then ti run this project:
```
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch racecar_potential_field_controller node.launch
```
After this the car starts instantly to navigate inside its space. If the user pushes the trigger button of Logitech F710 Wireless Gamepad then KIOS CAV turns into teleoperation mode. Whenever the user leaves the dead-man button autonomous navigation starts right away.
