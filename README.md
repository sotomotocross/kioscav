# kioscav
## Table of contents
* [Arduino setup](#arduino-setup)



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
$ rostopic echo /rang*_data
```

## asdsadasd

