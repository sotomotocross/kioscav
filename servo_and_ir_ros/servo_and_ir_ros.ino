/******************************************************************************
 * SparkFun_VL6180X_demo.ino
 * Example Sketch for VL6180x time of flight range finder.
 * Casey Kuhns @ SparkFun Electronics
 * 10/29/2014
 * https://github.com/sparkfun/SparkFun_ToF_Range_Finder-VL6180_Arduino_Library
 * 
 * The VL6180x by ST micro is a time of flight range finder that
 * uses pulsed IR light to determine distances from object at close
 * range.  The average range of a sensor is between 0-200mm
 * 
 * Resources:
 * This library uses the Arduino Wire.h to complete I2C transactions.
 * 
 * Development environment specifics:
 * 	IDE: Arduino 1.0.5
 * 	Hardware Platform: Arduino Pro 3.3V/8MHz
 * 	VL6180x Breakout Version: 1.0
 *  **Updated for Arduino 1.6.4 5/2015**

 * 
 * This code is beerware. If you see me (or any other SparkFun employee) at the
 * local pub, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ******************************************************************************/

#include <Wire.h>

#include <SparkFun_VL6180X.h>

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

#include <Servo.h> 
#include <std_msgs/Float64.h>

#define TCAADDR 0x70
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


VL6180xIdentification identification;
VL6180x sensor_0(0x29);
VL6180x sensor_1(0x29);
VL6180x sensor_2(0x29);
VL6180x sensor_3(0x29);
VL6180x sensor_4(0x29);
VL6180x sensor_5(0x29);


ros::NodeHandle  nh;
sensor_msgs::Range range0_msg;
sensor_msgs::Range range1_msg;
sensor_msgs::Range range2_msg;
sensor_msgs::Range range3_msg;
sensor_msgs::Range range4_msg;
sensor_msgs::Range range5_msg;

ros::Publisher pub_range0( "range0_data", &range0_msg);
ros::Publisher pub_range1( "range1_data", &range1_msg);
ros::Publisher pub_range2( "range2_data", &range2_msg);
ros::Publisher pub_range3( "range3_data", &range3_msg);
ros::Publisher pub_range4( "range4_data", &range4_msg);
ros::Publisher pub_range5( "range5_data", &range5_msg);

char frameid0[] = "/ir_ranger0";
char frameid1[] = "/ir_ranger1";
char frameid2[] = "/ir_ranger2";
char frameid3[] = "/ir_ranger3";
char frameid4[] = "/ir_ranger4";
char frameid5[] = "/ir_ranger5";

//servo stuff
Servo servo;

void servo_cb( const std_msgs::Float64& cmd_msg){
  servo.write(uint16_t(cmd_msg.data*180)); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::Float64> sub("/vesc/commands/servo/position", servo_cb);

void setup() {

  Serial.begin(57600); //Start Serial at 115200bps
  Wire.begin(); //Start I2C library
  
  delay(1); // delay .1s

  
  //*************INITIALIZING FIRST SENSOR*******************************   
  tcaselect(0);
  sensor_0.getIdentification(&identification); // Retrieve manufacture info from device memory

  if(sensor_0.VL6180xInit() != 0x29)
  { Serial.print(F("VL6180x Nr.1 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("VL6180x Nr.1 detected?\t")); Serial.println(F("Yes"));}
  
  sensor_0.VL6180xDefautSettings(); //Load default settings to get started.
  //**********************************************************************
  
  delay(1); // delay 1ms

  //*************INITIALIZING SECOND SENSOR*******************************  
  tcaselect(1);
  sensor_1.getIdentification(&identification); // Retrieve manufacture info from device memory

  if(sensor_1.VL6180xInit() != 0x29)
  { Serial.print(F("VL6180x Nr.2 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("VL6180x Nr.2 detected?\t")); Serial.println(F("Yes"));}
  
  sensor_1.VL6180xDefautSettings(); //Load default settings to get started.
  //*********************************************************************      

  delay(1); // delay 1ms

  //*************INITIALIZING THIRD SENSOR*******************************  
  tcaselect(2);
  sensor_2.getIdentification(&identification); // Retrieve manufacture info from device memory

  if(sensor_2.VL6180xInit() != 0x29)
  { Serial.print(F("VL6180x Nr.3 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("VL6180x Nr.3 detected?\t")); Serial.println(F("Yes"));}
  
  sensor_2.VL6180xDefautSettings(); //Load default settings to get started.
  //*********************************************************************      

  delay(1); // delay 1ms


  //*************INITIALIZING FOURTH SENSOR*******************************  
  tcaselect(3);
  sensor_3.getIdentification(&identification); // Retrieve manufacture info from device memory

  if(sensor_3.VL6180xInit() != 0x29)
  { Serial.print(F("VL6180x Nr.4 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("VL6180x Nr.4 detected?\t")); Serial.println(F("Yes"));}
  
  sensor_3.VL6180xDefautSettings(); //Load default settings to get started.
  //*********************************************************************      

  delay(1); // delay 1ms


  //*************INITIALIZING FIFTH SENSOR*******************************  
  tcaselect(4);
  sensor_4.getIdentification(&identification); // Retrieve manufacture info from device memory

  if(sensor_4.VL6180xInit() != 0x29)
  { Serial.print(F("VL6180x Nr.5 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("VL6180x Nr.5 detected?\t")); Serial.println(F("Yes"));}
  
  sensor_4.VL6180xDefautSettings(); //Load default settings to get started.
  //*********************************************************************      

  delay(1); // delay 1ms


  //*************INITIALIZING SIXTH SENSOR*******************************  
  tcaselect(5);
  sensor_5.getIdentification(&identification); // Retrieve manufacture info from device memory

  if(sensor_5.VL6180xInit() != 0x29)
  { Serial.print(F("VL6180x Nr.6 detected?\t")); Serial.println(F("No"));}
  else
  { Serial.print(F("VL6180x Nr.6 detected?\t")); Serial.println(F("Yes"));}
  
  sensor_5.VL6180xDefautSettings(); //Load default settings to get started.
  //*********************************************************************      

  delay(1); // delay 1ms


  nh.initNode();
  nh.advertise(pub_range0);
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  nh.advertise(pub_range5);
  
  range0_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range0_msg.header.frame_id =  frameid0;
  range0_msg.field_of_view = 0.01;
  range0_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range0_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers

  range1_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range1_msg.header.frame_id =  frameid1;
  range1_msg.field_of_view = 0.01;
  range1_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range1_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers

  range2_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range2_msg.header.frame_id =  frameid2;
  range2_msg.field_of_view = 0.01;
  range2_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range2_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers

  range3_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range3_msg.header.frame_id =  frameid3;
  range3_msg.field_of_view = 0.01;
  range3_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range3_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers

  range4_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range4_msg.header.frame_id =  frameid4;
  range4_msg.field_of_view = 0.01;
  range4_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range4_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers

  range5_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range5_msg.header.frame_id =  frameid5;
  range5_msg.field_of_view = 0.01;
  range5_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range5_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers

  //servo
  pinMode(13, OUTPUT);
  nh.subscribe(sub);
  servo.attach(9); //attach it to pin 9

}

void loop() {

  tcaselect(0);
  //Get Distances and report in mm
  float sensor0_dist = sensor_0.getDistance();

  tcaselect(1); 
  float sensor1_dist = sensor_1.getDistance();

  tcaselect(2);
  float sensor2_dist = sensor_2.getDistance();

  tcaselect(3);
  float sensor3_dist = sensor_3.getDistance();

  tcaselect(4);
  float sensor4_dist = sensor_4.getDistance();

  tcaselect(5);
  float sensor5_dist = sensor_5.getDistance();

  range0_msg.range = sensor0_dist;
  range0_msg.header.stamp = nh.now();
  pub_range0.publish(&range0_msg);

  range1_msg.range = sensor1_dist;
  range1_msg.header.stamp = nh.now();
  pub_range1.publish(&range1_msg);

  range2_msg.range = sensor2_dist;
  range2_msg.header.stamp = nh.now();
  pub_range2.publish(&range2_msg);

  range3_msg.range = sensor3_dist;
  range3_msg.header.stamp = nh.now();
  pub_range3.publish(&range3_msg);

  range4_msg.range = sensor4_dist;
  range4_msg.header.stamp = nh.now();
  pub_range4.publish(&range4_msg);

  range5_msg.range = sensor5_dist;
  range5_msg.header.stamp = nh.now();
  pub_range5.publish(&range5_msg);
    
  
  nh.spinOnce();

  delay(1);   
};
