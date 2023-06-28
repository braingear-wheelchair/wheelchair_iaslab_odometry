/*
   MPU-6050 Test

   This simple program reads and prints to the Serial Monitor window
   the raw X/Y/Z values for the accelerometer and the gyro
   It also calculates the pitch and roll values as well as the temperature
   in F and C.
    
   Connect VDD to 5V and GND to ground on the MCU
   Connect SCL to SCL on MCU and SDA to SDA on MCU

  Note that the correction values can be used to put in an offset to adjust the
  values toward 0 or in the case of the temperature to adjust it to match a
  reference temperature measurement device.
*/
#include <Wire.h>
#include <math.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>


// ROS 
ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data", &imu_msg);


const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double pitch,roll;
//===============================================================================
//  Initialization
//===============================================================================
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  nh.getHardware()->setBaud(115200); // default: 57600; then use cde: $rosrun rosserial_python serial_node.py /dev/ttyACM1 __name:=nodeMega1 _baud:=115200 
  nh.initNode();
  nh.advertise(imu_pub);

  imu_msg.header.frame_id = "imu";

}
//===============================================================================
//  Main
//===============================================================================
void loop(){
Wire.beginTransmission(MPU);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU,14,true);

int AcXoff,AcYoff,AcZoff,GyXoff,GyYoff,GyZoff;
int temp,toff;
double t,tx,tf;

//Acceleration data correction
AcXoff = -250;
AcYoff = 36;
AcZoff = 1200;

//Temperature correction
toff = -1400;

//Gyro correction
GyXoff = -335;
GyYoff = 250;
GyZoff = 170;

//read accel data and apply correction
AcX=(Wire.read()<<8|Wire.read()) * (8.0 / 65536.0) * 9.81;
AcY=(Wire.read()<<8|Wire.read()) * (8.0 / 65536.0) * 9.81;
AcZ=(Wire.read()<<8|Wire.read()) * (8.0 / 65536.0) * 9.81;

//read temperature data & apply correction
temp=(Wire.read()<<8|Wire.read()) + toff;

//read gyro data & apply correction
GyX=(Wire.read()<<8|Wire.read()) * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
GyY=(Wire.read()<<8|Wire.read()) * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
GyZ=(Wire.read()<<8|Wire.read()) * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

  // Calculate and convert temperature
  //tx=temp;
  //t = tx/340 + 36.53;     // Formula from data sheet
  //tf = (t * 9/5) + 32;    // Standard C to F conversion

  //get pitch/roll
  getAngle(AcX,AcY,AcZ);
  geometry_msgs::Quaternion quaternion = tf::createQuaternionFromYaw(roll,pitch,0.0) ;
  
  imu_msg.orientation =quaternion_tf2;
  

  imu_msg.angular_velocity.x = GyX;
  imu_msg.angular_velocity.y = GyY;
  imu_msg.angular_velocity.z = GyZ;

  imu_msg.linear_acceleration.x = AcX;
  imu_msg.linear_acceleration.y = AcY;
  imu_msg.linear_acceleration.z = AcZ;

  imu_pub.publish(&imu_msg);
  nh.spinOnce();


  delay(20);
}

//===============================================================================
//  GetAngle - Converts accleration data to pitch & roll
//===============================================================================
void getAngle(int Vx,int Vy,int Vz) {
double x = Vx;
double y = Vy;
double z = Vz;
pitch = atan(x/sqrt((y*y) + (z*z)));
roll = atan(y/sqrt((x*x) + (z*z)));
//convert radians into degrees
pitch = pitch * (180.0/3.14);
roll = roll * (180.0/3.14) ;
}
