// ROS
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

// Define Encoder 1 --> left wheel
#define encoder1PinA 2
#define encoder1PinB 4
#define encoder1ZeroIndex 5

//Define Encoder 2 --> right wheel
#define encoder2PinA 3
#define encoder2PinB 6
#define encoder2ZeroIndex 7

volatile long int encoder1Pos = 0;
volatile long int encoder2Pos = 0;

// ctdr: choose
float desiredPublishRate = 50.0; //[Hz]

const byte totalNrReadings = 2;

void setup() {

  // ROS 
  ros::NodeHandle  nh;
  std_msgs::Float32MultiArray encodercounter_msg;
  ros::Publisher encoder_pub("encoder_counter", &encodercounter_msg);

  nh.getHardware()->setBaud(115200); // default: 57600; then use cde: $rosrun rosserial_python serial_node.py /dev/ttyACM1 __name:=nodeMega1 _baud:=115200 
  nh.initNode();
  
  encodercounter_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*1);  
  
  encodercounter_msg.layout.dim[0].label = "height";
  encodercounter_msg.layout.dim[0].size = totalNrReadings;
  encodercounter_msg.layout.dim[0].stride = totalNrReadings;
  encodercounter_msg.layout.data_offset = 0;
  encodercounter_msg.data = (float *)malloc(sizeof(float)*totalNrReadings); // ctdr: 
  encodercounter_msg.data_length = totalNrReadings;  // ctdr: notice indication correctly/accordingly 
  
  nh.advertise(encoder_pub);
  
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder1ZeroIndex, INPUT);   

  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 
  pinMode(encoder2ZeroIndex, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, RISING);
  attachInterrupt(digitalPinToInterrupt(3), doEncoder2A, RISING);
  
}

void loop() {
  // // ROS
  noInterrupts();
  encodercounter_msg.data[0] = encoder1Pos; 
  encodercounter_msg.data[1] = encoder2Pos;
  interrupts();


  // publish the message
  encoder_pub.publish(&encodercounter_msg);

  // // conclude First + Second
  nh.spinOnce();
  
} // void loop()

void doEncoder1A(){
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
}

void doEncoder2A(){
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinB) == LOW) {  
      encoder2Pos = encoder2Pos + 1;         // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;         // CCW
    }
}
