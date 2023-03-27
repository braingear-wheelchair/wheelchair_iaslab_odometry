// ROS
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

// ROS 
ros::NodeHandle  nh;
std_msgs::Float32MultiArray encodercounter_msg;
ros::Publisher encoder_pub("encoder_counter", &encodercounter_msg);

// Define Encoder 1 --> left wheel
#define encoder1PinA 2
#define encoder1PinB 4
#define encoder1ZeroIndex 5

// Define Encoder 2 --> right wheel
#define encoder2PinA 3
#define encoder2PinB 6
#define encoder2ZeroIndex 7

// Define readEncoder status
#define read1A digitalRead(encoder1PinA)
#define read1B digitalRead(encoder1PinB)
#define read2A digitalRead(encoder2PinA)
#define read2B digitalRead(encoder2PinB)

volatile long int encoder1Pos = 0;
volatile long int encoder2Pos = 0;

// ctdr: choose
float desiredPublishRate = 50; //[Hz]
float desiredPublishSamplTime = 1.0/desiredPublishRate; //[sec]
unsigned long desiredPublishSamplTime_micros = (unsigned long)(desiredPublishSamplTime*1e6); //[microsecond]  

unsigned long unsLong_max = 2^32-1; //cf https://www.arduino.cc/reference/en/language/variables/data-types/unsignedlong/
unsigned long unsLong_min = 0; 

unsigned long time_actu_micros(0);
unsigned long time_prev_micros(0);

const byte totalNrReadings = 2;

void setup() {

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

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), isr1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), isr1B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), isr2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), isr2B, CHANGE);
  
}

void loop() {
  time_actu_micros = micros(); 

  // // ctdr: calc deltat betw two successive sensor readings, w detect overflow cf doc micros()   
  // def & safe ini
  unsigned long deltat_betw_succ_readings_micros = unsLong_max; //[microsec]
  
  if (time_actu_micros<time_prev_micros)  { // then detected overflow cf doc micros()
    deltat_betw_succ_readings_micros=(unsLong_max-time_prev_micros)+(time_actu_micros-unsLong_min) + 1; //+1 due to extra tic when jumping from unsLong_max to unsLong_min                                                     
  } else  {
    deltat_betw_succ_readings_micros=time_actu_micros-time_prev_micros; //[microsecond]
  } //if

  float deltat = ((float)deltat_betw_succ_readings_micros)*1e-6; //[sec]

  
  // // ROS
  noInterrupts();
  encodercounter_msg.data[0] = encoder1Pos; 
  encodercounter_msg.data[1] = encoder2Pos;
  interrupts();

  // publish the message
  encoder_pub.publish(&encodercounter_msg);

  // // conclude First + Second
  nh.spinOnce();
  
  // // calc deltat-loop-oneIter w detect overflow cf doc micros()
  // // ctdr: calc deltat betw two successive sensor readings, w detect overflow cf doc micros()   
  // def & safe ini
  unsigned long deltat_loop_oneIter_micros = desiredPublishSamplTime_micros; //[microsec]

  unsigned long time_startIter_micros = time_actu_micros;
  unsigned long time_endIter_micros = micros(); 
  
  if (time_endIter_micros<time_startIter_micros)  { // detect overflow cf doc micros()
    deltat_loop_oneIter_micros=(unsLong_max-time_startIter_micros)+(time_endIter_micros-unsLong_min)+1; //+1 due to extra tic when jumping from unsLong_max to unsLong_min                                                     
  } else  {
    deltat_loop_oneIter_micros=time_endIter_micros-time_startIter_micros; //[microsecond]
  } //if

  if (deltat_loop_oneIter_micros < desiredPublishSamplTime_micros) { // then it makes sense to wait for a while until starting the next iteration 
     //delayMicroseconds( (unsigned int)(desiredPublishSamplTime_micros-deltat_loop_oneIter_micros) );
     delay((unsigned long)((desiredPublishSamplTime_micros-deltat_loop_oneIter_micros)/1e3));
  } // if  
    
} // void loop()

void isr1A() {
  if(read1B != read1A) {
    encoder1Pos ++;
  } else {
    encoder1Pos --;
  }
}
void isr1B() {
  if (read1A == read1B) {
    encoder1Pos ++;
  } else {
    encoder1Pos --;
  }
}

void isr2A() {
  if(read2B != read2A) {
    encoder2Pos ++;
  } else {
    encoder2Pos --;
  }
}
void isr2B() {
  if (read2A == read2B) {
    encoder2Pos ++;
  } else {
    encoder2Pos --;
  }
}
