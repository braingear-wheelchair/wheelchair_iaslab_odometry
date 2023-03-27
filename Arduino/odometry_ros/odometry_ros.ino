

// ROS
//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

// ROS 
ros::NodeHandle  nh;
std_msgs::Float32MultiArray odometry_msg;
ros::Publisher odometry_pub("odometry_array", &odometry_msg);

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

long EncoderCount1_cts=-1;
long EncoderCount2_cts=-1;
char buff[8];

// ctdr: ini
long EncoderCount1_ini_cts=-1;
long EncoderCount2_ini_cts=-1;
 long EncoderCount1_prev_cts=-1;
 long EncoderCount2_prev_cts=-1;

unsigned long unsLong_max = 2^32-1; //cf https://www.arduino.cc/reference/en/language/variables/data-types/unsignedlong/
unsigned long unsLong_min = 0; 

unsigned long time_actu_micros(0); // def & ini
unsigned long time_prev_micros(0); // def & ini

float psi1_prev_rad = 0.0;
float psi2_prev_rad = 0.0; 
float   xU0par_prev = 0.0;
float   yU0par_prev = 0.0;
float   phi2_prev =0.0;
float deltat_prev = 0.0;
float psi1_pro_rad = 0.0;

// ctdr: whc parameters
float r  = 0.165; //+ 0.0051; //[m] -0.0051  radius of drive wheel
float lw = 0.550 + 0.07;   //[m]  length between the two drive wheels
float N_oneRot = 500.0;    //[cts]

const float N_smallPulley_rightMainWh = 26.0; //[nr-teeth]  30 (2.5mm pitch) or 36 (2mm pitch)
const float N_bigPulley_rightMainWh = 73.0; //[nr-teeth]  110 (2.5mm pitch) or 153 (2mm pitch) 

const float N_smallPulley_leftMainWh = 27.0; //[nr-teeth]  30 (2.5mm pitch) or 36 (2mm pitch)
const float N_bigPulley_leftMainWh  = 73.0; //[nr-teeth]  110 (2.5mm pitch) or 153 (2mm pitch) 

// ctdr: choose
float desiredPublishRate = 50.0; //[Hz]

// conseq
float desiredPublishSamplTime = 1.0/desiredPublishRate; //[sec]

// conseq: convert
unsigned long desiredPublishSamplTime_micros = (unsigned long)(desiredPublishSamplTime*1e6); //[microsecond]  


const byte totalNrReadings = 7;


void setup() {

  nh.getHardware()->setBaud(115200); // default: 57600; then use cde: $rosrun rosserial_python serial_node.py /dev/ttyACM1 __name:=nodeMega1 _baud:=115200 
  nh.initNode();
  
  odometry_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*1);  
  odometry_msg.layout.dim[0].label = "height";
  odometry_msg.layout.dim[0].size = totalNrReadings;
  odometry_msg.layout.dim[0].stride = totalNrReadings;
  odometry_msg.layout.data_offset = 0;
  odometry_msg.data = (float *)malloc(sizeof(float)*totalNrReadings); // ctdr: 
  odometry_msg.data_length = totalNrReadings;  // ctdr: notice indication correctly/accordingly 
  
  nh.advertise(odometry_pub);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder1ZeroIndex, INPUT);   


  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 
  pinMode(encoder2ZeroIndex, INPUT);


  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, RISING);
  attachInterrupt(digitalPinToInterrupt(3), doEncoder2A, RISING);
  EncoderCount1_ini_cts = encoder1Pos;
  EncoderCount2_ini_cts = encoder2Pos;

  EncoderCount1_prev_cts = encoder1Pos;
  EncoderCount2_prev_cts = encoder2Pos;
}

void loop() {
  // ctdr: the timestamping and the measurement reading should occur simultaneously herebelow (meaning: datastamp associated to data); in practice, there will be a few 16-MHz-cycles occuring in-between
  time_actu_micros = micros(); //[microsec] this line of code should be as closed as possible to the instant measurements (encoder reads) happen

  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, RISING);
  attachInterrupt(digitalPinToInterrupt(3), doEncoder2A, RISING);

    // // ctdr: calc deltat betw two successive sensor readings, w detect overflow cf doc micros()   
  // def & safe ini
  unsigned long deltat_betw_succ_readings_micros = unsLong_max; //[microsec]
  
  if (time_actu_micros<time_prev_micros)  { // then detected overflow cf doc micros()
    deltat_betw_succ_readings_micros=(unsLong_max-time_prev_micros)+(time_actu_micros-unsLong_min)+1; //+1 due to extra tic when jumping from unsLong_max to unsLong_min                                                     
  } else  {
    deltat_betw_succ_readings_micros=time_actu_micros-time_prev_micros; //[microsecond]
  } //if

  float deltat = ((float)deltat_betw_succ_readings_micros)*1e-6; //[sec]

// choose accordingly 
  float encShaft_leftWh_cts =  -encoder1Pos-EncoderCount1_ini_cts; //[counts]
  float encShaft_rightWh_cts  =  encoder2Pos-EncoderCount2_ini_cts; //[counts]

  // conseq
  float psi_smallPulley_rightWh_rad = encShaft_rightWh_cts *2.0*PI/N_oneRot; //[rad]
  float psi1_rad = psi_smallPulley_rightWh_rad *N_smallPulley_rightMainWh/N_bigPulley_rightMainWh;  //[rad]

  float psi_smallPulley_leftWh_rad = encShaft_leftWh_cts *2.0*PI/N_oneRot; //[rad]
  float psi2_rad = psi_smallPulley_leftWh_rad *N_smallPulley_leftMainWh/N_bigPulley_leftMainWh;    //[rad]

  //float dotpsi1_radps = (psi1_rad-psi1_prev_rad)/deltat;
  float dotpsi1_radps = (psi1_rad-psi1_prev_rad)/deltat;
  float dotpsi2_radps = (psi2_rad-psi2_prev_rad)/deltat;
  float v = (r/2.0)*(dotpsi1_radps+dotpsi2_radps); //[m/s] linear velocity
  float omega = (r/lw)*(dotpsi1_radps-dotpsi2_radps); // [rad/s] angular velocity

  float phi = (r/lw)*(psi1_rad-psi2_rad); //[rad] 

  // to-do: replace the 2 lines below where (xU0par,yU0par) = f(v) by (xU0par,yU0par) = f(psi1_rad,psi2_rad) 
  float xU0par = xU0par_prev + v*deltat*(float)(cos(phi)); //[m]
  float yU0par = yU0par_prev + v*deltat*(float)(sin(phi)); //[m]

  // // ROS
  // publish: positions
  odometry_msg.data[0] = xU0par; // [m] this is xU0parL{o^child} 
  odometry_msg.data[1] = yU0par; // [m] this is yU0parL{o^child} 
  odometry_msg.data[2] = phi; // [rad]

  // publish: velocities
  odometry_msg.data[3] = v; // [m/s] linear velocity
  odometry_msg.data[4] = omega; // [rad/s] angular velocity
  odometry_msg.data[5] = encoder1Pos; // 
  odometry_msg.data[6] = encoder2Pos; // [m/s] linear velocity in the y-dirextion
  //odometry_msg.data[7] = deltat;
  //odometry_msg.data[8] = deltat;






  // publish the message
  odometry_pub.publish(&odometry_msg);

  // // conclude First + Second
  nh.spinOnce();
  
  // ctdr: update
  EncoderCount1_prev_cts = EncoderCount1_cts;
  EncoderCount2_prev_cts = EncoderCount2_cts;
  psi1_prev_rad = psi1_rad;
  psi2_prev_rad = psi2_rad;
  xU0par_prev = xU0par;
  yU0par_prev = yU0par;
  time_prev_micros = time_actu_micros;
  deltat_prev = deltat;
  

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


void doEncoder1A(){

  // look for a low-to-high on channel A
  //if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  //}

}

void doEncoder2A(){

  // look for a low-to-high on channel A
  //if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinB) == LOW) {  
      encoder2Pos = encoder2Pos + 1;         // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;         // CCW
    }
  //}

}
