
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>



  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double th = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  double psi1_rad = 0.0;
  double psi2_rad = 0.0;
  double psi1_prev_rad = 0.0;
  double psi2_prev_rad = 0.0;
  double xU0par_prev = 0.0;
  double yU0par_prev = 0.0;
  double deltat_prev = 0.0;
  double vx_1 = 0.0;
  double vx_2 = 0.0;
  double vx_3 = 0.0;
  double vx_4 = 0.0;
  double vf_x = 0.0;
  double vf_x_1 = 0.0;
  double vf_x_2 = 0.0;
  double vf_x_3 = 0.0;
  double vf_x_4 = 0.0;
  double vf_th_1 = 0.0;
  double vf_th_2 = 0.0;
  double vf_th_3 = 0.0;
  double vf_th_4 = 0.0;
  double vf_th = 0.0;
  double vth_1 = 0.0;
  double vth_2 = 0.0;
  double vth_3 = 0.0;
  double vth_4 = 0.0;
  // ctdr: whc parameters
  double radius  = 0.165; //+ 0.0051; //[m] -0.0051  radius of drive wheel
  double lw = 0.550; //+ 0.07;   //[m]  length between the two drive wheels
  int N_oneRot = 500; 
  
  double N_smallPulley_rightMainWh = 26.0; 
  double N_bigPulley_rightMainWh = 73.0; 

  double N_smallPulley_leftMainWh = 28.0; 
  double N_bigPulley_leftMainWh  = 73.0; 
  long EncoderCount1_ini_cts=-1;
  long EncoderCount2_ini_cts=-1;
  long EncoderCount1_prev_cts=-1;
  long EncoderCount2_prev_cts=-1;
  volatile long int encoder1Pos = 0;
  volatile long int encoder2Pos = 0;
  long EncoderCount1_cts=-1;
  long EncoderCount2_cts=-1;

  bool new_msg = false;
  
void odometryCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
//ROS_INFO("/odometry: received msg: [%f,%f,%f]", msg->data[1], msg->data[4]);

encoder1Pos = msg->data[5]; // count left wheel 
encoder2Pos = msg->data[6]; // count right wheel 

 new_msg = true;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;



  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber odometry_sub = nh.subscribe("/odometry_array", 50, odometryCallback);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  EncoderCount1_ini_cts = encoder1Pos;
  EncoderCount2_ini_cts = encoder2Pos;

  EncoderCount1_prev_cts = encoder1Pos;
  EncoderCount2_prev_cts = encoder2Pos;

  ros::Rate r(30.0);
  while(nh.ok()){

    r.sleep();
    ros::spinOnce();
    
    if (new_msg == false) {
    	continue;
    }
    
    new_msg = false;
    	               // check for incoming messages
    current_time = ros::Time::now();


  float encShaft_leftWh_cts =  -encoder1Pos-EncoderCount1_ini_cts; //[counts]
  float encShaft_rightWh_cts  =  encoder2Pos-EncoderCount2_ini_cts; //[counts]

  // conseq
  double deltat = (current_time - last_time).toSec();
  double psi_smallPulley_rightWh_rad = encShaft_rightWh_cts *2.0*M_PI/N_oneRot; //[rad]
  double psi1_pro_rad = psi_smallPulley_rightWh_rad *N_smallPulley_rightMainWh/N_bigPulley_rightMainWh;  //[rad]

  double psi_smallPulley_leftWh_rad = encShaft_leftWh_cts *2.0*M_PI/N_oneRot; //[rad]
  double psi2_pro_rad = psi_smallPulley_leftWh_rad *N_smallPulley_leftMainWh/N_bigPulley_leftMainWh;    //[rad]
  
    //float dotpsi1_radps = (psi1_rad-psi1_prev_rad)/deltat;
  double dotpsi1_radps = (psi1_pro_rad-psi1_prev_rad)/(deltat + deltat_prev);
  double dotpsi2_radps = (psi2_pro_rad-psi2_prev_rad)/(deltat + deltat_prev);
  
   vx = (radius/2.0)*(dotpsi1_radps+dotpsi2_radps); //[m/s] linear velocity
   vth = (radius/lw)*(dotpsi1_radps-dotpsi2_radps); // [rad/s] angular velocity
   
   //filter
   vf_x =0.0001734*vx + 0.0006935*vx_1 + 0.00104*vx_2 + 0.0006935*vx_3 + 0.0001734*vx_4 + 3.354*vf_x_1-4.262 * vf_x_2 + 2.428 * vf_x_3 - 0.5226 *vf_x_4;
   
  vf_th =0.0001734*vth + 0.0006935*vth_1 + 0.00104*vth_2 + 0.0006935*vth_3 + 0.0001734*vth_4 + 3.354*vf_th_1-4.262 * vf_th_2 + 2.428 * vf_th_3 - 0.5226 *vf_th_4;

   th = (radius/lw)*(psi1_rad-psi2_rad); //[rad] 
   //  std::cerr << "dt: " << deltat - deltat_prev << std::endl;
   x = xU0par_prev + vf_x*deltat*(cos(th)); //[m]
   y = yU0par_prev + vf_x*deltat*(sin(th)); //[m]

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "wcias_base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "wcias_base_footprint";
    odom.twist.twist.linear.x = vf_x;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vf_th;
    
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    EncoderCount1_prev_cts = EncoderCount1_cts;
    EncoderCount2_prev_cts = EncoderCount2_cts;
    deltat_prev = deltat;
    psi1_prev_rad = psi1_rad;
    psi2_prev_rad = psi2_rad;
    psi1_rad = psi1_pro_rad;
    psi2_rad = psi2_pro_rad;
    xU0par_prev = x;
    yU0par_prev = y;
    
    //update variables for filters
    vx_4 = vx_3;
    vx_3 = vx_2;
    vx_2 = vx_1;
    vx_1 = vx;
    vth_4 = vth_3;
    vth_3 = vth_2;
    vth_2 = vth_1;
    vth_1 = vth;
    vf_x_4 = vf_x_3;
    vf_x_3 = vf_x_2;
    vf_x_2 = vf_x_1;
    vf_x_1 = vf_x;
    vf_th_4 = vf_th_3;
    vf_th_3 = vf_th_2;
    vf_th_2 = vf_th_1;
    vf_th_1 = vf_th;

    
    


    
  }
}



