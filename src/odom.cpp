#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>

struct state_vector {
  double x   = 0.0;
  double y   = 0.0;
  double z   = 0.0;
  double th  = 0.0;
  
  double vx  = 0.0;
  double vy  = 0.0;
  double vth = 0.0;
} state;
  
struct psi {
  double rad1      = 0.0;
  double rad2      = 0.0;
  double prev_rad1 = 0.0;
  double prev_rad2 = 0.0;
} psi;

double deltat_prev = 0.0;

struct wheelchair {
  // ctdr: whc parameters
  double radius  = 0.165; //+ 0.0051; //[m] -0.0051  radius of drive wheel
  double lw      = 0.550; //+ 0.07;   //[m]  length between the two drive wheels
  int N_oneRot   = 500; 
  
  double N_smallPulley_rightMainWh  = 26	; 
  double N_bigPulley_rightMainWh    = 71.53; 

  double N_smallPulley_leftMainWh   = 28; 
  double N_bigPulley_leftMainWh     = 71.63; 
} wheelchair;

struct encoder_vector {
  double Count1_ini_cts     =  1;
  double Count2_ini_cts     = -1;
  double Count1_prev_cts    =  1;
  double Count2_prev_cts    = -1;
  volatile long int Pos1  =  0;
  volatile long int Pos2  =  0;
  long Count1_cts         =  1;
  long Count2_cts         = -1;
} encoder;

bool new_msg = false;

struct temporal_delta_values {
  double deltat, psi1_pro_rad, psi2_pro_rad;
};

void resetCallback(const nav_msgs::Odometry& msg)
{
  // TODO: Check if it is necessary to read the encoder position by half
  encoder.Count1_ini_cts = encoder.Pos1 / 2;
  encoder.Count2_ini_cts = encoder.Pos2 / 2;
  
  encoder.Count1_prev_cts = encoder.Pos1;
  encoder.Count2_prev_cts = encoder.Pos2;

  psi.rad1      = 0.0;
  psi.rad2      = 0.0;
  psi.prev_rad1 = 0.0;
  psi.prev_rad2 = 0.0;

  state.x   = msg.pose.pose.position.x;
  state.y   = msg.pose.pose.position.y;
  state.z   = msg.pose.pose.position.z;
  state.th  = 0; // TODO: check the presence of this component
  
  state.vx  = msg.twist.twist.linear.x;
  state.vy  = msg.twist.twist.linear.y;
  state.vth = msg.twist.twist.angular.z; 
}
  
void encoderCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  encoder.Pos1 = msg->data[0]; // count left wheel 
  encoder.Pos2 = msg->data[1]; // count right wheel 

  new_msg = true;
}

nav_msgs::Odometry generate_odometry_message(struct state_vector state, geometry_msgs::Quaternion odom_quat, ros::Time current_time){
  nav_msgs::Odometry odom;

  odom.header.stamp     = current_time;
  odom.header.frame_id  = "wcias_odom";

  //set the position
  odom.pose.pose.position.x  = state.x;
  odom.pose.pose.position.y  = state.y;
  odom.pose.pose.position.z  = state.z;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "wcias_base_footprint";
  odom.twist.twist.linear.x  = state.vx;
  odom.twist.twist.linear.y  = state.vy;
  odom.twist.twist.angular.z = state.vth;
  
  float std = 1e-3f;
  float stdv = 1e-3f;
  odom.pose.covariance[0] = std;
  odom.pose.covariance[7] = std;
  odom.pose.covariance[14] = std;
  odom.pose.covariance[21] = std;
  odom.pose.covariance[28] = std;
  odom.pose.covariance[35] = std;
  
  odom.twist.covariance[0] = stdv;
  odom.twist.covariance[7] = stdv;
  odom.twist.covariance[14] = stdv;
  odom.twist.covariance[21] = stdv;
  odom.twist.covariance[28] = stdv;
  odom.twist.covariance[35] = stdv;
  

  return odom;
}

/* geometry_msgs::TransformStamped generate_odometry_tranform_message(struct state_vector state, geometry_msgs::Quaternion odom_quat, ros::Time current_time){
  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp       = current_time;
  odom_trans.header.frame_id    = "wcias_odom";
  odom_trans.child_frame_id     = "wcias_base_footprint";

  odom_trans.transform.translation.x = state.x;
  odom_trans.transform.translation.y = state.y;
  odom_trans.transform.translation.z = 0.0; // Set the 0 on the plane
  odom_trans.transform.rotation      = odom_quat;

  return odom_trans;
} */

struct temporal_delta_values update_global_odometry_variables(ros::Time current_time, ros::Time last_time){
  // TODO: review this code
  float encShaft_leftWh_cts   =  - (encoder.Pos1 - encoder.Count1_ini_cts); //[counts]
  float encShaft_rightWh_cts  =    (encoder.Pos2 - encoder.Count2_ini_cts); //[counts]

  // conseq
  double deltat = (current_time - last_time).toSec();
  double psi_smallPulley_rightWh_rad = encShaft_rightWh_cts * M_PI / wheelchair.N_oneRot; //[rad]
  double psi1_pro_rad = psi_smallPulley_rightWh_rad * wheelchair.N_smallPulley_rightMainWh / wheelchair.N_bigPulley_rightMainWh;  //[rad]

  double psi_smallPulley_leftWh_rad  = encShaft_leftWh_cts * M_PI / wheelchair.N_oneRot; //[rad]
  double psi2_pro_rad = psi_smallPulley_leftWh_rad  * wheelchair.N_smallPulley_leftMainWh / wheelchair.N_bigPulley_leftMainWh;    //[rad]
  
  //float dotpsi1_radps = (psi1_rad-psi1_prev_rad)/deltat;
  double dotpsi1_radps = (psi1_pro_rad - psi.prev_rad1) / (deltat + deltat_prev);
  double dotpsi2_radps = (psi2_pro_rad - psi.prev_rad2) / (deltat + deltat_prev);
  
  struct temporal_delta_values return_values;
  return_values.deltat       = deltat;
  return_values.psi1_pro_rad = psi1_pro_rad;  
  return_values.psi2_pro_rad = psi2_pro_rad;   

  state.vx  = (wheelchair.radius / 2.0)            * (dotpsi1_radps + dotpsi2_radps); //[m/s] linear velocity
  state.vth = (wheelchair.radius / wheelchair.lw ) * (dotpsi1_radps - dotpsi2_radps); // [rad/s] angular velocity
   
  state.th  = (wheelchair.radius / wheelchair.lw ) * (psi.rad1 - psi.rad2); //[rad] 

  state.x = state.x + state.vx * cos(state.th) * deltat ; //[m]
  state.y = state.y + state.vx * sin(state.th) * deltat ; //[m]

  return return_values;
}

void update_global_filter_variables(ros::Time current_time, struct temporal_delta_values delta_valus){
  encoder.Count1_prev_cts = encoder.Count1_cts;
  encoder.Count2_prev_cts = encoder.Count2_cts;
  deltat_prev             = delta_valus.deltat;
  psi.prev_rad1           = psi.rad1;
  psi.prev_rad2           = psi.rad2;
  psi.rad1                = delta_valus.psi1_pro_rad;
  psi.rad2                = delta_valus.psi2_pro_rad;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;

  ros::Publisher  odom_pub     = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber odometry_sub = nh.subscribe("/encoder_counter", 50, encoderCallback);

  // TODO: find a better name for the reset point topic
  // ros::Subscriber odometry_reset = nh.subscribe("/odometry_point", 50, reset_odometry_callback);

  //tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  
  current_time = ros::Time::now();
  last_time    = ros::Time::now();
  
  encoder.Count1_ini_cts = encoder.Pos1;
  encoder.Count2_ini_cts = encoder.Pos2;

  encoder.Count1_prev_cts = encoder.Pos1;
  encoder.Count2_prev_cts = encoder.Pos2;

  // try to follow the reader of the encoders
  ros::Rate r(50.0);
  
  while(nh.ok()){

    r.sleep();
    ros::spinOnce();
    
    if (new_msg == false) {
    	continue;
    }
    
    new_msg = false;
    	               // check for incoming messages
    current_time = ros::Time::now();

    struct temporal_delta_values delta_valus = update_global_odometry_variables(current_time, last_time);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state.th);

    //first, we'll publish the transform over tf -> Its automatically done by the urdf scheme
    //geometry_msgs::TransformStamped odom_trans = generate_odometry_tranform_message(state, odom_quat, current_time);
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom = generate_odometry_message(state, odom_quat, current_time);
    odom_pub.publish(odom);

    last_time = current_time;
    update_global_filter_variables(current_time, delta_valus);

  }
}
