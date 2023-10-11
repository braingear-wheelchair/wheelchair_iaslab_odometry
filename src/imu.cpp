#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

/*
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
*/

#include <sensor_msgs/Imu.h>

/*
  double gyro_x = 0.0;
  double gyro_y = 0.0;
  double gyro_z = 0.0;
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z= 0.0;
  double w= 0.0;
  */

sensor_msgs::Imu real_imu;

void imuCallback(const sensor_msgs::Imu msg)
{
    real_imu = msg; /*
   x = msg->orientation.x;
   y = msg->orientation.y;
   z = msg->orientation.z;
   w = msg->orientation.w;
   accel_x = msg->linear_acceleration.x;
   accel_y = msg->linear_acceleration.y;
   accel_z = msg->linear_acceleration.z; 
   gyro_x = msg->angular_velocity.x;
   gyro_y = msg->angular_velocity.y;
   gyro_z = msg->angular_velocity.z; */
}


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh;

  ros::Time current_time = ros::Time::now();

  ros::Publisher imu_pub  = nh.advertise<sensor_msgs::Imu>("imu_data", 50);
  ros::Subscriber imu_sub = nh.subscribe("/lidar/lidar_camera/imu", 50, imuCallback);

  /*
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  tf::TransformBroadcaster imu_broadcaster;
  tf::Transform transform;
  */
  
  sensor_msgs::Imu   imu_data;

  ros::Rate r(50.0);
  while (nh.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    /*
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "lidar_camera_link";
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    transformStamped.child_frame_id = "imu";

    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0.0,0.0,0.0,1) );
    // imu_broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "lidar_camera_link", "imu"));
    //br.sendTransform(transformStamped);
    //CHECK if I can remove this because maybe is duplicate*/

    imu_data = real_imu;

    imu_data.header.stamp = current_time;
    imu_data.header.frame_id = "imu";



    imu_data.orientation.w =  real_imu.orientation.w;
    imu_data.orientation.x =  real_imu.orientation.z;
    imu_data.orientation.y = -real_imu.orientation.x;
    imu_data.orientation.z = -real_imu.orientation.y;
    imu_data.linear_acceleration.x =  real_imu.linear_acceleration.z;
    imu_data.linear_acceleration.y = -real_imu.linear_acceleration.x;
    imu_data.linear_acceleration.z = -real_imu.linear_acceleration.y;
    // TODO: setup the correct infomation covariance
    //imu_data.linear_acceleration_covariance[0] = -1;
    imu_data.angular_velocity.x =  real_imu.angular_velocity.z;
    imu_data.angular_velocity.y = -real_imu.angular_velocity.x;
    imu_data.angular_velocity.z = -real_imu.angular_velocity.y;
    //imu_data.angular_velocity_covariance[0] = -1;
    imu_pub.publish(imu_data);
    
    r.sleep();
}
};


