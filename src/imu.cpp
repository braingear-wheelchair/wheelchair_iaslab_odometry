#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>


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


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   x = msg->orientation.x;
   y = msg->orientation.y;
   z = msg->orientation.z;
   w = msg->orientation.w;
   accel_x = msg->linear_acceleration.x;
   accel_y = msg->linear_acceleration.y;
   accel_z = msg->linear_acceleration.z; 
   gyro_x = msg->angular_velocity.x;
   gyro_y = msg->angular_velocity.y;
   gyro_z = msg->angular_velocity.z; 
}


int main(int argc, char** argv){
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle nh;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 50);
  ros::Subscriber imu_sub = nh.subscribe("/lidar/lidar_camera/imu", 50, imuCallback);
  tf::TransformBroadcaster imu_broadcaster;
  tf::Transform transform;

  ros::Rate r(50.0);
  while (nh.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0.0,0.0,0.0,1) );
  imu_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lidar_camera_link", "imu"));
  

           sensor_msgs::Imu   imu_data;
           imu_data.header.stamp = current_time;
           imu_data.header.frame_id = "imu";

            imu_data.orientation.w = w;
            imu_data.orientation.x = z;
            imu_data.orientation.y = -x;
            imu_data.orientation.z = -y;
            imu_data.linear_acceleration.x = accel_z;
            imu_data.linear_acceleration.y = -accel_x;
            imu_data.linear_acceleration.z = -accel_y;
            imu_data.linear_acceleration_covariance[0] = -1;
            imu_data.angular_velocity.x = gyro_z;
            imu_data.angular_velocity.y = -gyro_x;
            imu_data.angular_velocity.z = -gyro_y;
            imu_data.angular_velocity_covariance[0] = -1;
            imu_pub.publish(imu_data);
    last_time = current_time;
    r.sleep();
}
};


