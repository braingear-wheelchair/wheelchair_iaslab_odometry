cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_iaslab_odometry)


find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
                         rospy
                         actionlib
                         actionlib_msgs
			 std_msgs
			 geometry_msgs
                         sensor_msgs
                         nav_msgs
                         tf)

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom src/odom.cpp)
target_link_libraries(odom ${catkin_LIBRARIES})

#add_executable(odom_old src/odom_old.cpp)
#target_link_libraries(odom_old ${catkin_LIBRARIES})

add_executable(imu src/imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})

