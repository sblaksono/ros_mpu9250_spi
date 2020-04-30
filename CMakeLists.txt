cmake_minimum_required(VERSION 2.8.3)
project(ros_mpu9250)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs genmsg)

add_message_files(DIRECTORY msg FILES ahrs.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/mpu9250_node.cpp src/MPU9250.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

