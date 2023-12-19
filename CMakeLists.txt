cmake_minimum_required(VERSION 3.0.2)
project(fingertip_dual_sensor)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES
    DualPressureStamped.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_custom_target(PREPARE_ROS_LIBRARIES
    COMMAND rosrun rosserial_arduino make_libraries.py .
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib
)
