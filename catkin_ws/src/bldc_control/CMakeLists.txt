cmake_minimum_required(VERSION 2.8.3)

project(bldc_control)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

find_library(maestro REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(bldc_control_publisher src/bldc_control_publisher.cpp)
target_link_libraries(bldc_control_publisher ${catkin_LIBRARIES})

add_executable(bldc_control_subscriber src/bldc_control_subscriber.cpp)
target_link_libraries(bldc_control_subscriber ${catkin_LIBRARIES} maestro usb-1.0)
