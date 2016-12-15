cmake_minimum_required(VERSION 2.8.3)
project(pf_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  message_filters
  genmsg
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)

message(STATUS "OPENCV INCLUDE DIRS ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OPENCV LIBS REQUIRED ${OpenCV_LIBS}")

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib std_msgs sensor_msgs 
  DEPENDS OpenCV
)

include_directories(SYSTEM  include
                            ${Boost_INCLUDE_DIRS} 
                            ${catkin_INCLUDE_DIRS} 
                            ${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/pf_localization.cpp include/pf_localization.h)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} pf_localization_generate_messages_cpp)



