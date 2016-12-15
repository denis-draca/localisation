cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  nav_msgs
  genmsg
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib nav_msgs
  DEPENDS OpenCV
)


include_directories(SYSTEM  include
                            ${Boost_INCLUDE_DIRS} 
                            ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS} )

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/astar.cpp include/astar.h)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
add_dependencies(${PROJECT_NAME} astar_generate_messages_cpp)


