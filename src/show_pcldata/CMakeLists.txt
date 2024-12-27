cmake_minimum_required(VERSION 3.0.2)
project(show_pcldata)

SET(CMAKE_BUILD_TYPE Debug) #debug模式
# SET(CMAKE_BUILD_TYPE Release) #debug模式

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  gazebo_msgs 
  visualization_msgs
  pcl_ros
  pcl_conversions
  )
find_package(Eigen3)
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_node
#  CATKIN_DEPENDS roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}  
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS} 
)


add_executable(show_pcldata src/show_data.cpp)
target_link_libraries(show_pcldata ${catkin_LIBRARIES} pcl_visualization ${OCTOMAP_LIBRARIES})

