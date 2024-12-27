cmake_minimum_required(VERSION 2.8.3)
project(planeoct_server)
# SET(CMAKE_BUILD_TYPE Debug) #debug模式
SET(CMAKE_BUILD_TYPE Release) #Release
set(CMAKE_CXX_FLAGS_RELEASE"-O3")
set(
  PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  dynamic_reconfigure
  nodelet
  )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
# find_package(octomap 1.9.8 REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}  
)
link_directories(${OpenCV_LIBRARY_DIRS})
catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)




message("---------${CUDA_include_directory}-------")
message("---------${CUDA_LIBRARIES}-------")
message("---------${CUDA_LIBRARIES}-------")

# set(LINK_LIBS ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${CUDA_LIBRARIES} pcl_visualization)
# cuda_add_executable(cudatest src/cudatest.cpp src/addition.cu)
# target_link_libraries(cudatest ${LINK_LIBS})

set(LINK_LIBS ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} pcl_visualization ${OpenCV_LIBRARIES})
add_executable(test1 src/Teclas.cpp src/Gridsurf.cpp)
target_link_libraries(test1 ${LINK_LIBS})