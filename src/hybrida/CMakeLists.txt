cmake_minimum_required(VERSION 2.8.3)
project(hybrida)
# SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror
set(CMAKE_CXX_FLAGS "-std=c++11")
set(OMPL_DIR /home/r/Robot_LIB/omplapp-1.5.1-Source/ompl/)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    # ompl    
    std_msgs
    )
find_package(OpenCV 4.2.0 REQUIRED)
set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/astar_sur.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/nav_uneven.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uneven_opt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/non_uniform_bspline.cpp
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar_sur.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nav_uneven.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Surf.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/non_uniform_bspline.hpp
    )
    set(NLOPT_INC_DIRS /usr/local/include)
    set(nlopt_lib /usr/local/lib/libnlopt.so)
# add_library(hybridA ${SOURCES} ${HEADERS})
add_library(hybrida ${SOURCES})

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include  
    LIBRARIES hybrida  
    CATKIN_DEPENDS roscpp
)

# message("-----------${OMPL_INCLUDE_DIRS}-------------")
# message("-----------${OMPL_LIBRARIES}-------------")
# message("-----------${OMPL_DIRS}-------------")
## OPEN MOTION PLANNING LIBRARY
# find_package(OMPL REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${NLOPT_INC_DIRS}  
    ${OpenCV_INCLUDE_DIRS}      
    # ${OMPL_INCLUDE_DIRS}
    )
    # ${OMPL_LIBRARIES}
link_directories(${OpenCV_LIBRARY_DIRS})
add_executable(hybrid_astar src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES} ${nlopt_lib} ${OpenCV_LIBRARIES})