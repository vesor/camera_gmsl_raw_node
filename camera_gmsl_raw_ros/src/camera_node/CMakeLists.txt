cmake_minimum_required(VERSION 2.8)
project(camera_node)
#set(CMAKE_BUILD_TYPE Debug) #TODO: only for debug
add_definitions(
        -std=c++11
        -std=c++0x
        -DPCL_NO_PRECOMPILE
        -DELPP_THREAD_SAFE
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
#enable for camera frame
# cv_bridge
)
set(OpenCV_INCLUDE_DIRS /usr/local/opencv320/include)
set(OpenCV_LIBRARIES /usr/local/opencv320/lib)
#find_package(OpenCV)


#set(CMAKE_BUILD_TYPE Debug)

set(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/external/
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs cv_bridge
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/external
)

file(GLOB SRC_CODE
    "src/*.cpp"
)

add_executable(${PROJECT_NAME} ${SRC_CODE})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} lib_camera_gmsl_raw)#${OpenCV_LIBRARIES})

