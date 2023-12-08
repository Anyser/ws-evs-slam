cmake_minimum_required(VERSION 2.8.3)
project(evs_slam)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()


#add_compile_options(-std=c+11)

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

rosbuild_genmsg()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)


find_package (catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	cv_bridge
	image_transport
	tf
	tf2_geometry_msgs
	tf2_ros
	sensor_msgs
	dynamic_reconfigure
	message_generation
)



find_package(OpenCV REQUIRED)
find_package(PythonInterp 3 REQUIRED)

#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libEVS_SLAM.so
#${PROJECT_SOURCE_DIR}/../../../lib/libboost_system.so
#${PROJECT_SOURCE_DIR}/../../../lib/libboost_filesystem.so
-lboost_system
)

#set(LIBS_ROS
#${PROJECT_SOURCE_DIR}//lib/lib${PROJECT_NAME}.so
#${OpenCV_LIBS}
#${catkin_LIBRARIES}
#)

add_message_files(
	FILES
	array2D.msg
	Num.msg
	msg_mask.msg
)


generate_messages(
	DEPENDENCIES
	std_msgs
)



catkin_package (
	CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
   	LIBRARIES {PROJECT_NAME} libDBoW2 libg2o

)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})


rosbuild_add_executable (ros_rgbd_zed
src/ros_rgbd_zed.cc
)

target_link_libraries(ros_rgbd_zed
${LIBS}
${catkin_LIBRARIES}
)


rosbuild_add_executable (ros_stereo_zed
src/ros_stereo_zed.cc
)

target_link_libraries(ros_stereo_zed
${LIBS}
${catkin_LIBRARIES}
)





