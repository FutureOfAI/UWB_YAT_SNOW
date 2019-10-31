set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--copy-dt-needed-entries -Wl,-bq. rpath-link,${DIR_TMPFS}/lib")

set(CMAKE_CXX_FLAGS -fopenmp)
cmake_minimum_required(VERSION 2.8.3)
project(vision_zigzag)

ADD_DEFINITIONS(-DCPU_ONLY=1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(TENGINE_LIBS tengine protobuf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial  
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  #Eigen3
  #eigen
  #eigen_stl_containers
  message_generation
)

set(OpenCV_DIR /home/rock/opencv-3.3.1/build/install/share/OpenCV/)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  #edgeStatus.msg
)

add_service_files(
  FILES
  edgeSrv.srv
  edgeSrvmulty.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES log_gps
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/spdlog
  /home/rock/ncnn_build/install/include
  #/home/wangbin/ncnn/build/install/include
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#LINK_DIRECTORIES(/home/wangbin/ncnn/build/install/lib/)

#FIND_PACKAGE( OpenMP REQUIRED) 
#if(OPENMP_FOUND) 
#	message("OPENMP FOUND")
#	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}") 
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") 
#	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}") 
#endif()

#add_executable(vision_proc src/vision_proc.cpp src/nnet.cpp)
#target_link_libraries(vision_proc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /home/rock/ncnn_build/install/lib/libncnn.a)

add_executable(multy_test test/multy_test.cpp src/nnet.cpp)
target_link_libraries(multy_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /home/rock/ncnn_build/install/lib/libncnn.a)

# add_executable(vision_proc src/vision_proc.cpp src/mnet.cpp)
# target_link_libraries(vision_proc ${TENGINE_LIBS} ${catkin_LIBRARIES} /usr/lib/aarch64-linux-gnu/libopencv_core.so.2.4 
# /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.2.4 /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.2.4 
# /usr/lib/aarch64-linux-gnu/libglog.so /usr/lib/aarch64-linux-gnu/libboost_system.so /home/rock/software/tengine/build/libtengine.so)

#add_executable(vision_motion src/vision_motion.cpp)
#add_dependencies(vision_motion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(vision_motion ${catkin_LIBRARIES})

add_executable(vision_test src/vision_test.cpp)
add_dependencies(vision_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_test ${catkin_LIBRARIES})

add_executable(vision_random src/vision_random.cpp)
add_dependencies(vision_random ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_random ${catkin_LIBRARIES})

add_executable(vision_zigzag src/vision_zigzag.cpp)
add_dependencies(vision_zigzag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_zigzag ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

# add_executable(joy_control src/joy_control.cpp)
# target_link_libraries(joy_control ${catkin_LIBRARIES})


