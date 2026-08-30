cmake_minimum_required(VERSION 2.8.3)
project(as_navigation)
 
## Compile as C++11, supported in ROS Kinetic and newer
#  add_compile_options(-std=c++11)
 # 显式指定使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  sensor_msgs
  tf2_ros
  yolov8_ros_msgs
  ar_track_alvar_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
# import jetson-inference and jetson-utils packages.
# note that if you didn't do "sudo make install"
# while building jetson-inference, this will error.


# CUDA is required
find_package(CUDA)

## Generate services in the 'srv' folder
# add_service_files(FILES AddTwoInts.srv)
# generate_messages(DEPENDENCIES std_msgs)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#catkin_python_setup()

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES vision_to_mavros
  CATKIN_DEPENDS roscpp message_runtime yolov8_ros_msgs quadrotor_msgs geometry_msgs mavros_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
)

link_directories(
  # /usr/lib/aarch64-linux-gnu/tegra
  # /usr/lib/aarch64-linux-gnu
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  # ${catkin_LIB_DIRS})
)
# add_executable(navigation_with_waypoint_node src/navigation_with_waypoint.cpp)
add_library(asnav_api_2d src/api_2d.cpp)
add_library(asnav_api_3d src/api_3d.cpp)
target_link_libraries(asnav_api_2d ${catkin_LIBRARIES})
target_link_libraries(asnav_api_3d ${catkin_LIBRARIES})
## Specify libraries to link a library or executable target against
# target_link_libraries(navigation_with_waypoint_node
#   ${catkin_LIBRARIES}
# )

# add directory for libnvbuf-utils to program

# compile the detectnet program#
# cuda_add_executable(navigation_with_waypoint_node src/flight_control.cpp src/navigation_with_waypoint.cpp )
# cuda_add_executable(feisi_node src/flight_control.cpp src/feisi.cpp )
cuda_add_executable(waypoints_put_node src/flight_control.cpp src/waypoints_put1.cpp )
add_executable(asnav_2d_node src/competition_2d.cpp)
add_executable(asnav_3d_node src/competition_3d.cpp)
# link detectnet to jetson-inference library
# target_link_libraries(navigation_with_waypoint_node jetson-inference ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(feisi_node jetson-inference ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(waypoints_put_node nvvpi  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(asnav_2d_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  asnav_api_2d
  lib
)
target_link_libraries(asnav_3d_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  asnav_api_3d
  lib
)
# install(TARGETS detectnet DESTINATION bin)

# for legacy users of detectnet-camera (will be deprecated)
# cuda_add_executable(detectnet-camera ${detectnetSources})
# target_link_libraries(detectnet-camera jetson-inference)
# install(TARGETS detectnet-camera DESTINATION bin)

# for legacy users of detectnet-console (will be deprecated)
# cuda_add_executable(detectnet-console ${detectnetSources})
# set_target_properties(detectnet-console PROPERTIES COMPILE_FLAGS "-DHEADLESS" )
# target_link_libraries(detectnet-console jetson-inference)
# install(TARGETS detectnet-console DESTINATION bin)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
## Mark executables and/or libraries for installation
# install(TARGETS  position_to_mavros_node
#   set_mode_node
#   collision_avoidance_vfh_node
#   avoidance_car_vfh_node
#   collision_avoidance_matlabvfh_node
#   circular_node circular_car_node 
#   ros_nav_quadrotor_node 
#   mission_car_offboard_node
#   landing_quadrotor_node
#   tracking_quadrotor_node
#   tracking_down_kcf_quadrotor_node
#   tracking_ardupilot_node
#   auto_2d_nav_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY
#   launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# 添加可执行文件z




