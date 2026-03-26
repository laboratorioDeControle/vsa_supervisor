cmake_minimum_required(VERSION 3.8)
project(vsa_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(neptus_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(../imc_neptus_ros2/src/imc_ros2_bridge/include
                    ../imc_neptus_ros2/src/imc_ros2_bridge/external/imc-cxx
                    ../imc_neptus_ros2/src/imc_ros2_bridge/external/md5
                    /usr/include/jsoncpp)

## Declare a C++ library
add_library( md5 ../imc_neptus_ros2/src/imc_ros2_bridge/external/md5/md5.cpp )
add_library( imc_factory ../imc_neptus_ros2/src/imc_ros2_bridge/external/imc-cxx/IMC/Base/Factory.cpp )

file(GLOB IMC_ROS_SOURCE_FILES
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/udp_link.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/imc_handle.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/imc_to_ros/Goto.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/imc_to_ros/Abort.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/imc_to_ros/Heartbeat.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/imc_to_ros/PlanDB.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/imc_to_ros/PlanControl.cpp

 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/Heartbeat.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/Goto.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/RemoteState.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/GpsFix.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/GpsNavData.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/EstimatedState.cpp
 ../imc_neptus_ros2/src/imc_ros2_bridge/src/ros_to_imc/VehicleState.cpp
)

add_executable(vsa_supervisor_node src/vsa_supervisor_node.cpp
                                   src/guidance/math_operations.cpp
                                   src/plan_db/maneuver.cpp
                                   src/plan_db/plan_specification.cpp
                                   src/plan_db/plan_db.cpp
                                   src/guidance/guidance.cpp
                                   src/guidance/control_algorithms/pid.cpp
                                   src/guidance/filters/low_pass_filter.cpp
                                    ${IMC_ROS_SOURCE_FILES})
                                  
target_link_libraries(vsa_supervisor_node md5 imc_factory jsoncpp)

ament_export_targets()
ament_target_dependencies(vsa_supervisor_node 
                          rclcpp
                          jsoncpp
                          tf2
                          tf2_geometry_msgs
                          std_msgs
                          sensor_msgs
                          nav_msgs
                          neptus_msgs
                          Eigen3)

install(TARGETS
  vsa_supervisor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()