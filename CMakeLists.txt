cmake_minimum_required(VERSION 3.5.1)
project(rs_to_velodyne)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#########
# Setup #
#########

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)


set(DEPENDENCIES
  rclcpp
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
)


include_directories(
	include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
)


##############
# Executable #
##############

add_executable(${PROJECT_NAME} src/rs_to_velodyne.cpp)

ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()