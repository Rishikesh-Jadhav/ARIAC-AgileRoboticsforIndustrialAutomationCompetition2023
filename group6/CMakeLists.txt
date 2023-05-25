cmake_minimum_required(VERSION 3.8)
project(group6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


include_directories(include/group6)

set(SENSOR_CAMERA_DEPENDENCIES
  rclcpp
  std_srvs
  ariac_msgs
  geometry_msgs
  tf2  
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

foreach(Dependency IN ITEMS ${SENSOR_CAMERA_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


set(FLOOR_ROBOT_DEPENDENCIES
  rclcpp 
  moveit_ros_planning_interface
  ariac_msgs
  competitor_interfaces
  shape_msgs
)

foreach(Dependency IN ITEMS ${FLOOR_ROBOT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


# set(CEILING_ROBOT_DEPENDENCIES
#   rclcpp 
#   moveit_ros_planning_interface
#   ariac_msgs
#   competitor_interfaces
#   shape_msgs
# )

# foreach(Dependency IN ITEMS ${CEILING_ROBOT_DEPENDENCIES})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# Executables
add_executable(sensor_camera_exe src/sensor_camera.cpp src/main.cpp)
ament_target_dependencies(sensor_camera_exe ${SENSOR_CAMERA_DEPENDENCIES})

add_executable(floor_robot_exe src/floor_robot.cpp)
ament_target_dependencies(floor_robot_exe ${FLOOR_ROBOT_DEPENDENCIES})

# add_executable(ceiling_robot_exe src/ceiling_robot.cpp)
# ament_target_dependencies(ceiling_robot_exe ${CEILING_ROBOT_DEPENDENCIES})

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sensor_camera_exe
  floor_robot_exe
  # ceiling_robot_exe
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch include config rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
