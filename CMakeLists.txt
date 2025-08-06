cmake_minimum_required(VERSION 3.8)
project(agv_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)

# # Find libmodbus
# find_library(MODBUS_LIBRARY modbus)
# if(NOT MODBUS_LIBRARY)
#   message(FATAL_ERROR "libmodbus not found")
# endif()

# add_executable(vel_to_serial src/vel_cmd_to_serial.cpp)

# # Link ROS2 dependencies
# ament_target_dependencies(vel_to_serial rclcpp geometry_msgs)

# # Link libmodbus
# target_link_libraries(vel_to_serial ${MODBUS_LIBRARY})

# install(TARGETS
#   vel_to_serial
#   DESTINATION lib/${PROJECT_NAME}
# )

# FOR LASER NODES
# need to create shared library for composable node
add_library(scan_merger_v2_component SHARED src/scan_merger_v2.cpp)
target_include_directories(scan_merger_v2_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scan_merger_v2_component rclcpp_components rclcpp sensor_msgs geometry_msgs tf2_ros tf2)

# Register as composable node and create executable
rclcpp_components_register_node(
    scan_merger_v2_component
    PLUGIN "ScanMergerV2"  # Replace with your actual class name
    EXECUTABLE scan_merger_v2
)

# FOR CAMERA NODES
# need to create shared library for composable node
add_library(tag_transform_component SHARED src/tag_transform.cpp)
target_include_directories(tag_transform_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tag_transform_component rclcpp_components rclcpp geometry_msgs tf2_ros tf2 tf2_geometry_msgs apriltag_msgs)

# Register as composable node and create executable
rclcpp_components_register_node(
    tag_transform_component
    PLUGIN "agv_hardware::TagTransformNode"  # Replace with your actual class name
    EXECUTABLE tag_transform
)

install(TARGETS
  scan_merger_v2_component  # Install the component library
  scan_merger_v2           # Install the executable
  tag_transform_component
  tag_transform
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python scripts
install(PROGRAMS
  src/vel_cmd_to_serial.py
  src/ZLAC8015D.py
  src/graceful_algo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()