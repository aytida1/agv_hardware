cmake_minimum_required(VERSION 3.8)
project(agv_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros)

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

# Install Python scripts
install(PROGRAMS
  src/vel_cmd_to_serial.py
  src/ZLAC8015D.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()