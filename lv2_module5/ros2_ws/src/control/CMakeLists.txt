cmake_minimum_required(VERSION 3.8)
project(control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ROS 와 무관한 계산 · 시리얼 모듈
add_library(control_core
  src/base_kinematics.cpp
  src/arm_command.cpp
  src/serial_bridge.cpp
)
target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 노드: 주기마다 ① RX → ② Base → ③ Arm → ④ TX
add_executable(control_master src/control_master.cpp)
target_link_libraries(control_master
  control_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

install(TARGETS control_core control_master
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

# Python 도구: OpenCR IMU 단독 드라이버 (ros2 run control imu_driver.py)
install(PROGRAMS scripts/imu_driver.py DESTINATION lib/${PROJECT_NAME})

ament_package()
