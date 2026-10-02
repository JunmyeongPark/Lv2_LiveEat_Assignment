cmake_minimum_required(VERSION 3.8)
project(perception)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)

# TODO: targets

ament_package()

find_package(OpenCV REQUIRED)
find_package(ncnn REQUIRED)            # ncnn을 소스 빌드 후 install하면 생김
find_package(message_filters REQUIRED)

add_library(perception_core src/detector.cpp src/depth_extractor.cpp)
target_include_directories(perception_core PUBLIC include)
target_link_libraries(perception_core ${OpenCV_LIBS} ncnn)

add_executable(perception_master src/perception_master.cpp)
target_link_libraries(perception_master perception_core
  rclcpp::rclcpp cv_bridge::cv_bridge message_filters::message_filters
  ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS})

install(TARGETS perception_master DESTINATION lib/${PROJECT_NAME})