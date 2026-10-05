cmake_minimum_required(VERSION 3.8)
project(perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)   # 벤치마크 수치가 Debug 빌드로 왜곡되지 않도록
endif()

# ---- 빌드 구성 ----
# 와플파이 배포 (기본): NCNN 백엔드 + perception_master 만
#   colcon build --packages-select perception --cmake-args -Dncnn_DIR=$HOME/ncnn-install/lib/cmake/ncnn
# 백엔드 비교·벤치마크 (tools/benchmark):
#   ... -DPERCEPTION_WITH_ONNX=ON -DONNXRUNTIME_ROOT=$HOME/onnxruntime -DPERCEPTION_BUILD_BENCH=ON
option(PERCEPTION_WITH_NCNN "Build NCNN backend" ON)
option(PERCEPTION_WITH_ONNX "Build ONNX Runtime backend (backend comparison only)" OFF)
option(PERCEPTION_BUILD_BENCH "Build detector_bench (offline benchmark)" OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime install dir (contains include/ and lib/)")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

# ---- perception_core: Detector(공통 + 백엔드), DepthExtractor ----
add_library(perception_core STATIC
  src/detector.cpp
  src/detector_factory.cpp
  src/depth_extractor.cpp)
set_target_properties(perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_core PUBLIC ${OpenCV_LIBS})

set(PERCEPTION_RPATH "")

if(PERCEPTION_WITH_NCNN)
  find_package(ncnn REQUIRED)   # ncnn 소스 빌드 후 install, 또는 -Dncnn_DIR=<prefix>/lib/cmake/ncnn
  target_sources(perception_core PRIVATE src/ncnn_detector.cpp)
  target_link_libraries(perception_core PUBLIC ncnn)
  target_compile_definitions(perception_core PUBLIC PERCEPTION_WITH_NCNN)
endif()

if(PERCEPTION_WITH_ONNX)
  find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_ROOT}/include ${ONNXRUNTIME_ROOT}/include/onnxruntime
          /usr/local/include/onnxruntime /usr/include/onnxruntime)
  find_library(ORT_LIBRARY onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib /usr/local/lib)
  if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIBRARY)
    message(FATAL_ERROR "ONNX Runtime not found. Set -DONNXRUNTIME_ROOT=<dir> or -DPERCEPTION_WITH_ONNX=OFF")
  endif()
  target_sources(perception_core PRIVATE src/onnx_detector.cpp)
  target_include_directories(perception_core PUBLIC ${ORT_INCLUDE_DIR})
  target_link_libraries(perception_core PUBLIC ${ORT_LIBRARY})
  target_compile_definitions(perception_core PUBLIC PERCEPTION_WITH_ONNX)
  get_filename_component(ORT_LIB_DIR ${ORT_LIBRARY} DIRECTORY)
  list(APPEND PERCEPTION_RPATH ${ORT_LIB_DIR})
endif()

if(NOT PERCEPTION_WITH_NCNN AND NOT PERCEPTION_WITH_ONNX)
  message(FATAL_ERROR "Enable at least one backend (PERCEPTION_WITH_NCNN / PERCEPTION_WITH_ONNX)")
endif()

# ---- perception_master 노드 ----
add_executable(perception_master src/perception_master.cpp)
# ament_target_dependencies는 Lyrical에서 삭제됨 → 패키지가 내보내는 CMake target에 직접 링크
target_link_libraries(perception_master perception_core
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS}
  cv_bridge::cv_bridge message_filters::message_filters)

set_target_properties(perception_master PROPERTIES INSTALL_RPATH "${PERCEPTION_RPATH}")
install(TARGETS perception_master DESTINATION lib/${PROJECT_NAME})

# 배포 모델 → share/perception/models (launch/perception.launch.py가 이 경로를 사용)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

# ---- detector_bench: bag/영상 오프라인 벤치마크 (테스트용) ----
if(PERCEPTION_BUILD_BENCH)
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  add_executable(detector_bench tools/detector_bench.cpp)
  target_link_libraries(detector_bench perception_core
    rclcpp::rclcpp ${sensor_msgs_TARGETS} cv_bridge::cv_bridge
    rosbag2_cpp::rosbag2_cpp rosbag2_storage::rosbag2_storage)
  target_compile_definitions(detector_bench PRIVATE BENCH_WITH_ROSBAG)
  set_target_properties(detector_bench PROPERTIES INSTALL_RPATH "${PERCEPTION_RPATH}")
  install(TARGETS detector_bench DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
