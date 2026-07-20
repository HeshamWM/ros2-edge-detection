cmake_minimum_required(VERSION 3.8)
project(pi_yolov26_detector)

# ---------- Compiler settings ----------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17 (required by rclcpp)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pi-native optimisation: enables ARM NEON SIMD when built on the Pi
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  add_compile_options(-march=native -mtune=native)
endif()

# ---------- Find dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Collect all dependencies in one list for ament_target_dependencies()
set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  diagnostic_msgs
  cv_bridge
  image_transport
)

# ---------- Shared component library ----------
# Builds Yolov26DetectorNode as a composable component.
add_library(yolov26_detector_component SHARED
  src/yolov26_detector_node.cpp
  src/perf_monitor.cpp
)

target_include_directories(yolov26_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(yolov26_detector_component ${DEPENDENCIES})
target_link_libraries(yolov26_detector_component ${OpenCV_LIBS})

# Register the node as a composable component
rclcpp_components_register_node(yolov26_detector_component
  PLUGIN "pi_yolov26_detector::Yolov26DetectorNode"
  EXECUTABLE yolov26_detector_standalone
)

# ---------- Install ----------
install(TARGETS yolov26_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
