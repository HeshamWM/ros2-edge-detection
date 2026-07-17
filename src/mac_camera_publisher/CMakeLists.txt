cmake_minimum_required(VERSION 3.8)
project(mac_camera_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# Shared component library
add_library(camera_publisher_component SHARED
  src/camera_publisher_node.cpp
)

target_include_directories(camera_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_publisher_component ${DEPENDENCIES})
target_link_libraries(camera_publisher_component ${OpenCV_LIBS})

# Register component
rclcpp_components_register_node(camera_publisher_component
  PLUGIN "mac_camera_publisher::CameraPublisherNode"
  EXECUTABLE camera_publisher_standalone
)

# Install
install(TARGETS camera_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
