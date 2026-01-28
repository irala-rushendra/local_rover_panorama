cmake_minimum_required(VERSION 3.8)
project(final_panorama)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(panorama_service_node src/panorama_service_node.cpp)

ament_target_dependencies(panorama_service_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS panorama_service_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
