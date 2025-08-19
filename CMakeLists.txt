cmake_minimum_required(VERSION 3.16)
project(dht22_sensor)

# Dependencias ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# libgpiod via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

add_executable(dht22_node src/dht22_node.cpp)
target_include_directories(dht22_node PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_libraries(dht22_node ${GPIOD_LIBRARIES})
ament_target_dependencies(dht22_node rclcpp sensor_msgs)

install(TARGETS dht22_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
