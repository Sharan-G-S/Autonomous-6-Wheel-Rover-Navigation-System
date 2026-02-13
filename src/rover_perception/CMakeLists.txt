cmake_minimum_required(VERSION 3.8)
project(rover_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY config launch scripts models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/object_detector.py
  scripts/obstacle_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
