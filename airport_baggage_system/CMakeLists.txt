cmake_minimum_required(VERSION 3.8)
project(airport_baggage_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros2_kdl_package REQUIRED)
find_package(nav2_msgs REQUIRED) 


add_executable(iiwa_manager_cpp src/iiwa_manager.cpp)


ament_target_dependencies(iiwa_manager_cpp
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  ros_gz_interfaces
  ros2_kdl_package
)

add_executable(fra2mo_manager_cpp src/fra2mo_manager.cpp)

ament_target_dependencies(fra2mo_manager_cpp
   rclcpp 
   rclcpp_action 
   std_msgs 
   nav2_msgs 
   geometry_msgs
   ros_gz_interfaces
)


install(TARGETS
  fra2mo_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  iiwa_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/color_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
