cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

add_executable(ros2_kdl_node src/ros2_kdl_node.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)
target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
  rclcpp_action  
  action_msgs    
)

rosidl_get_typesupport_target(
  kdl_node_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

target_link_libraries(ros2_kdl_node
  ${kdl_node_target})

add_executable(ros2_vision_control_node src/ros2_vision_control_node.cpp src/kdl_robot.cpp src/kdl_control.cpp)
target_include_directories(ros2_vision_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_vision_control_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_vision_control_node
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node_example
  orocos_kdl
  rclcpp
)

add_executable(action_client_node src/action_client_node.cpp)
target_include_directories(action_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_client_node
  rclcpp
  rclcpp_action
  geometry_msgs
  action_msgs
)

rosidl_get_typesupport_target(
  client_node_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

target_link_libraries(action_client_node
  ${client_node_target})

add_executable(action_server_node src/action_server_node.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)
target_include_directories(action_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(action_server_node
  rclcpp
  rclcpp_action
  geometry_msgs
  action_msgs
  orocos_kdl
  kdl_parser
)

rosidl_get_typesupport_target(
  server_node_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

target_link_libraries(action_server_node
  ${server_node_target})


install(TARGETS
  ros2_kdl_node
  ros2_vision_control_node
  ros2_kdl_node_example
  action_client_node
  action_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Salta il controllo copyright
  set(ament_cmake_copyright_FOUND TRUE)
  # Salta cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
