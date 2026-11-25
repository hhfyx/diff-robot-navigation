cmake_minimum_required(VERSION 3.8)
project(diffbot_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/diffbot_system.cpp
  src/diffbot_serial_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  gazebo_dev
  gazebo_ros2_control
)

# 硬件插件声明：这两个类都挂在 gazebo_ros2_control::GazeboSystemInterface /
# hardware_interface::SystemInterface 下，供 controller_manager 的 hardware_components 加载。
pluginlib_export_plugin_description_file(gazebo_ros2_control diffbot_control_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES diffbot_control_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
