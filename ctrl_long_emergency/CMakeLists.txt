cmake_minimum_required(VERSION 3.8)
project(ctrl_long_emergency)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

include_directories(include)

add_executable(ctrl_long_emergency src/ctrlLongEmergency.cpp)
ament_target_dependencies(ctrl_long_emergency rclcpp crp_msgs autoware_control_msgs)


install(TARGETS
  ctrl_long_emergency
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
