cmake_minimum_required(VERSION 3.8)
project(plan_long_emergency)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)
find_package(utils REQUIRED)

include_directories(include)

add_executable(plan_long_emergency "src/motionHandler.cpp")
ament_target_dependencies(plan_long_emergency rclcpp crp_msgs autoware_planning_msgs tier4_planning_msgs)
target_link_libraries(plan_long_emergency utils::utils)

install(TARGETS
  plan_long_emergency
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()