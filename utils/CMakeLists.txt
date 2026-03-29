cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(utils
  src/trajectoryCalculation.cpp
)

target_include_directories(utils PUBLIC
  include
)

install(
  TARGETS utils
  DESTINATION lib
)
add_executable(test_trajectory test/test_trajectory.cpp)
target_link_libraries(test_trajectory utils)
install(TARGETS test_trajectory DESTINATION lib/${PROJECT_NAME})
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

ament_package()