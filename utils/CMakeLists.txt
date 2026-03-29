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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS utils
  EXPORT utils_targets
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(utils_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()