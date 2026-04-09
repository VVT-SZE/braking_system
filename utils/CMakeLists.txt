cmake_minimum_required(VERSION 3.8)
project(utils)

add_library(utils src/trajectoryCalculation.cpp)

target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
