cmake_minimum_required(VERSION 3.1)
project(nmpc_solver CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

set(MODEL_DIR ${CMAKE_SOURCE_DIR}/examples/quad1d)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(src)
include_directories(${MODEL_DIR})

add_subdirectory(${MODEL_DIR})
add_subdirectory(src)
add_executable(sim ${MODEL_DIR}/main.cpp)

target_link_libraries(sim simulator)
target_link_libraries(sim nmpcsolver)
target_link_libraries(sim model)
