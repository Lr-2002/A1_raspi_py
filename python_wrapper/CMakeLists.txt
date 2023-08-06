cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk_python_wrapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3")

find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

include_directories(../include)

link_directories(../lib)
file(GLOB_RECURSE sources "cpp/*.cpp")

set(EXTRA_LIBS -pthread rt m libunitree_legged_sdk_amd64.so lcm) # change to libunitree_legged_sdk_arm64.so when working with the NX

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib/python)

add_subdirectory(third-party/pybind11)

pybind11_add_module(robot_interface python_interface.cpp)
target_link_libraries(robot_interface ${EXTRA_LIBS})

