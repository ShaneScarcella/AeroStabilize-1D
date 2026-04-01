cmake_minimum_required(VERSION 3.10)
project(AeroStabilize-1D)

# Ensure we use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the executable
add_executable(AeroStabilize-1D src/main.cpp)