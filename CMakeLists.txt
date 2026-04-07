cmake_minimum_required(VERSION 3.10)
project(AeroStabilize-1D)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(AeroStabilize-1D 
    src/main.cpp
    src/PhysicsEngine.cpp
    src/PIDController.cpp
    src/TelemetryLogger.cpp
)