cmake_minimum_required(VERSION 3.16)
project(AeroStabilize-1D)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(aerostabilize_core STATIC
    src/Config.cpp
    src/PhysicsEngine.cpp
    src/PIDController.cpp
    src/TelemetryLogger.cpp
    src/PerformanceAnalyzer.cpp
)
target_include_directories(aerostabilize_core PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(AeroStabilize-1D src/main.cpp)
target_link_libraries(AeroStabilize-1D PRIVATE aerostabilize_core)

# Place config next to the executable so runs from the build directory work.
configure_file(
    "${CMAKE_SOURCE_DIR}/config.txt"
    "${CMAKE_BINARY_DIR}/config.txt"
    COPYONLY
)

include(CTest)
if(BUILD_TESTING)
    include(FetchContent)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    FetchContent_MakeAvailable(googletest)
    enable_testing()
    add_subdirectory(tests)
endif()
