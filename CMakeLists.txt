cmake_minimum_required(VERSION 3.14)
project(servo_st VERSION 1.0.0 LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# ------------------------------------------------------------------------------
# Library configuration
# Assuming a header-only library since there are no source files in the base directory other than the ones in app/
# If you add source files for the library, change INTERFACE to STATIC or SHARED
# ------------------------------------------------------------------------------
add_library(servo_st INTERFACE)

target_include_directories(servo_st INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# Executables (Tools)
# ------------------------------------------------------------------------------

# st_scan tool
add_executable(st_scan app/st_scan.cpp)
target_link_libraries(st_scan PRIVATE servo_st)

# st_cli tool
add_executable(st_cli app/st_cli.cpp)
target_link_libraries(st_cli PRIVATE servo_st)
