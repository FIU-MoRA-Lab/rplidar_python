cmake_minimum_required(VERSION 3.4)
project(lidar_module)

# Find the pybind11 package
find_package(pybind11 REQUIRED)

# Add the SDK include directory (replace `sdk/include` with the actual path)
include_directories(${CMAKE_SOURCE_DIR}/rplidar_sdk/sdk/include)

# Define the Python module
pybind11_add_module(rplidar lidar_wrap.cpp)

# Link any libraries required by the SDK
target_link_libraries(rplidar PRIVATE ${CMAKE_SOURCE_DIR}/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a)  # Replace with actual library file name and path if needed
