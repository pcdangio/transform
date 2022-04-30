cmake_minimum_required(VERSION 3.2)

# Set up project.
project(transform
        VERSION 1.0
        DESCRIPTION "A C++ library for performing geometric transformations.")

# Find dependencies.
find_package(Eigen3 REQUIRED)

# Create library.
add_library(transform STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/transform.cpp)

# Link library to dependencies.
target_link_libraries(transform
    Eigen3::Eigen)

# Set include directories for internal/external use.
target_include_directories(transform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)