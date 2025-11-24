cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0048 NEW)

project(Velox VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Options
option(VELOX_BUILD_TESTS "Build tests" ON)

# Subdirectories
add_subdirectory(src)

if(VELOX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
