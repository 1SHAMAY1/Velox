project(VeloxVisualizer)

cmake_minimum_required(VERSION 3.11)

include(FetchContent)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
)
FetchContent_MakeAvailable(raylib)

add_executable(VeloxVisualizer Main.cpp icon.rc)

target_link_libraries(VeloxVisualizer PRIVATE raylib Velox)

set_target_properties(VeloxVisualizer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Copy assets to the output directory
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)
