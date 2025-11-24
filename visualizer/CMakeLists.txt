include(FetchContent)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/heads/master.tar.gz
)

FetchContent_MakeAvailable(raylib)

if(WIN32)
    add_executable(VeloxVisualizer Main.cpp app.rc)
else()
    add_executable(VeloxVisualizer Main.cpp)
endif()

# Copy assets to bin directory
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../bin/Release)
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../bin/Debug)

target_link_libraries(VeloxVisualizer PRIVATE raylib Velox Psapi)

# Copy DLL to output directory on Windows
if(WIN32)
    add_custom_command(TARGET VeloxVisualizer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:Velox>
        $<TARGET_FILE_DIR:VeloxVisualizer>
    )
endif()
