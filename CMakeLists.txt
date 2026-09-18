cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0048 NEW)

project(Velox VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

enable_testing()

# Options
option(VELOX_SIMD "Enable SIMD math optimizations" ON)

# Subdirectories
add_subdirectory(src)
add_subdirectory(visualizer)

# Tests & Benchmarks
set(TEST_FILES
    test_math
    test_ecs
    test_primitives
    test_joints_softbody
    test_gameplay_components
    test_raycasting
    test_rigidbody_stacking
    test_rigidbody_restitution
    test_rigidbody_ccd
    test_rigidbody_ccd_bullet
    test_rigidbody_friction_slope
    test_softbody_area_volume
    test_softbody_shape_match
    test_softbody_shape_matching
    test_chaos_stress
    test_physics_stress
    test_decentralized_behavior
)

foreach(TEST_NAME ${TEST_FILES})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/${TEST_NAME}.cpp")
        add_executable(${TEST_NAME} tests/${TEST_NAME}.cpp)
        target_link_libraries(${TEST_NAME} PRIVATE Velox)
        target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endif()
endforeach()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/benchmark_physics.cpp")
    add_executable(benchmark_physics benchmarks/benchmark_physics.cpp)
    target_link_libraries(benchmark_physics PRIVATE Velox)
    target_include_directories(benchmark_physics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

include(FetchContent)

# Upstream Native Box2D
FetchContent_Declare(
    box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v2.4.1
    GIT_SHALLOW TRUE
)
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_TESTBED OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(box2d)

# Upstream Native Chipmunk2D
FetchContent_Declare(
    chipmunk
    GIT_REPOSITORY https://github.com/slembcke/Chipmunk2D.git
    GIT_TAG master
    GIT_SHALLOW TRUE
)
set(BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(BUILD_STATIC ON CACHE BOOL "" FORCE)
set(BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(INSTALL_DEMOS OFF CACHE BOOL "" FORCE)
set(INSTALL_STATIC OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(chipmunk)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/benchmark_real_engines.cpp")
    add_executable(benchmark_real_engines benchmarks/benchmark_real_engines.cpp)
    target_link_libraries(benchmark_real_engines PRIVATE Velox box2d chipmunk_static)
    target_include_directories(benchmark_real_engines PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include 
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${box2d_SOURCE_DIR}/include
        ${chipmunk_SOURCE_DIR}/include
    )
endif()


