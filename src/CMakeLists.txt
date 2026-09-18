# Velox Source

# Collect sources (globally for now, can be explicit later)
file(GLOB_RECURSE VELOX_SOURCES 
    "core/*.cpp" "core/*.h"
    "math/*.cpp" "math/*.h"
    "physics/*.cpp" "physics/*.h"
    "api/*.cpp" "api/*.h"
)

add_library(Velox STATIC ${VELOX_SOURCES})

# Include directories
target_include_directories(Velox PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/math
    ${CMAKE_CURRENT_SOURCE_DIR}/physics
    ${CMAKE_CURRENT_SOURCE_DIR}/api
)

# Compiler flags & optimizations
if(MSVC)
    target_compile_options(Velox PRIVATE /W4 /O2 /arch:AVX2 /fp:fast /openmp)
else()
    target_compile_options(Velox PRIVATE -Wall -Wextra -O3 -mavx2 -ffast-math -fopenmp)
endif()

target_compile_definitions(Velox PUBLIC VELOX_STATIC)

if(VELOX_SIMD)
    target_compile_definitions(Velox PRIVATE VELOX_SIMD)
endif()