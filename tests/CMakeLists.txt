# Tests

add_executable(VeloxTests StressTest.cpp)

target_link_libraries(VeloxTests PRIVATE Velox)
