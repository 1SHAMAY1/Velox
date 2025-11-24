@echo off
rem Clean previous build artifacts
if exist build_viz rmdir /s /q build_viz
mkdir build_viz

rem Configure the project
cmake -S . -B build_viz -DCMAKE_BUILD_TYPE=Release

rem Build the visualizer executable
cmake --build build_viz --config Release --target VeloxVisualizer

rem Run the visualizer
start "Velox Visualizer" "build_viz\bin\Release\VeloxVisualizer.exe"
