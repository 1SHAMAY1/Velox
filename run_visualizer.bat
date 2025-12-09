@echo off
rem Configure the project (incremental)
if not exist build_viz mkdir build_viz
cmake -S . -B build_viz -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

rem Build the visualizer executable
cmake --build build_viz --config Release --target VeloxVisualizer

rem Run the visualizer if build succeeds
if %ERRORLEVEL% EQU 0 (
    start "Velox Visualizer" "build_viz\bin\Release\VeloxVisualizer.exe"
) else (
    echo Build failed!
)
