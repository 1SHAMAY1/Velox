@echo off
if exist "build\bin\VeloxVisualizer.exe" (
    echo Starting Velox Visualizer...
    start "" "build\bin\VeloxVisualizer.exe"
) else (
    echo Visualizer not found at build\bin\VeloxVisualizer.exe! Building...
    cmake --build build --target VeloxVisualizer
    if %ERRORLEVEL% EQU 0 (
        start "" "build\bin\VeloxVisualizer.exe"
    ) else (
        echo Build failed!
    )
)
