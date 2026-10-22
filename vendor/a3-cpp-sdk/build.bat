@echo off
setlocal
cd /d "%~dp0"

if exist build\CMakeCache.txt (
    findstr /C:"Visual Studio 17 2022" build\CMakeCache.txt >nul 2>&1
    if not errorlevel 1 (
        echo 检测到旧的 VS 2022 缓存，正在删除 build 目录...
        rmdir /s /q build
    )
)

cmake -S . -B build -G "Visual Studio 18 2026" -A x64
if errorlevel 1 (
    echo.
    echo Visual Studio 2026 生成器失败，尝试 VS 2022...
    cmake -S . -B build -G "Visual Studio 17 2022" -A x64
)
if errorlevel 1 (
    echo.
    echo 若提示 generator 不匹配，先删掉 build 目录再运行本脚本。
    exit /b 1
)
cmake --build build --config Release
if errorlevel 1 exit /b 1
echo.
echo HTML 上位机: build\Release\a3_web_bridge.exe
echo 示例: build\Release\a3_example.exe --list
echo 然后浏览器打开 http://127.0.0.1:8080/
