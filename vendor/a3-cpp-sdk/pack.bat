@echo off
setlocal
cd /d "%~dp0"

if not exist build\CMakeCache.txt (
    echo Run build.bat first.
    exit /b 1
)

cmake --build build --config Release
if errorlevel 1 exit /b 1

ctest --test-dir build -C Release --output-on-failure --no-tests=error
if errorlevel 1 exit /b 1

cmake --install build --config Release --prefix dist\a3-cpp-sdk
if errorlevel 1 exit /b 1

cmake -E chdir dist cmake -E tar cf a3-cpp-sdk.zip --format=zip a3-cpp-sdk
if errorlevel 1 exit /b 1

echo.
echo SDK pack: dist\a3-cpp-sdk
echo   include\   public headers
echo   lib\       a3_sdk.lib
echo   bin\       a3_example.exe a3_web_bridge.exe
echo Ship that folder, not the H7 firmware tree.
