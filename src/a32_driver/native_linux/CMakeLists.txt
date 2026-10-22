cmake_minimum_required(VERSION 3.16)
project(a32_native_linux LANGUAGES CXX)
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The A3.2 POSIX driver must be built on Linux")
endif()
find_package(Threads REQUIRED)
include(CTest)
# Keep upstream source unchanged and build its protocol tests with the adapter.
# Only the SDK's protocol helpers are called: POSIX transport/lifecycle remains
# here so connecting, stopping or destroying a handle never powers motors off.
set(A3_SDK_BUILD_SHARED OFF CACHE BOOL "Build the SDK into the native adapter" FORCE)
set(A3_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Do not build vendor device examples" FORCE)
set(A3_SDK_BUILD_TESTS "${BUILD_TESTING}" CACHE BOOL "Test the bundled SDK" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/a3-cpp-sdk" "a3-cpp-sdk")
set_target_properties(a3_sdk PROPERTIES CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES)
add_library(a32_native SHARED a32_native_linux.cpp protocol.cpp)
target_compile_features(a32_native PRIVATE cxx_std_17)
target_compile_options(a32_native PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_link_libraries(a32_native PRIVATE Threads::Threads A3::a3_sdk)
set_target_properties(a32_native PROPERTIES CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES)
set(A32_NATIVE_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/../bin" CACHE PATH "Native library output directory")
set_target_properties(a32_native PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${A32_NATIVE_OUTPUT}")
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME linux_offline_transport COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_offline.py" "$<TARGET_FILE:a32_native>")
endif()
