cmake_minimum_required(VERSION 3.15)
project(manus_glove LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

set(MANUS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/manus")
set(MANUS_LIB_DIR "${MANUS_SDK_DIR}/lib")
set(MANUS_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/linux-x86_64")

pybind11_add_module(manus_glove
    src/ManusGlove.cpp
    src/bindings.cpp
)

target_include_directories(manus_glove PRIVATE "${MANUS_SDK_DIR}/include")
# The vendor library has no SONAME. Linking its absolute path would bake this
# checkout's location into DT_NEEDED, so link by its library name instead.
target_link_directories(manus_glove PRIVATE "${MANUS_LIB_DIR}")
target_link_libraries(manus_glove PRIVATE ManusSDK)

set_target_properties(manus_glove PROPERTIES
    BUILD_RPATH "\$ORIGIN/../../vendor/manus/lib;\$ORIGIN/../../runtime/linux-x86_64"
    INSTALL_RPATH "\$ORIGIN/../../vendor/manus/lib;\$ORIGIN/../../runtime/linux-x86_64"
    BUILD_WITH_INSTALL_RPATH TRUE
)

target_link_options(manus_glove PRIVATE "-Wl,--disable-new-dtags")

include(CTest)
if(BUILD_TESTING)
    foreach(test_name snapshot lifecycle)
        add_executable(test_${test_name} tests/test_${test_name}.cpp src/ManusGlove.cpp)
        target_include_directories(test_${test_name} PRIVATE src "${MANUS_SDK_DIR}/include")
        target_link_directories(test_${test_name} PRIVATE "${MANUS_LIB_DIR}")
        target_link_libraries(test_${test_name} PRIVATE ManusSDK pthread)
        target_compile_options(test_${test_name} PRIVATE -UNDEBUG)
        target_link_options(test_${test_name} PRIVATE "-Wl,--disable-new-dtags"
            "-Wl,-rpath-link,${MANUS_RUNTIME_DIR}")
        set_target_properties(test_${test_name} PROPERTIES
            BUILD_WITH_INSTALL_RPATH TRUE
            INSTALL_RPATH "\$ORIGIN/../../vendor/manus/lib;\$ORIGIN/../../runtime/linux-x86_64")
        add_test(NAME manus_${test_name} COMMAND test_${test_name})
    endforeach()
    target_link_options(test_snapshot PRIVATE
        "-Wl,--wrap=CoreSdk_GetRawSkeletonInfo" "-Wl,--wrap=CoreSdk_GetRawSkeletonData")
    foreach(sdk_function InitializeIntegrated InitializeCore ShutDown
            RegisterCallbackForOnLog RegisterCallbackForOnDisconnect RegisterCallbackForSystemStream
            RegisterCallbackForRawSkeletonStream RegisterCallbackForLandscapeStream
            RegisterCallbackForErgonomicsStream InitializeCoordinateSystemWithVUH
            ConnectToHost LookForHosts GetNumberOfAvailableHostsFound
            GetAvailableHostsFound SetRawSkeletonHandMotion)
        target_link_options(test_lifecycle PRIVATE "-Wl,--wrap=CoreSdk_${sdk_function}")
    endforeach()
endif()
