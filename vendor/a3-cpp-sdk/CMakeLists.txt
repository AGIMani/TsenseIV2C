cmake_minimum_required(VERSION 3.15)
project(a3-cpp-sdk VERSION 1.1.15 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(A3_SDK_IS_TOPLEVEL ON)
else()
    set(A3_SDK_IS_TOPLEVEL OFF)
endif()

option(A3_SDK_BUILD_SHARED "Build a3_sdk as a shared library" OFF)
option(A3_SDK_BUILD_EXAMPLES "Build example programs and web bridge" ${A3_SDK_IS_TOPLEVEL})
option(A3_SDK_BUILD_TESTS "Build protocol unit tests" ${A3_SDK_IS_TOPLEVEL})

if(MSVC)
    add_compile_options(/utf-8 /EHsc)
endif()

if(UNIX)
    # glibc hides cfmakeraw / getifaddrs when CMAKE_CXX_EXTENSIONS=OFF (-std=c++17).
    add_compile_definitions(_DEFAULT_SOURCE)
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/a3/Version.h
    @ONLY
)

if(A3_SDK_BUILD_SHARED)
    set(A3_SDK_LIB_TYPE SHARED)
else()
    set(A3_SDK_LIB_TYPE STATIC)
endif()

add_library(a3_sdk ${A3_SDK_LIB_TYPE}
    src/Protocol.cpp
    src/Stroke.cpp
    src/UsbCdc.cpp
    src/Tcp5001.cpp
    src/A3HandApi.cpp
)
add_library(A3::a3_sdk ALIAS a3_sdk)
target_compile_features(a3_sdk PUBLIC cxx_std_17)
if(MSVC)
    target_compile_options(a3_sdk PRIVATE /EHsc)
endif()

set_target_properties(a3_sdk PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    OUTPUT_NAME a3_sdk
    EXPORT_NAME a3_sdk
    WINDOWS_EXPORT_ALL_SYMBOLS OFF
)
if(UNIX)
    set_target_properties(a3_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
if(A3_SDK_BUILD_SHARED AND NOT WIN32)
    set_target_properties(a3_sdk PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
endif()

target_include_directories(a3_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(A3_SDK_BUILD_SHARED)
    target_compile_definitions(a3_sdk PUBLIC A3_SDK_SHARED PRIVATE A3_SDK_BUILDING)
endif()

if(WIN32)
    target_link_libraries(a3_sdk PUBLIC ws2_32 setupapi winmm uuid iphlpapi)
elseif(UNIX)
    find_package(Threads REQUIRED)
    target_link_libraries(a3_sdk PUBLIC Threads::Threads)
endif()

if(A3_SDK_BUILD_EXAMPLES)
    add_executable(a3_example examples/enable_and_hold.cpp)
    target_link_libraries(a3_example PRIVATE a3_sdk)

    add_executable(a3_web_bridge examples/web_bridge.cpp)
    target_link_libraries(a3_web_bridge PRIVATE a3_sdk)

    add_custom_command(TARGET a3_web_bridge POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/webui/static
            $<TARGET_FILE_DIR:a3_web_bridge>/webui/static
    )
endif()

if(A3_SDK_BUILD_TESTS)
    enable_testing()
    add_executable(a3_test_protocol tests/test_protocol.cpp)
    target_link_libraries(a3_test_protocol PRIVATE a3_sdk)
    add_test(NAME a3_test_protocol COMMAND a3_test_protocol)
    add_executable(a3_test_web_commands tests/test_web_commands.cpp)
    target_link_libraries(a3_test_web_commands PRIVATE a3_sdk)
    add_test(NAME a3_test_web_commands COMMAND a3_test_web_commands)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS a3_sdk
    EXPORT a3-cpp-sdk-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
if(A3_SDK_BUILD_EXAMPLES)
    install(TARGETS a3_example a3_web_bridge
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/webui/static
        DESTINATION ${CMAKE_INSTALL_BINDIR}/webui
    )
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/a3/Version.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/a3
)
install(DIRECTORY webui/ DESTINATION ${CMAKE_INSTALL_DATADIR}/a3-cpp-sdk/webui)
if(A3_SDK_IS_TOPLEVEL)
    install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/README.md
        ${CMAKE_CURRENT_SOURCE_DIR}/README.en.md
        ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
        ${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.md
        ${CMAKE_CURRENT_SOURCE_DIR}/COMPAT.md
        ${CMAKE_CURRENT_SOURCE_DIR}/linux/99-a3-h7-cdc.rules
        DESTINATION .
    )
endif()

install(EXPORT a3-cpp-sdk-targets
    FILE a3-cpp-sdk-targets.cmake
    NAMESPACE A3::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/a3-cpp-sdk
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/a3-cpp-sdk-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/a3-cpp-sdk-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/a3-cpp-sdk
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/a3-cpp-sdk-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/a3-cpp-sdk-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/a3-cpp-sdk-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/a3-cpp-sdk
)
