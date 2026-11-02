cmake_minimum_required(VERSION 3.16)
project(wheel_rail LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "This SDK uses Linux SocketCAN")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(_wheel_rail_examples_default OFF)
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(_wheel_rail_examples_default ON)
endif()
option(WHEEL_RAIL_BUILD_EXAMPLES "Build monitor and explicit control examples" ${_wheel_rail_examples_default})

add_library(wheel_rail_sdk
    src/error.cpp
    src/protocol.cpp
    src/socketcan.cpp
    src/client.cpp
)
add_library(wheel_rail::sdk ALIAS wheel_rail_sdk)
set_target_properties(wheel_rail_sdk PROPERTIES EXPORT_NAME sdk)
target_compile_features(wheel_rail_sdk PUBLIC cxx_std_17)
target_include_directories(wheel_rail_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(wheel_rail_sdk PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WHEEL_RAIL_BUILD_EXAMPLES)
    foreach(name monitor control)
        add_executable(wheel_rail_${name} examples/${name}.cpp)
        target_link_libraries(wheel_rail_${name} PRIVATE wheel_rail::sdk)
        install(TARGETS wheel_rail_${name} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    endforeach()
endif()

install(TARGETS wheel_rail_sdk EXPORT wheel_railTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/wheel_rail)
install(DIRECTORY examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/wheel_rail/examples)
configure_package_config_file(cmake/wheel_railConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/wheel_railConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wheel_rail)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wheel_railConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wheel_rail)
install(EXPORT wheel_railTargets NAMESPACE wheel_rail::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wheel_rail)
