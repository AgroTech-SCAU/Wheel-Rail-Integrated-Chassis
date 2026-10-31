cmake_minimum_required(VERSION 3.20)
project(wheel_rail_sdk VERSION 1.0.0 LANGUAGES C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(WHEEL_RAIL_SDK_STM32_PORT "Build the existing STM32 HAL adapter" ON)
option(WHEEL_RAIL_SDK_BUILD_TESTS "Build tests with a simulated board (native compiler)" OFF)
option(WHEEL_RAIL_SDK_BUILD_EXAMPLES "Compile the upper application example" OFF)
set(WHEEL_RAIL_SDK_HAL_TARGET "" CACHE STRING
    "Existing CubeMX/HAL CMake target providing headers and MCU definitions")

add_library(wheel_rail_sdk STATIC
    src/service/chassis_service.c
    src/device/benmo_drive_motor.c
    src/device/rs06_steer_motor.c
    src/device/fs_ia10b.c
    src/domain/steer_home_control.c
    src/domain/steer_wheel_kinematics.c
    src/infra/log.c
)
add_library(wheel_rail::sdk ALIAS wheel_rail_sdk)
set_target_properties(wheel_rail_sdk PROPERTIES EXPORT_NAME sdk)
target_compile_features(wheel_rail_sdk PUBLIC c_std_11)
target_include_directories(wheel_rail_sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/service>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/domain>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/wheel_rail_sdk>
    PRIVATE src/device src/infra src/platform
)

if(WHEEL_RAIL_SDK_STM32_PORT)
    target_sources(wheel_rail_sdk PRIVATE
        src/platform/stm32_board_port.c
        src/platform/stm32_fdcan_port.c
        src/platform/stm32_uart_port.c
        src/platform/stm32_time_port.c
    )
    if(WHEEL_RAIL_SDK_HAL_TARGET)
        if(NOT TARGET ${WHEEL_RAIL_SDK_HAL_TARGET})
            message(FATAL_ERROR "WHEEL_RAIL_SDK_HAL_TARGET must name an existing target")
        endif()
        # Installed users supply their own HAL; do not export a source-tree target.
        target_link_libraries(wheel_rail_sdk PRIVATE
            $<BUILD_INTERFACE:${WHEEL_RAIL_SDK_HAL_TARGET}>)
    endif()
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(wheel_rail_sdk PRIVATE -Wall -Wextra)
    if(NOT MSVC)
        target_link_libraries(wheel_rail_sdk PUBLIC m)
    endif()
endif()

if(WHEEL_RAIL_SDK_BUILD_EXAMPLES)
    add_library(wheel_rail_upper_app_example STATIC sdk/examples/upper_app.c)
    target_link_libraries(wheel_rail_upper_app_example PRIVATE wheel_rail::sdk)
endif()

install(TARGETS wheel_rail_sdk EXPORT WheelRailSDKTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES sdk/wheel_rail_sdk.h src/service/chassis_service.h
    src/domain/steer_wheel_kinematics.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wheel_rail_sdk)
install(FILES src/platform/stm32_board_port.h src/platform/stm32_fdcan_port.h
    src/platform/stm32_uart_port.h src/platform/stm32_time_port.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wheel_rail_sdk/ports)
install(DIRECTORY sdk/examples/ DESTINATION ${CMAKE_INSTALL_DATADIR}/wheel_rail_sdk/examples)
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DATADIR}/wheel_rail_sdk)
configure_package_config_file(cmake/WheelRailSDKConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/WheelRailSDKConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/WheelRailSDK)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/WheelRailSDKConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/WheelRailSDKConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/WheelRailSDKConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/WheelRailSDK)
install(EXPORT WheelRailSDKTargets NAMESPACE wheel_rail::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/WheelRailSDK)

if(WHEEL_RAIL_SDK_BUILD_TESTS)
    if(WHEEL_RAIL_SDK_STM32_PORT OR CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR "Tests require STM32_PORT=OFF and a native compiler")
    endif()
    enable_testing()
    add_executable(wheel_rail_sdk_test tests/sdk_test.c)
    target_include_directories(wheel_rail_sdk_test PRIVATE src/platform)
    target_link_libraries(wheel_rail_sdk_test PRIVATE wheel_rail::sdk)
    # Separate processes keep the production singleton lifecycle intact.
    foreach(scenario external remote timeout_wrap init_failure)
        add_test(NAME sdk_${scenario} COMMAND wheel_rail_sdk_test ${scenario})
    endforeach()
endif()
