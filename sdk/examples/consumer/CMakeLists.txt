cmake_minimum_required(VERSION 3.20)
project(wheel_rail_upper_app_consumer LANGUAGES C)

# Configure with -DCMAKE_PREFIX_PATH=<SDK install prefix> and the same ARM toolchain.
find_package(WheelRailSDK 1.0 CONFIG REQUIRED)
add_library(upper_app_consumer STATIC ../upper_app.c)
target_link_libraries(upper_app_consumer PRIVATE wheel_rail::sdk)
# The final MCU firmware also provides startup, HAL or custom ports, and a linker script.
