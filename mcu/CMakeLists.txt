cmake_minimum_required(VERSION 3.16)
project(wheel_rail_mcu LANGUAGES C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

add_library(wheel_rail_mcu_core STATIC
    src/app/entry.c
    src/device/wr_can_protocol.c
    src/device/benmo_drive_motor.c
    src/device/rs06_steer_motor.c
    src/device/fs_ia10b.c
    src/domain/steer_home_control.c
    src/domain/steer_wheel_kinematics.c
    src/infra/log.c
    src/service/chassis_service.c
    src/service/host_link_service.c
)
target_include_directories(wheel_rail_mcu_core PUBLIC
    src/app src/device src/domain src/infra src/platform src/service)
target_link_libraries(wheel_rail_mcu_core PUBLIC m)
