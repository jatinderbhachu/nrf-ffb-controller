cmake_minimum_required(VERSION 3.20.0)

set(BOARD promicro_nrf52840/nrf52840/uf2)

# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
find_package(Zephyr REQUIRED HINTS C:/ncs/main_branch/zephyr)

project(openffboard)

target_include_directories(
    app
    PRIVATE
    src/
)

target_sources(
    app 
    PRIVATE
    src/main.c
)