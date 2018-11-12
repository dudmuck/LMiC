cmake_minimum_required(VERSION 3.6)

set(CMAKE_TOOLCHAIN_FILE "../arm-gcc-toolchain.cmake")
project(lmic C ASM)

set(target "MOTE_L152RC" CACHE STRING "target chosen by the user at CMake configure time")
set_property(CACHE target PROPERTY STRINGS NUCLEO_L073RZ NUCLEO_L152RE NRF52_DK NUCLEO_L476RG MOTE_L152RC)

set(radio "sx126x" CACHE STRING "radio chosen by the user at CMake configure time")
set_property(CACHE radio PROPERTY STRINGS sx1272 sx1276 sx126x sx1280)

set(lorawan_region "eu868" CACHE STRING "region chosen by the user at CMake configure time")
set_property(CACHE lorawan_region PROPERTY STRINGS us915 eu868 as923)

set(stm32cube_drivers "" CACHE PATH "/home/user/STM32Cube_FW_Lx_Vx.xx.x/Drivers")
set(nrf_sdk "" CACHE PATH "/home/user/nRF5_SDK_x.x.x_xxxxxxx")

set(softdevice "s132" CACHE STRING "softdevice chosen by the user at CMake configure time")
set_property(CACHE softdevice PROPERTY STRINGS s112 s140 s132)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -Wall -fmessage-length=0 -ffunction-sections")


list(APPEND lmic_sources
    ${PROJECT_SOURCE_DIR}/hal/lpTimeout.c
    ${PROJECT_SOURCE_DIR}/hal/mbed_critical.c
    ${PROJECT_SOURCE_DIR}/hal/mbed_critical_section_api.c
    ${PROJECT_SOURCE_DIR}/hal/ticker_api.c
    ${PROJECT_SOURCE_DIR}/hal/sleep_manager.c
    ${PROJECT_SOURCE_DIR}/hal/timer_event.c
    ${PROJECT_SOURCE_DIR}/hal/lp_ticker_api.c
    ${PROJECT_SOURCE_DIR}/hal/us_ticker_api.c
    ${PROJECT_SOURCE_DIR}/hal/wait_api_no_rtos.c

    ${PROJECT_SOURCE_DIR}/lmic/lmic.c
    ${PROJECT_SOURCE_DIR}/lmic/aes.c
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -Wall -fmessage-length=0 -ffunction-sections")


list(APPEND lmic_sources
    ${PROJECT_SOURCE_DIR}/hal/lpTimeout.c
    ${PROJECT_SOURCE_DIR}/hal/mbed_critical.c
    ${PROJECT_SOURCE_DIR}/hal/mbed_critical_section_api.c
    ${PROJECT_SOURCE_DIR}/hal/ticker_api.c
    ${PROJECT_SOURCE_DIR}/hal/sleep_manager.c
    ${PROJECT_SOURCE_DIR}/hal/timer_event.c
    ${PROJECT_SOURCE_DIR}/hal/lp_ticker_api.c
    ${PROJECT_SOURCE_DIR}/hal/us_ticker_api.c
    ${PROJECT_SOURCE_DIR}/hal/wait_api_no_rtos.c

    ${PROJECT_SOURCE_DIR}/lmic/lmic.c
    ${PROJECT_SOURCE_DIR}/lmic/aes.c
)

if (${radio} STREQUAL "sx1276")
    add_definitions(-DCFG_sx1276_radio)
    list(APPEND lmic_sources ${PROJECT_SOURCE_DIR}/lmic/radio_sx127x.c)
endif ()

if (${lorawan_region} STREQUAL "us915")
    list(APPEND lmic_sources ${PROJECT_SOURCE_DIR}/lmic/lmic_us915.c)
    add_definitions(-DCFG_us915)
endif ()

list(APPEND targetincs
    ${PROJECT_SOURCE_DIR}
)

SET(CMAKE_EXE_LINKER_FLAGS "-specs=nosys.specs -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections")


if (${target} STREQUAL "NRF52_DK")
    string(TOUPPER ${softdevice} sd)
    add_definitions(-DNRF52832_XXAA -DSOFTDEVICE_PRESENT -DBOARD_PCA10040 -D${sd})

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0 -fno-diagnostics-show-caret -mcpu=cortex-m4 -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb -mtp=soft")

    list(APPEND targetincs
        ${PROJECT_SOURCE_DIR}/target_nordic/
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/target_nrf52
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/target_nrf52/target_mcu_nrf52832/device
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/target_nrf52/target_mcu_nrf52832/config
        ${nrf_sdk}/modules/nrfx/mdk
        ${nrf_sdk}/modules/nrfx/hal
        ${nrf_sdk}/modules/nrfx
        ${nrf_sdk}/modules/nrfx/templates/nRF52832
        ${nrf_sdk}/modules/nrfx/drivers/include
        ${nrf_sdk}/components/boards
        ${nrf_sdk}/components/toolchain/cmsis/include
        ${nrf_sdk}/components/libraries/util
        ${nrf_sdk}/components/libraries/timer
        ${nrf_sdk}/components/libraries/atomic
        ${nrf_sdk}/components/libraries/uart
        ${nrf_sdk}/components/libraries/bsp
        ${nrf_sdk}/components/libraries/button
        ${nrf_sdk}/components/libraries/pwr_mgmt
        ${nrf_sdk}/components/libraries/experimental_log
        ${nrf_sdk}/components/libraries/experimental_log/src
        ${nrf_sdk}/components/libraries/strerror
        ${nrf_sdk}/components/libraries/experimental_memobj
        ${nrf_sdk}/components/libraries/balloc
        ${nrf_sdk}/components/libraries/fifo
        ${nrf_sdk}/components/libraries/fstorage
        ${nrf_sdk}/components/softdevice/${softdevice}/headers
        ${nrf_sdk}/components/softdevice/common
        ${nrf_sdk}/components/ble/common
        ${nrf_sdk}/components/ble/nrf_ble_gatt
        ${nrf_sdk}/components/ble/nrf_ble_qwr
        ${nrf_sdk}/components/ble/ble_advertising
        ${nrf_sdk}/components/ble/ble_link_ctx_manager
        ${nrf_sdk}/components/ble/ble_services/ble_nus
        ${nrf_sdk}/components/libraries/experimental_section_vars
        ${nrf_sdk}/integration/nrfx
        ${nrf_sdk}/integration/nrfx/legacy/
        ${nrf_sdk}/external/fprintf
    )

    list(APPEND lmic_sources
        ${PROJECT_SOURCE_DIR}/target_nordic/debug.c
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/nvm.c
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/us_ticker.c
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/my_lp_ticker.c
        ${PROJECT_SOURCE_DIR}/target_nordic/target_nrf5x/target_nrf52/sleep.c
        ${nrf_sdk}/components/libraries/uart/app_uart_fifo.c
        ${nrf_sdk}/components/libraries/util/app_error.c
        ${nrf_sdk}/components/libraries/fifo/app_fifo.c
        ${nrf_sdk}/components/libraries/atomic/nrf_atomic.c
        ${nrf_sdk}/components/libraries/experimental_log/src/nrf_log_frontend.c
        ${nrf_sdk}/components/libraries/experimental_memobj/nrf_memobj.c
        ${nrf_sdk}/modules/nrfx/drivers/src/nrfx_uart.c
        ${nrf_sdk}/modules/nrfx/drivers/src/nrfx_rtc.c
        ${nrf_sdk}/integration/nrfx/legacy/nrf_drv_clock.c
        ${nrf_sdk}/integration/nrfx/legacy/nrf_drv_uart.c
    )

    set(hello_app
        ${PROJECT_SOURCE_DIR}/examples/hello/ses/main.c
        ${PROJECT_SOURCE_DIR}/examples/hello/ses/hello_dual.c
    )
    set(transmit_app
        ${PROJECT_SOURCE_DIR}/examples/transmit/ses/main.c
        ${PROJECT_SOURCE_DIR}/examples/transmit/ses/transmit.c
    )

    #SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T\"${PROJECT_SOURCE_DIR}/todo.ld\" ")
endif ()

if (${target} STREQUAL "NUCLEO_L152RE")
    set(TARGET_STM32L1 1)
    list(APPEND targetincs
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/target_nucleo_l152re/device
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/target_nucleo_l152re
    )
    list(APPEND lmic_sources
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/target_nucleo_l152re/device/toolchain_gcc_arm/startup_stm32.s
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/target_nucleo_l152re/device/system_clock.c
        ${PROJECT_SOURCE_DIR}/target_stm/rtc_api.c
        ${PROJECT_SOURCE_DIR}/target_stm/lp_ticker_rtc.c
    )

    add_definitions(-DTARGET_FF_ARDUINO -DCLOCK_SOURCE=USE_PLL_HSE_EXTC -DSTM32L152xE)

    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T\"${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/target_nucleo_l152re/device/toolchain_gcc_arm/LinkerScript.ld\" ")
endif ()

if (${target} STREQUAL "NUCLEO_L073RZ")
    set(TARGET_STM32L0 1)
    list(APPEND targetincs
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/target_nucleo_l073rz/device
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/target_nucleo_l073rz
    )
    list(APPEND lmic_sources
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/target_nucleo_l073rz/device/toolchain_gcc_arm/startup_stm32.s
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/target_nucleo_l073rz/device/system_clock.c
    )

    add_definitions(-DTARGET_FF_ARDUINO -DCLOCK_SOURCE=USE_PLL_HSE_EXTC)

    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T\"${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/target_nucleo_l073rz/device/toolchain_gcc_arm/LinkerScript.ld\" ")
endif ()

if (${TARGET_STM32L1})
    set(TARGET_STM32 1)
    list(APPEND targetincs
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1
    )

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m3 -mthumb -mfloat-abi=soft")

    set(stm32_hal ${stm32cube_drivers}/STM32L1xx_HAL_Driver)
    list(APPEND lmic_sources
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/device/system_stm32l1xx.c
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l1/nvm.c
        ${stm32_hal}/Src/stm32l1xx_hal_tim.c
        ${stm32_hal}/Src/stm32l1xx_hal_rcc.c
        ${stm32_hal}/Src/stm32l1xx_hal_rcc_ex.c
        ${stm32_hal}/Src/stm32l1xx_hal.c
        ${stm32_hal}/Src/stm32l1xx_hal_cortex.c
        ${stm32_hal}/Src/stm32l1xx_hal_uart.c
        ${stm32_hal}/Src/stm32l1xx_hal_rtc.c
        ${stm32_hal}/Src/stm32l1xx_hal_rtc_ex.c
        ${stm32_hal}/Src/stm32l1xx_hal_pwr.c
        ${stm32_hal}/Src/stm32l1xx_hal_gpio.c
        ${stm32_hal}/Src/stm32l1xx_hal_flash_ex.c
        ${stm32_hal}/Src/stm32l1xx_hal_flash.c
        ${stm32_hal}/Src/stm32l1xx_hal_spi.c
        ${stm32_hal}/Src/stm32l1xx_hal_spi_ex.c
    )

    add_definitions(-DTARGET_STM32L1)
endif (${TARGET_STM32L1})

if (${TARGET_STM32L0})
    set(TARGET_STM32 1)
    list(APPEND targetincs
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0
    )

    set(stm32_hal ${stm32cube_drivers}/STM32L0xx_HAL_Driver)
    list(APPEND lmic_sources
        ${PROJECT_SOURCE_DIR}/target_stm/lp_ticker_lptim.c
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/nvm.c
        ${PROJECT_SOURCE_DIR}/target_stm/target_stm32l0/device/system_stm32l0xx.c
        ${stm32_hal}/Src/stm32l0xx_hal.c
        ${stm32_hal}/Src/stm32l0xx_hal_cortex.c
        ${stm32_hal}/Src/stm32l0xx_hal_gpio.c
        ${stm32_hal}/Src/stm32l0xx_hal_uart.c
        ${stm32_hal}/Src/stm32l0xx_hal_uart_ex.c
        ${stm32_hal}/Src/stm32l0xx_hal_rcc.c
        ${stm32_hal}/Src/stm32l0xx_hal_rcc_ex.c
        ${stm32_hal}/Src/stm32l0xx_hal_dma.c
        ${stm32_hal}/Src/stm32l0xx_hal_pwr.c
        ${stm32_hal}/Src/stm32l0xx_hal_tim.c
        ${stm32_hal}/Src/stm32l0xx_hal_flash_ex.c
        ${stm32_hal}/Src/stm32l0xx_hal_flash.c
        ${stm32_hal}/Src/stm32l0xx_hal_spi.c
        ${stm32_hal}/Src/stm32l0xx_hal_lptim.c
    )

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft")

    add_definitions(-DTARGET_STM32L0)
endif (${TARGET_STM32L0})

if (${TARGET_STM32})
    list(APPEND targetincs
        ${PROJECT_SOURCE_DIR}/target_stm
        ${stm32_hal}/Inc
        ${stm32cube_drivers}/CMSIS/Include
    )
    list(APPEND lmic_sources
        ${PROJECT_SOURCE_DIR}/target_stm/hal_tick_overrides.c
        ${PROJECT_SOURCE_DIR}/target_stm/hal_spi.c
        ${PROJECT_SOURCE_DIR}/target_stm/us_ticker.c
        ${PROJECT_SOURCE_DIR}/target_stm/syscalls.c
        ${PROJECT_SOURCE_DIR}/target_stm/hal.c
        ${PROJECT_SOURCE_DIR}/target_stm/sleep.c
        ${PROJECT_SOURCE_DIR}/target_stm/debug.c
        ${PROJECT_SOURCE_DIR}/target_stm/mbed_overrides.c
    )

    add_definitions(-DTARGET_STM32)

    set(hello_app ${PROJECT_SOURCE_DIR}/examples/hello/main_dual.c)
    set(transmit_app ${PROJECT_SOURCE_DIR}/examples/transmit/transmit_main.c)
endif (${TARGET_STM32})

include_directories(${targetincs}
    ${PROJECT_SOURCE_DIR}/lmic
    ${PROJECT_SOURCE_DIR}/hal
)

set(lmic_lib "lmic_${target}")

add_library(${lmic_lib} STATIC ${lmic_sources})

set(hello_executable "hello_${target}")

add_executable(
    ${hello_executable}
    ${hello_app}
)
target_link_libraries(${hello_executable} ${lmic_lib})
set_target_properties(${hello_executable} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${hello_executable} POST_BUILD COMMAND arm-none-eabi-size ${hello_executable}.elf)
add_custom_command(TARGET ${hello_executable} POST_BUILD COMMAND arm-none-eabi-objcopy -O binary ${hello_executable}.elf ${hello_executable}.bin)

set(transmit_executable "transmit_${target}")

add_executable(
    ${transmit_executable}
    ${transmit_app}
)
target_link_libraries(${transmit_executable} ${lmic_lib})
set_target_properties(${transmit_executable} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${transmit_executable} POST_BUILD COMMAND arm-none-eabi-size ${transmit_executable}.elf)
add_custom_command(TARGET ${transmit_executable} POST_BUILD COMMAND arm-none-eabi-objcopy -O binary ${transmit_executable}.elf ${transmit_executable}.bin)

