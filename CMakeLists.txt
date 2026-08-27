cmake_minimum_required(VERSION 3.22)

project(stm32zet6 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(STM32_CPU_FLAGS
    -mcpu=cortex-m3
    -mthumb
    -mfloat-abi=soft
)

set(STM32_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
)

set(STM32_SOURCES
    Core/Src/gpio.c
    Core/Src/imu_uart.c
    Core/Src/main.c
    Core/Src/stm32f1xx_hal_msp.c
    Core/Src/stm32f1xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32f1xx.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Startup/startup_stm32f103zetx.s
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

add_executable(${PROJECT_NAME} ${STM32_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${STM32_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F103xE
)

foreach(flag IN LISTS STM32_CPU_FLAGS)
    target_compile_options(${PROJECT_NAME} PRIVATE
        "$<$<COMPILE_LANGUAGE:C>:${flag}>"
        "$<$<COMPILE_LANGUAGE:ASM>:${flag}>"
    )
endforeach()

target_compile_options(${PROJECT_NAME} PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-ffunction-sections>"
    "$<$<COMPILE_LANGUAGE:C>:-fdata-sections>"
    "$<$<COMPILE_LANGUAGE:C>:-Wall>"
    "$<$<CONFIG:Debug>:-Og>"
    "$<$<CONFIG:Debug>:-g3>"
    "$<$<CONFIG:Release>:-Os>"
)

set(STM32_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F103ZETX_FLASH.ld)
target_link_options(${PROJECT_NAME} PRIVATE
    ${STM32_CPU_FLAGS}
    --specs=nano.specs
    --specs=nosys.specs
    -T${STM32_LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)
target_link_libraries(${PROJECT_NAME} PRIVATE m)

find_program(STM32_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(STM32_SIZE arm-none-eabi-size REQUIRED)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${STM32_OBJCOPY} -O ihex
            $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${STM32_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating ${PROJECT_NAME}.hex"
)
