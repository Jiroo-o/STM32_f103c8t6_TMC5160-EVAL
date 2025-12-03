cmake_minimum_required(VERSION 3.22)

# Project setup
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")

project(f103_5160EVAL C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# MCU setup
add_definitions(-DSTM32F103xB -DUSE_HAL_DRIVER -DDEBUG)

# Include directories
include_directories(
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)

# Source files
file(GLOB_RECURSE SOURCES 
    "Core/Src/*.c"
    "Drivers/STM32F1xx_HAL_Driver/Src/*.c"
    "Core/Startup/*.s"
)

# Linker script
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F103C8TX_FLASH.ld")

# Executable
add_executable(${PROJECT_NAME} ${SOURCES} ${LINKER_SCRIPT})

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -Wall -Wextra -g3 -O0 -ffunction-sections -fdata-sections
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    --specs=nano.specs
    --specs=nosys.specs
)

# Post-build: Generate HEX and BIN
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)
