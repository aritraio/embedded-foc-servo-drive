# STM32 cross-compilation toolchain (arm-none-eabi-gcc).
# Usage: cmake -DTARGET_STM32=ON -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake -B build-stm32
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

find_program(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
find_program(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

if(NOT CMAKE_C_COMPILER)
    message(WARNING "arm-none-eabi-gcc not found in PATH. STM32 build will fail; "
        "install with 'brew install --cask gcc-arm-embedded' (macOS) or "
        "'sudo apt install gcc-arm-none-eabi' (Linux).")
    # Fall back to host compiler so configure step still succeeds for inspection.
    set(CMAKE_C_COMPILER cc)
    set(CMAKE_CXX_COMPILER c++)
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Cortex-M4F (STM32G474): hard float, single precision FPU.
add_compile_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections)
add_link_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/linker_script.ld -nostartfiles)

add_compile_definitions(STM32G474xx USE_HAL_REGISTER STM32_TARGET=1)
