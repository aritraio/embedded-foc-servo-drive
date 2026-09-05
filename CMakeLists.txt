cmake_minimum_required(VERSION 3.20)
project(embedded_foc_servo_drive VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(TARGET_SIL "Build native Software-in-the-Loop simulator + tests" ON)
option(TARGET_STM32 "Build STM32 bare-metal firmware image" OFF)

include(cmake/CompilerFlags.cmake)

# ---- Control library: pure algorithmic core, zero HAL dependency ----
add_library(foc_control STATIC
    firmware/control/src/foc_math.c
    firmware/control/src/svpwm.c
    firmware/control/src/pid_controller.c
    firmware/control/src/foc_core.c
    firmware/control/src/motion_profiler.c
    firmware/control/src/disturbance_observer.c
)
target_include_directories(foc_control PUBLIC
    firmware/control/inc
    firmware/app/inc
)
target_link_libraries(foc_control PUBLIC m)
set_target_properties(foc_control PROPERTIES C_STANDARD 11)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(foc_control PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Werror;-Wdouble-promotion;-Wshadow>
    )
endif()

# ---- Comms library: COBS + CRC32 + telemetry ----
add_library(foc_comms STATIC
    firmware/comms/src/packet_protocol.c
    firmware/comms/src/telemetry.c
)
target_include_directories(foc_comms PUBLIC firmware/comms/inc firmware/app/inc)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(foc_comms PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Werror;-Wdouble-promotion;-Wshadow>
    )
endif()

# ---- Drivers + HAL (host-compilable: register maps mocked when not on ARM) ----
add_library(foc_drivers STATIC
    firmware/drivers/src/as5048a_encoder.c
    firmware/drivers/src/drv8301_gate_driver.c
    firmware/hal/src/stm32g4_pwm.c
    firmware/hal/src/stm32g4_adc.c
    firmware/hal/src/stm32g4_spi.c
    firmware/hal/src/sil_hal_mock.c
)
target_include_directories(foc_drivers PUBLIC
    firmware/drivers/inc firmware/hal/inc firmware/app/inc firmware/control/inc
)
target_link_libraries(foc_drivers PUBLIC foc_control)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(foc_drivers PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Werror;-Wdouble-promotion;-Wshadow>
    )
endif()

# ---- App (state machine + ISRs) ----
add_library(foc_app STATIC
    firmware/app/src/foc_app.c
    firmware/app/src/isr_handlers.c
)
target_include_directories(foc_app PUBLIC
    firmware/app/inc firmware/control/inc firmware/comms/inc
    firmware/drivers/inc firmware/hal/inc
)
target_link_libraries(foc_app PUBLIC foc_control foc_comms foc_drivers)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(foc_app PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Werror;-Wdouble-promotion;-Wshadow>
    )
endif()

if(TARGET_SIL)
    # ---- PMSM physics simulator (C++) ----
    add_library(pmsm_sim STATIC
        simulation/src/pmsm_model.cpp
        simulation/src/sil_simulator.cpp
    )
    target_include_directories(pmsm_sim PUBLIC
        simulation/inc firmware/control/inc firmware/app/inc
        firmware/comms/inc firmware/drivers/inc firmware/hal/inc
    )
    target_link_libraries(pmsm_sim PUBLIC foc_control foc_comms foc_drivers)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(pmsm_sim PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Werror
            -Wdouble-promotion -Wshadow -Wnon-virtual-dtor -Wold-style-cast
        )
    endif()

    add_executable(sil_main simulation/src/sil_main.cpp)
    target_link_libraries(sil_main PRIVATE pmsm_sim foc_app)

    # ---- Tests ----
    enable_testing()
    find_package(GTest REQUIRED)

    function(foc_add_test name src)
        add_executable(${name} ${src})
        target_include_directories(${name} PRIVATE
            simulation/inc firmware/control/inc firmware/comms/inc
            firmware/app/inc firmware/drivers/inc firmware/hal/inc)
        target_link_libraries(${name} PRIVATE pmsm_sim foc_app foc_control foc_comms foc_drivers GTest::gtest GTest::gtest_main)
        # Tests use slightly relaxed -Wno-conversion for gtest headers interaction,
        # but our own code remains warning-clean.
        if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
            target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -Werror
                -Wno-conversion -Wno-double-promotion)
        endif()
        add_test(NAME ${name} COMMAND ${name})
    endfunction()

    foc_add_test(test_smoke tests/test_smoke.cpp)
    foc_add_test(test_foc_math tests/test_foc_math.cpp)
    foc_add_test(test_pid tests/test_pid.cpp)
    foc_add_test(test_sil_openloop tests/test_sil_openloop.cpp)
    foc_add_test(test_sil_closed_loop tests/test_sil_closed_loop.cpp)
    foc_add_test(test_motion_profiler tests/test_motion_profiler.cpp)
    foc_add_test(test_hal_fault tests/test_hal_fault.cpp)
    foc_add_test(test_packet_protocol tests/test_packet_protocol.cpp)
endif()

if(TARGET_STM32)
    add_executable(foc_firmware.elf firmware/app/src/main.c)
    target_link_libraries(foc_firmware.elf PRIVATE foc_app foc_control foc_comms foc_drivers)
endif()
