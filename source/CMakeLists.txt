cmake_minimum_required(VERSION 2.8)

project(wifi_usb_psu)

set(COMPILER_OPTIONS "-Wall -Werror")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${COMPILER_OPTIONS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILER_OPTIONS} -std=c++11")

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm-none-eabi")

    # force stm builds to debug.  This is a hack as when release is set, it automatically 
    # sets the invalid -O3 flag on the assembler.
    set(CMAKE_BUILD_TYPE Debug)

    # shared libraries
    add_subdirectory(CMSIS)
    add_subdirectory(StdPeriph_Driver)
    add_subdirectory(STM32_USB-FS-Device_Driver)
    add_subdirectory(startup)
    
    # enable asm for stm startup.s file
    enable_language(ASM)

    # override toolchain flags
    set(COMPILER_OPTIONS "-mthumb -mcpu=cortex-m3 -mlittle-endian -mthumb-interwork ")
    # -fno-builtin -fno-strict-aliasing -std=c99 
    SET(CMAKE_C_FLAGS "${COMPILER_OPTIONS}  -lm -lc --specs=nano.specs " CACHE INTERNAL "c compiler flags")
    SET(CMAKE_CXX_FLAGS "${COMPILER_OPTIONS}  -lm -lc --specs=nano.specs " CACHE INTERNAL "cxx compiler flags")
    SET(CMAKE_ASM_FLAGS "${COMPILER_OPTIONS}" CACHE INTERNAL "asm compiler flags")
    # -mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd   -Wl,-Map=linker.map -Wl,-cref  -Wl,--gc-sections
    SET(CMAKE_EXE_LINKER_FLAGS "${COMPILER_OPTIONS} -Wl,-Map=linker.map -Wl,-cref -Wl,--gc-sections" CACHE INTERNAL "exe link flags")
    
    # shared flags, so we can have different ones if needed for the bootloader and app
    set(COMPILER_OPTIONS_STM "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Wl,-Map=output.map -Wl,--gc-sections -lm")
    set(CMAKE_C_FLAGS_STM   "${CMAKE_C_FLAGS}   ${COMPILER_OPTIONS_STM}")
    set(CMAKE_CXX_FLAGS_STM "${CMAKE_CXX_FLAGS} ${COMPILER_OPTIONS_STM}")

    # set compiler optimisations
    set(COMPILER_OPTIMISATION "-g -O0")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS_STM}   ${COMPILER_OPTIMISATION}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_STM} ${COMPILER_OPTIMISATION}")

    # add startup and linker file
    set(STARTUP_ASM_FILE "${startup_stm32_SOURCE_DIR}/startup_stm32.s")
    set_property(SOURCE ${STARTUP_ASM_FILE} PROPERTY LANGUAGE ASM)
    set(LINKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/LinkerScript.ld")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COMPILER_OPTIONS} -T${LINKER_FILE}")

    # add the source code directory
    add_subdirectory(src)
endif()

message(STATUS "System Processor      : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS 
    "BUILD FLAGS:\n"
    "   c flags     = ${CMAKE_C_FLAGS}\n"
    "   c++ flags   = ${CMAKE_CXX_FLAGS}\n"
    "   asm flags   = ${CMAKE_ASM_FLAGS}\n"
    "   ld flags    = ${CMAKE_EXE_LINKER_FLAGS}\n"
)
