cmake_minimum_required(VERSION 2.8)

project(stm32f103_wifi_usb_psu)

set(PP_DEFINES " -DSTM32F1 -DSTM32F103C8Tx -DSTM32 -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD")
#set(PP_DEFINES "${PP_DEFINES} ")


include_directories(
    ${CMSIS_SOURCE_DIR}/core
    ${CMSIS_SOURCE_DIR}/device
    ${StdPeriph_Driver_SOURCE_DIR}/inc
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/inc
    inc
)

# uncomment/ comment as required
file(GLOB STM_STD_PERIPH_LIB_SRC
    ${StdPeriph_Driver_SOURCE_DIR}/src/misc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_adc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_bkp.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_can.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_cec.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_crc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_dac.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_dbgmcu.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_dma.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_exti.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_flash.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_fsmc.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_gpio.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_i2c.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_iwdg.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_pwr.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_rcc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_rtc.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_sdio.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_spi.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_tim.c
    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_usart.c
#    ${StdPeriph_Driver_SOURCE_DIR}/src/stm32f10x_wwdg.c
)

file(GLOB STM32_USB_DEVICE_SRC   
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_core.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_init.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_int.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_mem.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_regs.c
    ${STM32_USB-FS-Device_Driver_SOURCE_DIR}/src/usb_sil.c
)

file(GLOB C_SOURCE
    syscalls.c
    debug_uart.c
    esp8266.c
    flash.c
    http_server.c
    hw_config.c
    led_pattern.c
    main.c
    mcp41xxx.c
    spi_driver.c
    stm32f10x_it.c
    system_stm32f10x.c
    usb_desc.c
    usb_endp.c
    usb_istr.c
    usb_prop.c
    usb_pwr.c
    
    ${CMSIS_SOURCE_DIR}/core/core_cm3.c
    ${STM_STD_PERIPH_LIB_SRC}
    ${STM32_USB_DEVICE_SRC}
)

set_source_files_properties(${C_SOURCE}
    PROPERTIES COMPILE_FLAGS ${PP_DEFINES}
)
    
add_executable(${PROJECT_NAME}.elf
    ${STARTUP_ASM_FILE}
    ${C_SOURCE}
)

target_link_libraries(${PROJECT_NAME}.elf)

add_custom_target ( ${PROJECT_NAME}.hex ALL
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
)
add_custom_target ( ${PROJECT_NAME}.bin ALL
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.bin"
)

# debug target
add_custom_target ( debug-${PROJECT_NAME}
    DEPENDS ${PROJECT_NAME}.elf 
    COMMAND ${CMAKE_GDB} ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Starting gdb for ${PROJECT_NAME}"
)

install(
    PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMPONENT surface
    DESTINATION opt/midas/resources/hermes/
)
