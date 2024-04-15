# I target a recent cmake, it shouldn't be a problem on a dev machine
cmake_minimum_required(VERSION 3.11)
# Project name
project("pacman_pisibot")

# Product filename
set(PRODUCT_NAME pacman_pisibot)

## AVR Chip Configuration
# 8Mhz, this should match the crystal on your board,
# I use 8Mhz and 3.3V for the lowest power consumption
set(F_CPU 32000000UL)
# CPU, you can find the list here:
# https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html
set(MCU atxmega32a4u)
# Default Baudrate for UART, read avr include/util/setbaud.h for usage
set(BAUD 9600)
# The programmer to use, read avrdude manual for list
set(PROG_TYPE jtag2pdi)

# Use AVR GCC toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CXX_COMPILER avr-g++)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_ASM_COMPILER avr-gcc)


# Pass defines to compiler
add_definitions(
        -DF_CPU=${F_CPU}
        -D__AVR_ATxmega32A4U__
)
# mmcu MUST be passed to bot the compiler and linker, this handle the linker
set(CMAKE_EXE_LINKER_FLAGS -mmcu=${MCU})

add_compile_options(
        -mmcu=${MCU} # MCU
        -std=gnu99 # C99 standard
        -Os # optimize
        -Wall # enable warnings
        -Wextra
        -Wno-main
        -Wundef
        #-pedantic
        #-Werror
        -Wfatal-errors
        -Wl,--relax,--gc-sections
        -g
        -gdwarf-2
        -funsigned-char # a few optimizations
        -funsigned-bitfields
        -fpack-struct
        -fshort-enums
        -ffunction-sections
        -fdata-sections
        -fno-split-wide-types
        -fno-tree-scev-cprop
)

# Create one target
add_executable(${PRODUCT_NAME}
        main.c
        drive_control.c
        cmd_control.c
        drivers/adc.c
        drivers/board.c
        drivers/com.c
        drivers/gyro.c
        drivers/motor.c
        drivers/PisiBot.c
        drivers/drivers/adc_driver.c
        drivers/drivers/clksys_driver.c
        drivers/drivers/dma_driver.c
        drivers/drivers/port_driver.c
        drivers/drivers/qdec_driver.c
        drivers/drivers/tc_driver.c
        drivers/drivers/twi_master_driver.c
        drivers/drivers/usart_driver.c
)

# Rename the output to .elf as we will create multiple files
set_target_properties(${PRODUCT_NAME} PROPERTIES OUTPUT_NAME ${PRODUCT_NAME}.elf)

# Strip binary for upload
add_custom_target(strip ALL avr-strip ${PRODUCT_NAME}.elf DEPENDS ${PRODUCT_NAME})

# Transform binary into hex file, we ignore the eeprom segments in the step
add_custom_target(hex ALL avr-objcopy -R .eeprom -O ihex ${PRODUCT_NAME}.elf ${PRODUCT_NAME}.hex DEPENDS strip)
# Transform binary into hex file, this is the eeprom part (empty if you don't
# use eeprom static variables)
add_custom_target(eeprom avr-objcopy -j .eeprom  --set-section-flags=.eeprom="alloc,load"  --change-section-lma .eeprom=0 -O ihex ${PRODUCT_NAME}.elf ${PRODUCT_NAME}.eep DEPENDS strip)

# Erase all the contents from the microcontroller
# (necessary for uploading new code)
add_custom_target(dfu-erase dfu-programmer atxmega32a4u erase --force)

# Upload new code to the microcontroller
# (super user privileges needed - e.g. sudo make dfup)
add_custom_target(dfu-upload dfu-programmer atxmega32a4u flash ${PRODUCT_NAME}.hex DEPENDS hex dfu-erase)

add_custom_target(dfus dfu-programmer atxmega32a4u launch DEPENDS dfu-upload)

# Clean extra files
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${PRODUCT_NAME}.hex;${PRODUCT_NAME}.eeprom;${PRODUCT_NAME}.lst")

