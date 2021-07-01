This repository contains low level drivers used by an STM32F103RB board based on a MCU Cortex M3

The low level drivers can control the following peripherals:

-GPIO
-SPI
-I2C
-UART

stm32f103.h is a header file which includes general definitions for the MCU, among these are Base Addresses of Peripheral in order
to handle and configure them.

In order to build any example in examples folder, execute make to generate an executable file to flash to your device, you must specify which example you want to build by overriding the variable output as follows (note source file is passed as simple text): 

make output=000_GPIO_volatile

In case you don't pass an argument to outputt variable, by default 000_GPIO_volatile.c example will be built.

You can generate all driver object files (not linked yet), by using:
make drivers

You can clean or remove all object, .elf and binary files previously generated by using:
make clean

You can flash your Nucleo Board by using:
make flash
