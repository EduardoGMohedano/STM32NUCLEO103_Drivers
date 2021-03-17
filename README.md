This repository contains low level drivers used by an STM32F103RB board based on a MCU Cortex M3

The low level drivers can control the following peripherals:

-GPIO
-SPI
-I2C
-UART

sm32f103.h is a header file which includes general definitions for the MCU, among these are Base Addresses of Peripheral in order
to handle and configure them.

Do make to generate and executable file to flash to your device. In order to build these drivers you should modify the main.c file
