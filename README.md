# cm_mcu
Microcontroller source code, initially targeting the [TI Tiva TM4C1290NCPDT](https://www.ti.com/product/TM4C1290NCPDT) on the Apollo command module. This is a Cortex-M4F 32 bit processor.


The ambition is to make this source code a makefile/gcc project.

For the compiler use the [generic GNU ARM compiler](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm). Since this is a bare-metal application we are using the "arm-none-eabi" version of the tools (i.e., gcc becomes arm-none-eabi-gcc). This compiler is available as a part of the Petalinux suite but it has some weird options on how glibc is compiled (if you link against this.) This compiler is available for Windows, Linux and MacOS. 

The code will use the Tivaware driver library since this is stored in the ROM of the TM4C devices.

Utimately we will likely use [FreeRTOS](https://www.freertos.org/RTOS-Cortex-M3-M4.html) to provide a basic RTOS.

Currently investigating using the [Black Magic Probe](https://github.com/blacksphere/blackmagic/wiki) as the JTAG programmer for the device.
