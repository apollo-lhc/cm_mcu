# cm_mcu
Microcontroller source code, initially targeting the [TI Tiva TM4C1290NCPDT](https://www.ti.com/product/TM4C1290NCPDT) on the Apollo command module. This is a Cortex-M4F 32 bit processor.

## Project
The project is a makefile project; you can also use the Eclipse-based [GNU MCU Eclipse](https://gnu-mcu-eclipse.github.io) IDE which integrates well with the Segger debugger. An Eclipse project is included in the repo. Again the build proceeds via `make` even if you use the IDE.  Follow the instructions on this page, also for windows you'll need to install `make`, `echo` and `rm` (as explained on the GNU MCU web page, see below.) You will also need a command-line `git`. The windows compilation has not been extensively tested.
## Compiler and source code
For the compiler use the [generic GNU ARM compiler](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm). Since this is a bare-metal application we are using the "arm-none-eabi" version of the tools (i.e., gcc becomes arm-none-eabi-gcc). This compiler is available as a part of the Petalinux suite but it has some weird options on how glibc is compiled (if you link against this.) This compiler is available for Windows, Linux and MacOS. 

The code uses the Tivaware driver library since this is stored in the ROM of the TM4C devices. (No install is required.)

### FreeRTOS
We are also using [FreeRTOS](https://freertos.org) also to provide basicl multi-tasking. Install FreeRTOS from the above link somewhere and then set the environment variables as follows:
```bash
export FREERTOS_ROOT=/base/of/install/FreeRTOS/Source
export FREERTOS_PLUS_ROOT=/base/of/install/FreeRTOS-Plus/Source
```
The second link is needed for some CLI handling tools.

## programmer/debugger
We have settled on the [Segger J-LINK EDU programmer](https://www.segger.com) ($60 USD, plus another $60 if you want to get the custom Xilinx JTAG header adapter). 


## Terminal Emulator
For the terminal emulation, it is best if your program sends \n when you hit enter, and interprets \n as \n\r in typical unix fashion. 
Unfortunately, the venerable gnu `screen` does not appear to accomodate this, and using `stty` is not a solution either. `miniterm.py` which is included in pyserial does this automatically.
