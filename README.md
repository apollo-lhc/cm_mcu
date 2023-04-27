# cm_mcu ![CI Status](https://github.com/apollo-lhc/cm_mcu/actions/workflows/c-cpp.yml/badge.svg)
Microcontroller source code, initially targeting the [TI Tiva TM4C1290NCPDT](https://www.ti.com/product/TM4C1290NCPDT) on the Apollo command module. This is a Cortex-M4F 32 bit processor.

## Project
The project is a makefile project; you can also use the Eclipse-based [GNU MCU Eclipse](https://gnu-mcu-eclipse.github.io) IDE which integrates well with the Segger debugger. An Eclipse project is included in the repo. Again the build proceeds via `make` even if you use the IDE.  Follow the instructions on this page, also for windows you'll need to install `make`, `echo` and `rm` (as explained on the GNU MCU web page, see below.) You will also need a command-line `git`. The windows compilation has not been extensively tested.
## Compiler and source code
For the compiler use the [generic GNU ARM compiler](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm), 10-2020-q4 release. Since this is a bare-metal application we are using the "arm-none-eabi" version of the tools (i.e., gcc becomes arm-none-eabi-gcc). This compiler is available as a part of the Petalinux suite but it has some weird options on how glibc is compiled so we are _not_ using it for this project.  The compiler linked above is available for Windows, Linux and MacOS. 
There is also partial support for clang, mainly to allow use of its syntax checking and static code analysis. 

To compile the source code simply type `make` at the top-level directory. To get debugging symbols type `make DEBUG=1`. To see the gory details of the build add `VERBOSE=1` to the command line.
```
% make -j `nproc` DEBUG=1
```
Note that you should not mix builds w/ and w/o `DEBUG=1`; if you do the build will fail or worse do weird things.

If the build fails remember you have to initialize the submodule.
```
% git submodule update --remote --recursive
```

The code uses the Tivaware driver library since this is stored in the ROM of the TM4C devices. (No install is required.)

The build has been extensively tested on MacOS and Linux (SC7/RHEL7).  See commments below if you want to compile in Windows.

### FreeRTOS
We are also using [FreeRTOS](https://freertos.org) to provide basic multi-tasking. It is included via a Git submodule.

### microrl
We are using the microrl library in our project. 

## programmer/debugger
We have settled on the [Segger J-LINK EDU programmer](https://www.segger.com) ($60 USD, plus another $60 if you want to get the custom Xilinx JTAG header adapter). 


## Terminal Emulator
For the terminal emulation, it is best if your program sends \n when you hit enter, and interprets \n as \n\r in typical unix fashion.  However, please print out \r\n on every printout to ensure broad compatibility across many tty clients.

## Software release versions
Please try to use the below versions to avoid unnecessary issues. In particular the `arm-none-eabi` compiler that comes with a lot of distributions is very old; please do not try to use it and instead grab the one from above. 

| Software | Release | 
|----------|---------|
| arm-none-eabi compiler | 10-2020-q4 | 
| FreeRTOS | 10.2 | 
| Tivaware | included in build|


## Continuous Integration (CI)
We use Github Actions to do a verification of the code base during pull requests. The PR triggers a simply build of the code with warnings as errors and fails if there are compliation warnings or errors. The CI also builds binaries when new releases are created and populates the release page with these binaries, which are the authoratative ones (these binaries are built without debugging.) 

## Code structure overview
The repository contains two directors that are imported from the tivaware distribution (`inc` and `driverlib`). The `common` directory contains software that can be used across several projects. The `projects` directory contains the main program files. The main binary lives in `projects/cm_mcu`.

## Documentation
See the wiki page on this github for documentation.

## Windows
For Windows users, WSL works well on command line, but we haven't figured out how to make eclipse recognize the path to its /bin/ directory. If using cygwin, set FREERTOS_PATH in the format of d:/FreeRTOSv10.2.1/FreeRTOS/Source instead /cydrive/d/FreeRTOSv10.2.1/FreeRTOS/Source. Known issue with cygwin: one has to do make clean in between make.
