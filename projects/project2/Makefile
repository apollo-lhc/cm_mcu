#******************************************************************************
# 
# Makefile
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=TM4C1290NCPDT

#
# The base directory for TivaWare.
#
ROOT=../..

# if the environment variable is not set, this is used
FREERTOS_ROOT?=../../../FreeRTOSv10.2.0/FreeRTOS/Source
FREERTOS_PLUS_ROOT?=../../../FreeRTOSv10.2.0/FreeRTOS-Plus/Source

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

CFLAGS+=-Wno-pointer-compare ## for configASSERT macro
#
# Where to find source files that do not live in this directory.
#
VPATH=../drivers
VPATH+=../../../../utils
# board specific 
VPATH+=../../common
# FreeRTOS
VPATH+=${FREERTOS_ROOT}
VPATH+=${FREERTOS_ROOT}/portable/GCC/ARM_CM4F
VPATH+=${FREERTOS_ROOT}/portable/MemMang
VPATH+=${FREERTOS_PLUS_ROOT}/FreeRTOS-Plus-CLI
# hack
VPATH+=${FREERTOS_PLUS_ROOT}/../Demo/Common/FreeRTOS_Plus_CLI_Demos
#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=..
IPATH+=${ROOT}
IPATH+=${FREERTOS_ROOT}/include
IPATH+=${FREERTOS_ROOT}/portable/GCC/ARM_CM4F
IPATH+=${FREERTOS_PLUS_ROOT}/FreeRTOS-Plus-CLI

# GIT 
GIT_VERSION="$(shell git describe --abbrev=4 --dirty --always --tags)"

CFLAGS+= -DFIRMWARE_VERSION=\"$(GIT_VERSION)\"


#
# The default rule, which causes the project to be built.
#
all: ${COMPILER}
all: ${COMPILER}/project2.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}


#
# Rules for building the project
# 
${COMPILER}/project2.axf: ${COMPILER}/project2.o
${COMPILER}/project2.axf: ${COMPILER}/pinout.o
${COMPILER}/project2.axf: ${COMPILER}/pinsel.o
${COMPILER}/project2.axf: ${COMPILER}/i2c_reg.o
${COMPILER}/project2.axf: ${COMPILER}/uart.o
${COMPILER}/project2.axf: ${COMPILER}/utils.o
${COMPILER}/project2.axf: ${COMPILER}/smbus.o
${COMPILER}/project2.axf: ${COMPILER}/smbus_units.o
${COMPILER}/project2.axf: ${COMPILER}/power_ctl.o
${COMPILER}/project2.axf: ${COMPILER}/CommandLineTask.o
${COMPILER}/project2.axf: ${COMPILER}/PowerSupplyTask.o
${COMPILER}/project2.axf: ${COMPILER}/ADCMonitorTask.o
${COMPILER}/project2.axf: ${COMPILER}/MonitorTask.o
${COMPILER}/project2.axf: ${COMPILER}/FireFlyTask.o
${COMPILER}/project2.axf: ${COMPILER}/LedTask.o
${COMPILER}/project2.axf: ${COMPILER}/startup_${COMPILER}.o
## FreeRTOS
${COMPILER}/project2.axf: ${COMPILER}/tasks.o
${COMPILER}/project2.axf: ${COMPILER}/queue.o
${COMPILER}/project2.axf: ${COMPILER}/port.o
${COMPILER}/project2.axf: ${COMPILER}/list.o
${COMPILER}/project2.axf: ${COMPILER}/stream_buffer.o
${COMPILER}/project2.axf: ${COMPILER}/heap_4.o
${COMPILER}/project2.axf: ${COMPILER}/FreeRTOS_CLI.o
#${COMPILER}/project2.axf: ${COMPILER}/Sample-CLI-commands.o
${COMPILER}/project2.axf: ${COMPILER}/printf-stdarg.o
## TivaWare
${COMPILER}/project2.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/project2.axf: project2.ld
SCATTERgcc_project2=project2.ld
ENTRY_project2=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA1 -DUSE_FREERTOS

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
