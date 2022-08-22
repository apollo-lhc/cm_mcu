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
BASE=$(shell cd ../..; pwd)

# if the environment variable is not set, this is used
FREERTOS_ROOT?=../../../FreeRTOSv10.2.0/FreeRTOS/Source

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

ifndef DEBUG
CFLAGS+=-Werror
endif 

ifdef REV2
CFLAGS += -DREV2
endif 
ifdef REV1
CFLAGS += -DREV1
endif 
CFLAGS+=-DPRINTF_INCLUDE_CONFIG_H

CFLAGS+=-Wno-pointer-compare ## for configASSERT macro
#CFLAGS+=-fstack-protector 

ifeq (${COMPILER}, clang)
## for clang complaining about FreeRTOS code
## this is not the right way to solve this
CFLAGS+=-Wno-int-in-bool-context
endif

#
# Where to find source files that do not live in this directory.
#
VPATH=${BASE}/driverlib
VPATH+=commands
# board specific 
VPATH+=${BASE}/common
# FreeRTOS
VPATH+=${FREERTOS_ROOT}
VPATH+=${FREERTOS_ROOT}/portable/GCC/ARM_CM4F
VPATH+=${FREERTOS_ROOT}/portable/MemMang
#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=..
IPATH+=${ROOT}
IPATH+=${FREERTOS_ROOT}/include
IPATH+=${FREERTOS_ROOT}/portable/GCC/ARM_CM4F

# GIT 
GIT_VERSION="$(shell git --no-pager  show -s --format="commit date %ci  %d"  ; git describe  --dirty --always --tags )"

CFLAGS+= -DFIRMWARE_VERSION=\"$(GIT_VERSION)\"


#
# The default rule, which causes the project to be built.
#
all: ${COMPILER}
all: ${COMPILER}/cm_mcu.axf

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
${COMPILER}/cm_mcu.axf: ${COMPILER}/cm_mcu.o
ifdef REV1
${COMPILER}/cm_mcu.axf: ${COMPILER}/pinout_rev1.o
endif
ifdef REV2
${COMPILER}/cm_mcu.axf: ${COMPILER}/pinout_rev2.o
endif 
${COMPILER}/cm_mcu.axf: ${COMPILER}/pinsel.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/i2c_reg.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/LocalUart.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/utils.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/smbus.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/log.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/clocksynth.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/softuart.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/smbus_units.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/smbus_helper.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/power_ctl.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/I2CCommunication.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/CommandLineTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/EEPROMTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/PowerSupplyTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/ADCMonitorTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/MonitorTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/MonitorI2CTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/I2CSlaveTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/GenericAlarmTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/AlarmUtilities.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/LedTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/microrl.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/ZynqMonTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/InterruptHandlers.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/InitTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/WatchdogTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/LocalTasks.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/BoardCommands.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/BufferCommands.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/EEPROMCommands.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/I2CCommands.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/SensorControl.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/startup_${COMPILER}.o
## FreeRTOS
${COMPILER}/cm_mcu.axf: ${COMPILER}/tasks.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/queue.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/port.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/list.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/stream_buffer.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/heap_4.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/printf.o
## TivaWare
${COMPILER}/cm_mcu.axf: ${COMPILER}/uart.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/sysctl.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/gpio.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/interrupt.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/hibernate.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/adc.o

#${COMPILER}/cm_mcu.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/cm_mcu.axf: cm_mcu.ld
SCATTERgcc_cm_mcu=cm_mcu.ld
ENTRY_cm_mcu=ResetISR
CFLAGS+=-DTARGET_IS_TM4C129_RA1 -DUSE_FREERTOS

# this rule forces the main function to be recompiled when
# any of the dependencies are out of date, to get the
# git and compile time info right.
SRCS=$(shell ls *.c ../../common/*.c)
HDRS=$(shell ls *.h ../../common/*.h)
${COMPILER}/cm_mcu.o: ${SRCS} ${HDRS}
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

.PHONY: ${COMPILER}
