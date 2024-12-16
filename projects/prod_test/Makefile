#******************************************************************************
# 
# Makefile
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=TM4C1290NCPDT

# # The base directory for TivaWare.
#
ROOT=../..
BASE=$(shell cd ../..; pwd)

# FreeRTOS submodule
FREERTOS_ROOT=$(BASE)/FreeRTOS-Kernel

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
error "REV1 is not supported"
endif 
CFLAGS+=-DPRINTF_INCLUDE_CONFIG_H

CFLAGS+=-Wno-pointer-compare ## for configASSERT macro
#CFLAGS+=-fstack-protector 

ifeq (${COMPILER}, clang)
## for clang complaining about FreeRTOS code
## this is not the right way to solve this
CFLAGS+= -Wno-int-in-bool-context \
         -Wstrict-prototypes
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
VPATH+=${BASE}/projects/cm_mcu/commands
#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=..
IPATH+=${ROOT}
IPATH+=${FREERTOS_ROOT}/include
IPATH+=${FREERTOS_ROOT}/portable/GCC/ARM_CM4F

# GIT 
GIT_VERSION="$(shell git describe --abbrev=4 --dirty --always --tags)"

CFLAGS+= -DFIRMWARE_VERSION=\"$(GIT_VERSION)\"
CFLAGS+=-DTARGET_IS_TM4C129_RA2
#ugh
#CFLAGS:=$(filter-out -DDEBUG,$(CFLAGS))

#
# The default rule, which causes the project to be built.
#
all: ${COMPILER}
all: ${COMPILER}/prod_test.axf

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
${COMPILER}/prod_test.axf: ${COMPILER}/prod_test.o
${COMPILER}/prod_test.axf: ${COMPILER}/LocalUart.o
${COMPILER}/prod_test.axf: ${COMPILER}/CommandLineTask.o
${COMPILER}/prod_test.axf: ${COMPILER}/microrl.o
${COMPILER}/prod_test.axf: ${COMPILER}/utils.o
${COMPILER}/prod_test.axf: ${COMPILER}/printf.o
${COMPILER}/prod_test.axf: ${COMPILER}/InterruptHandlers.o
${COMPILER}/prod_test.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/prod_test.axf: ${COMPILER}/printf.o
${COMPILER}/prod_test.axf: ${COMPILER}/pinsel.o
${COMPILER}/prod_test.axf: ${COMPILER}/commands.o
${COMPILER}/prod_test.axf: ${COMPILER}/power_ctl.o
${COMPILER}/prod_test.axf: ${COMPILER}/BoardCommands.o

# TIVAWARE
${COMPILER}/prod_test.axf: ${COMPILER}/sysctl.o
${COMPILER}/prod_test.axf: ${COMPILER}/pinout_rev2.o
${COMPILER}/prod_test.axf: ${COMPILER}/gpio.o
${COMPILER}/prod_test.axf: ${COMPILER}/uart.o
## FreeRTOS
${COMPILER}/prod_test.axf: ${COMPILER}/tasks.o
${COMPILER}/prod_test.axf: ${COMPILER}/queue.o
${COMPILER}/prod_test.axf: ${COMPILER}/port.o
${COMPILER}/prod_test.axf: ${COMPILER}/list.o
${COMPILER}/prod_test.axf: ${COMPILER}/stream_buffer.o
${COMPILER}/prod_test.axf: ${COMPILER}/heap_4.o

${COMPILER}/prod_test.axf: prod_test.ld
SCATTERgcc_prod_test=prod_test.ld
ENTRY_prod_test=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA1 -DUSE_FREERTOS

# this rule forces the main function to be recompiled when
# any of the dependencies are out of date, to get the
# git and compile time info right.
SRCS=$(shell ls *.c )
HDRS=$(shell ls *.h )
${COMPILER}/prod_test.o: ${SRCS} ${HDRS}
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

.PHONY: ${COMPILER}
