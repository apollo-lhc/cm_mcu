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
ROOT=..


#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=..
IPATH+=${ROOT}

# GIT 
GIT_VERSION="$(shell git describe --abbrev=4 --dirty --always --tags)"

CFLAGS+= -DFIRMWARE_VERSION=\"$(GIT_VERSION)\"


#
# The default rule, which causes the project to be built.
#
all: ${COMPILER}
all: ${COMPILER}/bl_main.axf

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
${COMPILER}/bl_main.axf: ${COMPILER}/bl_main.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_uart.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_check.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_packet.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_autobaud.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_flash.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_startup_${COMPILER}.o
## FreeRTOS
SCATTERgcc_bl_main=bl_link.ld
ENTRY_bl_main=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA1 

# this rule forces the main function to be recompiled when
# any of the dependencies are out of date, to get the
# git and compile time info right.
SRCS=$(shell ls *.c )
HDRS=$(shell ls *.h )
${COMPILER}/bl_main.o: ${SRCS} ${HDRS}
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
