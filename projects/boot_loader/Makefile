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


#
# Include the common make definitions.
#
include ${ROOT}/makedefs

ifdef REV2
CFLAGS += -DREV2
endif 
ifdef REV1
CFLAGS += -DREV1
endif 
ifdef REV3
CFLAGS += -DREV3
endif 
# make sure only one of REV1, REV2, REV3 is defined
ifeq ($(words $(REV1) $(REV2) $(REV3)),0)
$(error "No Revision defined")
else ifneq ($(words $(REV1) $(REV2) $(REV3)),1)
$(error "Multiple Revisions defined")
endif


#
# Where to find header files that do not live in the source directory.
#
IPATH=.
IPATH+=..
IPATH+=${ROOT}

## where to find sources
VPATH=.
VPATH+=${ROOT}/driverlib
VPATH+=../../common

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
${COMPILER}/bl_main.axf: ${COMPILER}/bl_flash.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_userhooks.o
${COMPILER}/bl_main.axf: ${COMPILER}/LocalUart.o
${COMPILER}/bl_main.axf: ${COMPILER}/microrl.o
${COMPILER}/bl_main.axf: ${COMPILER}/printf.o
${COMPILER}/bl_main.axf: ${COMPILER}/bl_startup_${COMPILER}.o
# TIVAWARE
${COMPILER}/bl_main.axf: ${COMPILER}/sysctl.o
${COMPILER}/bl_main.axf: ${COMPILER}/interrupt.o
${COMPILER}/bl_main.axf: ${COMPILER}/cpu.o

#${COMPILER}/bl_main.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a

SCATTERgcc_bl_main=bl_link.ld
ENTRY_bl_main=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA2 

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

.PHONY: ${COMPILER}
