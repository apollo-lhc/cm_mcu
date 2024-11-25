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
error "REV1 is not supported"
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
${COMPILER}/prod_test.axf: ${COMPILER}/microrl.o
${COMPILER}/prod_test.axf: ${COMPILER}/printf.o
${COMPILER}/prod_test.axf: ${COMPILER}/startup_${COMPILER}.o
# TIVAWARE
${COMPILER}/prod_test.axf: ${COMPILER}/sysctl.o
${COMPILER}/prod_test.axf: ${COMPILER}/interrupt.o
${COMPILER}/prod_test.axf: ${COMPILER}/cpu.o


SCATTERgcc_prod_test=prod_test.ld
ENTRY_prod_test=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA2 

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
