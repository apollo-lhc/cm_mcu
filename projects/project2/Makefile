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

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../drivers
VPATH+=../../../../utils
# board specific 
VPATH+=../../board_specific
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
#
# The default rule, which causes the project to be built.
#
all: ${COMPILER}
all: ${COMPILER}/project2.axf
all: size

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


size: ${COMPILER}/project2.axf
	${SIZE} $<

#
# Rules for building the project
#p
${COMPILER}/project2.axf: ${COMPILER}/project2.o
${COMPILER}/project2.axf: ${COMPILER}/pinout.o
${COMPILER}/project2.axf: ${COMPILER}/pinsel.o
${COMPILER}/project2.axf: ${COMPILER}/startup_${COMPILER}.o
## freeRTOS
${COMPILER}/project2.axf: ${COMPILER}/tasks.o
${COMPILER}/project2.axf: ${COMPILER}/queue.o
${COMPILER}/project2.axf: ${COMPILER}/port.o
${COMPILER}/project2.axf: ${COMPILER}/list.o
${COMPILER}/project2.axf: ${COMPILER}/heap_4.o
## TivaWare
${COMPILER}/project2.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/project2.axf: project2.ld
SCATTERgcc_project2=project2.ld
ENTRY_project2=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C129_RA1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
