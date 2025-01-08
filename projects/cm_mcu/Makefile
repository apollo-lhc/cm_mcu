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
CFLAGS += -DREV1
endif 
ifdef REV3
CFLAGS += -DREV3
endif
CFLAGS+=-DPRINTF_INCLUDE_CONFIG_H -DUSE_ERROR_EPROM

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
	@rm -rf ${COMPILER} ${wildcard *~} ZynqMon_addresses.c ZynqMon_addresses.h ${ROOT}/PL_MEM*.xml MonI2C_addresses.*

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

## Zynq address table -- MCU code
ZYNQMON_YAML_FILES=$(wildcard ${ROOT}/sm_cm_config/data/PL_MEM*.yml)
#
# for building tar files in a release
#
release: ${ZYNQMON_YAML_FILES}
	@for f in ${ZYNQMON_YAML_FILES}; do \
		if [ 'x${VERBOSE}' = x ]; then \
			echo "  PY    $(notdir $$f)"; \
		else \
			echo ${ROOT}/sm_cm_config/src/xml_generate.py -d ${ROOT} $$f; \
		fi; \
		python ${ROOT}/sm_cm_config/src/xml_generate.py -d ${ROOT} $$f; \
	done

#
# Zynq address table -- MCU code
#
ZynqMon_addresses.c ZynqMon_addresses.h &: ${ZYNQMON_YAML_FILES}
	@if [ 'x${VERBOSE}' = x ];                                            \
	then                                                                 \
		echo "  PY    $(notdir $?)";                                               \
	else                                                                 \
		echo ${ROOT}/sm_cm_config/src/mcu_generate.py -o $@ ${ZYNQMON_YAML_FILES};    \
	fi
	@python ${ROOT}/sm_cm_config/src/mcu_generate.py -o $@ ${ZYNQMON_YAML_FILES}

#
# Rules for building the MonI2C_address.c file
#
MONI2C_YAML_FILES=$(wildcard ${ROOT}/sm_cm_config/data/MON_I2C*.yml)

MonI2C_addresses.c: ${MONI2C_YAML_FILES}
	@if [ 'x${VERBOSE}' = x ];                                            \
	then                                                                 \
		echo "  PY    $(notdir $?)";                                               \
	else                                                                 \
		echo ${ROOT}/sm_cm_config/src/mon_generate.py -o $@ ${MONI2C_YAML_FILES};    \
	fi
	@python ${ROOT}/sm_cm_config/src/mon_generate.py -o $@ ${MONI2C_YAML_FILES}


# these rules are required to force the generation of the ZynqMon_addresses.c
# and .h file. The second one depends on the .h file only, but putting that
# in as a target causes the rule above to be called 2x. So we lie to make.
${COMPILER}/ZynqMon_addresses.o: ZynqMon_addresses.c 
${COMPILER}/ZynqMonTask.o: ZynqMon_addresses.c
#
# Rules for building the project
# 
${COMPILER}/cm_mcu.axf: ${COMPILER}/cm_mcu.o
ifdef REV1
${COMPILER}/cm_mcu.axf: ${COMPILER}/pinout_rev1.o
else ifdef REV2
${COMPILER}/cm_mcu.axf: ${COMPILER}/pinout_rev2.o
else ifdef REV3
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
${COMPILER}/cm_mcu.axf: ${COMPILER}/MonitorTaskI2C.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/I2CSlaveTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/GenericAlarmTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/AlarmUtilities.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/LedTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/microrl.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/ZynqMon_addresses.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/MonI2C_addresses.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/ZynqMonTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/InterruptHandlers.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/InitTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/MonUtils.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/WatchdogTask.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/LocalTasks.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/BoardCommands.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/SoftwareCommands.o
${COMPILER}/cm_mcu.axf: ${COMPILER}/FireflyUtils.o
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
${COMPILER}/cm_mcu.axf: ${COMPILER}/Semaphore.o
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

.PHONY: ${COMPILER} release
