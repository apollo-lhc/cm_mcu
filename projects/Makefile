
include ../makedefs

DIRS=blinky uart_echo cm_mcu #i2c-sensors 

all:
	@for d in ${DIRS}; do \
	   ${MAKE} -s -C $$d ;\
	done

clean:
	@for d in ${DIRS}; do \
	   ${MAKE} -s -C $$d clean ;\
	done
