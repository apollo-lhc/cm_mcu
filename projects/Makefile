
include ../makedefs

DIRS=blinky uart_echo project0 #i2c-sensors #project2

all:
	@for d in ${DIRS}; do \
	   make -C $$d ;\
	done

clean:
	@for d in ${DIRS}; do \
	   make -C $$d clean ;\
	done
