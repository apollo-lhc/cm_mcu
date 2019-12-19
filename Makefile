
include ./makedefs


DIRS=driverlib projects boot_loader

all: 
	@for d in ${DIRS}; do \
	   ${MAKE} DEBUG=1 -C $$d ;\
	done

clean:
	@for d in ${DIRS}; do \
	   ${MAKE} -C $$d clean ;\
	done

