
include ./makedefs

DIRS=driverlib projects

all:
	@for d in ${DIRS}; do \
	   make -C $$d ;\
	done

