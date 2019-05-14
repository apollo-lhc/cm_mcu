
include ./makedefs

DIRS=driverlib projects

all: common/version.h
	@for d in ${DIRS}; do \
	   ${MAKE} -C $$d ;\
	done

clean:
	@for d in ${DIRS}; do \
	   ${MAKE} -C $$d clean ;\
	done

.PHONY: common/version.h
common/version.h: 
	@echo " GIT    common/version.h"
ifeq ($(OS), Windows_NT)
	@echo #define FIRMWARE_VERSION "$(shell git describe --always --dirty)" > $@
else
	@echo "#define FIRMWARE_VERSION \"$(shell git describe --always --dirty)\"" > $@
endif
