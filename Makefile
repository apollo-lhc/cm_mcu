
include ./makedefs

ifdef REV1
ifdef REV2
$(error "REV1 and REV2 are mutually exclusive")
endif
endif

ifndef REV1
ifndef REV2
$(error "Must define either REV1 or REV2")
endif
endif


DIRS=driverlib projects 
DIRSCLEAN=$(addsuffix .clean,$(DIRS))

all:  $(DIRS)

# these rules ensure that the library exists before you try to use
# them.
projects: driverlib
boot_loader: driverlib

$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean


.PHONY: all clean $(DIRS) $(DIRSCLEAN)
