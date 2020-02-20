
include ./makedefs


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
