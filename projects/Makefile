
include ../makedefs

DIRS=blinky uart_echo cm_mcu #i2c-sensors 

DIRSCLEAN=$(addsuffix .clean,$(DIRS))

all:  $(DIRS)

# these rules ensure that the library exists before you try to use
# them.

$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean


.PHONY: all clean $(DIRS) $(DIRSCLEAN)
