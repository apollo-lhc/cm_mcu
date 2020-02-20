

DIRS=boot_loader blinky uart_echo cm_mcu #i2c-sensors 

DIRSCLEAN=$(addsuffix .clean,$(DIRS))

all:  $(DIRS)


$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean


.PHONY: all clean $(DIRS) $(DIRSCLEAN)
