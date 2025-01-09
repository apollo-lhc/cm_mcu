

DIRS=boot_loader cm_mcu #i2c-sensors 
ifneq "$(or  $(REV3), $(REV2))" ""
DIRS+=prod_test # only build prod_test if REV2 or REV3 is defined
endif

DIRSCLEAN=$(addsuffix .clean,$(DIRS))

all:  $(DIRS)


$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean


.PHONY: all clean $(DIRS) $(DIRSCLEAN)
