
include ./makedefs

ifdef REV1
ifdef REV2
$(error "REV1 and REV2 are mutually exclusive")
endif
endif

ifndef REV1
ifndef REV2
REV2=1
export REV2
endif
endif


DIRS=projects 
DIRSCLEAN=$(addsuffix .clean,$(DIRS))

all:  $(DIRS)

$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean


.PHONY: all clean $(DIRS) $(DIRSCLEAN)
