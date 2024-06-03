
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

all:  check-and-reinit-submodules $(DIRS) 

$(DIRS):
	@$(MAKE) -C $@

clean: $(DIRSCLEAN)


$(DIRSCLEAN): %.clean:
	@$(MAKE) -C $* clean

check-and-reinit-submodules:
	@if git submodule status | egrep -q '^[-]|^[+]' ; then \
            echo "INFO: Need to reinitialize git submodules"; \
            git submodule update --init; \
	fi

release:
	@$(MAKE) -C projects/cm_mcu release

check-for-pr: format
	@./buildall.sh


# 2nd dollar sign in grep command is to pass along a single dollar sign to egrep itself
format:
	run-clang-format.py $(shell git diff --diff-filter=AM --name-only master | egrep '\.[ch]$$')

format-apply:
	run-clang-format.py -i $(shell git diff --diff-filter=AM --name-only master | egrep '\.[ch]$$')	

.PHONY: all clean $(DIRS) $(DIRSCLEAN) check-and-reinit-submodules
