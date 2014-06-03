TAR = mcpat
BUILD_DIR ?= ../../build
ODIR = $(BUILD_DIR)/mcpat

.PHONY: all dbg opt clean clean_dbg clean_opt

all: opt

dbg: $(TAR).mk $(ODIR)/obj_dbg
	@$(MAKE) TAG=dbg ODIR=$(ODIR) -C . -f $(TAR).mk

opt: $(TAR).mk $(ODIR)/obj_opt
	@$(MAKE) TAG=opt ODIR=$(ODIR) -C . -f $(TAR).mk

$(ODIR)/obj_dbg:
	mkdir -p $@

$(ODIR)/obj_opt:
	mkdir -p $@

clean: clean_dbg clean_opt

clean_dbg: $(ODIR)/obj_dbg
	@$(MAKE) TAG=dbg ODIR=$(ODIR) -C . -f $(TAR).mk clean
	rm -rf $<

clean_opt: $(ODIR)/obj_opt
	@$(MAKE) TAG=opt ODIR=$(ODIR) -C . -f $(TAR).mk clean
	rm -rf $<

