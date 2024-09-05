DRIVER  := gem5_bridge
SOURCES := gem5_bridge gem5_ops gem5_poke
GROUP   := gem5

ifneq ($(KERNELRELEASE),)
obj-m := $(DRIVER).o
$(DRIVER)-objs := $(addprefix ./src/,$(addsuffix .o,$(SOURCES)))

# Build Parameters:
# - GEM5OPS_BASE : base physical address of gem5ops MMIO range
# - GEM5OPS_SIZE : size of gem5ops MMIO range
ifneq ($(GEM5OPS_BASE),)
ccflags-y += -DGEM5OPS_BASE=$(GEM5OPS_BASE)
endif
ifneq ($(GEM5OPS_SIZE),)
ccflags-y += -DGEM5OPS_SIZE=$(GEM5OPS_SIZE)
endif

else
PWD         ?= $(shell pwd)
KVERSION    ?= $(shell uname -r)
KMAKEDIR    ?= /lib/modules/$(KVERSION)/build
MODLOADCONF ?= /etc/modules-load.d/$(GROUP).conf

.PHONY: build
build:
	make -C $(KMAKEDIR) M=$(PWD) modules

.PHONY: install
install:
	make -C $(KMAKEDIR) M=$(PWD) INSTALL_MOD_DIR=$(GROUP) modules_install
	# May need to manually run `depmod --quick` after this

.PHONY: autoload
autoload:
	echo $(DRIVER) >> $(MODLOADCONF)

.PHONY: clean
clean:
	make -C $(KMAKEDIR) M=$(PWD) clean

endif
