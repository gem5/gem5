#=======================================================================
# Makefile for riscv-tests/isa
#-----------------------------------------------------------------------

XLEN ?= 64

src_dir 	:= ./isa
bin_dir 	:= ../../bin/riscv
dump_dir 	:= ../../dump/riscv

include $(src_dir)/rv64ui/Makefrag
include $(src_dir)/rv64uc/Makefrag
include $(src_dir)/rv64um/Makefrag
include $(src_dir)/rv64ua/Makefrag
include $(src_dir)/rv64uf/Makefrag
include $(src_dir)/rv64ud/Makefrag
include $(src_dir)/rv64si/Makefrag
include $(src_dir)/rv64mi/Makefrag
include $(src_dir)/rv64uamt/Makefrag
include $(src_dir)/rv64samt/Makefrag

default: all

#--------------------------------------------------------------------
# Build rules
#--------------------------------------------------------------------

RISCV_PREFIX ?= riscv$(XLEN)-unknown-elf-
RISCV_GCC ?= $(RISCV_PREFIX)gcc
RISCV_GCC_OPTS ?= -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles
RISCV_OBJDUMP ?= $(RISCV_PREFIX)objdump --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data
RISCV_SIM ?= spike

vpath %.S $(src_dir)

#------------------------------------------------------------
# Build assembly tests

%.dump: %
	mkdir -p $(dump_dir)
	$(RISCV_OBJDUMP) $(bin_dir)/$< > $(dump_dir)/$@

%.out: %
	$(RISCV_SIM) --isa=rv64gc $< 2> $@

define compile_template

$$($(1)_ps_tests): $(1)-ps-%: $(1)/%.S
	mkdir -p $(bin_dir)
	$$(RISCV_GCC) $(2) $$(RISCV_GCC_OPTS) -I$(src_dir)/../env/ps -I$(src_dir)/macros/scalar -I$(src_dir)/macros/mt -T$(src_dir)/../env/ps/link.ld $$< -o $(bin_dir)/$$@
$(1)_ps_env_tests += $$($(1)_ps_tests)

$(1)_tests_dump = $$(addsuffix .dump, $$($(1)_tests))

$(1): $$($(1)_tests_dump)

.PHONY: $(1)

p_env_tests 	+= $$($(1)_p_env_tests)
v_env_tests 	+= $$($(1)_v_env_tests)
ps_env_tests 	+= $$($(1)_ps_env_tests)

endef

$(eval $(call compile_template,rv64ui,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64uc,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64um,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64ua,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64uf,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64ud,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64si,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64mi,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64uamt,-march=rv64g -mabi=lp64))
$(eval $(call compile_template,rv64samt,-march=rv64g -mabi=lp64))

ps_env_tests_dump = $(addsuffix .dump, $(ps_env_tests))

#------------------------------------------------------------
# Targets

all: 	ps
# build tests with ps environment
ps: 	$(ps_env_tests_dump)

#------------------------------------------------------------
# Clean up

clean:
	rm -rf $(bin_dir) $(dump_dir)
