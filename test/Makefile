# $Id$

CC= gcc
CXX= g++

CURDIR?= $(shell /bin/pwd)
SRCDIR?= ..
TARGET?= alpha

TEST_SRCDIR?= $(SRCDIR)/test
ARCH_SRCDIR?= $(SRCDIR)/arch/$(TARGET)
BASE_SRCDIR?= $(SRCDIR)/base
SIM_SRCDIR?= $(SRCDIR)/sim
CACHE_SRCDIR?= $(SRCDIR)/sim/cache
OLD_SRCDIR= $(SRCDIR)/old

vpath % $(TEST_SRCDIR)
vpath % $(BASE_SRCDIR)
vpath % $(SIM_SRCDIR)
vpath % $(CACHE_SRCDIR)
vpath % $(OLD_SRCDIR)

CCFLAGS= -g -O0 -MMD -I. -I$(SRCDIR) -I- -DTRACING_ON=0

default:
	@echo "You must specify a target"

targetarch:
	ln -s ../arch/$(TARGET) targetarch

bitvectest: bitvectest.o
	$(CXX) $(LFLAGS) -o $@ $^

circletest: circletest.o circlebuf.o
	$(CXX) $(LFLAGS) -o $@ $^

cprintftest: cprintftest.o cprintf.o
	$(CXX) $(LFLAGS) -o $@ $^

initest: initest.o str.o inifile.o cprintf.o
	$(CXX) $(LFLAGS) -o $@ $^

lrutest: lru_test.o
	$(CXX) $(LFLAGS) -o $@ $^

nmtest: nmtest.o object_file.o symtab.o misc.o str.o
	$(CXX) $(LFLAGS) -o $@ $^

offtest: offtest.o
	$(CXX) $(LFLAGS) -o $@ $^

rangetest: rangetest.o str.o
	$(CXX) $(LFLAGS) -o $@ $^

stattest: cprintf.o hostinfo.o misc.o sim_time.o statistics.o stattest.o str.o
	$(CXX) $(LFLAGS) -o $@ $^

strnumtest: strnumtest.o str.o
	$(CXX) $(LFLAGS) -o $@ $^

symtest: misc.o symtest.o symtab.o str.o
	$(CXX) $(LFLAGS) -o $@ $^

tokentest: tokentest.o str.o
	$(CXX) $(LFLAGS) -o $@ $^

tracetest: tracetest.o trace.o trace_flags.o cprintf.o str.o misc.o
	$(CXX) $(LFLAGS) -o $@ $^

clean:
	@rm -f *.o *.d *test *~ .#* *.core core
.PHONY: clean

# C++ Compilation
%.o: %.cc
	@echo '$(CXX) $(CCFLAGS) -c $(notdir $<) -o $@'
	@$(CXX) $(CCFLAGS) -c $< -o $@

# C Compilation
%.o: %.c
	@echo '$(CC) $(CCFLAGS) -c $(notdir $<) -o $@'
	@$(CC) $(CCFLAGS) -c $< -o $@

-include *.d
