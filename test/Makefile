# $Id$

CC?= gcc
CXX?= g++
PYTHON?=/usr/bin/env python

CURDIR?= $(shell /bin/pwd)
SRCDIR?= $(CURDIR)/..

CCFLAGS= -g -O0 -MMD -I. -I$(SRCDIR) -I- -DTRACING_ON=0
MYSQL= -I/usr/include/mysql -L/usr/lib/mysql -lmysqlclient

VPATH=$(SRCDIR):$(CURDIR)

default:
	@echo "You must specify a target"

base/traceflags.cc base/traceflags.hh: $(SRCDIR)/base/traceflags.py
	mkdir -p base; \
	cd base; \
	$(PYTHON) $<

bitvectest: test/bitvectest.cc
	$(CXX) $(CCFLAGS) -o $@ $^

circletest: test/circletest.cc base/circlebuf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

cprintftest: test/cprintftest.cc base/cprintf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

initest: test/initest.cc base/str.cc base/inifile.cc base/cprintf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

lrutest: test/lru_test.cc
	$(CXX) $(CCFLAGS) -o $@ $^

nmtest: test/nmtest.cc base/object_file.cc base/symtab.cc base/misc.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

offtest: test/offtest.cc
	$(CXX) $(CCFLAGS) -o $@ $^

rangetest: test/rangetest.cc base/range.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

STATTEST+= base/cprintf.cc base/hostinfo.cc base/misc.cc base/mysql.cc
STATTEST+= base/python.cc base/str.cc base/time.cc 
STATTEST+= base/statistics.cc base/stats/mysql.cc base/stats/python.cc
STATTEST+= base/stats/statdb.cc base/stats/text.cc base/stats/visit.cc
STATTEST+= test/stattest.cc 
stattest: $(STATTEST)
	$(CXX) $(CCFLAGS) $(MYSQL) -o $@ $^ 

strnumtest: test/strnumtest.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

symtest: test/symtest.cc base/misc.cc base/symtab.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

tokentest: test/tokentest.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

TRACE+=test/tracetest.cc base/trace.cc base/trace_flags.cc base/cprintf.cc
TRACE+=base/str.cc base/misc.cc
tracetest: $(TRACE)
	$(CXX) $(CCFLAGS) -o $@ $^

clean:
	@rm -rf *test *~ .#* *.core core base
.PHONY: clean
