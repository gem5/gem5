# Copyright (c) 2006-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Nathan Binkert
#          Steve Reinhardt

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

bitvectest: unittest/bitvectest.cc
	$(CXX) $(CCFLAGS) -o $@ $^

circletest: unittest/circletest.cc base/circlebuf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

cprintftest: unittest/cprintftest.cc base/cprintf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

cprintftime: unittest/cprintftime.cc base/cprintf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

initest: unittest/initest.cc base/str.cc base/inifile.cc base/cprintf.cc
	$(CXX) $(CCFLAGS) -o $@ $^

lrutest: unittest/lru_test.cc
	$(CXX) $(CCFLAGS) -o $@ $^

nmtest: unittest/nmtest.cc base/output.cc base/hostinfo.cc base/cprintf.cc base/misc.cc base/loader/object_file.cc base/loader/symtab.cc base/misc.cc base/str.cc base/loader/aout_object.cc base/loader/ecoff_object.cc base/loader/elf_object.cc
	$(CXX) $(CCFLAGS) -I/n/ziff/z/binkertn/build/work/ALPHA_FS -lelf -o $@ $^

offtest: unittest/offtest.cc
	$(CXX) $(CCFLAGS) -o $@ $^

rangetest: unittest/rangetest.cc base/range.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

STATTEST+= base/cprintf.cc base/hostinfo.cc base/misc.cc base/mysql.cc
STATTEST+= base/python.cc base/str.cc base/time.cc 
STATTEST+= base/statistics.cc base/stats/mysql.cc base/stats/python.cc
STATTEST+= base/stats/statdb.cc base/stats/text.cc base/stats/visit.cc
STATTEST+= unittest/stattest.cc 
stattest: $(STATTEST)
	$(CXX) $(CCFLAGS) $(MYSQL) -o $@ $^ 

strnumtest: unittest/strnumtest.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

symtest: unittest/symtest.cc base/misc.cc base/symtab.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

tokentest: unittest/tokentest.cc base/str.cc
	$(CXX) $(CCFLAGS) -o $@ $^

TRACE+=unittest/tracetest.cc base/trace.cc base/trace_flags.cc base/cprintf.cc
TRACE+=base/str.cc base/misc.cc
tracetest: $(TRACE)
	$(CXX) $(CCFLAGS) -o $@ $^

clean:
	@rm -rf *test *~ .#* *.core core base
.PHONY: clean
