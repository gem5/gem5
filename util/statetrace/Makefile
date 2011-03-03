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
# Authors: Gabe Black

CXX      := g++
INCLUDES := -I ./ -I ./arch
CXXFLAGS := -O3 -ggdb

define build-obj
$(CXX) -c $(patsubst %.o,%.cc,$@) -o $@ $(INCLUDES) $(CXXFLAGS)
endef

define final-link
$(CXX) $(INCLUDES) $(CXXFLAGS) -o $@ $^
endef

all: statetrace

statetrace.o: statetrace.cc tracechild.hh regstate.hh
	$(build-obj)
tracechild.o: tracechild.cc tracechild.hh regstate.hh
	$(build-obj)
tracechild_arch.o: statetrace.cc tracechild.hh regstate.hh arch/tracechild_arm.hh arch/tracechild_arm.cc arch/tracechild_i386.hh arch/tracechild_i386.cc arch/tracechild_amd64.cc arch/tracechild_amd64.hh arch/tracechild_sparc.cc arch/tracechild_sparc.hh
	$(build-obj)

statetrace: statetrace.o tracechild.o tracechild_arch.o
	$(final-link)

clean:
	rm -f *.o statetrace
