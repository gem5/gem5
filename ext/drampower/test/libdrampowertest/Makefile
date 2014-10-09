# Copyright (c) 2012-2014, TU Delft
# Copyright (c) 2012-2014, TU Eindhoven
# Copyright (c) 2012-2014, TU Kaiserslautern
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Matthias Jung, Omar Naji, Sven Goossens
#

ifeq ($(COVERAGE),1)
	GCOVFLAGS := -fprofile-arcs -ftest-coverage
else
	GCOVFLAGS :=
endif

# Optimization flags. Usually you should not optimize until you have finished
# debugging, except when you want to detect dead code.
OPTCXXFLAGS ?=

# Debugging flags.
DBGCXXFLAGS ?= -g ${GCOVFLAGS}

# Common warning flags shared by both C and C++.
WARNFLAGS := -W -pedantic-errors -Wextra -Werror \
             -Wformat -Wformat-nonliteral -Wpointer-arith \
             -Wcast-align -Wconversion -Wall -Werror

# Sum up the flags.
CXXFLAGS := -O ${WARNFLAGS} ${DBGCXXFLAGS} ${OPTCXXFLAGS} -std=c++98

DRAMPOWER_PATH ?= ../..
MYPATH := ${DRAMPOWER_PATH}/test/libdrampowertest
USE_XERCES ?= 1

# Name of the generated binary.
BINARY := ${MYPATH}/library_test

ifeq ($(USE_XERCES),1)
	LIBS := -lxerces-c -ldrampowerxml -ldrampower
else
	LIBS := -ldrampower
endif

all:
	g++ ${MYPATH}/lib_test.cc ${CXXFLAGS} -iquote ${DRAMPOWER_PATH}/src -DUSE_XERCES=${USE_XERCES} -L${DRAMPOWER_PATH}/src/ ${LIBS} -o $(BINARY)

clean:
	rm -f $(BINARY)

coverageclean:
	$(RM) lib_test.gcno lib_test.gcda

test: all
	./$(BINARY) ${DRAMPOWER_PATH}/memspecs/MICRON_1Gb_DDR2-1066_16bit_H.xml

.PHONY: clean test
