# Copyright (c) 2025 The Regents of the University of California
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

# This Makefile compiles the hypercall-exit binary, used in the
# "hypercall-exit-check.py" script to test the hypercall exit functionality.
#
# Notes:
# * This must be compiled on an X86 host.

CC = g++

# This is the path to the gem5 root. Needs updated when added to gem5-resources
# source repo.
GEM5_ROOT = ../../../../..

CFLAGS = -Wall -Werror -O2 -static -I $(GEM5_ROOT)/include

SRC = hypercall-exit.cpp
OBJ = $(SRC:.cpp=.o)

TARGET = hypercall-exit

all: $(TARGET)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJ) $(GEM5_ROOT)/util/m5/build/x86/out

	$(CC) $(CFLAGS) -o $@ $(OBJ) -lm5 -L $(GEM5_ROOT)/util/m5/build/x86/out

$(GEM5_ROOT)/util/m5/build/x86/out:
	cd $(GEM5_ROOT)/util/m5 && scons build/x86/out/m5 -j `nproc`

clean:
	- rm -r $(OBJ) $(TARGET)  $(GEM5_ROOT)/util/m5/build/x86/out

.PHONY: all clean
