# Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
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

include toolchain.mk

.DEFAULT_GOAL := main.elf

OBJS = boot.o syscalls.o main.o isr.o m5op.o

.PHONY: clean

$(OBJS): Makefile toolchain.mk
main.o: main.c bench.h inputs.h ../harness_clstr_hw_defines.h
isr.o: isr.c bench.h

main.elf: $(OBJS) $(LNK_SCRIPT) Makefile toolchain.mk
	$(CC) $(LNK_FILE_OPT) -o $@ $(OBJS) $(LNK_OPT)

m5op.o: $(M5OP_SRC)
	$(CC) $(CPPFLAGS) -x assembler-with-cpp -c -o $@ $<

boot.o: boot.s Makefile
	$(CPP) $(CPPFLAGS) boot.s | $(AS) $(ASFLAGS) -o boot.o

clean:
	rm -f $(OBJS) main.elf
