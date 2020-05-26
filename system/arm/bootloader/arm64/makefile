# Copyright (c) 2019-2020 ARM Limited
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

CROSS_COMPILE = aarch64-linux-gnu-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld

.SUFFIXES:

BUILDDIR = .
DESTDIR = $(error Please set DESTDIR to wanted installation directory)

CFLAGS = -march=armv8-a
CPPFLAGS = -DPHYS_OFFSET=0x80000000 \
		   -DUART_BASE=0x1c090000 -DSYSREGS_BASE=0x1c010000 \
		   -Dkernel=0x80080000 \
		   -Dmbox=0x8000fff8 -Ddtb=0x80000100

LDFLAGS = -N -Ttext 0x00000010 -static

.PHONY: all clean install mkdir

all: mkdir $(BUILDDIR)/boot_emm.arm64 \
	$(BUILDDIR)/boot.arm64  \
	$(BUILDDIR)/boot_v2.arm64

#v1 has a GIC V2
$(BUILDDIR)/boot_emm.o: CPPFLAGS += -UGICV3 -DGIC_CPU_BASE=0x2c002000 \
	-DGIC_DIST_BASE=0x2c001000
$(BUILDDIR)/boot.o: CPPFLAGS += -UGICV3 -DGIC_CPU_BASE=0x2c002000 \
	-DGIC_DIST_BASE=0x2c001000

#V2 has a GIC V3
$(BUILDDIR)/boot_v2.o: CPPFLAGS += -DGICV3 -DGIC_REDIST_BASE=0x2c010000 \
	-DGIC_DIST_BASE=0x2c000000

$(BUILDDIR)/%.arm64: $(BUILDDIR)/%.o
	$(LD) -o $@ $< $(LDFLAGS)

$(BUILDDIR)/%.o: boot.S
	$(CC) $(CPPFLAGS) -c $< -o $@ $(CFLAGS)

install:
	mkdir -p $(DESTDIR)
	install -m 644 $(BUILDDIR)/boot_emm.arm64 \
		$(BUILDDIR)/boot.arm64 \
		$(BUILDDIR)/boot_v2.arm64 \
		$(DESTDIR)/.

mkdir:
	mkdir -p $(BUILDDIR)

clean:
	rm -f $(BUILDDIR)/*.o
	rm -f $(BUILDDIR)/boot_emm.arm64 $(BUILDDIR)/boot.arm64 $(BUILDDIR)/boot_v2.arm64
