# Copyright (c) 2019 ARM Limited
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

ARCH = x86
CC = gcc
CFLAGS = -static -O2
OUT = $(OUTDIR)/m5_exit
OUTDIR = ./
SQUASHFS = $(OUT).squashfs
SQUASHFS_ROOT = $(OUTDIR)/squashfs_root
INIT = $(SQUASHFS_ROOT)/sbin/init

.PHONY: all squashfs

all: $(OUT)

$(OUT): m5-exit.c
	$(CROSS_COMPILE)$(CC) \
	-DM5 \
	-I $(CURDIR)/../../../../include/ \
	-o $@ \
	$(CFLAGS) \
	$(CURDIR)/../../../../util/m5/m5op_$(ARCH).S \
	$<

squashfs: $(SQUASHFS)

$(SQUASHFS): $(OUT)
	mkdir -p $$(dirname $(INIT))
	cp $< $(INIT)
	mksquashfs $(SQUASHFS_ROOT) $(SQUASHFS)

clean:
	rm -fr $(SQUASHFS) $(SQUASHFS_ROOT) $(OUT)
