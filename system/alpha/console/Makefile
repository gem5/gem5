# Copyright (c) 2005 The Regents of The University of Michigan
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
# Authors: Nathan L. Binkert
#          Ali G. Saidi

# Point to the M5 diretory so we can get some headers
M5?=../../..

### If we are not compiling on an alpha, we must use cross tools ###    
ifneq ($(shell uname -m), alpha)
CROSS_COMPILE?=alpha-unknown-linux-gnu-
endif
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)ld

DBMENTRY= fffffc0000010000
CFLAGS=-I . -I ../h -I$(M5)/src/dev/alpha -I$(M5)/util/m5/ -fno-builtin -Wa,-m21164
OBJS=dbmentry.o printf.o paljtokern.o paljtoslave.o console.o m5op.o

all: console

m5op.o: $(M5)/util/m5/m5op_alpha.S
	$(CC) $(CFLAGS) -nostdinc -o $@ -c $<

%.o: %.S
	$(CC) $(CFLAGS) -nostdinc -o $@ -c $<

%.o: %.c
	$(CC)  -g3 $(CFLAGS) -o $@ -c $<

console: $(OBJS)
	$(LD) -o console -N -Ttext $(DBMENTRY) -non_shared $(OBJS) -lc

clean:
	rm -f *.o console
