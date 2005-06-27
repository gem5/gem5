# Copyright (c) 2005 
# The Regents of The University of Michigan
# All Rights Reserved
#
# This code is part of the M5 simulator, developed by Nathan Binkert,
# Erik Hallnor, Steve Raasch, and Steve Reinhardt, with contributions
# from Ron Dreslinski, Dave Greene, Lisa Hsu, Ali Saidi, and Andrew
# Schultz.
#
# Permission is granted to use, copy, create derivative works and
# redistribute this software and such derivative works for any purpose,
# so long as the copyright notice above, this grant of permission, and
# the disclaimer below appear in all copies made; and so long as the
# name of The University of Michigan is not used in any advertising or
# publicity pertaining to the use or distribution of this software
# without specific, written prior authorization.
#
# THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
# UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND WITHOUT
# WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER EXPRESS OR
# IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE REGENTS OF
# THE UNIVERSITY OF MICHIGAN SHALL NOT BE LIABLE FOR ANY DAMAGES,
# INCLUDING DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, WITH RESPECT TO ANY CLAIM ARISING OUT OF OR IN CONNECTION
# WITH THE USE OF THE SOFTWARE, EVEN IF IT HAS BEEN OR IS HEREAFTER
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

### If we are not compiling on an alpha, we must use cross tools ###    
ifneq ($(shell uname -m), alpha)
CROSS_COMPILE?=alpha-unknown-linux-gnu-
endif
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)ld

DBMENTRY= fffffc0000010000
CFLAGS=-I . -I ../h -I$(M5)/dev -fno-builtin -Wa,-m21164
OBJS=dbmentry.o printf.o paljtokern.o paljtoslave.o
TLOBJS+=$(OBJS) console_tl.o
TSOBJS+=$(OBJS) console_ts.o

### Make sure that the M5 variable is set ###
ifndef M5
$(error The M5 variable must be set)
endif

all: console_tl console_ts

%.o: %.S
	$(CC) $(CFLAGS) -nostdinc -o $@ -c $<

console_ts.o: console.c
	$(CC)  -g3 $(CFLAGS) -D TSUNAMI -o $@ -c $<

console_tl.o: console.c
	$(CC)  -g3 $(CFLAGS) -D TLASER  -o $@ -c $< 

printf.o: printf.c 
	$(CC)  -g3 $(CFLAGS) -o $@ -c $<

console_ts: $(TSOBJS)
	$(LD) -o console_ts -N -Ttext $(DBMENTRY) -non_shared $(TSOBJS) -lc

console_tl: $(TLOBJS)
	$(LD) -o console_tl -N -Ttext $(DBMENTRY) -non_shared $(TLOBJS) -lc

clean:
	rm -f *.o console_t?
