# Copyright (c) 2003, 2004
# The Regents of The University of Michigan
# All Rights Reserved
#
# This code is part of the M5 simulator.
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
#
# Authors: Nathan L. Binkert
#          Ali G. Saidi

# Makefile for palcode
# Works on alpha-linux and builds elf executable

### If we are not compiling on an alpha, we must use cross tools ###    
ifneq ($(shell uname -m), alpha)
CROSS_COMPILE?=alpha-unknown-linux-gnu-
endif
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)ld

CFLAGS=-I . -I ../h -nostdinc -nostdinc++ -Wa,-m21164
LDFLAGS=-Ttext 0x4000

TLOBJS = osfpal.o platform_tlaser.o
TLOBJS_COPY = osfpal_cache_copy.o platform_tlaser.o
TLOBJS_COPY_UNALIGNED = osfpal_cache_copy_unaligned.o platform_tlaser.o
TSOBJS = osfpal.o platform_tsunami.o
TSBOBJS = osfpal.o platform_bigtsunami.o
TSOBJS_COPY = osfpal_cache_copy.o platform_tsunami.o
TSOBJS_COPY_UNALIGNED = osfpal_cache_copy_unaligned.o platform_bigtsunami.o

all: tlaser tsunami tsunami_b64

all_copy: tlaser tlaser_copy tsunami tsunami_b64 tsunami_copy

osfpal.o: osfpal.S
	$(CC) $(CFLAGS) -o $@ -c $<

osfpal_cache_copy.o: osfpal.S
	$(CC) $(CFLAGS) -DCACHE_COPY -o $@ -c $<

osfpal_cache_copy_unaligned.o: osfpal.S
	$(CC) $(CFLAGS) -DCACHE_COPY -DCACHE_COPY_UNALIGNED -o $@ -c $<

platform_tlaser.o: platform.S
	$(CC) $(CFLAGS) -DTLASER -o $@ -c $<

platform_tsunami.o: platform.S
	$(CC) $(CFLAGS) -DTSUNAMI -o $@ -c $<

platform_bigtsunami.o: platform.S
	$(CC) $(CFLAGS) -DBIG_TSUNAMI -o $@ -c $<

tlaser:  $(TLOBJS)
	$(LD) $(LDFLAGS) -o tl_osfpal $(TLOBJS)

tlaser_copy: $(TLOBJS_COPY) $(TLOBJS_COPY_UNALIGNED)
	$(LD) $(LDFLAGS) -o tl_osfpal_cache $(TLOBJS_COPY)
	$(LD) $(LDFLAGS) -o tl_osfpal_unalign $(TLOBJS_COPY_UNALIGNED)

tsunami: $(TSOBJS)
	$(LD) $(LDFLAGS) -o ts_osfpal $(TSOBJS)

tsunami_b64: $(TSBOBJS)
	$(LD) $(LDFLAGS) -o tsb_osfpal $(TSBOBJS)

tsunami_copy: $(TSOBJS_COPY) $(TSOBJS_COPY_UNALIGNED)
	$(LD) $(LDFLAGS) -o ts_osfpal_cache $(TSOBJS_COPY)
	$(LD) $(LDFLAGS) -o ts_osfpal_unalign $(TSOBJS_COPY_UNALIGNED)

clean:
	rm -f *.o tl_osfpal tl_osfpal_cache tl_osfpal_unalign ts_osfpal \
	ts_osfpal_cache ts_osfpal_unalign tsb_osfpal
