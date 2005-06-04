#Copyright (c) 2003, 2004
#The Regents of The University of Michigan
#All Rights Reserved
#
#This code is part of the M5 simulator, developed by Nathan Binkert,
#Erik Hallnor, Steve Raasch, and Steve Reinhardt, with contributions
#from Ron Dreslinski, Dave Greene, Lisa Hsu, Ali Saidi, and Andrew
#Schultz.
#
#Permission is granted to use, copy, create derivative works and
#redistribute this software and such derivative works for any purpose,
#so long as the copyright notice above, this grant of permission, and
#the disclaimer below appear in all copies made; and so long as the
#name of The University of Michigan is not used in any advertising or
#publicity pertaining to the use or distribution of this software
#without specific, written prior authorization.
#
#THIS SOFTWARE IS PROVIDED AS IS, WITHOUT REPRESENTATION FROM THE
#UNIVERSITY OF MICHIGAN AS TO ITS FITNESS FOR ANY PURPOSE, AND WITHOUT
#WARRANTY BY THE UNIVERSITY OF MICHIGAN OF ANY KIND, EITHER EXPRESS OR
#IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED WARRANTIES OF
#MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE REGENTS OF
#THE UNIVERSITY OF MICHIGAN SHALL NOT BE LIABLE FOR ANY DAMAGES,
#INCLUDING DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
#DAMAGES, WITH RESPECT TO ANY CLAIM ARISING OUT OF OR IN CONNECTION
#WITH THE USE OF THE SOFTWARE, EVEN IF IT HAS BEEN OR IS HEREAFTER
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

#Makefile for palcode
#Works on alpha-linux and builds elf executable

### If we are not compiling on an alpha, we must use cross tools ###    
ifneq ($(ARCHNAME), alpha)
CROSS_COMPILE?=alpha-unknown-linux-gnu-
endif
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)ld

CFLAGS=-I . -I ../h -D SIMOS -D BUILD_PALCODE -nostdinc -nostdinc++ -Wa,-m21164
LDFLAGS=-Ttext 0x4000

TLOBJS = osfpal.o  platform_tlaser.o
TLOBJS_COPY = osfpal_cache_copy.o osfpal_cache_copy_unaligned.o
TSOBJS = osfpal.o platform_m5.o
TSBOBJS = osfpal.o platform_m5b.o
TSOBJS_COPY = osfpal_cache_copy.o osfpal_cache_copy_unaligned.o

%.o: %.S
	$(CC) $(CFLAGS) -o $@ -c $<

all: tsunami tsunami_b64
	
platform_m5b.o: platform_m5.S
	$(CC) $(CFLAGS) -DBIG_TSUNAMI -o $@ -c $<
	
tlaser:  $(TLOBJS)
	$(LD) $(LDFLAGS) -o tl_osfpal osfpal.o platform_tlaser.o

tlaser_copy: $(TLOBJS_COPY) $(TLOBJS)
	$(LD) $(LDFLAGS) -o tl_osfpal_cache osfpal_cache_copy.o platform_tlaser.o
	$(LD) $(LDFLAGS) -o tl_osfpal_unalign osfpal_cache_copy_unaligned.o platform_tlaser.o

tsunami: $(TSOBJS)
	$(LD) $(LDFLAGS) -o ts_osfpal osfpal.o platform_m5.o

tsunami_b64: $(TSBOBJS)
	$(LD) $(LDFLAGS) -o tsb_osfpal osfpal.o platform_m5b.o

tsunami_copy: $(TSOBJS) $(TSOBJS_COPY)
	$(LD) $(LDFLAGS) -o ts_osfpal_cache  osfpal_cache_copy.o platform_m5.o
	$(LD) $(LDFLAGS) -o ts_osfpal_unalign osfpal_cache_copy_unaligned.o platform_m5.o

clean:
	rm -f *.o tl_osfpal tl_osfpal_cache tl_osfpal_unalign ts_osfpal \
	ts_osfpal_cache ts_osfpal_unalign tsb_osfpal
