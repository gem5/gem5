DBMENTRY	= fffffc0000010000
INCLUDES        = -I$(PALCODE) -I$(INCLUDEH) -I$(M5)/dev

SOURDIR = ./
PALCODE  = ../palcode
INCLUDEH = ../h

ARCHNAME=$(shell uname -m)

### If we are not compiling on an alpha, we must use cross tools ###    
ifneq ($(ARCHNAME), alpha)
CC=alpha-unknown-linux-gnu-gcc
AS=alpha-unknown-linux-gnu-as
CXX=alpha-unknown-linux-gnu-g++
LD=alpha-unknown-linux-gnu-ld
endif

### Make sure that the M5 variable is set ###
ifndef M5
$(error The M5 variable must be set)
endif

all: console_tl console_ts

dbmentry.o: dbmentry.s 
	g++ -I ../h -I ../palcode -E -P -nostdinc -nostdinc++ -x c++ dbmentry.s | \
	$(AS) -m 21164 -o dbmentry.s.o

console_ts.o: console.c
	$(CC)  -g3 $(INCLUDES) -D _TIME_T -D TSUNAMI -nostdinc++ -o console_ts.o -c console.c

console_tl.o: console.c
	$(CC)  -g3 $(INCLUDES) -D _TIME_T -D TLASER  -nostdinc++ -o console_tl.o -c console.c

printf.o: printf.c 
	$(CC)  -g3 $(INCLUDES) -nostdinc++ -o $*.o -c $*.c

paljtokern.s.o: paljtokern.s
	g++ -I ../h -E -P -nostdinc -nostdinc++ -x c++ paljtokern.s | \
	$(AS) -m 21164 -o paljtokern.s.o

paljtoslave.s.o: paljtoslave.s
	g++ -I ../h -E -P -nostdinc -nostdinc++ -x c++ paljtoslave.s | \
	$(AS) -m 21164 -o paljtoslave.s.o

console_ts: console_ts.o dbmentry.o printf.o paljtokern.s.o paljtoslave.s.o 
	$(LD) -o console_ts  -N -Ttext $(DBMENTRY) -non_shared \
	dbmentry.s.o console_ts.o printf.o paljtokern.s.o paljtoslave.s.o -lc

console_tl: console_tl.o dbmentry.o printf.o paljtokern.s.o paljtoslave.s.o 
	$(LD) -o console_tl  -N -Ttext $(DBMENTRY) -non_shared \
	dbmentry.s.o console_tl.o printf.o paljtokern.s.o paljtoslave.s.o -lc

install: console
	scp console zizzer.eecs.umich.edu:/z/m5/system/testing/binaries/console

clean:
	rm -f *.o console *.strip paljtokern.c paljtoslave.c
