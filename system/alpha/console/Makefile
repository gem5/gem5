DBMENTRY	= fffffc0000010000
CFLAGS=-I . -I ../h -I$(M5)/dev -nostdinc++ -Wa,-m21164

ARCHNAME=$(shell uname -m)

### If we are not compiling on an alpha, we must use cross tools ###    
ifneq ($(ARCHNAME), alpha)
CC=alpha-unknown-linux-gnu-gcc
AS=alpha-unknown-linux-gnu-as
LD=alpha-unknown-linux-gnu-ld
endif

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
	$(CC)  -g3 $(CFLAGS) -D _TIME_T -D TSUNAMI -o $@ -c $<

console_tl.o: console.c
	$(CC)  -g3 $(CFLAGS) -D _TIME_T -D TLASER  -o $@ -c $< 

printf.o: printf.c 
	$(CC)  -g3 $(CFLAGS) -o $@ -c $<

console_ts: $(TSOBJS)
	$(LD) -o console_ts  -N -Ttext $(DBMENTRY) -non_shared $(TSOBJS) -lc

console_tl: $(TLOBJS)
	$(LD) -o console_tl  -N -Ttext $(DBMENTRY) -non_shared $(TLOBJS) -lc

install: console
	scp console zizzer.eecs.umich.edu:/z/m5/system/testing/binaries/console

clean:
	rm -f *.o console_t?
