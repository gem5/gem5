#Makefile for palcode
#Works on alpha-linux and builds elf executable

#19 December 2003 - Ali Saidi

GAS = as
CC = g++
LD = ld
#CFLAGS=-I ../h -E -P -D SIMOS -nostdinc -nostdinc++ -x c++
CFLAGS=-I . -E -P -D SIMOS -D BUILD_PALCODE -nostdinc -nostdinc++ -x c++
GASFLAGS=-m21164
LDFLAGS=-Ttext 0x4000

SOURCES=platform_tlaser.s osfpal.s
PREPROC := $(SOURCES:.s=.i)
OBJS := $(SOURCES:.s=.o)
    
%.i: %.s
	$(CC) $(CFLAGS) $< > $@

%.o: %.i
	$(GAS) $(GASFLAGS) -o $@ $< 


all: $(PREPROC) $(OBJS)
	ld $(LDFLAGS) -o osfpal $(OBJS) 

clean:
	rm -f *.o *.i osfpal
