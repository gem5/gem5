#Makefile for palcode
#Works on alpha-linux and builds elf executable

#19 December 2003 - Ali Saidi

GAS = alpha-elf-as
CC = alpha-elf-g++
LD = alpha-elf-ld
#CFLAGS=-I ../h -E -P -D SIMOS -nostdinc -nostdinc++ -x c++
CFLAGS=-I . -I ../h -E -P -D SIMOS -D BUILD_PALCODE -nostdinc -nostdinc++ -x c++
GASFLAGS=-m21164
LDFLAGS=-Ttext 0x4000

SOURCES=osfpal.s platform_tlaser.s platform_m5.s
PREPROC := $(SOURCES:.s=.i)
TLOBJS = osfpal.o platform_tlaser.o
TSOBJS = osfpal.o platform_m5.o
    
%.i: %.s
	$(CC) $(CFLAGS) $< > $@

%.o: %.i
	$(GAS) $(GASFLAGS) -o $@ $< 

all: tlaser tsunami
        
tlaser: $(PREPROC) $(TLOBJS)
	$(LD) $(LDFLAGS) -o tl_osfpal $(TLOBJS) 

tsunami: $(PREPROC) $(TSOBJS)
	$(LD) $(LDFLAGS) -o ts_osfpal $(TSOBJS) 

clean:
	rm -f *.o *.i osfpal ts_osfpal tl_osfpal
 
