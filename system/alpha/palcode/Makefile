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

SOURCES=osfpal.s osfpal_cache_copy.s osfpal_cache_copy_unaligned.s platform_tlaser.s \
	 platform_m5.s
PREPROC := $(SOURCES:.s=.i)
TLOBJS = osfpal.o osfpal_cache_copy.o osfpal_cache_copy_unaligned.o platform_tlaser.o
TSOBJS = osfpal.o osfpal_cache_copy.o osfpal_cache_copy_unaligned.o platform_m5.o

%.i: %.s
	$(CC) $(CFLAGS) $< > $@

%.o: %.i
	$(GAS) $(GASFLAGS) -o $@ $< 

all: tlaser tsunami

tlaser: $(PREPROC) $(TLOBJS)
	$(LD) $(LDFLAGS) -o tl_osfpal osfpal.o platform_tlaser.o
	$(LD) $(LDFLAGS) -o tl_osfpal_cache osfpal_cache_copy.o platform_tlaser.o
	$(LD) $(LDFLAGS) -o tl_osfpal_unalign osfpal_cache_copy_unaligned.o platform_tlaser.o

tsunami: $(PREPROC) $(TSOBJS)
	$(LD) $(LDFLAGS) -o ts_osfpal osfpal.o platform_m5.o
	$(LD) $(LDFLAGS) -o ts_osfpal_cache  osfpal_cache_copy.o platform_m5.o
	$(LD) $(LDFLAGS) -o ts_osfpal_unalign osfpal_cache_copy_unaligned.o platform_m5.o

clean:
	rm -f *.o *.i tl_osfpal tl_osfpal_cache tl_osfpal_unalign ts_osfpal \
	ts_osfpal_cache ts_osfpal_unalign
