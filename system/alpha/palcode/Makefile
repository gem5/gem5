#Makefile for palcode
#Works on alpha-linux and builds elf executable

GAS = alpha-linux-as
CC = alpha-linux-gcc
LD = alpha-linux-ld
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
