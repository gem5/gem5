
all: console

DBMENTRY	= fffffc0000010000
INCLUDES        = -I$(PALCODE) -I$(INCLUDEH) -I$(M5)/dev

SOURDIR = ./
PALCODE  = ../palcode
INCLUDEH = ../h
CC=gcc
#AS=gas

dbmentry.o: dbmentry.s 
	$(AS)  $(INCLUDES) -nointrinsics -o $*.o $*.s

console.o: console.c
	$(CC)  -g3 $(INCLUDES) -nointrinsics -o $*.o -c $*.c

printf.o: printf.c 
	$(CC)  -g3 $(INCLUDES) -nointrinsics -o $*.o -c $*.c

paljtokern.s.o: paljtokern.s
	g++ -I ../palcode -E -P -nostdinc -nostdinc++ -x c++ paljtokern.s | \
	gas -m 21164 -o paljtokern.s.o

paljtoslave.s.o: paljtoslave.s
	g++ -I ../palcode -E -P -nostdinc -nostdinc++ -x c++ paljtoslave.s | \
	gas -m 21164 -o paljtoslave.s.o

paljtokern.c: paljtokern.s.o
	echo 'unsigned int palJToKern[] = {' > paljtokern.c
	dis paljtokern.s.o | awk '{print "0x"$$2","}' >> paljtokern.c
	echo "0x0\n};" >> paljtokern.c

paljtoslave.c: paljtoslave.s.o
	echo "unsigned int palJToSlave[] = {" > paljtoslave.c
	dis paljtoslave.s.o | awk '{print "0x"$$2","}' >> paljtoslave.c
	echo "0x0\n};" >> paljtoslave.c

paljtokern.c.o: paljtokern.c
	$(CC) -g3 -nointrinsics -o paljtokern.c.o -c paljtokern.c

paljtoslave.c.o: paljtoslave.c
	$(CC) -g3 -nointrinsics -o paljtoslave.c.o -c paljtoslave.c

console: console.o dbmentry.o printf.o paljtokern.c.o paljtoslave.c.o 
	$(LD) -o console  -N -T $(DBMENTRY) -non_shared \
	dbmentry.o console.o printf.o paljtokern.c.o paljtoslave.c.o -lc

install: console
	scp console zizzer.eecs.umich.edu:/z/m5/system/testing/binaries/console

clean:
	rm -f *.o console *.strip paljtokern.c paljtoslave.c
