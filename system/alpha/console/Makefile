
all: console

DBMENTRY	= fffffc0000010000
INCLUDES        = -I$(PALCODE) -I$(INCLUDEH) -I$(SIMTOOLS)/cpus-alpha/simos
#-I/udir/bugnion/ebfw/h -I/udir/bugnion/ebfw/palcode/xxm

SOURDIR = ./
EB_TOOLBOX = /wrl/proj/simos/bin/tools/osf
PALCODE  = ../palcode
INCLUDEH = ../h

dbmentry.o: dbmentry.s 
	$(AS)  $(INCLUDES) -o $*.o $*.s

console.o: console.c 
	$(CC)  -g3 $(INCLUDES) -o $*.o -c $*.c

printf.o: printf.c 
	$(CC)  -g3 $(INCLUDES) -o $*.o -c $*.c

paljtokern.o: $(SOURCEDIR)paljtokern.s
	rm $(OBJDIR)paljtokern.i
	cpp -I$(PALCODE) $(SOURCEDIR)paljtokern.s $(OBJDIR)paljtokern.i
	$(EB_TOOLBOX)/gas -21164 -o $(OBJDIR)paljtokern.aout.o $(OBJDIR)paljtokern.i
	$(EB_TOOLBOX)/astrip -a $(OBJDIR)paljtokern.aout.o $(OBJDIR)paljtokern.strip
	$(EB_TOOLBOX)/strip2array $(OBJDIR)paljtokern.strip $(OBJDIR)paljtokern.c
	$(CC) $(CCFLAGS) -o $(OBJDIR)paljtokern.o -c $(INCLUDES) $(OBJDIR)paljtokern.c

paljtoslave.o: $(SOURCEDIR)paljtoslave.s
	rm $(OBJDIR)paljtoslave.i
	cpp -I$(PALCODE) $(SOURCEDIR)paljtoslave.s $(OBJDIR)paljtoslave.i
	$(EB_TOOLBOX)/gas -21164 -o $(OBJDIR)paljtoslave.aout.o $(OBJDIR)paljtoslave.i
	$(EB_TOOLBOX)/astrip -a $(OBJDIR)paljtoslave.aout.o $(OBJDIR)paljtoslave.strip
	$(EB_TOOLBOX)/strip2array $(OBJDIR)paljtoslave.strip $(OBJDIR)paljtoslave.tmp
	sed -e 's/palJToKern/palJToSlave/' $(OBJDIR)paljtoslave.tmp >$(OBJDIR)paljtoslave.c
	$(CC) $(CCFLAGS) -o $(OBJDIR)paljtoslave.o -c $(INCLUDES) $(OBJDIR)paljtoslave.c

console: console.o dbmentry.o printf.o paljtokern.o paljtoslave.o 
	$(LD) -o console  -N -T $(DBMENTRY) -non_shared dbmentry.o console.o \
	printf.o paljtokern.o paljtoslave.o -lc

clean:
	rm *.o; rm console; rm *.strip; rm paljtokern.c paljtoslave.c; touch paljtokern.i paljtoslave.i
