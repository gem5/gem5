# 
# $Id: Makefile,v 1.2 1997/12/16 01:18:38 bugnion Exp $
#
# Revision History:
#
# $Log: Makefile,v $
# Revision 1.2  1997/12/16 01:18:38  bugnion
# Removed bogus TLASER offsets from palcode build. -- roll over
# simultaneously with the simulation tree
#
# Revision 1.1.1.1  1997/10/30 23:27:18  verghese
# current 10/29/97
#
#
# Set environment variables to point to various things:
#
#	EB_TOOLBOX - Where your toolset is located
#

EB_TOOLBOX =  /wrl/proj/simos/bin/tools/osf
CPP	  =	/usr/bin/cpp
AS	  =	$(EB_TOOLBOX)/gas
LD	  =	$(EB_TOOLBOX)/gld
DIS	  =	$(EB_TOOLBOX)/alist
STRIP	  =	$(EB_TOOLBOX)/astrip
PVC	  =	$(EB_TOOLBOX)/pvc
MAKEDEP	  =	$(CPP) -MM

#
# Define KDEBUG if you want a special unprivileged CALL_PAL 
# breakpoint trap handler for remote kernel debugging.
#
# Define CONSOLE_ENTRY to change the sys$enter_console
# transfer address.  Default CONSOLE_ENTRY value is 0x10000.
#
# Define DISABLE_CRD to disable CRD. Note that reset sets MCES so that
# correctable errors are ignored anyway, but this actually stops the
# interrupt.
#

DEFINES   =	 -DDISABLE_CRD -DSIMOS -DBUILD_PALCODE
DEFINES  +=  -I$(SIMTOOLS)/cpus-alpha/simos

CPPFLAGS  =
ASFLAGS   =	-21164
#LDFLAGS   =	-Tstrip 0 -Thdr -N
#LDFLAGS   =	-Tstrip 2000 -Thdr -N  # removed bugnion
LDFLAGS   =	-Tstrip 4000 -Thdr -N
# Source files:
#
#   This is the only block in which the list of source files should change.
#
#	SFILES - assembler source files
#	HFILES - header files
#

SFILES	= osfpal.s platform.s

HFILES	= dc21164.h \
	  osf.h \
	  macros.h \
	  ev5_impure.h \
	  cserve.h \
	  platform.h

# Intermediate files:
#
#   This block should not change.
#

IFILES	= $(SFILES:.s=.i)

# Object files:
#
#   This block should not change.
#

OFILES 	= $(IFILES:.i=.o)

.DEFAULT:
	co -u $<

.SUFFIXES:
.SUFFIXES: .s .i .o

.s.i:
	$(CPP) $(CPPFLAGS) $(DEFINES) $< $*.i

osfpal.nh: osfpal
	$(STRIP) -a osfpal $@
	$(DIS) osfpal >osfpal.dis

osfpal: $(OFILES)
	echo '$OFILES= ' $(OFILES)
	$(LD) $(LDFLAGS) -o $@ $(OFILES)

osfpal.o: osfpal.i
	$(AS) $(ASFLAGS) -o $@ osfpal.i

platform.o: platform.i
	$(AS) $(ASFLAGS)  -o $@ platform.i

pvc: osfpal.lis osfpal.nh osfpal.ent osfpal.map
	(export PVC_PAL PVC_ENTRY PVC_MAP PVC_CPU;	\
	 PVC_PAL=osfpal.nh;				\
	 PVC_ENTRY=osfpal.ent;				\
	 PVC_MAP=osfpal.map;				\
	 PVC_CPU=ev5;					\
	 $(PVC);)

osfpal.lis: osfpal
	$(DIS) osfpal > $@

osfpal.map: osfpal
	$(DIS) -m osfpal > $@
	
depend:
	@cat < /dev/null > makedep
	@(for i in $(SFILES); do echo $$i; \
	    $(MAKEDEP) $(DEFINES) $$i | 					\
		awk '{ if ($$1 != prev) {if (rec != "") print rec; 		\
		    rec = $$0; prev = $$1; } 					\
		    else { if (length(rec $$2) > 78) { print rec; rec = $$0; } 	\
		    else rec = rec " " $$2 } }		 			\
		    END { print rec }' | sed 's/\.o/\.i/'			\
		    >> makedep; done)
	@echo '/^# DO NOT DELETE THIS LINE/+1,$$d' > eddep
	@echo '$$r makedep' >> eddep
	@echo 'w' >> eddep
	@cp Makefile Makefile.bak
	@ed - Makefile < eddep
	@rm -f eddep makedep
	@echo '# DEPENDENCIES MUST END AT END OF FILE' >> Makefile
	@echo '# IF YOU PUT STUFF HERE IT WILL GO AWAY' >> Makefile
	@echo '# see make depend above' >> Makefile

clean:
	rm -f core $(OFILES) $(IFILES)

clobber: clean
	rm -f osfpal.lis osfpal.nh osfpal.map osfpal

rcsinfo:
	rlog RCS/*

rcsget:
	co -u $(HFILES) $(SFILES)

# DO NOT DELETE THIS LINE
osfpal.i:	osfpal.s
platform.i:	platform.s ./cserve.h ./platform.h
# DEPENDENCIES MUST END AT END OF FILE
# IF YOU PUT STUFF HERE IT WILL GO AWAY
# see make depend above



