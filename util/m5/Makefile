AS=as
CC=cc
LD=cc

CCFLAGS=-O2
#LDFLAGS=-non_shared

all: m5

m5: m5op.o m5.o
	$(LD) $(LDFLAGS) -o $@ $>
	strip $@

clean:
	@rm -f m5 *.o *.d *~ .#*

.SUFFIXES:
.SUFFIXES:.o .c .s

# C Compilation
.c.o:
	$(CC) $(CCFLAGS) -o $@ -c $<

# Assembly
.s.o:
	$(AS) $(ASFLAGS) -o $@ $<
