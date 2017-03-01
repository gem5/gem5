# This file (pa-risc.Makefile) is part of the port of QuickThreads for
# PA-RISC 1.1 architecture.  This file is a machine dependent makefile
# for QuickThreads.  It was written in 1994 by Uwe Reder
# (`uereder@cip.informatik.uni-erlangen.de') for the Operating Systems
# Department (IMMD4) at the University of Erlangen/Nuernberg Germany.

# `Normal' configuration.

CC = cc -Aa

.s.o:
	/usr/ccs/bin/as -o $@ $<
