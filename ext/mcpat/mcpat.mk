TARGET = mcpat
SHELL = /bin/sh
.PHONY: all depend clean
.SUFFIXES: .cc .o

ifndef NTHREADS
  NTHREADS = 4
endif


LIBS = 
INCS = -lm

ifeq ($(TAG),dbg)
  DBG = -Wall 
  OPT = -ggdb -g -O0 -DNTHREADS=1 -Icacti
else
  DBG = 
  OPT = -O3 -msse2 -mfpmath=sse -DNTHREADS=$(NTHREADS) -Icacti
  #OPT = -O0 -DNTHREADS=$(NTHREADS)
endif

#CXXFLAGS = -Wall -Wno-unknown-pragmas -Winline $(DBG) $(OPT) 
CXXFLAGS = -Wno-unknown-pragmas $(DBG) $(OPT) 
CXX = g++
CC  = gcc

VPATH = cacti

SRCS  = \
  Ucache.cc \
  arbiter.cc \
  area.cc \
  array.cc \
  bank.cc \
  basic_circuit.cc \
  basic_components.cc \
  bus_interconnect.cc \
  cachearray.cc \
  cachecontroller.cc \
  cacheunit.cc \
  cacti_interface.cc \
  component.cc \
  core.cc \
  crossbar.cc \
  decoder.cc \
  htree2.cc \
  interconnect.cc \
  io.cc \
  iocontrollers.cc \
  logic.cc \
  main.cc \
  mat.cc \
  memoryctrl.cc \
  noc.cc \
  nuca.cc \
  parameter.cc \
  router.cc \
  subarray.cc \
  system.cc \
  technology.cc \
  uca.cc \
  wire.cc \
  xmlParser.cc

OBJS = $(patsubst %.cc,$(ODIR)/obj_$(TAG)/%.o,$(SRCS))

all: $(ODIR)/obj_$(TAG)/$(TARGET)
	cp -f $< $(ODIR)/$(TARGET)

$(ODIR)/obj_$(TAG)/$(TARGET) : $(OBJS)
	$(CXX) $^ -o $@ $(INCS) $(CXXFLAGS) $(LIBS) -pthread

$(ODIR)/obj_$(TAG)/%.o : %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-rm -f *.o $(ODIR)/$(TARGET)


