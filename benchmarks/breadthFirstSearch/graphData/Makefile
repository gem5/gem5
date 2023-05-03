include parallelDefs

COMMON = dataGen.h utils.h IO.h parseCommandLine.h graph.h graphIO.h graphUtils.h parallel.h sequence.h blockRadixSort.h deterministicHash.h transpose.h
GENERATORS = rMatGraph gridGraph powerGraph randLocalGraph addWeights randDoubleVector fromAdjIdx adjToEdgeArray adjElimSelfEdges

.PHONY: all clean
all: $(GENERATORS)

$(COMMON) :
	ln -s ../../common/$@ .

%.o : %.C $(COMMON)
	$(PCC) $(PCFLAGS) -c $< -o $@

rMatGraph : rMatGraph.o
	$(PCC) $(PLFLAGS) -o $@ rMatGraph.o

gridGraph : gridGraph.o
	$(PCC) $(PLFLAGS) -o $@ gridGraph.o

powerGraph : powerGraph.o
	$(PCC) $(PLFLAGS) -o $@ powerGraph.o

randLocalGraph : randLocalGraph.o
	$(PCC) $(PLFLAGS) -o $@ randLocalGraph.o

addWeights : addWeights.o
	$(PCC) $(PLFLAGS) -o $@ addWeights.o

fromAdjIdx : fromAdjIdx.o
	$(PCC) $(PLFLAGS) -o $@ fromAdjIdx.o

randDoubleVector : randDoubleVector.o
	$(PCC) $(PLFLAGS) -o $@ randDoubleVector.o

adjToEdgeArray : adjToEdgeArray.C $(COMMON)
	$(PCC) $(PCFLAGS) $(PLFLAGS) -o $@ adjToEdgeArray.C

adjElimSelfEdges : adjElimSelfEdges.C $(COMMON)
	$(PCC) $(PCFLAGS) $(PLFLAGS) -o $@ adjElimSelfEdges.C

clean :
	rm -f *.o $(GENERATORS)
	make clean -s -C data

cleansrc :
	make -s clean
	rm -f $(COMMON)
