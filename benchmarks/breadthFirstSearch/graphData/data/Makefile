GENERATORS = ../gridGraph ../rMatGraph ../randLocalGraph ../addWeights ../randDoubleVector

$(GENERATORS) :
	cd ..; make -s $(subst ../,,$@)

clean :
	rm -f *0*

2Dgrid_E_% : ../gridGraph
	../gridGraph -d 2 $(subst 2Dgrid_E_,,$@) $@

3Dgrid_E_% : ../gridGraph
	../gridGraph -d 3 $(subst 3Dgrid_E_,,$@) $@

rMatGraph_E_5_% : ../rMatGraph
	../rMatGraph  -a .5 -b .1 -m `expr 5 \* $(subst rMatGraph_E_5_,,$@)` $(subst rMatGraph_E_5_,,$@) $@

randLocalGraph_E_5_% : ../randLocalGraph
	../randLocalGraph -d 3 -m `expr 5 \* $(subst randLocalGraph_E_5_,,$@)` $(subst randLocalGraph_E_5_,,$@) $@

2Dgrid_WE_% : ../addWeights
	make -s $(subst WE,E,$@)
	../addWeights $(subst WE,E,$@) $@

3Dgrid_WE_% : ../addWeights
	make -s $(subst WE,E,$@)
	../addWeights $(subst WE,E,$@) $@

rMatGraph_WE_% : ../addWeights
	make -s $(subst WE,E,$@)
	../addWeights $(subst WE,E,$@) $@

randLocalGraph_WE_5_% : ../addWeights
	make -s $(subst WE,E,$@)
	../addWeights $(subst WE,E,$@) $@

2Dgrid_J_% : ../gridGraph
	../gridGraph -j -d 2 -o $(subst 2Dgrid_J_,,$@) $@

3Dgrid_J_% : ../gridGraph
	../gridGraph -j -d 3  $(subst 3Dgrid_J_,,$@) $@

rMatGraph_J_5_% : ../rMatGraph
	../rMatGraph -j -a .5 -b .1 -m `expr 5 \* $(subst rMatGraph_J_5_,,$@)` $(subst rMatGraph_J_5_,,$@) $@

randLocalGraph_J_5_% : ../randLocalGraph
	../randLocalGraph -j -d 3 -m `expr 5 \* $(subst randLocalGraph_J_5_,,$@)` $(subst randLocalGraph_J_5_,,$@) $@

rMatGraph_J_500_% : ../rMatGraph
	../rMatGraph -j -a .5 -b .1 -m `expr 500 \* $(subst rMatGraph_J_500_,,$@)` $(subst rMatGraph_J_500_,,$@) $@

randLocalGraph_J_500_% : ../randLocalGraph
	../randLocalGraph -j -d 3 -m `expr 500 \* $(subst randLocalGraph_J_500_,,$@)` $(subst randLocalGraph_J_500_,,$@) $@

rMatGraph_E_500_% : ../rMatGraph
	../rMatGraph -a .5 -b .1 -m `expr 500 \* $(subst rMatGraph_E_500_,,$@)` $(subst rMatGraph_E_500_,,$@) $@

randLocalGraph_E_500_% : ../randLocalGraph
	../randLocalGraph -d 3 -m `expr 500 \* $(subst randLocalGraph_E_500_,,$@)` $(subst randLocalGraph_E_500_,,$@) $@

randDoubleVector_% : ../randDoubleVector
	../randDoubleVector $(subst randDoubleVector_,,$@) $@
