#!/usr/bin/env bash
#build/X86/gem5.opt configs/example/se.py -n 4 --cpu-type=detailed --caches -c tests/test-progs/hello/bin/x86/linux/hello
#build/X86/gem5.opt configs/example/se.py -n 4 --cpu-type=detailed --caches -c /home/itecgo/Archimulator/benchmarks/Olden_Custom1/mst/ht/mst
#~ build/X86_MESI_CMP_directory/gem5.opt configs/example/se.py --num-cpus=16 --cpu-type=detailed --caches --ruby --num-dirs=16 --topology=Mesh --mesh-rows=4  -c tests/test-progs/hello/bin/x86/linux/hello
#~ build/X86_MESI_CMP_directory/gem5.opt configs/example/se.py --num-cpus=16 --cpu-type=detailed --caches --ruby  -c tests/test-progs/hello/bin/x86/linux/hello
build/X86_MESI_CMP_directory/gem5.opt configs/example/se.py --num-cpus=16 --cpu-type=detailed --caches --ruby --l1i_size=32kB --l1d_size=32kB --l2_size=8MB --num-l2caches=16 --topology=Mesh --garnet-network=fixed --mesh-rows=4 --num-dir=16  -c tests/test-progs/hello/bin/x86/linux/hello

#build/ARM/gem5.opt configs/example/se.py -n 4 --cpu-type=detailed --caches -c tests/test-progs/hello/bin/arm/linux/hello
#build/ARM/gem5.opt configs/example/se.py -n 4 --cpu-type=detailed --caches --ruby -c /home/itecgo/Archimulator/benchmarks/Olden_Custom1/mst/baseline/mst.arm -o 100
#build/ARM/gem5.opt configs/example/se.py -n 4 -c /home/itecgo/Archimulator/benchmarks/Olden_Custom1/mst/baseline/mst.arm -o 100
