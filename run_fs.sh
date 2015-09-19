#! /bin/sh
#build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --num-cpus=4 --num-dirs=4 --topology=Mesh --mesh-rows=1 --garnet-network=fixed --cpu-type=timing --script=run_script.rcS
build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --num-cpus=4 --num-dirs=4 --cpu-type=timing --script=run_script.rcS