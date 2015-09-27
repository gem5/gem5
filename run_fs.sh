#! /bin/sh
build/X86_MESI_Two_Level/gem5.fast configs/example/fs.py --cpu-type=detailed --num-cpus=4 --num-dirs=4 --caches --ruby --num-l2caches=4 --l1d_size=8kB --l1i_size=8kB --l2_size=128kB --topology=Mesh --mesh-rows=2 --garnet-network=fixed --script=run_script.rcS
