#! /bin/sh
#build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --cpu-type=timing --num-cpus=4 --caches --l2cache --num-l2caches=4 --l1d_size=32kB --l1i_size=32kB --l2_size=256kB --script=run_script.rcS
#build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --num-cpus=4 --script=ext/parsec/2.1/run_scripts/blackscholes_4c_simsmall_chkpts.rcS
#build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --cpu-type=timing --num-cpus=4 --caches --l2cache --num-l2caches=4 --l1d_size=32kB --l1i_size=32kB --l2_size=256kB --checkpoint-restore=1

build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --cpu-type=timing --caches --l2cache --disk-image=BigDataBench-gem5.img --kernel=vmlinux-22-22-64 --num-cpus=4
