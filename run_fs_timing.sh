#! /bin/sh
#build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --cpu-type=timing --num-cpus=4 --caches --l2cache --num-l2caches=4 --l1d_size=32kB --l1i_size=32kB --l2_size=256kB --script=run_script.rcS
build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --num-cpus=4 --script=ext/parsec/2.1/run_scripts/blackscholes_4c_simsmall_chkpts.rcS
build/X86_MESI_Two_Level/gem5.opt configs/example/fs.py --cpu-type=timing --num-cpus=4 --caches --l2cache --num-l2caches=4 --l1d_size=32kB --l1i_size=32kB --l2_size=256kB --checkpoint-restore=1
