#! /bin/sh
build/X86_MESI_Two_Level/gem5.fast configs/mcpat/fs_mcpat.py --cpu-type=timing --num-cpus=4 --caches --l2cache --num-l2caches=4 --l1d_size=32kB --l1i_size=32kB --l2_size=256kB --script=run_script.rcS
