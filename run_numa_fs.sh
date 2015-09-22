#! /bin/sh
build/X86_MESI_Two_Level/gem5.debug --debug-flags=Cache configs/example/fs.py --cpu-type=timing --num-cpus=4 --caches --l2cache --script=run_script.rcS