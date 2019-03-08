# !/usr/bin/python3
#
# Test hello with modification
# Author: Yicheng Wang
# Date: 03/07/2018

# build/X86/gem5.opt --debug-flags=DRAM configs/example/se.py
# -c tests/test-progs/hello/bin/x86/linux/hello
# --cpu-type=DerivO3CPU --caches
# --mem-type=DDR4_2400_16x4 > ~/Desktop/run.log

import subprocess
import os
import sys
from sys import argv



# Run simulation on selected benchmark
def main(debugFlag):
    # Change current working directory to the binary folder
    os.chdir('/home/ywang/Research/gem5')

    args = []
    # Gem5 configurations
    gem5 = '/home/ywang/Research/gem5/build/X86/gem5.opt'
    debug = '--debug-flags=' + debugFlag
    gem5_output = '--outdir=/home/ywang/Desktop/gem5_output'
    script = '/home/ywang/Research/gem5/configs/example/se.py'
    args.extend([gem5,debug,gem5_output,script])

    # Binary
    program = '--cmd=tests/test-progs/hello/bin/x86/linux/hello'
    args.extend([program])

    # CPU configurations
    cpu_type = '--cpu-type=DerivO3CPU'
    args.append(cpu_type)

    # Memory configurations
    mem_type = '--mem-type=DDR4_2400_16x4'
    mem_channles = '--mem-channels=1'
    mem_ranks = '--mem-ranks=1'
    mem_size = '--mem-size=16GB'
    caches = '--caches'
    args.extend([mem_type,mem_channles,mem_ranks,mem_size,caches])
    subprocess.Popen(args)
    return 0


if __name__ == "__main__":
    script, debugFlag = argv
    main(debugFlag)
