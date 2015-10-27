#!/usr/bin/python

#
# A Python script to run all checkpoint-based experiments for PARSEC 2.1 benchmarks.
#
# Copyright (C) Min Cai 2015
#

import os

# benches = ['blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim',
#            'ferret', 'fluidanimate', 'freqmine', 'streamcluster',
#            'swaptions', 'vips', 'x264']
benches = ['blackscholes']

# num_threads = [1, 2, 4, 8, 16, 32]
num_threads = [4]

for bench in benches:
    for num_thread in num_threads:
        dir = 'results/alpha_ccnuma/' + bench +'/' + str(num_thread) + 'c/'

        os.system('mkdir -p ' + dir)

        cmd_first_run = 'build/ALPHA_MESI_Two_Level/gem5.opt -d ' + dir + ' configs/numa/ccnuma_fs.py --num-cpus=' \
                        + str(num_thread) + ' --script=ext/parsec/2.1/run_scripts/' \
                        + bench + '_' + str(num_thread) + 'c_simsmall_ckpts.rcS'
        print cmd_first_run
        os.system(cmd_first_run)

        cmd_second_run = 'build/ALPHA_MESI_Two_Level/gem5.opt -d ' + dir + ' configs/numa/ccnuma_fs.py --cpu-type=timing --num-cpus=' \
                         + str(num_thread) \
                         + ' --caches --l2cache --num-l2caches=1' \
                         + ' --l1d_size=32kB --l1i_size=32kB --l2_size=256kB --checkpoint-restore=1 --restore-with-cpu=timing'
        print cmd_second_run
        os.system(cmd_second_run)

        cmd_mcpat = './mcpat.py --config=' + dir + 'config.json --stats=' + dir + 'stats.txt --mcpat_in=' + dir + 'mcpat_in.xml --mcpat_out=' + dir + 'mcpat_out.txt'
        print cmd_mcpat
        os.system(cmd_mcpat)