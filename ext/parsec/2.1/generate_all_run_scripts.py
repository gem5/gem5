#!/usr/bin/python

#
# A Python script to generate all common-case runscripts for parsec 2.1 benchmarks.
#
# Copyright (C) Min Cai 2015
#

import os

benches = ['blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim',
          'ferret', 'fluidanimate', 'freqmine', 'streamcluster',
          'swaptions', 'vips', 'x264', 'rtview']

num_threads = [1, 2, 4, 8, 16, 32]

for bench in benches:
    for num_thread in num_threads:
        cmd = './writescripts.pl ' + bench + ' ' + str(num_thread)
        os.system(cmd)

        cmd_with_chpts = cmd + ' --ckpts'
        os.system(cmd_with_chpts)