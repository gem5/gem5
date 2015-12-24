#!/usr/bin/python

#
# A Python script to run all instruction count based experiments for CPU2006 benchmarks.
#
# Copyright (C) Min Cai 2015
#

import os


def run(bench, input_set, l2_size, l2_assoc, l2_tags, num_threads):
    dir = 'results/x86_SE/' + bench + '/' +  input_set + '/' + l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + str(num_threads) + 'c/'

    os.system('rm -fr ' + dir)
    os.system('mkdir -p ' + dir)

    cmd = 'build/X86_MESI_Two_Level/gem5.opt -d ' + dir + ' configs/example/se.py --num-cpus=' + str(num_threads)\
         + ' --cpu-type=detailed  --caches --l2cache --num-l2caches=1' \
         + ' --l1i_size=32kB  --l1d_size=32kB --l1i_assoc=4 --l2_size=' + l2_size + ' --l2_assoc=' + str(l2_assoc) + ' --l2_tags=' + l2_tags \
         + ' --standard-switch=100000000 --warmup-insts=100000000 -I 200000000 --mem-size=2048MB' \
         + ' --bench=' + bench
    print cmd
    os.system(cmd)

def run_experiments(bench):
    run(bench, 'ref', '8MB', 16, 'LRU', 1)

run_experiments('429.mcf')
