#!/usr/bin/python

#
# A Python script to run all multi-core experiments for PARSEC 2.1 benchmarks.
#
# Copyright (C) Min Cai 2015
#

import os
import multiprocessing as mp


def run(bench, input_set, l2_size, l2_assoc, l2_tags, num_threads):
    dir = 'results/alpha_no_checkpoints/' + bench + '/' + input_set + '/' + l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + str(num_threads) + 'c/'

    os.system('rm -fr ' + dir)
    os.system('mkdir -p ' + dir)

    cmd_run = 'build/ALPHA_MESI_Two_Level/gem5.opt -d ' + dir + ' configs/example/fs.py --two-phase --cpu-type=timing --num-cpus=' \
              + str(num_threads) + ' --script=ext/parsec/2.1/run_scripts/' \
              + bench + '_' + str(num_threads) + 'c_' + input_set + '.rcS' \
              + ' --caches --l2cache --num-l2caches=1' \
              + ' --l1d_size=32kB --l1i_size=32kB --l2_size=' + l2_size + ' --l2_assoc=' + str(l2_assoc) + ' --l2_tags=' + l2_tags
    print cmd_run
    os.system(cmd_run)


def run_as_task(task):
    bench, input_set, l2_size, l2_assoc, l2_tags, num_threads = task
    run(bench, input_set, l2_size, l2_assoc, l2_tags, num_threads)

tasks = []


def run_experiments():
    num_processes = mp.cpu_count()
    pool = mp.Pool(num_processes)
    pool.map(run_as_task, tasks)

    pool.close()
    pool.join()


def add_task(bench, input_set, l2_size, l2_assoc, l2_tags, num_threads):
    task = bench, input_set, l2_size, l2_assoc, l2_tags, num_threads
    tasks.append(task)

# input_sets = ['simsmall', 'simmedium', 'simlarge']
# input_sets = ['simsmall']
# input_sets = ['simmedium']
input_sets = ['simlarge']


def add_tasks(bench, input_set):
    add_task(bench, input_set, '256kB', 8, 'LRU', 4)
    add_task(bench, input_set, '512kB', 8, 'LRU', 4)
    add_task(bench, input_set, '1MB', 8, 'LRU', 4)
    add_task(bench, input_set, '2MB', 8, 'LRU', 4)
    add_task(bench, input_set, '4MB', 8, 'LRU', 4)
    add_task(bench, input_set, '8MB', 8, 'LRU', 4)

    add_task(bench, input_set, '256kB', 8, 'LRU', 1)
    add_task(bench, input_set, '256kB', 8, 'LRU', 2)
    add_task(bench, input_set, '256kB', 8, 'LRU', 8)
    add_task(bench, input_set, '256kB', 8, 'LRU', 16)

    add_task(bench, input_set, '256kB', 8, 'IbRDP', 4)
    add_task(bench, input_set, '256kB', 8, 'RRIP', 4)
    add_task(bench, input_set, '256kB', 8, 'DBRSP', 4)

for input_set in input_sets:
    add_tasks('blackscholes', input_set)
    add_tasks('bodytrack', input_set)
    add_tasks('canneal', input_set)
    add_tasks('dedup', input_set)
    add_tasks('facesim', input_set)
    add_tasks('ferret', input_set)
    add_tasks('fluidanimate', input_set)
    add_tasks('freqmine', input_set)
    add_tasks('streamcluster', input_set)
    add_tasks('swaptions', input_set)
    add_tasks('vips', input_set)
    add_tasks('x264', input_set)

run_experiments()
