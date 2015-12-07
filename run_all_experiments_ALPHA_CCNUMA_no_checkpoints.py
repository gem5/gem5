#!/usr/bin/python

#
# A Python script to run all CC-NUMA multi-core experiments for PARSEC 2.1 benchmarks.
#
# Copyright (C) Min Cai 2015
#

import os
import multiprocessing as mp


def run(bench, input_set, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain, numa_cache_size,
        numa_cache_assoc, numa_cache_tags):
    num_threads = num_domains * num_cpus_per_domain

    dir = 'results/alpha_ccnuma_no_checkpoints/' + bench + '/' + input_set + '/' + \
          l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + \
          str(num_domains) + 'd/' + str(num_cpus_per_domain) + 'c/'
          # + numa_cache_size + '/' + str(numa_cache_assoc) + 'way/' + numa_cache_tags + '/'

    os.system('rm -fr ' + dir)
    os.system('mkdir -p ' + dir)

    cmd_run = 'build/ALPHA_MESI_Two_Level/gem5.opt -d ' + dir + ' configs/numa/ccnuma_fs.py --two-phase --cpu-type=timing --num-cpus=' \
              + str(num_threads) + ' --script=ext/parsec/2.1/run_scripts/' \
              + bench + '_' + str(num_threads) + 'c_' + input_set + '.rcS' \
              + ' --caches --l2cache --num-l2caches=1' \
              + ' --l1d_size=32kB --l1i_size=32kB --l2_size=' + l2_size + ' --l2_assoc=' + str(l2_assoc) + ' --l2_tags=' + l2_tags \
              + ' --num_domains=' + str(num_domains) + ' --num_cpus_per_domain=' + str(num_cpus_per_domain) \
              + ' --numa_cache_size=' + numa_cache_size + ' --numa_cache_assoc=' + str(numa_cache_assoc) + ' --numa_cache_tags=' + numa_cache_tags \
              + ' --mem_size_per_domain=256MB'
    print cmd_run
    os.system(cmd_run)


def run_as_task(task):
    bench, input_set, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain, numa_cache_size, numa_cache_assoc, numa_cache_tags = task
    run(bench, input_set, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain, numa_cache_size, numa_cache_assoc, numa_cache_tags)

tasks = []


def run_experiments():
    num_processes = mp.cpu_count()
    pool = mp.Pool(num_processes)
    pool.map(run_as_task, tasks)

    pool.close()
    pool.join()


def add_task(bench, input_set, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain, numa_cache_size, numa_cache_assoc, numa_cache_tags):
    task = (bench, input_set, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain, numa_cache_size, numa_cache_assoc, numa_cache_tags)
    tasks.append(task)

# input_sets = ['simsmall', 'simmedium', 'simlarge']
# input_sets = ['simsmall']
# input_sets = ['simmedium']
input_sets = ['simlarge']


def add_tasks(bench, input_set):
    add_task(bench, input_set, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')

    add_task(bench, input_set, '512kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '1MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '2MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '4MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '8MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')

    add_task(bench, input_set, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'LRU', 2, 4, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'LRU', 2, 8, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'LRU', 4, 1, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'LRU', 4, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'LRU', 4, 4, '1kB', 8, 'LRU')

    add_task(bench, input_set, '256kB', 8, 'IbRDP', 2, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'RRIP', 2, 2, '1kB', 8, 'LRU')
    add_task(bench, input_set, '256kB', 8, 'DBRSP', 2, 2, '1kB', 8, 'LRU')

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