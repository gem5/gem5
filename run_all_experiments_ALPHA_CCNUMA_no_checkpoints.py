#!/usr/bin/python

#
# A Python script to run all checkpoint-based experiments for PARSEC 2.1 benchmarks.
#
# Copyright (C) Min Cai 2015
#

import os


def run(bench, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain):
    num_threads = num_domains * num_cpus_per_domain

    dir = 'results/alpha_ccnuma_no_checkpoints/' + bench + '/' + l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + str(
        num_domains) + 'd/' + str(num_cpus_per_domain) + 'c/'

    os.system('rm -fr ' + dir)
    os.system('mkdir -p ' + dir)

    cmd_run = 'build/ALPHA_MESI_Two_Level/gem5.opt -d ' + dir + ' configs/numa/ccnuma_fs.py --two-phase --cpu-type=timing --num-cpus=' \
              + str(num_threads) + ' --script=ext/parsec/2.1/run_scripts/' \
              + bench + '_' + str(num_threads) + 'c_simsmall.rcS' \
              + ' --caches --l2cache --num-l2caches=1' \
              + ' --l1d_size=32kB --l1i_size=32kB --l2_size=' + l2_size + ' --l2_assoc=' + str(l2_assoc) + ' --l2_tags=' + l2_tags \
              + ' --num_domains=' + str(num_domains) + ' --num_cpus_per_domain=' + str(num_cpus_per_domain) \
              + ' --mem_size_per_domain=256MB'
    print cmd_run
    os.system(cmd_run)

    cmd_mcpat = './mcpat.py --config=' + dir + 'config.json --stats=' + dir + 'stats.txt --mcpat_in=' + dir + 'mcpat_in.xml --mcpat_out=' + dir + 'mcpat_out.txt'
    print cmd_mcpat
    os.system(cmd_mcpat)


# benches = ['blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim',
#            'ferret', 'fluidanimate', 'freqmine', 'streamcluster',
#            'swaptions', 'vips', 'x264']
benches = ['blackscholes']

# num_domains_range = [2, 4]
num_domains_range = [2]

# num_cpus_per_domain_range = [1, 2, 4]
num_cpus_per_domain_range = [2]

# l2_size_range = ['256kB', '512kB', '1MB', '2MB']
l2_size_range = ['256kB']

# l2_assoc_range = [4, 8, 16, 32]
l2_assoc_range = [8]

# l2_tags_range = ['LRU', 'Random']
l2_tags_range = ['LRU']
# l2_tags_range = ['Random']

# for bench in benches:
#     for l2_size in l2_size_range:
#         for l2_assoc in l2_assoc_range:
#             for l2_tags in l2_tags_range:
#                 for num_domains in num_domains_range:
#                     for num_cpus_per_domain in num_cpus_per_domain_range:
#                         run(bench, l2_size, l2_assoc, l2_tags, num_domains, num_cpus_per_domain)


def run_experiments(bench):
    run(bench, '256kB', 8, 'LRU', 2, 2)
    run(bench, '512kB', 8, 'LRU', 2, 2)
    run(bench, '1MB', 8, 'LRU', 2, 2)
    run(bench, '2MB', 8, 'LRU', 2, 2)
    run(bench, '4MB', 8, 'LRU', 2, 2)
    run(bench, '8MB', 8, 'LRU', 2, 2)

    run(bench, '256kB', 8, 'LRU', 2, 1)
    run(bench, '256kB', 8, 'LRU', 2, 2)
    run(bench, '256kB', 8, 'LRU', 4, 1)
    run(bench, '256kB', 8, 'LRU', 4, 2)
    run(bench, '256kB', 8, 'LRU', 4, 4)

run_experiments('blackscholes')
# run_experiments('bodytrack')
# run_experiments('canneal')
# run_experiments('dedup')
# run_experiments('facesim')
# run_experiments('ferret')
# run_experiments('fluidanimate')
# run_experiments('freqmine')
# run_experiments('streamcluster')
# run_experiments('swaptions')
# run_experiments('vips')
# run_experiments('x264')
