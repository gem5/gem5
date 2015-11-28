#!/usr/bin/python

#
# A Python script to generate experiment result CSV files from the specified GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

from multicore import *
from ccnuma import *
import csv
import pygal


def multicore(bench, l2_size, l2_assoc, l2_tags, num_threads):
    work_dir = '../../results/alpha_no_checkpoints/' + bench + '/' + \
               l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + \
               str(num_threads) + 'c/'

    experiment = MulticoreExperiment(work_dir, bench, l2_size, l2_assoc, l2_tags, gen_mcpat_xml_file=gen_mcpat_xml_file)
    experiment.num_threads = num_threads
    return experiment


def ccnuma(bench, l2_size, l2_assoc, l2_tags,
           num_domains, num_cpus_per_domain,
           numa_cache_size, numa_cache_assoc, numa_cache_tags):
    work_dir = '../../results/alpha_ccnuma_no_checkpoints/' + bench + '/' + \
               l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + \
               str(num_domains) + 'd/' + str(num_cpus_per_domain) + 'c/'
    # + numa_cache_size + '/' + str(numa_cache_assoc) + 'way/' + numa_cache_tags + '/'

    experiment = CCNUMAExperiment(work_dir, bench, l2_size, l2_assoc, l2_tags, gen_mcpat_xml_file=gen_mcpat_xml_file)

    experiment.num_domains = num_domains
    experiment.num_cpus_per_domain = num_cpus_per_domain

    experiment.numa_cache_size = numa_cache_size
    experiment.numa_cache_assoc = numa_cache_assoc
    experiment.numa_cache_tags = numa_cache_tags

    return experiment


def generate_csv_multicore_experiments(benches):
    print 'Generating result CSV files for multicore experiments'

    experiments_l2sizes = []

    for bench in benches:
        experiments_l2sizes.append(multicore(bench, '256kB', 8, 'LRU', 4))
        experiments_l2sizes.append(multicore(bench, '512kB', 8, 'LRU', 4))
        experiments_l2sizes.append(multicore(bench, '1MB', 8, 'LRU', 4))
        experiments_l2sizes.append(multicore(bench, '2MB', 8, 'LRU', 4))
        experiments_l2sizes.append(multicore(bench, '4MB', 8, 'LRU', 4))
        experiments_l2sizes.append(multicore(bench, '8MB', 8, 'LRU', 4))

    generate_csv(MulticoreExperiment, '../../multicore_l2sizes.csv', experiments_l2sizes)

    experiments_l2tags = []

    for bench in benches:
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'LRU', 4))
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'IbRDP', 4))
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'RRIP', 4))
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'DBRSP', 4))

    generate_csv(MulticoreExperiment, '../../multicore_l2tags.csv', experiments_l2tags)

    experiments_topologies = []

    for bench in benches:
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 1))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 2))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 4))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 8))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 16))

    generate_csv(MulticoreExperiment, '../../multicore_topologies.csv', experiments_topologies)


def generate_csv_ccnuma_experiments(benches):
    print 'Generating result CSV files for CC-NUMA experiments'

    experiments_l2sizes = []

    for bench in benches:
        experiments_l2sizes.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2sizes.append(ccnuma(bench, '512kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2sizes.append(ccnuma(bench, '1MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2sizes.append(ccnuma(bench, '2MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2sizes.append(ccnuma(bench, '4MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2sizes.append(ccnuma(bench, '8MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))

    generate_csv(CCNUMAExperiment, '../../ccnuma_l2sizes.csv', experiments_l2sizes)

    experiments_l2tags = []

    for bench in benches:
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'IbRDP', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'RRIP', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'DBRSP', 2, 2, '1kB', 8, 'LRU'))

    generate_csv(CCNUMAExperiment, '../../ccnuma_l2tags.csv', experiments_l2tags)

    experiments_topologies = []

    for bench in benches:
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 4, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 8, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 4, 1, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 4, 2, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 4, 4, '1kB', 8, 'LRU'))

    generate_csv(CCNUMAExperiment, '../../ccnuma_topologies.csv', experiments_topologies)


def generate_csv(experiment_cls, csv_file_name, experiments):
    if not gen_csv_file:
        return

    with open(csv_file_name, 'w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)

        writer.writerow(experiment_cls.dump_head_row())

        for experiment in experiments:
            print 'Generating CSV row for experiment under "' + experiment.dir + '"'
            writer.writerow(experiment.dump_row())


def generate_plot(plot_file_name, benches, experiment_funcs, y_title, y_func):
    if not gen_plot_file:
        return

    bar_chart = pygal.Bar()
    bar_chart.y_title = y_title
    bar_chart.x_labels = map(str, benches)

    for x_label, experiment_func in experiment_funcs:
        values = []
        for bench in benches:
            experiment = experiment_func(bench)
            values.append(y_func(experiment))
        bar_chart.add(x_label, values)

    print 'Generating plot "' + plot_file_name + '"'
    bar_chart.render_to_png(plot_file_name, dpi=300)


# gen_mcpat_xml_file = True
gen_mcpat_xml_file = False

# gen_csv_file = True
gen_csv_file = False

gen_plot_file = True
# gen_plot_file = False

if __name__ == '__main__':
    generate_csv_multicore_experiments(
        [
            'blackscholes',
            'bodytrack',
            'canneal',
            'dedup',
            'facesim',
            'ferret',
            'fluidanimate',
            'freqmine',
            'streamcluster',
            'swaptions',
            'vips',
            'x264'
        ]
    )

    generate_csv_ccnuma_experiments(
        [
            'blackscholes',
            'bodytrack',
            'canneal',
            'dedup',
            'facesim',
            'ferret',
            'fluidanimate',
            'freqmine',
            'streamcluster',
            'swaptions',
            'vips',
            'x264'
        ]
    )

    benches = ['blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim', 'ferret', 'fluidanimate', 'freqmine',
          'streamcluster', 'swaptions', 'vips', 'x264']

    generate_plot('../../multicore_l2sizes.png', benches, [
        ('256kB', lambda bench: multicore(bench, '256kB', 8, 'LRU', 4)),
        ('512kB', lambda bench: multicore(bench, '512kB', 8, 'LRU', 4)),
        ('1MB', lambda bench: multicore(bench, '1MB', 8, 'LRU', 4)),
        ('2MB', lambda bench: multicore(bench, '2MB', 8, 'LRU', 4)),
        ('4MB', lambda bench: multicore(bench, '4MB', 8, 'LRU', 4)),
        ('8MB', lambda bench: multicore(bench, '8MB', 8, 'LRU', 4))
    ], 'L2 Miss Rate', lambda experiment: experiment.l2_miss_rate())

    generate_plot('../../multicore_l2tags.png', benches, [
        ('LRU', lambda bench: multicore(bench, '256kB', 8, 'LRU', 4)),
        ('IbRDP', lambda bench: multicore(bench, '256kB', 8, 'IbRDP', 4)),
        ('RRIP', lambda bench: multicore(bench, '256kB', 8, 'RRIP', 4)),
        ('DBRSP', lambda bench: multicore(bench, '256kB', 8, 'DBRSP', 4))
    ], 'L2 Miss Rate', lambda experiment: experiment.l2_miss_rate())

    generate_plot('../../multicore_topologies.png', benches, [
        ('1', lambda bench: multicore(bench, '256kB', 8, 'LRU', 1)),
        ('2', lambda bench: multicore(bench, '256kB', 8, 'LRU', 2)),
        ('4', lambda bench: multicore(bench, '256kB', 8, 'LRU', 4)),
        ('8', lambda bench: multicore(bench, '256kB', 8, 'LRU', 8)),
        ('16', lambda bench: multicore(bench, '256kB', 8, 'LRU', 16)),
    ], '# Cycles', lambda experiment: experiment.num_cycles())

    generate_plot('../../ccnuma_l2sizes.png', benches, [
        ('256kB', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('512kB', lambda bench: ccnuma(bench, '512kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('1MB', lambda bench: ccnuma(bench, '1MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('2MB', lambda bench: ccnuma(bench, '2MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('4MB', lambda bench: ccnuma(bench, '4MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('8MB', lambda bench: ccnuma(bench, '8MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
    ], 'L2 Miss Rate', lambda experiment: experiment.l2_miss_rate())

    generate_plot('../../ccnuma_l2tags.png', benches, [
        ('LRU', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('IbRDP', lambda bench: ccnuma(bench, '256kB', 8, 'IbRDP', 2, 2, '1kB', 8, 'LRU')),
        ('RRIP', lambda bench: ccnuma(bench, '256kB', 8, 'RRIP', 2, 2, '1kB', 8, 'LRU')),
        ('DBRSP', lambda bench: ccnuma(bench, '256kB', 8, 'DBRSP', 2, 2, '1kB', 8, 'LRU'))
    ], 'L2 Miss Rate', lambda experiment: experiment.l2_miss_rate())

    generate_plot('../../ccnuma_topologies.png', benches, [
        ('[2, 1]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU')),
        ('[2, 2]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU')),
        ('[2, 4]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 4, '1kB', 8, 'LRU')),
        ('[2, 8]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 8, '1kB', 8, 'LRU')),
        ('[4, 1]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 4, 1, '1kB', 8, 'LRU')),
        ('[4, 2]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 4, 2, '1kB', 8, 'LRU')),
        ('[4, 4]', lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 4, 4, '1kB', 8, 'LRU')),
    ], '# Cycles', lambda experiment: experiment.num_cycles())
