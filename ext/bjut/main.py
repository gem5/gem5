#!/usr/bin/python

#
# A Python script to generate experiment result CSV files from the specified GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

from multicore import *
from ccnuma import *
import csv
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


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

    generate_csv(MulticoreExperiment, '../../multicore_l2sizes.csv', experiments_l2sizes,
                 lambda bench: multicore(bench, '256kB', 8, 'LRU', 4))

    experiments_l2tags = []

    for bench in benches:
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'LRU', 4))
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'IbRDP', 4))
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'RRIP', 4))
        experiments_l2tags.append(multicore(bench, '256kB', 8, 'DBRSP', 4))

    generate_csv(MulticoreExperiment, '../../multicore_l2tags.csv', experiments_l2tags,
                 lambda bench: multicore(bench, '256kB', 8, 'LRU', 4))

    experiments_topologies = []

    for bench in benches:
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 1))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 2))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 4))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 8))
        experiments_topologies.append(multicore(bench, '256kB', 8, 'LRU', 16))

    generate_csv(MulticoreExperiment, '../../multicore_topologies.csv', experiments_topologies,
                 lambda bench: multicore(bench, '256kB', 8, 'LRU', 1))


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

    generate_csv(CCNUMAExperiment, '../../ccnuma_l2sizes.csv', experiments_l2sizes,
                 lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))

    experiments_l2tags = []

    for bench in benches:
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'IbRDP', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'RRIP', 2, 2, '1kB', 8, 'LRU'))
        experiments_l2tags.append(ccnuma(bench, '256kB', 8, 'DBRSP', 2, 2, '1kB', 8, 'LRU'))

    generate_csv(CCNUMAExperiment, '../../ccnuma_l2tags.csv', experiments_l2tags,
                 lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))

    experiments_topologies = []

    for bench in benches:
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 4, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 2, 8, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 4, 1, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 4, 2, '1kB', 8, 'LRU'))
        experiments_topologies.append(ccnuma(bench, '256kB', 8, 'LRU', 4, 4, '1kB', 8, 'LRU'))

    generate_csv(CCNUMAExperiment, '../../ccnuma_topologies.csv', experiments_topologies,
                 lambda bench: ccnuma(bench, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU'))


def generate_csv(experiment_cls, csv_file_name, experiments, baseline_experiment_func):
    with open(csv_file_name, 'w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)

        writer.writerow(experiment_cls.dump_head_row())

        for experiment in experiments:
            print 'Generating CSV row for experiment under "' + experiment.dir + '"'
            writer.writerow(experiment.dump_row(baseline_experiment_func(experiment.bench)))


def generate_plot(csv_file_name, plot_file_name, x, y, hue, y_title):
    print 'Generating plot "' + plot_file_name + '"'

    sns.set(font_scale=1.5)

    sns.set_style("white", {"legend.frameon": True})

    df = pd.read_csv(csv_file_name)

    ax = sns.barplot(data=df, x=x, y=y, hue=hue, palette=sns.color_palette("Paired"))
    ax.set_xlabel('')
    ax.set_ylabel(y_title)

    labels = ax.get_xticklabels()
    ax.set_xticklabels(labels, rotation=45)

    legend = ax.legend()
    legend.set_label('')

    fig = ax.get_figure()
    fig.tight_layout()
    fig.savefig(plot_file_name)

    plt.clf()
    plt.close('all')


# gen_mcpat_xml_file = True
gen_mcpat_xml_file = False

if __name__ == '__main__':
    benches = ['blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim', 'ferret', 'fluidanimate', 'freqmine',
          'streamcluster', 'swaptions', 'vips', 'x264']

    generate_csv_multicore_experiments(benches)
    generate_csv_ccnuma_experiments(benches)

    generate_plot('../../multicore_l2sizes.csv', '../../multicore_l2sizes-l2_miss_rate.pdf',
                  'bench', 'l2_miss_rate', 'l2_size', 'L2 Miss Rate')

    generate_plot('../../multicore_l2sizes.csv', '../../multicore_l2sizes-l2_runtime_dynamic_energy.pdf',
                  'bench', 'l2_runtime_dynamic_energy', 'l2_size', 'L2 Runtime Dynamic Energy (J)')

    generate_plot('../../multicore_l2tags.csv', '../../multicore_l2tags-l2_miss_rate.pdf',
                  'bench', 'l2_miss_rate', 'l2_tags', 'L2 Miss Rate')

    generate_plot('../../multicore_l2tags.csv', '../../multicore_l2tags-l2_runtime_dynamic_energy.pdf',
                  'bench', 'l2_runtime_dynamic_energy', 'l2_tags', 'L2 Runtime Dynamic Energy (J)')

    generate_plot('../../multicore_topologies.csv', '../../multicore_topologies-speedup.pdf',
                  'bench', 'speedup', 'num_threads', 'Speedup')

    generate_plot('../../multicore_topologies.csv', '../../multicore_topologies-system_runtime_dynamic_energy.pdf',
                  'bench', 'system_runtime_dynamic_energy', 'num_threads', 'System Runtime Dynamic Energy (J)')

    generate_plot('../../ccnuma_l2sizes.csv', '../../ccnuma_l2sizes-l2_miss_rate.pdf',
                  'bench', 'l2_miss_rate', 'l2_size', 'L2 Miss Rate')

    generate_plot('../../ccnuma_l2sizes.csv', '../../ccnuma_l2sizes-l2_runtime_dynamic_energy.pdf',
                  'bench', 'l2_runtime_dynamic_energy', 'l2_size', 'L2 Runtime Dynamic Energy (J)')

    generate_plot('../../ccnuma_l2tags.csv', '../../ccnuma_l2tags-l2_miss_rate.pdf',
                  'bench', 'l2_miss_rate', 'l2_tags', 'L2 Miss Rate')

    generate_plot('../../ccnuma_l2tags.csv', '../../ccnuma_l2tags-l2_runtime_dynamic_energy.pdf',
                  'bench', 'l2_runtime_dynamic_energy', 'l2_tags', 'L2 Runtime Dynamic Energy (J)')

    generate_plot('../../ccnuma_topologies.csv', '../../ccnuma_topologies-speedup.pdf',
                  'bench', 'speedup', 'topology', 'Speedup')

    generate_plot('../../ccnuma_topologies.csv', '../../ccnuma_topologies-system_runtime_dynamic_energy.pdf',
                  'bench', 'system_runtime_dynamic_energy', 'topology', 'System Runtime Dynamic Energy (J)')
