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


def cpu2006_multicore(bench, input_set, l2_size, l2_assoc, l2_tags, num_threads):
    work_dir = '../../results/x86_SE/' + bench + '/' + input_set + '/' + \
               l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + \
               str(num_threads) + 'c/'

    experiment = MulticoreExperiment(ExperimentType.THREE_PHASE_SE_SIMULATION, work_dir, bench, l2_size, l2_assoc, l2_tags, 0, gen_mcpat_xml_file, False)
    experiment.num_threads = num_threads
    return experiment


def parsec_multicore(bench, input_set, l2_size, l2_assoc, l2_tags, num_threads):
    work_dir = '../../results/alpha_no_checkpoints/' + bench + '/' + input_set + '/' + \
               l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + \
               str(num_threads) + 'c/'

    experiment = MulticoreExperiment(ExperimentType.FOUR_PHASE_FS_SIMULATION, work_dir, bench, l2_size, l2_assoc, l2_tags, 2, gen_mcpat_xml_file, True)
    experiment.num_threads = num_threads
    return experiment


def parsec_ccnuma(bench, input_set, l2_size, l2_assoc, l2_tags,
                  num_domains, num_cpus_per_domain,
                  numa_cache_size, numa_cache_assoc, numa_cache_tags):
    work_dir = '../../results/alpha_ccnuma_no_checkpoints/' + bench + '/' + input_set + '/' + \
               l2_size + '/' + str(l2_assoc) + 'way/' + l2_tags + '/' + \
               str(num_domains) + 'd/' + str(num_cpus_per_domain) + 'c/'
    # + numa_cache_size + '/' + str(numa_cache_assoc) + 'way/' + numa_cache_tags + '/'

    experiment = CCNUMAExperiment(ExperimentType.FOUR_PHASE_FS_SIMULATION, work_dir, bench, l2_size, l2_assoc, l2_tags, 2, gen_mcpat_xml_file, True)

    experiment.num_domains = num_domains
    experiment.num_cpus_per_domain = num_cpus_per_domain

    experiment.numa_cache_size = numa_cache_size
    experiment.numa_cache_assoc = numa_cache_assoc
    experiment.numa_cache_tags = numa_cache_tags

    return experiment


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
    fig.savefig(plot_file_name + '.jpg')

    plt.clf()
    plt.close('all')


gen_mcpat_xml_file = True
# gen_mcpat_xml_file = False

def generate_cpu2006_experiments_results():
    def generate_csv_multicore_experiments(benches, input_set):
        print 'Generating result CSV files for CPU2006 multicore experiments'

        experiment_l2_sizes = []

        for bench in benches:
            experiment_l2_sizes.append(cpu2006_multicore(bench, input_set, '8MB', 16, 'LRU', 1))

        generate_csv(MulticoreExperiment, '../../results/multicore_l2sizes_cpu2006_' + input_set + '.csv', experiment_l2_sizes,
                     lambda bench: cpu2006_multicore(bench, 'ref', '8MB', 16, 'LRU', 1))

    benches = ['429.mcf']

    input_sets = ['ref']

    for input_set in input_sets:
        generate_csv_multicore_experiments(benches, input_set)

def generate_parsec_experiments_results():
    def generate_csv_multicore_experiments(benches, input_set):
        print 'Generating result CSV files for PARSEC multicore experiments'

        experiments_l2sizes = []

        for bench in benches:
            experiments_l2sizes.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 4))
            experiments_l2sizes.append(parsec_multicore(bench, input_set, '512kB', 8, 'LRU', 4))
            experiments_l2sizes.append(parsec_multicore(bench, input_set, '1MB', 8, 'LRU', 4))
            experiments_l2sizes.append(parsec_multicore(bench, input_set, '2MB', 8, 'LRU', 4))
            experiments_l2sizes.append(parsec_multicore(bench, input_set, '4MB', 8, 'LRU', 4))
            experiments_l2sizes.append(parsec_multicore(bench, input_set, '8MB', 8, 'LRU', 4))

        generate_csv(MulticoreExperiment, '../../results/multicore_l2sizes_' + input_set + '.csv', experiments_l2sizes,
                     lambda bench: parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 4))

        experiments_l2tags = []

        for bench in benches:
            experiments_l2tags.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 4))
            experiments_l2tags.append(parsec_multicore(bench, input_set, '256kB', 8, 'IbRDP', 4))
            experiments_l2tags.append(parsec_multicore(bench, input_set, '256kB', 8, 'RRIP', 4))
            # experiments_l2tags.append(multicore(bench, input_set, '256kB', 8, 'DBRSP', 4))

        generate_csv(MulticoreExperiment, '../../results/multicore_l2tags_' + input_set + '.csv', experiments_l2tags,
                     lambda bench: parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 4))

        experiments_topologies = []

        for bench in benches:
            experiments_topologies.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 1))
            experiments_topologies.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 2))
            experiments_topologies.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 4))
            experiments_topologies.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 8))
            experiments_topologies.append(parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 16))

        generate_csv(MulticoreExperiment, '../../results/multicore_topologies_' + input_set + '.csv', experiments_topologies,
                     lambda bench: parsec_multicore(bench, input_set, '256kB', 8, 'LRU', 1))


    def generate_csv_ccnuma_experiments(benches, input_set):
        print 'Generating result CSV files for PARSEC CC-NUMA experiments'

        experiments_l2sizes = []

        for bench in benches:
            experiments_l2sizes.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2sizes.append(parsec_ccnuma(bench, input_set, '512kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2sizes.append(parsec_ccnuma(bench, input_set, '1MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2sizes.append(parsec_ccnuma(bench, input_set, '2MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2sizes.append(parsec_ccnuma(bench, input_set, '4MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2sizes.append(parsec_ccnuma(bench, input_set, '8MB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))

        generate_csv(CCNUMAExperiment, '../../results/ccnuma_l2sizes_' + input_set + '.csv', experiments_l2sizes,
                     lambda bench: parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))

        experiments_l2tags = []

        for bench in benches:
            experiments_l2tags.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2tags.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'IbRDP', 2, 2, '1kB', 8, 'LRU'))
            experiments_l2tags.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'RRIP', 2, 2, '1kB', 8, 'LRU'))
            # experiments_l2tags.append(ccnuma(bench, input_set, '256kB', 8, 'DBRSP', 2, 2, '1kB', 8, 'LRU'))

        generate_csv(CCNUMAExperiment, '../../results/ccnuma_l2tags_' + input_set + '.csv', experiments_l2tags,
                     lambda bench: parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))

        experiments_topologies = []

        for bench in benches:
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU'))
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 2, '1kB', 8, 'LRU'))
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 4, '1kB', 8, 'LRU'))
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 8, '1kB', 8, 'LRU'))
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 4, 1, '1kB', 8, 'LRU'))
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 4, 2, '1kB', 8, 'LRU'))
            experiments_topologies.append(parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 4, 4, '1kB', 8, 'LRU'))

        generate_csv(CCNUMAExperiment, '../../results/ccnuma_topologies_' + input_set + '.csv', experiments_topologies,
                     lambda bench: parsec_ccnuma(bench, input_set, '256kB', 8, 'LRU', 2, 1, '1kB', 8, 'LRU'))

    benches = ['blackscholes', 'bodytrack', 'canneal', 'dedup', 'facesim', 'ferret', 'fluidanimate', 'freqmine',
               'streamcluster', 'swaptions', 'vips', 'x264']

    # input_sets = ['simsmall', 'simmedium', 'simlarge']
    input_sets = ['simsmall']
    # input_sets = ['simmedium']
    # input_sets = ['simlarge']

    for input_set in input_sets:
        generate_csv_multicore_experiments(benches, input_set)
        generate_csv_ccnuma_experiments(benches, input_set)

        generate_plot('../../results/multicore_l2sizes_' + input_set + '.csv',
                      '../../results/multicore_l2sizes-l2_miss_rate_' + input_set + '.pdf',
                      'bench', 'l2_miss_rate', 'l2_size', 'L2 Miss Rate')

        generate_plot('../../results/multicore_l2sizes_' + input_set + '.csv',
                      '../../results/multicore_l2sizes-l2_runtime_dynamic_energy_' + input_set + '.pdf',
                      'bench', 'l2_runtime_dynamic_energy', 'l2_size', 'L2 Runtime Dynamic Energy (J)')

        generate_plot('../../results/multicore_l2tags_' + input_set + '.csv',
                      '../../results/multicore_l2tags-l2_miss_rate_' + input_set + '.pdf',
                      'bench', 'l2_miss_rate', 'l2_tags', 'L2 Miss Rate')

        generate_plot('../../results/multicore_l2tags_' + input_set + '.csv',
                      '../../results/multicore_l2tags-l2_runtime_dynamic_energy_' + input_set + '.pdf',
                      'bench', 'l2_runtime_dynamic_energy', 'l2_tags', 'L2 Runtime Dynamic Energy (J)')

        generate_plot('../../results/multicore_topologies_' + input_set + '.csv',
                      '../../results/multicore_topologies-speedup_' + input_set + '.pdf',
                      'bench', 'speedup', 'num_threads', 'Speedup')

        generate_plot('../../results/multicore_topologies_' + input_set + '.csv',
                      '../../results/multicore_topologies-system_runtime_dynamic_energy_' + input_set + '.pdf',
                      'bench', 'system_runtime_dynamic_energy', 'num_threads', 'System Runtime Dynamic Energy (J)')

        generate_plot('../../results/ccnuma_l2sizes_' + input_set + '.csv',
                      '../../results/ccnuma_l2sizes-l2_miss_rate_' + input_set + '.pdf',
                      'bench', 'l2_miss_rate', 'l2_size', 'L2 Miss Rate')

        generate_plot('../../results/ccnuma_l2sizes_' + input_set + '.csv',
                      '../../results/ccnuma_l2sizes-l2_runtime_dynamic_energy_' + input_set + '.pdf',
                      'bench', 'l2_runtime_dynamic_energy', 'l2_size', 'L2 Runtime Dynamic Energy (J)')

        generate_plot('../../results/ccnuma_l2tags_' + input_set + '.csv',
                      '../../results/ccnuma_l2tags-l2_miss_rate_' + input_set + '.pdf',
                      'bench', 'l2_miss_rate', 'l2_tags', 'L2 Miss Rate')

        generate_plot('../../results/ccnuma_l2tags_' + input_set + '.csv',
                      '../../results/ccnuma_l2tags-l2_runtime_dynamic_energy_' + input_set + '.pdf',
                      'bench', 'l2_runtime_dynamic_energy', 'l2_tags', 'L2 Runtime Dynamic Energy (J)')

        generate_plot('../../results/ccnuma_topologies_' + input_set + '.csv',
                      '../../results/ccnuma_topologies-speedup_' + input_set + '.pdf',
                      'bench', 'speedup', 'topology', 'Speedup')

        generate_plot('../../results/ccnuma_topologies_' + input_set + '.csv',
                      '../../results/ccnuma_topologies-system_runtime_dynamic_energy_' + input_set + '.pdf',
                      'bench', 'system_runtime_dynamic_energy', 'topology', 'System Runtime Dynamic Energy (J)')


if __name__ == '__main__':
    generate_cpu2006_experiments_results()
    # generate_parsec_experiments_results()
