#
# A Python script to generate CC-NUMA experiment result CSV files from the specified GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

from mcpat import *


class CCNUMAExperiment(McPATEnabledExperiment):
    num_domains = -1
    num_cpus_per_domain = -1

    numa_cache_size = None
    numa_cache_assoc = None
    numa_cache_tags = None

    @classmethod
    def dump_head_row(cls):
        return [
            'bench',
            'l2_size',
            'l2_assoc',
            'l2_tags',
            'num_domains',
            'num_cpus_per_domain',
            'sim_ticks',
            'num_cycles',
            'l2_miss_rate',
            'l2_replacements',
            'numa_cache_downward_miss_rate',
            'numa_cache_upward_miss_rate',
            'system_bus_snoops'
        ]

    def dump_row(self):
        return [
            self.bench,
            self.l2_size,
            self.l2_assoc,
            self.l2_tags,
            self.num_domains,
            self.num_cpus_per_domain,
            self.sim_ticks(),
            self.num_cycles(),
            self.l2_miss_rate(),
            self.l2_replacements(),
            self.numa_cache_downward_miss_rate(),
            self.numa_cache_upward_miss_rate(),
            self.system_bus_snoops()
        ]

    def numa_cache_downward_id(self, i=None):
        return '' if self.configs is None else ('l2' if i is None else self.configs.execute('$.system.numa_caches_downward[' + str(i) + '].name'))

    def num_numa_caches_downward(self):
        return -1 if self.configs is None else self.configs.execute('len($.system.numa_caches_downward)')

    def numa_cache_downward_miss_rate(self):
        if self.stats is None:
            return -1

        miss_rates = []

        if self.numa():
            for i in range(self.num_numa_caches_downward()):
                key = 'system.' + self.numa_cache_downward_id(i) + '.overall_miss_rate::total'
                miss_rates.append(float(self.stats[key] if key in self.stats else 0))
        else:
            print 'numa_cache_downward_miss_rate is meaningless in non-NUMA experiments.'
            sys.exit(-1)

        return sum(miss_rates) / float(len(miss_rates))

    def numa_cache_upward_id(self, i=None):
        return '' if self.configs is None else ('l2' if i is None else self.configs.execute('$.system.numa_caches_upward[' + str(i) + '].name'))

    def num_numa_caches_upward(self):
        return -1 if self.configs is None else self.configs.execute('len($.system.numa_caches_upward)')

    def numa_cache_upward_miss_rate(self):
        if self.stats is None:
            return -1

        miss_rates = []

        if self.numa():
            for i in range(self.num_numa_caches_upward()):
                key = 'system.' + self.numa_cache_upward_id(i) + '.overall_miss_rate::total'
                miss_rates.append(float(self.stats[key] if key in self.stats else 0))
        else:
            print 'numa_cache_upward_miss_rate is meaningless in non-NUMA experiments.'
            sys.exit(-1)

        return sum(miss_rates) / float(len(miss_rates))

    def system_bus_snoops(self):
        return -1 if self.stats is None else str(int(self.stats['system.system_bus.snoops']))