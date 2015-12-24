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
            'topology',

            'sim_ticks',
            'num_cycles',
            'speedup',

            'system_subthreshold_leakage_power',
            'system_gate_leakage_power',
            'system_runtime_dynamic_power',
            'system_runtime_dynamic_energy',
            'system_total_runtime_energy',

            'l2_subthreshold_leakage_power',
            'l2_gate_leakage_power',
            'l2_runtime_dynamic_power',
            'l2_runtime_dynamic_energy',
            'l2_total_runtime_energy',

            'l2_mpki',
            'l2_miss_rate',
            'l2_replacements',

            'numa_cache_downward_miss_rate',
            'numa_cache_upward_miss_rate',

            'system_bus_snoops'
        ]

    def dump_row(self, baseline_experiment):
        return [
            self.bench,

            self.l2_size,
            self.l2_assoc,
            self.l2_tags,

            self.num_domains,
            self.num_cpus_per_domain,
            '[' + str(self.num_domains) + ',' + str(self.num_cpus_per_domain) + ']',

            self.sim_ticks(),
            self.num_cycles(),
            baseline_experiment.num_cycles() / float(self.num_cycles()),

            self.system_subthreshold_leakage_power(),
            self.system_gate_leakage_power(),
            self.system_runtime_dynamic_power(),
            self.system_runtime_dynamic_energy(),
            self.system_total_runtime_energy(),

            self.l2_subthreshold_leakage_power(),
            self.l2_gate_leakage_power(),
            self.l2_runtime_dynamic_power(),
            self.l2_runtime_dynamic_energy(),
            self.l2_total_runtime_energy(),

            self.l2_mpki(),
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
                miss_rates.append(-1 if self.stats is None or key not in self.stats else float(self.stats[key]))
        else:
            print 'numa_cache_downward_miss_rate is meaningless in non-NUMA experiments.'
            sys.exit(-1)

        return sum(miss_rates) / float(len(miss_rates))

    def numa_cache_upward_id(self, i=None):
        key = '$.system.numa_caches_upward[' + str(i) + '].name'
        return '' if self.configs is None else ('l2' if i is None else self.configs.execute(key))

    def num_numa_caches_upward(self):
        key = 'len($.system.numa_caches_upward)'
        return -1 if self.configs is None else self.configs.execute(key)

    def numa_cache_upward_miss_rate(self):
        if self.stats is None:
            return -1

        miss_rates = []

        if self.numa():
            for i in range(self.num_numa_caches_upward()):
                key = 'system.' + self.numa_cache_upward_id(i) + '.overall_miss_rate::total'
                miss_rates.append(-1 if self.stats is None or key not in self.stats else float(self.stats[key]))
        else:
            print 'numa_cache_upward_miss_rate is meaningless in non-NUMA experiments.'
            sys.exit(-1)

        return sum(miss_rates) / float(len(miss_rates))

    def system_bus_snoops(self):
        key = 'system.system_bus.snoops'
        return -1 if self.stats is None or key not in self.stats else int(self.stats[key])