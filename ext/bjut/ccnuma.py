#
# A Python script to generate CC-NUMA experiment result CSV files from the specified GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

from mcpat import *


class CCNUMAExperiment(McPATEnabledExperiment):
    num_domains = -1
    num_cpus_per_domain = -1

    @classmethod
    def dump_head_row(cls):
        return [
            'bench',
            'l2_size',
            'l2_assoc',
            'l2_tags',
            'num_domains',
            'num_cpus_per_domain',
            'num_cycles',
            'l2_miss_rate'
        ]

    def dump_row(self):
        return [
            self.bench,
            self.l2_size,
            self.l2_assoc,
            self.l2_tags,
            self.num_domains,
            self.num_cpus_per_domain,
            str(self.num_cycles()),
            str(self.l2_miss_rate())
        ]