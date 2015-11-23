#
# A Python script to generate multicore experiment result CSV files from the specified GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

from mcpat import *


class MulticoreExperiment(McPATEnabledExperiment):
    num_threads = -1

    @classmethod
    def dump_head_row(cls):
        return [
            'bench',
            'l2_size',
            'l2_assoc',
            'l2_tags',
            'num_threads',
            'sim_ticks',
            'num_cycles',
            'system_total_runtime_energy',
            'l2_total_runtime_energy',
            'l2_miss_rate',
            'l2_replacements'
        ]

    def dump_row(self):
        return [
            self.bench,
            self.l2_size,
            self.l2_assoc,
            self.l2_tags,
            self.num_threads,
            self.sim_ticks(),
            self.num_cycles(),
            self.system_total_runtime_energy(),
            self.l2_total_runtime_energy(),
            self.l2_miss_rate(),
            self.l2_replacements()
        ]
