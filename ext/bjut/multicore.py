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
            'num_cycles',
            'l2_miss_rate'
        ]

    def dump_row(self):
        return [
            self.bench,
            self.l2_size,
            self.l2_assoc,
            self.l2_tags,
            self.num_threads,
            self.num_cycles(),
            self.l2_miss_rate()
        ]
