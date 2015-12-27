#
# A Python script for reading GEM5 config and stat files.
#
# Copyright (C) Min Cai 2015
#

import os
import traceback
import collections
import json
from objectpath import *
from pyparsing import Word, Optional, ParseException, printables, nums, restOfLine
from flufl.enum import Enum


class ExperimentType(Enum):
    FOUR_PHASE_FS_SIMULATION, THREE_PHASE_SE_SIMULATION = range(2)


class Experiment:
    def __init__(self, type, dir, bench=None, l2_size=None, l2_assoc=None, l2_tags=None, section_num_to_use=2):
        self.type = type
        self.dir = dir
        self.bench = bench
        self.l2_size = l2_size
        self.l2_assoc = l2_assoc
        self.l2_tags = l2_tags
        self.section_num_to_use = section_num_to_use

        self.configs = self.read_configs()
        self.stats = self.read_stats()

    def read_configs(self):
        try:
            with open(self.config_json_file_name()) as config_json_file:
                config = json.load(config_json_file)
            return Tree(config)
        except:
            traceback.print_exc()
            return None

    def read_stats(self):
        stat_rule = Word(printables) + Word('nan.%' + nums) + Optional(restOfLine)

        stats = collections.OrderedDict()

        try:
            with open(self.stats_file_name()) as stats_file:
                i = 0
                for stat_line in stats_file:
                    if 'End Simulation Statistics' in stat_line:
                        i += 1
                    elif i == self.section_num_to_use:
                        try:
                            stat = stat_rule.parseString(stat_line)
                            key = stat[0]
                            value = stat[1]
                            stats[key] = value
                        except ParseException:
                            pass

            return stats
        except:
            traceback.print_exc()
            return None

    def config_json_file_name(self):
        return os.path.join(self.dir, 'config.json')

    def stats_file_name(self):
        return os.path.join(self.dir, 'stats.txt')

    def mcpat_in_xml_file_name(self):
        return os.path.join(self.dir, 'mcpat_in.xml')

    def mcpat_out_file_name(self):
        return os.path.join(self.dir, 'mcpat_out.txt')

    def num_cpus(self):
        return -1 if self.configs is None else self.configs.execute('len($.system.cpu)')

    def num_l2caches(self):
        return -1 if self.configs is None else self.configs.execute('len($.system.l2cache)') if self.numa() else 1

    def numa(self):
        return False if self.configs is None else self.configs.execute('len($.system.numa_caches_upward)') > 0

    def cpu_id(self, i, l1=False):
        if self.type == ExperimentType.THREE_PHASE_SE_SIMULATION:
            if not l1:
                id = 'switch_cpus_1'
            else:
                id = 'cpu'
        elif self.type == ExperimentType.FOUR_PHASE_FS_SIMULATION:
            if not l1:
                id = 'switch_cpus'
            else:
                id = 'cpu'
        else:
            raise NotImplementedError

        return '' if self.configs is None else self.configs.execute('$.system.' + id + '[' + str(i) + '].name')

    def l2_id(self, i=None):
        return '' if self.configs is None else ('l2' if i is None else self.configs.execute('$.system.l2cache[' + str(i) + '].name'))

    def sim_seconds(self):
        key = 'system.' + self.cpu_id(0) + '.sim_seconds'
        return -1 if self.stats is None or key not in self.stats else float(self.stats[key])

    def sim_ticks(self):
        key = 'sim_ticks'
        return -1 if self.stats is None or key not in self.stats else int(self.stats[key])

    def num_cycles(self):
        key = 'system.' + self.cpu_id(0) + '.numCycles'
        return -1 if self.stats is None or key not in self.stats else int(self.stats[key])

    def l2_mpki(self):
        if self.stats is None:
            return -1

        num_l2_misses = 0
        num_committed_instructions = 0

        if self.numa():
            for i in range(self.num_l2caches()):
                key = 'system.' + self.l2_id(i) + '.overall_misses::total'
                num_l2_misses += (0 if self.stats is None or key not in self.stats else int(self.stats[key]))
        else:
                key = 'system.' + self.l2_id() + '.overall_misses::total'
                num_l2_misses += (0 if self.stats is None or key not in self.stats else int(self.stats[key]))

        for i in range(self.num_cpus()):
                key = 'system.' + self.cpu_id(i) + '.committedInsts'
                num_committed_instructions += (0 if self.stats is None or key not in self.stats else int(self.stats[key]))

        return num_l2_misses * 1000 / float(num_committed_instructions)

    def l2_miss_rate(self):
        if self.stats is None:
            return -1

        l2_miss_rates = []

        if self.numa():
            for i in range(self.num_l2caches()):
                key = 'system.' + self.l2_id(i) + '.overall_miss_rate::total'
                l2_miss_rates.append(-1 if self.stats is None or key not in self.stats else float(self.stats[key]))
        else:
            key = 'system.' + self.l2_id() + '.overall_miss_rate::total'
            l2_miss_rates.append(-1 if self.stats is None or key not in self.stats else float(self.stats[key]))

        return sum(l2_miss_rates) / float(len(l2_miss_rates))

    def l2_replacements(self):
        if self.stats is None:
            return -1

        l2_replacements = []

        if self.numa():
            for i in range(self.num_l2caches()):
                key = 'system.' + self.l2_id(i) + '.tags.replacements'
                l2_replacements.append(-1 if self.stats is None or key not in self.stats else int(self.stats[key]))
        else:
            key = 'system.' + self.l2_id() + '.tags.replacements'
            l2_replacements.append(-1 if self.stats is None or key not in self.stats else int(self.stats[key]))

        return sum(l2_replacements)

    @classmethod
    def dump_head_row(cls):
        pass

    def dump_row(self, baseline_experiment):
        pass
