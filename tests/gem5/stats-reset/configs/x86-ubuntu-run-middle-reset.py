# Copyright (c) 2023 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This script is based on
configs/example/gem5_library/x86-ubuntu-run-with-kvm-no-perf.py

This test runs an X86 Ubuntu 24.04 boot for 15 billion ticks in total. At the
5 billion tick mark, it dumps and resets the stats. It then runs for another
10 billion ticks before dumping the stats again. We check the stats by adding
the values from the two stats dumps together and comparing it against the stats
from a simulation without resets.

"""

import math
import sys
from collections import defaultdict

import m5

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
)

from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)

cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
    l1d_size="64KiB", l1i_size="64KiB", l2_size="1MiB"
)

from gem5.components.memory import DualChannelDDR4_2400

memory = DualChannelDDR4_2400(size="3GiB")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=2,
)

# Here we setup the board. The X86Board allows for Full-System X86 simulations.
board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)


workload = obtain_resource("x86-ubuntu-24.04-boot-with-systemd")
board.set_workload(workload)


def exit_event_handler():
    print("First exit: kernel booted")
    processor.switch()
    yield False  # gem5 is now executing systemd startup
    print("Second exit: Started `after_boot.sh` script")
    # m5.stats.dump()
    yield False  # gem5 is now executing the `after_boot.sh` script
    print("Third exit: Finished `after_boot.sh` script")
    # The after_boot.sh script will run a script if it is passed via
    # m5 readfile. This is the last exit event before the simulation exits.
    yield True


def max_tick_handler_middle_reset():
    m5.stats.dump()
    m5.stats.reset()
    simulator.set_max_ticks(10000000000)  # 10,000,000,000
    yield False
    m5.stats.dump()


simulator = Simulator(
    board=board,
    on_exit_event={
        # Here we want override the default behavior for the first m5 exit
        # exit event.
        ExitEvent.EXIT: exit_event_handler(),
        ExitEvent.MAX_TICK: max_tick_handler_middle_reset(),
    },
)

simulator.set_max_ticks(5000000000)  # 5,000,000,000

simulator.run()

middle_reset_dict = defaultdict(list)
no_reset_dict = defaultdict(list)


def read_stats_files(filepath, stats_dict):
    with open(filepath) as stats:
        for line in stats:
            tmp = line.split()
            if len(tmp) > 1:
                stats_dict[tmp[0]].append(tmp[1])
    stats_dict.pop("----------", None)


# checks to see if any items in a list match with a key/ part of a key.
# This is used to detect stats that appear for several components
def is_match_key(key, match_list):
    if len([item for item in match_list if item in key]) == 0:
        return False
    else:
        return True


def check_both_nan(val_1, val_2):
    if math.isnan(val_1) and math.isnan(val_2):
        return True
    return False


read_stats_files("./gem5/stats-reset/base-case-stats.txt", no_reset_dict)
read_stats_files(f"{m5.options.outdir}/stats.txt", middle_reset_dict)

not_reset_properly = {}

# These stats are either constant, should carry over across resets, or have to do with the host
exclude_from_check = [
    "hostSeconds",
    "hostTickRate",
    "hostMemory",
    "hostInstRate",
    "hostOpRate",
    "UNDEFINED",
    "stdev",
    # Below are stats that I am not sure should reset/ I don't know how they are calculated
    "occupancies",
    "tagsInUse",
    "bwRead",
    "bwInstRead",
    "bwTotal",
    "bwWrite",
]

check_same = [
    "finalTick",
    "simFreq",
    "snoopFanout::max_value",
    "board.clk_domain.clock",
    "board.clk_domain.voltage_domain.voltage",
    # Below are stats that may or may not be constant/intended to be consistently increasing throughout the simulation.
    "ageTaskId_1024",
    "ratioOccsTaskId",
    "peakBW",
    "clk_domain.clock",
]  #


# Currently, the stats in this list are not being checked. Some stats can be calculated from the values of the first and second dumps using a weighted average based on the number of ticks elasped in each dump, but others cannot.
check_avg = [
    "demandMissRate",
    "overallMissRate",
    "missRate",
    "avgRefs",
    "avgOccs",
    "snoopFanout::mean",
    "idleFraction",
    "notIdleFraction",
    "fetchRate",
    "branchRate",
    "averagePower",
    # Below are stats that may or may not be averages
    "cpi",
    "ipc",
]
valid_avg = {}

# If a stat is present when you don't reset but goes missing if you reset,
# something is wrong
for key, value in no_reset_dict.items():
    if key not in middle_reset_dict:
        not_reset_properly[key] = [value, "missing from middle_reset_dict"]


# check stats that count up over time but reset when m5.stats.reset is called.
# These stats can be added together.

# total value = stat from 0 to 5 billion ticks + stat from 5 billion to
# 15 billion ticks

for key, value in middle_reset_dict.items():
    no_reset_val = float(no_reset_dict[key][0])

    if (
        not is_match_key(key, check_same)
        and not is_match_key(key, check_avg)
        and not is_match_key(key, exclude_from_check)
    ):
        if len(value) == 2:
            middle_reset_sum = float(value[0]) + float(value[1])
            # The energy stats may be slightly off due to floating point errors. This rounds them to avoid that.
            if is_match_key(key, ["Energy"]):
                middle_reset_sum = round(middle_reset_sum, 4)
                no_reset_val = round(no_reset_val, 4)
            if (
                not check_both_nan(middle_reset_sum, no_reset_val)
                and no_reset_val != middle_reset_sum
            ):
                not_reset_properly[key] = [
                    no_reset_val,
                    middle_reset_sum,
                    "length 2",
                ]
        elif len(value) == 1:
            middle_reset_sum = float(value[0])
            if (
                not check_both_nan(middle_reset_sum, no_reset_val)
                and no_reset_val != middle_reset_sum
            ):
                not_reset_properly[key] = [
                    no_reset_val,
                    middle_reset_sum,
                    "length 1",
                ]


# Check stats that are averages, rates, or constants/stats that don't reset
# with m5.stats.reset().
for key, value in middle_reset_dict.items():

    first_part_val = float(value[0])
    no_reset_val = float(no_reset_dict[key][0])

    if len(value) == 2:
        second_part_val = float(value[1])

        # if is_match_key(key, check_avg):
        #     temp_sum = round(first_part_val * (1/3) + second_part_val * (2/3), 4)
        #     if temp_sum != round(no_reset_val, 4):
        #         not_reset_properly[key] = [no_reset_val, temp_sum, "average"]
        #     else:
        #         valid_avg[key] = [no_reset_val]

        # check stats that are constants and/or maximum values
        if is_match_key(key, check_same):
            if second_part_val != no_reset_val:
                not_reset_properly[key] = [
                    no_reset_val,
                    first_part_val,
                    second_part_val,
                    "constant",
                ]
    elif len(value) == 1:
        # for now, skip stats that are averages
        # if is_match_key(key, check_avg):
        #     temp_sum = round(first_part_val, 4)
        #     if temp_sum != round(no_reset_val, 4):
        #         not_reset_properly[key] = [no_reset_val, temp_sum, "average"]
        #     else:
        #         valid_avg[key] = [no_reset_val]

        # check stats that are constants and/or maximum values
        if is_match_key(key, check_same):
            if first_part_val != no_reset_val:
                not_reset_properly[key] = [
                    no_reset_val,
                    first_part_val,
                    "constant",
                ]

for item in not_reset_properly.items():
    print(item)

if len(not_reset_properly) > 0:
    print(f"{len(not_reset_properly)} stats did not reset properly!")
    sys.exit(1)
