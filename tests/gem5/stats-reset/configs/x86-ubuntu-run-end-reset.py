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

This test runs an X86 Ubuntu 24.04 boot for 15 billion ticks. It resets at 15
billion ticks, then dumps the stats. With the exception of a few stats that
should not be reset with m5.stats.reset(), all of the stats should have a value
of 0, nan, or simply not be present. To confirm that all of the stats have been
reset, we check their values against stats from a simulation with the
same configuration and workload that runs for 15 billion ticks without
resetting. We skip over stats that have values of 0 in the run that doesn't
reset, as we cannot tell whether the stat has reset or not.

"""

import sys

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

# Main memory
from gem5.components.memory import DualChannelDDR4_2400

memory = DualChannelDDR4_2400(size="3GiB")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.ATOMIC,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=2,
)

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


def max_tick_handler_end_reset():
    m5.stats.reset()
    m5.stats.dump()
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        # Here we want override the default behavior for the first m5 exit
        # exit event.
        ExitEvent.EXIT: exit_event_handler(),
        ExitEvent.MAX_TICK: max_tick_handler_end_reset(),
    },
)

simulator.set_max_ticks(15000000000)  # 15,000,000,000 ticks
simulator.run()

end_reset_dict = {}
no_reset_dict = {}


def read_stats_files(filepath, stats_dict):
    with open(filepath) as stats:
        for line in stats:
            tmp = line.split()
            if len(tmp) > 1:
                stats_dict[tmp[0]] = tmp[1]
    stats_dict.pop("----------", None)


read_stats_files("./gem5/stats-reset/base-case-stats.txt", no_reset_dict)
read_stats_files(f"{m5.options.outdir}/stats.txt", end_reset_dict)

# These stats are either constant, should carry over across resets, or have to do with the host
exclude_from_check = [
    "finalTick",
    "simFreq",
    "hostSeconds",
    "hostMemory",
    "UNDEFINED",
    "clk_domain.clock",
    "voltage_domain.voltage",
    "peakBW",
]

# These are the stats that don't reset when m5.stats.reset() is called, and which I don't know if they are supposed to reset
# idleFraction is in the uncertain list because it could be calculated from the total number of ticks instead of the number of ticks elapsed between resets.
uncertain_exclude_from_check = [
    "tagsInUse",
    "occupancies",
    "avgOccs",
    "ageTaskId_1024",
    "ratioOccsTaskId",
    "pwrStateTime::IDLE",
    "pwrStateResidencyTicks::ON",
    "pwrStateResidencyTicks::OFF",
    "idleFraction",
    "notIdleFraction",
]


# checks to see if any items in a list match with a key/ part of a key.
# This is used to filter for stats that appear for several components
def is_match_key(key, match_list):
    if len([item for item in match_list if item in key]) == 0:
        return False
    else:
        return True


not_reset_properly = {}
for key, value in end_reset_dict.items():
    # Get stats that weren't reset properly.
    # We don't consider stats that had a value of 0 or nan in the simulation with no resets, as those values can't tell us anything. If the stats from both simulations have values of 0, there is no telling if the stat actually reset or not.
    if (
        not (value == "nan" or float(value) == 0.0)
        and not (no_reset_dict[key] == "0" or no_reset_dict[key] == "nan")
        and not is_match_key(key, exclude_from_check)
        and not is_match_key(key, uncertain_exclude_from_check)
    ):
        not_reset_properly[key] = value

for item in not_reset_properly.items():
    print(item)

if len(not_reset_properly) > 0:
    print(f"{len(not_reset_properly)} stats did not reset properly!")
    sys.exit(1)
