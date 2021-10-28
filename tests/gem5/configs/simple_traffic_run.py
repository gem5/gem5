# Copyright (c) 2021 The Regents of the University of California
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
This scripts is used for checking the correctness of statistics reported
by the gem5 simulator. It can excercise certain components in the memory
subsystem. The reported values could be used to compare against a validated
set of statistics.
"""

import m5
import argparse
import importlib

from os.path import join
from m5.objects import Root
from m5.stats import gem5stats
from gem5.components.boards.test_board import TestBoard
from gem5.components.processors.linear_generator import LinearGenerator
from gem5.components.processors.random_generator import RandomGenerator


generator_class_map = {
    "LinearGenerator": LinearGenerator,
    "RandomGenerator": RandomGenerator,
}

generator_initializers = dict(rate="20GB/s")


def cache_factory(cache_class):
    if cache_class == "NoCache":
        from gem5.components.cachehierarchies.classic.no_cache import NoCache

        return NoCache()
    elif cache_class == "MESITwoLevel":
        from gem5.components.cachehierarchies.ruby\
            .mesi_two_level_cache_hierarchy import (
            MESITwoLevelCacheHierarchy,
        )

        return MESITwoLevelCacheHierarchy(
            l1i_size="32KiB",
            l1i_assoc="8",
            l1d_size="32KiB",
            l1d_assoc="8",
            l2_size="256KiB",
            l2_assoc="4",
            num_l2_banks=1,
        )
    else:
        raise ValueError(f"The cache class {cache_class} is not supported.")


parser = argparse.ArgumentParser(
    description="A traffic generator that can be used to test a gem5 "
    "memory component."
)

parser.add_argument(
    "generator_class",
    type=str,
    help="The class of generator to use.",
    choices=["LinearGenerator", "RandomGenerator"],
)

parser.add_argument(
    "cache_class",
    type=str,
    help="The cache class to import and instantiate.",
    choices=["NoCache", "MESITwoLevel"],
)

parser.add_argument(
    "mem_module",
    type=str,
    help="The python module to import for memory.",
)

parser.add_argument(
    "mem_class", type=str, help="The memory class to import and instantiate."
)

parser.add_argument(
    "mem_args",
    nargs="*",
    help="The arguments needed to instantiate the memory class.",
)

args = parser.parse_args()

generator_class = generator_class_map[args.generator_class]
generator = generator_class(**generator_initializers)

cache_hierarchy = cache_factory(args.cache_class)

memory_class = getattr(
    importlib.import_module(args.mem_module), args.mem_class
)
memory = memory_class(*args.mem_args)

# We use the Test Board. This is a special board to run traffic generation
# tasks
motherboard = TestBoard(
    clk_freq="3GHz",
    processor=generator,  # We pass the traffic generator as the processor.
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

root = Root(full_system=False, system=motherboard)

m5.instantiate()

generator.start_traffic()
print("Beginning simulation!")
exit_event = m5.simulate()
print(
    "Exiting @ tick {} because {}.".format(m5.curTick(), exit_event.getCause())
)

stats = gem5stats.get_simstat(root)
json_out = join(m5.options.outdir, "stats.json")
with open(json_out, "w") as json_stats:
    stats.dump(json_stats, indent=2)
