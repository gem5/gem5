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
import argparse
import importlib
from pathlib import Path

import m5
from gem5.components.boards.test_board import TestBoard
from m5.objects import MemorySize
from m5.objects import Root
from m5.stats.gem5stats import get_simstat


def generator_factory(
    generator_class: str,
    generator_cores: int,
    mem_size: MemorySize,
):
    if generator_class == "LinearGenerator":
        from gem5.components.processors.linear_generator import LinearGenerator

        return LinearGenerator(
            duration="250us",
            rate="40GB/s",
            num_cores=generator_cores,
            max_addr=mem_size,
        )
    elif generator_class == "RandomGenerator":
        from gem5.components.processors.random_generator import RandomGenerator

        return RandomGenerator(
            duration="250us",
            rate="40GB/s",
            num_cores=generator_cores,
            max_addr=mem_size,
        )
    elif generator_class == "GUPSGenerator":
        if generator_cores != 1:
            raise ValueError(
                "Only one core should be used with GUPSGenerator. "
                "In order to use multiple cores of GUPS generator, use either "
                "GUPSGeneratorEP or GUPSGeneratorPAR.",
            )
        from gem5.components.processors.gups_generator import GUPSGenerator

        table_size = f"{int(mem_size / 2)}B"
        return GUPSGenerator(0, table_size, update_limit=1000, clk_freq="2GHz")
    elif generator_class == "GUPSGeneratorEP":
        from gem5.components.processors.gups_generator_ep import (
            GUPSGeneratorEP,
        )

        table_size = f"{int(mem_size / 2)}B"

        return GUPSGeneratorEP(
            generator_cores,
            0,
            table_size,
            update_limit=1000,
            clk_freq="2GHz",
        )
    elif generator_class == "GUPSGeneratorPAR":
        from gem5.components.processors.gups_generator_par import (
            GUPSGeneratorPAR,
        )

        table_size = f"{int(mem_size / 2)}B"
        return GUPSGeneratorPAR(
            generator_cores,
            0,
            table_size,
            update_limit=1000,
            clk_freq="2GHz",
        )
    else:
        raise ValueError(f"Unknown generator class {generator_class}")


def cache_factory(cache_class: str):
    if cache_class == "NoCache":
        from gem5.components.cachehierarchies.classic.no_cache import NoCache

        return NoCache()
    elif cache_class == "PrivateL1":
        from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
            PrivateL1CacheHierarchy,
        )

        return PrivateL1CacheHierarchy(l1d_size="32KiB", l1i_size="32KiB")
    elif cache_class == "PrivateL1PrivateL2":
        from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
            PrivateL1PrivateL2CacheHierarchy,
        )

        return PrivateL1PrivateL2CacheHierarchy(
            l1d_size="32KiB",
            l1i_size="32KiB",
            l2_size="256KiB",
        )
    elif cache_class == "MESITwoLevel":
        from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
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
    "memory component.",
)

parser.add_argument(
    "generator_class",
    type=str,
    help="The class of generator to use.",
    choices=[
        "LinearGenerator",
        "RandomGenerator",
        "GUPSGenerator",
        "GUPSGeneratorEP",
        "GUPSGeneratorPAR",
    ],
)

parser.add_argument(
    "generator_cores",
    type=int,
    help="The number of generator cores to use.",
)

parser.add_argument(
    "cache_class",
    type=str,
    help="The cache class to import and instantiate.",
    choices=["NoCache", "PrivateL1", "PrivateL1PrivateL2", "MESITwoLevel"],
)

parser.add_argument(
    "mem_module",
    type=str,
    help="The python module to import for memory.",
)

parser.add_argument(
    "mem_class",
    type=str,
    help="The memory class to import and instantiate.",
)

parser.add_argument(
    "mem_args",
    nargs="*",
    help="The arguments needed to instantiate the memory class.",
)

args = parser.parse_args()

cache_hierarchy = cache_factory(args.cache_class)

memory_class = getattr(
    importlib.import_module(args.mem_module),
    args.mem_class,
)
memory = memory_class(*args.mem_args)

generator = generator_factory(
    args.generator_class,
    args.generator_cores,
    memory.get_size(),
)

# We use the Test Board. This is a special board to run traffic generation
# tasks
motherboard = TestBoard(
    clk_freq="3GHz",
    generator=generator,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

root = Root(full_system=False, system=motherboard)

motherboard._pre_instantiate()
m5.instantiate()

generator.start_traffic()
print("Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}.")

simstats = get_simstat(
    [core.generator for core in generator.get_cores()],
    prepare_stats=True,
)
json_output = Path(m5.options.outdir) / "output.json"
with open(json_output, "w") as stats_file:
    simstats.dump(stats_file, indent=2)
