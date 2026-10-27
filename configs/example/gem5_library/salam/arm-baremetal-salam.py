# Copyright (c) 2026 Akanksha Chaudhari
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


# Run a bare-metal ARM program that uses gem5-SALAM accelerators, built from
# gem5 standard library components.
#
# One O3 core runs at 2 GHz on a 1 GHz board with private L1 caches, a
# shared L2, 16 GiB of single-channel DDR4-2400, and VExpress_GEM5_V1.
# The cache hierarchy and board use their ordinary component defaults.
#
# Usage
# -----
#
# ```
# scons build/ARM/gem5.opt --with-salam
# ./build/ARM/gem5.opt \
#     configs/example/gem5_library/salam/arm-baremetal-salam.py \
#     --binary <bench>/sw/main.elf --accpath <bench>
# ```
#
# where ``<bench>`` is a built benchmark directory holding ``config.yml``,
# for example ``configs/example/gem5_library/salam-benchmarks/src/bfs``.

import argparse

from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.salam.board import SALAMArmBoard
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser(
    description="Run a bare-metal ARM program with SALAM accelerators."
)
parser.add_argument(
    "--binary", required=True, help="The bare-metal ARM ELF to run."
)
parser.add_argument(
    "--accpath",
    required=True,
    help="The directory holding the accelerator configuration.",
)
parser.add_argument(
    "--acccfg",
    default="config.yml",
    help="The accelerator configuration file, relative to --accpath.",
)
parser.add_argument(
    "--max-ticks", type=int, default=None, help="Stop after this tick."
)
parser.add_argument(
    "--param",
    action="append",
    default=[],
    help="Override a child SimObject parameter relative to the board.",
)
args = parser.parse_args()

processor = SimpleProcessor(
    cpu_type=CPUTypes.O3, isa=ISA.ARM, num_cores=1, clk_freq="2GHz"
)

cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
    l1d_size="64KiB",
    l1i_size="32KiB",
    l2_size="2MiB",
)

board = SALAMArmBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=SingleChannelDDR4_2400(size="16GiB"),
    cache_hierarchy=cache_hierarchy,
)
board.set_bare_metal_binary(args.binary)
board.attach_accelerators(args.accpath, args.acccfg)

# Use gem5's existing command-line parameter mechanism after the accelerator
# children exist and before the simulator instantiates their C++ objects.
board.apply_config(args.param)

simulator = Simulator(board=board)
simulator.run(max_ticks=args.max_ticks)
print(
    f"Exiting @ tick {simulator.get_current_tick()} because "
    f"{simulator.get_last_exit_event_cause()}"
)
