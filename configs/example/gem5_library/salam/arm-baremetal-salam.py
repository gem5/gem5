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
# The machine matches the one ``configs/SALAM/fs.py`` builds with the options
# ``util/SALAM-tools/run_system.sh`` passes: one O3 core at 2 GHz on a 1 GHz
# system, private L1 caches and a shared L2 with the legacy cache parameters,
# 16 GiB of single-channel DDR4-2400 and the VExpress_GEM5_V1 platform.
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

from m5.objects import (
    NULL,
    BadAddr,
    IdeController,
    IOXBar,
    L2XBar,
    SystemXBar,
)

from gem5.components.cachehierarchies.classic.caches.l1dcache import L1DCache
from gem5.components.cachehierarchies.classic.caches.l1icache import L1ICache
from gem5.components.cachehierarchies.classic.caches.l2cache import L2Cache
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
cpu_clock = processor.get_clock_domain()


def no_prefetcher():
    return NULL


# The caches run on the core clock, as the legacy configuration's do.
def l1i():
    cache = L1ICache(
        size="32KiB",
        assoc=2,
        tag_latency=2,
        data_latency=2,
        response_latency=2,
        mshrs=4,
        tgts_per_mshr=20,
        writeback_clean=True,
        PrefetcherCls=no_prefetcher,
    )
    cache.is_read_only = True
    cache.clk_domain = cpu_clock
    return cache


def l1d():
    cache = L1DCache(
        size="64KiB",
        assoc=2,
        tag_latency=2,
        data_latency=2,
        response_latency=2,
        mshrs=4,
        tgts_per_mshr=20,
        PrefetcherCls=no_prefetcher,
    )
    cache.clk_domain = cpu_clock
    return cache


def l2():
    cache = L2Cache(
        size="2MiB",
        assoc=8,
        tag_latency=20,
        data_latency=20,
        response_latency=20,
        mshrs=20,
        tgts_per_mshr=12,
        PrefetcherCls=no_prefetcher,
    )
    cache.clk_domain = cpu_clock
    return cache


membus = SystemXBar(width=16)
membus.badaddr_responder = BadAddr(warn_access="warn")
membus.default = membus.badaddr_responder.pio

cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
    l1d_size="64KiB",
    l1i_size="32KiB",
    l2_size="2MiB",
    membus=membus,
    l1i_cache=l1i,
    l1d_cache=l1d,
    l2_cache=l2,
    l2bus=L2XBar(clk_domain=cpu_clock),
)

board = SALAMArmBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=SingleChannelDDR4_2400(size="16GiB"),
    cache_hierarchy=cache_hierarchy,
    iobus=IOXBar(),
)
# ArmBoard marks single-core systems as uniprocessors in MPIDR; the
# legacy configuration keeps ArmSystem's multiprocessor default.
board.multi_proc = True
board.set_bare_metal_binary(args.binary)
# This field supplies the bootloader, which this bare-metal run does not use.
board.gic_cpu_addr = 0
board.attach_accelerators(args.accpath, args.acccfg)
# The platform has no built-in storage controller; the legacy
# configuration adds an IDE controller without disks.
board.add_pci_device(IdeController(disks=[]))

# Use gem5's existing command-line parameter mechanism after the accelerator
# children exist and before the simulator instantiates their C++ objects.
board.apply_config(args.param)

simulator = Simulator(board=board)
simulator.run(max_ticks=args.max_ticks)
print(
    f"Exiting @ tick {simulator.get_current_tick()} because "
    f"{simulator.get_last_exit_event_cause()}"
)
