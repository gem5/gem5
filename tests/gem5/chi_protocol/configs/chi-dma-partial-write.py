# Copyright (c) 2026 Aalhad Sawane
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
Stress partial-line DMA writes on lines shared by CPU-side L1 caches in the
CHI `PrivateL1CacheHierarchy`.

`MemTest` testers act as cores and a further `MemTest` drives the Ruby DMA
port. All testers access the same small address range with single-byte
accesses, so the DMA requester issues `WriteUniquePtl` requests to lines
which the L1 caches hold in a shared state. The L1 caches are small to make
evictions and snoops frequent. `MemTest` checks every load against its
reference data, so the simulation fails on a protocol assertion or on
corrupted data.
"""

import argparse
import sys

import m5
from m5.objects import MemTest

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.test_board import TestBoard
from gem5.components.cachehierarchies.chi.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.abstract_generator import AbstractGenerator
from gem5.components.processors.abstract_generator_core import (
    AbstractGeneratorCore,
)
from gem5.utils.override import overrides
from gem5.utils.requires import requires

parser = argparse.ArgumentParser(
    description="Run MemTest testers on CPU-side L1 caches and on a Ruby "
    "DMA port of the CHI PrivateL1CacheHierarchy."
)
parser.add_argument(
    "--num-testers",
    type=int,
    default=4,
    help="The number of CPU-side MemTest testers.",
)
parser.add_argument(
    "--max-loads",
    type=int,
    default=10000,
    help="The number of loads each tester completes before exiting.",
)
args = parser.parse_args()

requires(coherence_protocol_required=CoherenceProtocol.CHI)


def make_tester() -> MemTest:
    # All testers share the same address range. MemTest gives each tester
    # its own byte within a cache line, so every line is shared.
    return MemTest(
        base_addr_1=0x100000,
        base_addr_2=0x400000,
        size=8192,
        max_loads=args.max_loads,
        percent_functional=0,
        percent_uncacheable=0,
        progress_interval=0,
    )


class MemTestCore(AbstractGeneratorCore):
    def __init__(self):
        super().__init__()
        self.generator = make_tester()

    @overrides(AbstractGeneratorCore)
    def connect_dcache(self, port):
        self.generator.port = port

    @overrides(AbstractGeneratorCore)
    def start_traffic(self):
        pass


class MemTestGenerator(AbstractGenerator):
    def __init__(self, num_cores: int):
        super().__init__(cores=[MemTestCore() for _ in range(num_cores)])

    @overrides(AbstractGenerator)
    def start_traffic(self):
        pass


class DMATestBoard(TestBoard):
    """A TestBoard with a MemTest tester connected to a DMA port."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.dma_tester = make_tester()

    @overrides(TestBoard)
    def has_dma_ports(self) -> bool:
        return True

    @overrides(TestBoard)
    def get_dma_ports(self):
        return [self.dma_tester.port]


board = DMATestBoard(
    clk_freq="2GHz",
    generator=MemTestGenerator(args.num_testers),
    memory=SingleChannelDDR3_1600(size="64MiB"),
    cache_hierarchy=PrivateL1CacheHierarchy(size="1KiB", assoc=2),
)

root = board._pre_instantiate()
m5.instantiate()
exit_event = m5.simulate()

print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")

# Each MemTest exits the simulation once it completes its loads. Any other
# exit cause is a failure.
if exit_event.getCause() != "maximum number of loads reached":
    sys.exit(1)
