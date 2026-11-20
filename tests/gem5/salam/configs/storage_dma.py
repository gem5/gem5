# Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
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

import argparse
import json

import m5
from m5.objects import (
    AddrRange,
    SALAMPowerModel,
    ScratchpadMemory,
)

from gem5.components.cachehierarchies.classic import (
    private_l1_shared_l2_cache_hierarchy as shared_l2,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.salam.board import SALAMArmBoard

parser = argparse.ArgumentParser(
    description="Exercise DMA traffic through characterized storage."
)
parser.add_argument("parameters", help="JSON storage-test parameters")
args = parser.parse_args()
with open(args.parameters) as stream:
    settings = json.load(stream)

board = SALAMArmBoard(
    clk_freq="1GHz",
    processor=SimpleProcessor(
        cpu_type=CPUTypes.TIMING, isa=ISA.ARM, num_cores=1, clk_freq="1GHz"
    ),
    memory=SingleChannelDDR4_2400(size="4GiB"),
    cache_hierarchy=shared_l2.PrivateL1SharedL2CacheHierarchy(
        l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
    ),
)
board.set_bare_metal_binary(settings["binary"])
board.attach_accelerators(settings["accpath"])
board.test_memory = ScratchpadMemory(
    range=AddrRange(0x2FF10000, size=4096),
    access_cycles=10000,
    bandwidth="8MiB/s",
    ppa=SALAMPowerModel(**settings["model"]),
)
board.test_memory.port = board.iobus.mem_side_ports
board.harness_clstr.coherency_bus.header_latency = 10
root = board._pre_instantiate()
m5.instantiate()
event = m5.simulate(4000000000)
print(f"Exiting @ tick {m5.curTick()} because {event.getCause()}")
