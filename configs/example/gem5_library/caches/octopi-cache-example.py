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
This script boots Ubuntu 20.04 with 8 timing cores in 1 CCD.

Usage
-----

```
scons build/ARM_MESI_Three_Level/gem5.opt -j `nproc`
./build/ARM_MESI_Three_Level/gem5.opt \
    configs/example/gem5_library/caches/octopi-cache-example.py
```
"""


from m5.objects import ArmDefaultRelease, VExpress_GEM5_Foundation

from gem5.utils.requires import requires
from gem5.components.boards.arm_board import ArmBoard
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.cachehierarchies.ruby.caches.mesi_three_level.octopi import (
    OctopiCache,
)
from gem5.isas import ISA
from gem5.coherence_protocol import CoherenceProtocol
from gem5.simulate.simulator import Simulator
from gem5.resources.resource import obtain_resource

num_ccds = 1  # CCDs
num_cores_per_ccd = 8  # 8 cores/CCD

# OctopiCache is built on top of gem5's MESI_Three_Level cache coherence
# protocol
requires(coherence_protocol_required=CoherenceProtocol.MESI_THREE_LEVEL)
cache_hierarchy = OctopiCache(
    l1i_size="32KiB",
    l1i_assoc=8,
    l1d_size="32KiB",
    l1d_assoc=8,
    l2_size="512KiB",
    l2_assoc=8,
    l3_size="32MiB",
    l3_assoc=16,
    num_core_complexes=num_ccds,
    is_fullsystem=True,
)

memory = DualChannelDDR4_2400(size="16GB")

# The number of cores must be consistent with
# num_core_complexes and num_cores_per_core_complexes
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING,
    isa=ISA.ARM,
    num_cores=num_ccds * num_cores_per_ccd,
)

release = ArmDefaultRelease()
platform = VExpress_GEM5_Foundation()

board = ArmBoard(
    clk_freq="4GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    release=release,
    platform=platform,
)

board.set_workload(obtain_resource("arm64-ubuntu-20.04-boot"))

simulator = Simulator(board=board)
simulator.run()
