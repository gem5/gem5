# Copyright (c) 2024 The Regents of the University of California
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
Script that runs a SpatterGen test with a specific trace file.
This script can be used as an example on how to use SpatterGenerator,
SpatterKernel, and its utilities to run a Spatter trace in gem5.

The script uses a spatter trace taken from the hpcgarage github repository.
Link to the original trace file:

https://github.com/hpcgarage/spatter/blob/main/standard-suite/app-traces/amg.json

It will create a system with `num_cores` SpatterGenerators and interleave the
trace by `intlv_size` elements in the `pattern` field from the trace.
Interleaving is done for assigning part of the access to each core.

Usage:
------

```
scons build/NULL/gem5.opt
./build/NULL/gem5.opt configs/example/gem5_library/spatter_gen/spatter-gen-test.py
```
"""
import argparse
import json
from pathlib import Path

import m5
from m5.objects import Root

from gem5.components.boards.test_board import TestBoard
from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.spatter_gen import (
    SpatterGenerator,
    prepare_kernels,
)
from gem5.simulate.simulator import Simulator

num_cores = 8
intlv_size = 128

memory = DualChannelDDR4_2400(size="8GiB")

generator = SpatterGenerator(
    processing_mode="synchronous", num_cores=num_cores
)

kernels = prepare_kernels(
    Path(__file__).parent / "traces/amg.json",
    num_cores,
    intlv_size,
    0,
    memory.get_size() // 2,
)
for kernel in kernels:
    generator.add_kernel(kernel)

board = TestBoard(
    clk_freq="4GHz",
    generator=generator,
    cache_hierarchy=PrivateL1CacheHierarchy(
        l1d_size="32KiB", l1i_size="32KiB"
    ),
    memory=memory,
)

simulator = Simulator(board=board, full_system=False)

simulator.run()
