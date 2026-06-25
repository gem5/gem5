# Copyright (c) 2026 Rajesh Gangam
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
Run a RISC-V SE binary on an O3 CPU with E-Trace enabled.  Produces a
protobuf trace file that can be decoded with util/decode_etrace.py.

Usage
-----

```
scons build/RISCV/gem5.opt
./build/RISCV/gem5.opt configs/example/gem5_library/riscv-etrace-se.py
```

After the simulation completes, decode the trace:

```
cd util && python3 decode_etrace.py ../m5out/system.processor.cores0\
.core.etrace.etrace.pb.gz
```
"""

from m5.objects import ETrace

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

cache_hierarchy = PrivateL1CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
)

memory = SingleChannelDDR3_1600("512MiB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.O3,
    isa=ISA.RISCV,
    num_cores=1,
)

for core in processor.get_cores():
    core.core.probeListener = ETrace(
        traceFile="etrace.pb.gz",
        resyncPeriod=10000,
    )

board = SimpleBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_se_binary_workload(
    obtain_resource("riscv-hello", resource_version="1.0.0")
)

simulator = Simulator(board=board)
simulator.run()

print(f"Simulation done. Exit cause: '{simulator.get_last_exit_event_cause()}'")
print("E-Trace output written to m5out/ directory")
