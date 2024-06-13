# Copyright (c) 2024 The Regents of the University of California.
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

"""An example of a single configuration script for defining multiple
simulations through the gem5 `multisim` module.

This script creates 6 full system simulations by interating through a suite
of benchmarks and different cores counts.

Usage
-----

1. To run all the simulations defined in this script::

```shell
<gem5-binary> -m gem5.utils.multisim \
    configs/example/gem5_library/multisim/multisim-fs-x86-npb.py
```

2. To run a specific simulation defined in this script:

```shell
<gem5-binary> configs/example/gem5_library/multisim/multisim-fs-x86-npb.py \
    <process_id> # e.g. npb-bt-a_cores-1
```

3. To list all the IDs of the simulations defined in this script:

```shell
<gem5-binary> configs/example/gem5_library/multisim/multisim-fs-x86-npb.py -l
```
"""

import m5

import gem5.utils.multisim as multisim
from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import (
    ExitEvent,
    Simulator,
)
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
)

from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)


def handle_workbegin():
    m5.stats.reset()
    processor.switch()
    yield False


def handle_workend():
    m5.stats.dump()
    yield True


# Here we imagine an experiment wanting to run each NPB benchmark on the same
# system twice: once with 1 core and once with 2 cores.
for benchmark in obtain_resource("npb-benchmark-suite"):
    for num_cores in [1, 2]:
        cache_hierarchy = MESITwoLevelCacheHierarchy(
            l1d_size="32kB",
            l1i_size="32kB",
            l2_size="256kB",
            l1d_assoc=8,
            l1i_assoc=8,
            l2_assoc=16,
            num_l2_banks=2,
        )
        memory = DualChannelDDR4_2400(size="3GB")
        processor = SimpleSwitchableProcessor(
            starting_core_type=CPUTypes.ATOMIC,
            switch_core_type=CPUTypes.TIMING,
            isa=ISA.X86,
            num_cores=num_cores,
        )
        board = X86Board(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

        board.set_workload(benchmark)

        simulator = Simulator(
            board=board,
            on_exit_event={
                ExitEvent.WORKBEGIN: handle_workbegin(),
                ExitEvent.WORKEND: handle_workend(),
            },
        )

        simulator.set_id(f"{benchmark.get_id()}_cores-{num_cores}")

        multisim.add_simulator(simulator)
