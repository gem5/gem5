# Copyright (c) 2024-2025 The Regents of the University of California.
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
An example of using a single configuration script to define and run multiple
simulations using gem5's `multisim` module.

This script creates 18 full system simulations by iterating through NPB
benchmarks and different core counts.

Usage
-----

1. To run all the simulations defined in this script:

```shell
<gem5-binary> -m gem5.utils.multisim \
    configs/example/gem5_library/multisim/multisim-fs-x86-npb.py
```

2. To run a specific simulation defined in this script:

```shell
<gem5-binary> configs/example/gem5_library/multisim/multisim-fs-x86-npb.py \
    <process_id> # e.g. npb-bt-s_cores-1
```

3. To list all the IDs of the simulations defined in this script:

```shell
<gem5-binary> configs/example/gem5_library/multisim/multisim-fs-x86-npb.py -l
```
"""

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
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
)

from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)

# Set the maximum number of concurrent processes to be 6.
multisim.set_num_processes(6)

# Here we imagine an experiment wanting to run each NPB benchmark on the same
# system twice: once with 1 core and once with 2 cores.

for benchmark in ["bt", "cg", "ep", "ft", "is", "lu", "mg", "sp", "ua"]:
    for num_cores in [1, 2]:
        cache_hierarchy = MESITwoLevelCacheHierarchy(
            l1d_size="32KiB",
            l1i_size="32KiB",
            l2_size="256KiB",
            l1d_assoc=8,
            l1i_assoc=8,
            l2_assoc=16,
            num_l2_banks=2,
        )
        memory = DualChannelDDR4_2400(size="3GiB")
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

        board.set_workload(
            obtain_resource(
                f"x86-ubuntu-24.04-npb-{benchmark}-s", resource_version="3.0.0"
            )
        )

        simulator = Simulator(board=board)

        # As this is just an example we will only run the simulation for a
        # billion ticks. An actual run could take days of time to simulate.
        #
        # We use `set_hypercall_absolute_max_ticks` to schedule a hypercall 6
        # exit at 1 billion ticks. The default hypercall 6 behavior is to
        # end simulation, but this can be overridden with a custom hypercall
        # handler. See `src/python/gem5/simulate/exit_handler.py` for more
        # information.
        simulator.set_hypercall_absolute_max_ticks(
            1_000_000_000, "To exit the simulation as this is just an example."
        )

        simulator.set_id(f"npb-{benchmark}-s_cores-{num_cores}")

        multisim.add_simulator(simulator=simulator)
