# Copyright (c) 2024 The Regents of the University of California
# All Rights Reserved.
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

This script is very simple and simply prints a simple message once for each
simulation, outputing the process id.

Usage
-----

1. To run all the simulations defined in this script::

```shell
<gem5-binary> -m gem5.utils.multisim \
    configs/example/gem5_library/multisim/multisim-print-this.py
```

2. To run a specific simulation defined in this script:

```shell
<gem5-binary> configs/example/gem5_library/multisim/multisim-print-this.py \
    process_id_1
```

3. To list all the IDs of the simulations defined in this script:

```shell
<gem5-binary> configs/example/gem5_library/multisim/multisim-print-this.py -l
```
"""


import gem5.utils.multisim as multisim
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

# Set the maximum number of concurrent processes to be 2.
multisim.set_num_processes(2)

for process_id in range(5):
    cache_hierarchy = NoCache()
    memory = SingleChannelDDR3_1600(size="32MB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.TIMING, isa=ISA.X86, num_cores=1
    )
    board = SimpleBoard(
        clk_freq="1GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
    board.set_se_binary_workload(
        binary=obtain_resource("x86-print-this"),
        arguments=[f"Hello from process {process_id}", 1],
    )
    multisim.add_simulator(Simulator(board=board, id=f"process_{process_id}"))
