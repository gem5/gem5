# Copyright (c) 2021-2025 The Regents of the University of California
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
This gem5 configuation script creates a simple board to run a matrix multiply
workload. The ISA of the board and workload depends on the argument passed
into the config.

This setup is close to the simplest setup possible using the gem5 library. It
does not contain any kind of caching, IO, or any non-essential components.

Usage
-----

```
cd tests
./main.py run gem5/regression_tests -t <number of threads to run tests with>

```
"""

import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory.simple import SingleChannelSimpleMemory
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import get_isa_from_str
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser()

parser.add_argument(
    "--isa",
    required=True,
    choices=["arm", "x86", "riscv"],
    help="Enter the isa to use.",
)

args = parser.parse_args()

# This check ensures the gem5 binary contains the ISA target. If not,
# an exception will be thrown.
requires(isa_required=get_isa_from_str(args.isa))

# In this setup we don't have a cache. `NoCache` can be used for such setups.
cache_hierarchy = NoCache()

# We use a simple memory system. Taken from
# tests/gem5/replacement_policies/configs/run_replacement_policy.py
memory = SingleChannelSimpleMemory(
    latency="30ns",
    latency_var="0ns",
    bandwidth="12.8GiB/s",
    size="512MiB",
)

# We use a simple Timing processor with one core.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=get_isa_from_str(args.isa), num_cores=1
)

# The gem5 library simple board which can be used to run simple SE-mode
# simulations.
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the workload. In this case we want to run a matrix multiply
# workload.
board.set_workload(
    obtain_resource(
        f"{args.isa}-matrix-multiply-run", resource_version="1.0.0"
    )
)

# Lastly we run the simulation.
simulator = Simulator(board=board)
simulator.run()

stats = simulator.get_stats()

# Compare values in the current run against reference IPC values calculated
# using simInsts / numCycles. The reference IPC values were obtained by running
# the tests on gem5 v25.0.0.1, commit ddd4ae35adb0a3df1f1ba11e9a973a5c2f8c2944.

reference_ipcs = {
    "arm": 0.006344991461591237,
    "riscv": 0.006273970221306115,
    "x86": 0.005040373616806491,
}

curr_ipc = (
    stats["simInsts"]["value"]
    / stats["board"]["processor"]["cores"]["value"][0]["core"]["numCycles"][
        "value"
    ]
)

print(f"Current IPC is: {curr_ipc}")
print(f"Comparison IPC is: {reference_ipcs[args.isa]}")
print(f"Current IPC is {curr_ipc/reference_ipcs[args.isa]:.2%} of reference")

if (
    curr_ipc < 0.98 * reference_ipcs[args.isa]
    or curr_ipc > 1.02 * reference_ipcs[args.isa]
):
    print(
        f"The current IPC is {curr_ipc}. Expected {reference_ipcs[args.isa]}"
    )
    exit(1)
