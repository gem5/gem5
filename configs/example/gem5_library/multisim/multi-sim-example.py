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

import itertools
from functools import partial

from example_callable import (
    run_riscvmatched_workload_diff_clocks,
    run_riscvmathed_workload,
)

from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.utils.multiprocessing.multi_sim import MultiSim

# getting the resources we want to run
workload_1 = obtain_resource("riscv-gapbs-tc-run")
workload_2 = obtain_resource("riscv-npb-is-size-s-run")

# creating a list of partial functions to run
# The sim_factory list is a list of tuples. Each tuple contains a partial function and the name of the output directory.
sim_factory = []

# This is another way to create the list of partial functions to run
# If you dont want to run a cross product of all the workloads and configurations
# sim_factory.append(
#     partial(run_riscvmathed_workload, workload_1)
# )
# sim_factory.append(
#     partial(run_riscvmathed_workload, workload_2)
# )

# This is a way tocreate a cross product of all the workloads and configurations

configs = [run_riscvmathed_workload, run_riscvmatched_workload_diff_clocks]
workloads = [workload_1, workload_2]
sim_factory = [
    partial(config, workload)
    for config, workload in itertools.product(configs, workloads)
]

multi_sim = MultiSim(sim_factory)

# running the simulations with 4 threads
multi_sim.run(2)
