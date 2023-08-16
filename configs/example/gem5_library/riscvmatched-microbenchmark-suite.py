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
This script shows how to use a suite. In this example, we will use the
RISCVMatchedBoard and the RISCV Vertical Microbenchmark Suite,
and show the different functionalities of the suite.

The print statements in the script are for illustrative purposes only,
and are not required to run the script.
"""

from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.prebuilt.riscvmatched.riscvmatched_board import RISCVMatchedBoard
from gem5.isas import ISA
from gem5.utils.requires import requires

requires(isa_required=ISA.RISCV)

# instantiate the riscv matched board with default parameters
board = RISCVMatchedBoard()

# obtain the RISC-V Vertical Microbenchmarks
microbenchmarks = obtain_resource("riscv-vertical-microbenchmarks")

# list all the microbenchmarks present in the suite
print("Microbenchmarks present in the suite:")
print("====================================")
for (
    id,
    resource_version,
    input_group,
    workload,
) in microbenchmarks.get_all_workloads():
    print(f"Workload ID: {id}")
    print(f"Workload Version: {resource_version}")
    print(f"Workload Input Groups: {input_group}")
    print(f"WorkloadResource Object: {workload}")
    print("====================================")

# list all the WorkloadResource objects present in the suite
for resource in microbenchmarks:
    print(f"WorkloadResource Object: {resource}")

# list all the available input groups in the suite
print("Input groups present in the suite:")
print(microbenchmarks.get_input_groups())

# for this example, we will filter the suite
# to run the Workload "riscv-cca-run"
# it has the input group 'cca', which is used as the filter
board.set_workload(list(microbenchmarks.with_input_group("cca"))[0])

# run the simulation with the RISCV Matched board
simulator = Simulator(board=board, full_system=False)
simulator.run()
print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(),
        simulator.get_last_exit_event_cause(),
    )
)
