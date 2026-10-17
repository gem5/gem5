# Copyright (c) 2021 The Regents of the University of California
# Copyright (c) 2022 Google Inc
# Copyright (c) 2026 Donmakino LLC
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

"""Check VIOTA prefix counts when O3 executes its pieces out of order."""
import inspect
import sys
from pathlib import Path

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic import (
    private_l1_private_l2_cache_hierarchy as l1_l2,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.simulator import Simulator

processor_args = dict(cpu_type=CPUTypes.O3, isa=ISA.RISCV, num_cores=1)
if "clk_freq" in inspect.signature(SimpleProcessor).parameters:
    processor_args["clk_freq"] = "1GHz"
processor = SimpleProcessor(**processor_args)
processor.cores[0].core.isa[0].vlen = 256
board = SimpleBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=SingleChannelDDR3_1600(size="512MiB"),
    cache_hierarchy=l1_l2.PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB"
    ),
)
binary = (
    Path(__file__).resolve().parents[3]
    / "test-progs/viota/bin/riscv/linux/viota"
)
board.set_se_binary_workload(BinaryResource(local_path=str(binary)))
simulator = Simulator(board=board)
simulator.run(max_ticks=10_000_000_000)
assert simulator.get_last_exit_event_cause() == (
    "exiting with last active thread context"
)
sys.exit(simulator.get_last_exit_event_code())
