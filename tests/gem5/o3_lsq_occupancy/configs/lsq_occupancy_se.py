# Copyright (c) 2026 Barcelona Supercomputing Center
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
Runs an RVV program on an O3 core whose memory instructions occupy a
varying number of LSQ entries, so that they grow and shrink their
occupancy under the given overflow mode.

With ``--drain-from``, the CPU drains ``--drains`` times from that tick,
running ``--drain-every`` ticks between drains, so that it drains with
resized entries and waiting accesses in flight.

With ``--switch-at``, the processor switches to the Timing CPU at that tick
and back to O3 ``--timing-ticks`` later.
"""

import argparse
import sys

import m5
from m5.objects import (
    LSQOpDesc,
    MaxOccupancy,
    PerCacheLineOccupancy,
    PerChunkOccupancy,
)
from m5.objects.FuncUnit import OpClass

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser()
parser.add_argument("resource", type=str)
parser.add_argument(
    "--mode", choices=["Bound", "Overcommit", "Replay"], required=True
)
parser.add_argument(
    "--switch-at", type=int, help="Tick at which to switch to the Timing CPU"
)
parser.add_argument(
    "--timing-ticks",
    type=int,
    default=1_000_000,
    help="Ticks to run on the Timing CPU before switching back to O3",
)
parser.add_argument(
    "--drain-from", type=int, help="Tick at which to start draining the CPU"
)
parser.add_argument(
    "--drains", type=int, default=100, help="Number of times to drain the CPU"
)
parser.add_argument(
    "--drain-every", type=int, default=5000, help="Ticks to run between drains"
)
args = parser.parse_args()

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.O3,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.RISCV,
    num_cores=1,
    clk_freq="3GHz",
)

# Memory instructions reserve 2 entries at dispatch, then occupy one entry
# per cache line or per 8 bytes, whichever is more. Scalar accesses shrink
# to 1 entry and vector accesses grow, with queues small enough to fill up.
memory_op_classes = [
    op_class
    for op_class in OpClass.vals
    if op_class.endswith(("MemRead", "MemWrite", "Load", "Store"))
]
for core in processor.get_cores():
    cpu = core.get_simobject()
    cpu.LQEntries = 8
    cpu.SQEntries = 8
    cpu.lsqOpDescs = [
        LSQOpDesc(
            opClasses=memory_op_classes,
            entries=2,
            policy=MaxOccupancy(
                policies=[
                    PerCacheLineOccupancy(),
                    PerChunkOccupancy(chunkSize="8B"),
                ]
            ),
        )
    ]
    cpu.lsqOverflowMode = args.mode

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=SingleChannelDDR3_1600(size="512MiB"),
    cache_hierarchy=PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB"
    ),
)
board.set_se_binary_workload(obtain_resource(args.resource))

simulator = Simulator(board=board)


def run_until(tick):
    """Runs until tick, failing if the program finishes first."""
    m5.scheduleTickExitAbsolute(tick)
    simulator.run()
    if simulator.get_current_tick() != tick:
        print(f"The program finished before tick {tick}", flush=True)
        sys.exit(1)


if args.drain_from is not None:
    # m5.drain() ignores exit events, so a bounded number of drains keeps
    # the program from finishing during one.
    run_until(args.drain_from)
    for _ in range(args.drains):
        m5.drain()
        run_until(simulator.get_current_tick() + args.drain_every)

if args.switch_at is not None:
    run_until(args.switch_at)
    # Switching drains the O3 CPU, which takes some ticks.
    simulator.switch_processor()
    print(
        f"Switched to Timing at tick {simulator.get_current_tick()}",
        flush=True,
    )
    run_until(simulator.get_current_tick() + args.timing_ticks)
    simulator.switch_processor()
    print(
        f"Switched back to O3 at tick {simulator.get_current_tick()}",
        flush=True,
    )

simulator.run()
print(
    f"Exiting @ tick {simulator.get_current_tick()} because "
    f"{simulator.get_last_exit_event_cause()}"
)
sys.exit(simulator.get_last_exit_event_code())
