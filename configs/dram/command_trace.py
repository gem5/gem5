# Copyright (c) 2026 CkingX256
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

"""Exercise DRAM command observations without an ISA or downloaded workload.

Run with --trace to attach the observer. Compare stats.txt with an otherwise
identical run without --trace. CSV rows are in scheduling order: sort by
(command_at, sequence) when analysing modeled command start times.
"""

import argparse

import m5
from m5.objects import (
    AddrRange,
    DDR3_1600_8x8,
    DramCommandTrace,
    IOXBar,
    MemCtrl,
    PyTrafficGen,
    Root,
    SrcClockDomain,
    System,
    VoltageDomain,
)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--trace", action="store_true")
parser.add_argument("--page-policy", choices=("open", "close"), default="open")
args = parser.parse_args()

system = System(mem_mode="timing", membus=IOXBar(width=32))
system.clk_domain = SrcClockDomain(
    clock="2GHz", voltage_domain=VoltageDomain()
)
system.mem_ranges = [AddrRange("256MiB")]
system.mem_ctrl = MemCtrl(
    dram=DDR3_1600_8x8(
        range=system.mem_ranges[0],
        ranks_per_channel=2,
        page_policy=args.page_policy,
        null=True,
    )
)
system.mem_ctrl.port = system.membus.mem_side_ports
system.generator = PyTrafficGen()
system.generator.port = system.membus.cpu_side_ports
system.system_port = system.membus.cpu_side_ports
if args.trace:
    system.trace = DramCommandTrace(dram=system.mem_ctrl.dram)
root = Root(full_system=False, system=system)
m5.instantiate()


def traffic():
    yield system.generator.createRandom(
        30000000, 0, 256 * 1024 * 1024, 64, 50, 10000, 10000, 0
    )
    yield system.generator.createIdle(1000000)
    yield system.generator.createExit(0)


system.generator.start(traffic())
event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {event.getCause()}")
