# Copyright (c) 2026 Jann Zhang
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

import argparse

import m5
from m5.objects import *

parser = argparse.ArgumentParser(description="TimingThreadBridge tester")
parser.add_argument("--config", required=True, help="TrafficGen config file")
parser.add_argument("--req-size", type=int, default=16)
parser.add_argument("--resp-size", type=int, default=16)
parser.add_argument(
    "--expect-retries",
    action="store_true",
    help="assert that backpressure caused at least one retry",
)
args = parser.parse_args()

# Requestor runs on event queue 0.
try:
    cpu = TrafficGen(config_file=args.config, eventq_index=0)
except NameError:
    m5.fatal("protobuf required for timing thread bridge test")

# Responder runs on event queue 1.
physmem = SimpleMemory(eventq_index=1)

# Bridge response side (in_port) runs on queue 0, request side (out_port)
# runs on queue 1. sim_quantum is in ticks (picoseconds): 1000 ticks == 1 ns.
# At 1 GHz one clock period is also 1 ns, so delay=1 cycle satisfies
# delay * clock_period >= sim_quantum.
bridge = TimingThreadBridge(
    eventq_index=0,
    peer_queue=1,
    delay=1,
    req_size=args.req_size,
    resp_size=args.resp_size,
)

system = System(
    cpu=cpu,
    physmem=physmem,
    membus=IOXBar(width=16),
    clk_domain=SrcClockDomain(clock="1GHz", voltage_domain=VoltageDomain()),
    bridge=bridge,
)

system.cpu.port = system.membus.cpu_side_ports
system.system_port = system.membus.cpu_side_ports
system.membus.mem_side_ports = system.bridge.in_port
system.bridge.out_port = system.physmem.port

root = Root(full_system=False, system=system, sim_quantum=1000)
root.system.mem_mode = "timing"

m5.instantiate()
exit_event = m5.simulate(20000000)
if exit_event.getCause() != "simulate() limit reached":
    exit(1)

issued = system.cpu.resolveStat("numPackets").value
completed_reads = system.cpu.resolveStat("totalReads").value
mem_reads = system.physmem.resolveStat("numReads").total
mem_writes = system.physmem.resolveStat("numWrites").total

assert issued > 0, "no packets were issued"
assert mem_reads > 0, "no read requests crossed the bridge"
assert completed_reads > 0, "no read responses crossed back"
assert mem_writes == 0, "expected a read-only workload"
assert completed_reads <= mem_reads <= issued, (
    f"unexpected request accounting: issued={issued} "
    f"mem_reads={mem_reads} completed_reads={completed_reads}"
)
if args.expect_retries:
    retries = system.cpu.resolveStat("numRetries").value
    assert retries > 0, "expected backpressure retries but saw none"
