# Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
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

# Native gem5 requestors exercise functional transfers without
# advancing simulated time or using a custom service hook.

import argparse
import json

import m5
from m5.objects import (
    AddrRange,
    Bridge,
    NoncoherentXBar,
    PyTrafficGen,
    RegisterBank,
    Root,
    SALAMPowerModel,
    ScratchpadMemory,
    SrcClockDomain,
    System,
    VoltageDomain,
)

parser = argparse.ArgumentParser(
    description="Exercise storage traffic and power accounting."
)
parser.add_argument("parameters", help="JSON storage-test parameters")
args = parser.parse_args()
with open(args.parameters) as stream:
    settings = json.load(stream)

system = System(mem_mode="timing", mem_ranges=[AddrRange(4096)])
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=VoltageDomain()
)
system.bus = NoncoherentXBar(
    width=1,
    frontend_latency=0,
    forward_latency=0,
    response_latency=1,
    header_latency=1,
)
model = SALAMPowerModel(**settings["model"])
if settings["kind"] == "spm":
    system.memory = ScratchpadMemory(
        range=system.mem_ranges[0],
        access_cycles=settings.get("access_cycles", 10),
        bandwidth=settings["bandwidth"],
        ppa=model,
    )
    system.memory.port = system.bus.mem_side_ports
else:
    system.memory = RegisterBank(
        range=system.mem_ranges[0],
        access_cycles=10,
        write_cycles=10,
        ppa=model,
    )
    # The private register interface can answer reads synchronously.
    # A standard bridge queues forwarding until the crossbar has recorded
    # its response route; this is the fixture topology, not a model delay.
    system.bridge = Bridge(ranges=system.mem_ranges, delay="0ns")
    system.bridge.cpu_side_port = system.bus.mem_side_ports
    system.bridge.mem_side_port = system.memory.reg_port
system.system_port = system.bus.cpu_side_ports
system.generators = [
    PyTrafficGen(elastic_req=True) for _ in range(settings["requestors"])
]
for gen in system.generators:
    gen.port = system.bus.cpu_side_ports
root = Root(full_system=False, system=system)
m5.instantiate()
proxy = system.getCCObject().physProxy
proxy.write(2048, b"functional-only")
assert proxy.read(2048, 15) == b"functional-only"
proxy.write(0, bytes(4096))
# Global dumps at tick zero and repeated ticks are suppressed by the
# Python statistics wrapper. Explicit-root snapshots preserve those
# zero-duration checks without adding any simulated delay.
root.preDumpStats()
m5.stats.prepare()
m5.stats.dump(roots=[root])

for index, gen in enumerate(system.generators):
    base = index * 256

    def traffic(gen=gen, base=base):
        # Three one-byte writes, two unaligned four-byte writes,
        # and four aligned four-byte reads. Data limits are exact.
        yield gen.createLinear(1000000, base + 3, base + 6, 1, 1, 1, 0, 3)
        yield gen.createLinear(1000000, base + 9, base + 17, 4, 1, 1, 0, 8)
        yield gen.createLinear(1000000, base, base + 16, 4, 1, 1, 100, 16)
        yield gen.createIdle(10000000)

    gen.start(traffic())
event = m5.simulate(1000000)
assert event.getCause() == "simulate() limit reached"
m5.stats.dump()
if settings["reset"]:
    m5.stats.reset()
event = m5.simulate(2000000)
assert event.getCause() == "simulate() limit reached"
m5.stats.dump()
# TrafficGen fills written bytes with its requestor ID. Check the exact
# affected byte locations and agreement of both writing phases. The DMA
# companion below additionally checks a fixed, independent payload.
for index in range(settings["requestors"]):
    payload = proxy.read(index * 256, 32)
    written = set(range(3, 6)) | set(range(9, 17))
    identity = payload[3]
    assert identity != 0
    assert all(
        value == (identity if pos in written else 0)
        for pos, value in enumerate(payload)
    )
    print("PAYLOAD", index, payload.hex())
proxy.write(2048, b"functional-after-traffic")
assert proxy.read(2048, 24) == b"functional-after-traffic"
m5.stats.dump(roots=[root])
m5.stats.dump(roots=[root])
if settings.get("reset_after_traffic", False):
    m5.stats.reset()
    root.preDumpStats()
    m5.stats.prepare()
    m5.stats.dump(roots=[root])
print("STORAGE_CHECK_PASSED", m5.curTick())
