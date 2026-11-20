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

import argparse
import json

import m5
from m5.objects import (
    AddrRange,
    RegisterBank,
    Root,
    SALAMDeviceTester,
    SALAMPowerModel,
    SimpleMemory,
    SrcClockDomain,
    System,
    VoltageDomain,
)

parser = argparse.ArgumentParser(
    description="Check functional and timing register-port interactions."
)
parser.add_argument("parameters", help="JSON storage-test parameters")
args = parser.parse_args()
with open(args.parameters) as stream:
    settings = json.load(stream)

system = System(mem_mode="timing", mem_ranges=[AddrRange(0x10000, size=4096)])
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=VoltageDomain()
)
system.backing = SimpleMemory(range=system.mem_ranges[0])
system.system_port = system.backing.port
system.memory = RegisterBank(
    range=AddrRange(0, size=4096),
    access_cycles=10,
    write_cycles=30,
    ppa=SALAMPowerModel(**settings["model"]),
)
system.testers = [SALAMDeviceTester(), SALAMDeviceTester()]
system.testers[0].port = system.memory.reg_port
system.testers[1].port = system.memory.load_port
root = Root(full_system=False, system=system)
m5.instantiate()
reg, load = system.testers


def until(tick):
    event = m5.simulate(tick - m5.curTick())
    assert event.getCause() == "simulate() limit reached"
    assert m5.curTick() == tick


def snapshot():
    root.preDumpStats()
    m5.stats.prepare()
    m5.stats.dump(roots=[root])


until(1)
reg.functionalWrite(8, 8, 0x8877665544332211)
assert load.functionalRead(8, 8) == 0x8877665544332211
load.functionalWrite(10, 2, 0xBBAA)
assert reg.functionalRead(8, 8) == 0x88776655BBAA2211
snapshot()
assert load.send(False, 0, 8, 0x0807060504030201)
assert reg.send(True, 0, 8, 0)
assert reg.getResponses()[-1][3] == 0
assert load.send(True, 0, 8, 0)
reg.functionalWrite(2, 2, 0xBBAA)
load.functionalWrite(12, 2, 0xDDCC)
for reader in (reg, load):
    assert reader.functionalRead(0, 8) == 0x08070605BBAA0201
snapshot()
until(11001)
# The read completes at 11000; the write waits until its buffered data
# becomes visible at 31000.
assert [r[0] for r in load.getResponses()] == [11000]
assert load.getResponses()[-1][3] == 0xBBAA0000
reg.blockResponsesUntil(20000)
assert reg.send(True, 0, 8, 0)
assert reg.send(True, 8, 8, 0)
assert reg.getResponseRejections() == [11001]
load.functionalWrite(4, 2, 0xEEFF)
snapshot()
until(20001)
assert [r[0] for r in reg.getResponses()] == [1, 20000, 20000]
assert reg.getResponses()[-2][3] == 0xEEFFBBAA0000
assert reg.getResponses()[-1][3] == 0x8877DDCCBBAA2211
until(31001)
assert [r[0] for r in load.getResponses()] == [11000, 31000]
assert reg.send(True, 0, 8, 0)
assert reg.getResponses()[-1][3] == 0x0807EEFFBBAA0201
for reader in (reg, load):
    assert reader.functionalRead(0, 8) == 0x0807EEFFBBAA0201
    assert reader.functionalRead(8, 8) == 0x8877DDCCBBAA2211
snapshot()
print("PAYLOAD", hex(reg.getResponses()[-1][3]))
print("STORAGE_CHECK_PASSED", m5.curTick())
