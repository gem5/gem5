# Copyright (c) 2026 Akanksha Chaudhari
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


# Focused production-device tests using a scripted ordinary requestor.
import argparse
import json
import os

import m5
from m5.objects import (
    AddrRange,
    RegisterBank,
    Root,
    SALAMDeviceTester,
    ScratchpadMemory,
    SimpleMemory,
    SrcClockDomain,
    StreamBuffer,
    System,
    VoltageDomain,
)

parser = argparse.ArgumentParser()
parser.add_argument("case")
args = parser.parse_args()
system = System()
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=VoltageDomain()
)
system.mem_mode = "timing"
system.mem_ranges = [AddrRange(0x10000, size=4096)]
system.backing = SimpleMemory(range=system.mem_ranges[0])
system.system_port = system.backing.port
system.testers = [SALAMDeviceTester(), SALAMDeviceTester()]
if args.case.startswith("stream-"):
    system.device = StreamBuffer(
        stream_address=0,
        stream_size=8,
        status_address=8,
        buffer_size=16,
        bandwidth="1000GiB/s",
    )
    system.testers[0].port = system.device.stream_out
    system.testers[1].port = system.device.stream_in
elif args.case.startswith("spm-"):
    system.device = ScratchpadMemory(
        range=AddrRange(0, size=4096),
        access_cycles=1,
        access_cycles_var=20 if args.case == "spm-order" else 0,
        bandwidth=(
            "1000000GiB/s" if args.case == "spm-boundaries" else "1000GiB/s"
        ),
    )
    system.testers[0].port = system.device.port
    system.testers[1].port = system.device.spm_ports
else:
    access_cycles = 10
    write_cycles = 10 if args.case == "register-boundaries" else 30
    if args.case.startswith("register-completion-"):
        access_cycles, write_cycles, _, _ = map(int, args.case.split("-")[2:])
    system.device = RegisterBank(
        range=AddrRange(0, size=4096),
        access_cycles=access_cycles,
        write_cycles=write_cycles,
    )
    system.testers[0].port = system.device.reg_port
    system.testers[1].port = system.device.load_port
root = Root(full_system=False, system=system)
m5.instantiate()


def until(tick):
    assert tick > m5.curTick()
    event = m5.simulate(tick - m5.curTick())
    assert event.getCause() == "simulate() limit reached"
    assert m5.curTick() == tick


until(1)
reg, load = system.testers
if args.case.startswith("register-completion-"):
    access, write, writer_index, reader_index = map(
        int, args.case.split("-")[2:]
    )
    writer = system.testers[writer_index]
    reader = system.testers[reader_index]
    value = 0x8877665544332211
    until(1000)
    writer.readOnNextWriteResponse(reader.getCCObject(), 0, 8)
    assert writer.send(False, 0, 8, value)
    write_ready = 1000 + max(access, write) * 1000
    if writer_index:
        # SimpleTimingPort's response queue schedules at least one tick
        # after enqueueing, even with zero modeled access latency.
        write_ready = max(write_ready, 1001)
    read_ready = write_ready
    if reader_index:
        read_ready = max(
            write_ready + 1,
            ((write_ready + 999) // 1000 + access) * 1000,
        )
    until(read_ready + 1)
    observed = {
        "expected_write_tick": write_ready,
        "expected_read_tick": read_ready,
        "expected_value": value,
        "write_responses": writer.getResponses(),
        "read_responses": reader.getResponses(),
    }
    with open(os.path.join(m5.options.outdir, "observation.json"), "w") as f:
        json.dump(observed, f, indent=2)
    assert writer.getResponses()[0][:2] == [write_ready, 0], observed
    assert reader.getResponses()[-1] == [read_ready, 0, 8, value], observed
elif args.case.startswith("register-batch-"):
    writer_index, reader_index = map(int, args.case.split("-")[2:])
    writer = system.testers[writer_index]
    reader = system.testers[reader_index]
    writer.functionalWrite(16, 8, 0x8877665544332211)
    until(1000)
    writer.readOnNextWriteResponse(reader.getCCObject(), 0, 8)
    assert writer.send(False, 0, 8, 0x0807060504030201)
    until(5000)
    assert writer.send(False, 8, 8, 0x1817161514131211)
    # A partial functional update must survive publication of this batch.
    reader.functionalWrite(2, 2, 0xBBAA)
    reader.functionalWrite(18, 2, 0xDDCC)
    until(30000)
    assert not writer.getResponses()
    until(42000)
    write_responses = writer.getResponses()[:2]
    # The queued load port sends at most one response per simulator tick;
    # both writes nevertheless belong to the batch published at 31000.
    second_tick = 31001 if writer_index else 31000
    assert [r[:2] for r in write_responses] == [[31000, 0], [second_tick, 8]]
    read_ready = 41000 if reader_index else 31000
    assert reader.getResponses()[-1] == [read_ready, 0, 8, 0x08070605BBAA0201]
    assert reader.functionalRead(8, 8) == 0x1817161514131211
    assert reader.functionalRead(16, 8) == 0x88776655DDCC2211
elif args.case.startswith("register-write-blocked-"):
    writer_index, reader_index = map(int, args.case.split("-")[3:])
    writer = system.testers[writer_index]
    reader = system.testers[reader_index]
    until(1000)
    writer.blockResponsesUntil(50000)
    writer.readOnNextWriteResponse(reader.getCCObject(), 0, 8)
    assert writer.send(False, 0, 8, 0x0807060504030201)
    until(40000)
    assert not writer.getResponses()
    assert writer.getResponseRejections() == [31000]
    # Functional writes must remain coherent while the completed timing
    # write's response is blocked, including on the other port.
    reader.functionalWrite(2, 2, 0xBBAA)
    until(61000)
    assert writer.getResponses()[0][:2] == [50000, 0]
    read_ready = 60000 if reader_index else 50000
    assert reader.getResponses()[-1] == [read_ready, 0, 8, 0x08070605BBAA0201]
elif args.case == "register-functional":
    for writer, reader in ((reg, load), (load, reg)):
        writer.functionalWrite(8, 8, 0x8877665544332211)
        assert reader.functionalRead(8, 8) == 0x8877665544332211
        writer.functionalWrite(10, 2, 0xBBAA)
        assert reader.functionalRead(8, 8) == 0x88776655BBAA2211
    until(50000)
    assert reg.functionalRead(8, 8) == 0x88776655BBAA2211
    m5.stats.dump()
    with open(os.path.join(m5.options.outdir, "stats.txt")) as stream:
        for line in stream:
            words = line.split()
            if words and words[0].startswith("system.device."):
                if any(
                    name in words[0]
                    for name in (
                        "numReads",
                        "numWrites",
                        "bytesRead",
                        "bytesWritten",
                    )
                ):
                    assert float(words[1]) == 0, line
elif args.case == "register-overlap":
    reg.functionalWrite(8, 8, 0x8877665544332211)
    assert load.send(False, 0, 8, 0x0807060504030201)
    # Timed reads retain the intentional visibility delay.
    assert reg.send(True, 0, 8, 0)
    assert reg.getResponses()[-1][3] == 0
    # Functional reads see the pending latest bytes through either port.
    assert reg.functionalRead(0, 8) == 0x0807060504030201
    assert load.functionalRead(0, 8) == 0x0807060504030201
    assert load.send(True, 0, 8, 0)  # queued old-data read response
    reg.functionalWrite(2, 2, 0xBBAA)
    load.functionalWrite(12, 2, 0xDDCC)
    until(20000)
    reads = [r for r in load.getResponses() if r[1] == 0 and r[3]]
    assert reads[-1][3] == 0xBBAA0000, reads
    assert reg.send(True, 0, 8, 0)
    assert reg.getResponses()[-1][3] == 0xBBAA0000
    until(40000)
    assert reg.send(True, 0, 8, 0)
    assert reg.getResponses()[-1][3] == 0x08070605BBAA0201
    assert load.functionalRead(8, 8) == 0x8877DDCC44332211
elif args.case == "register-boundaries":
    # Empty queue, singleton insertion, and equal-time FIFO order.
    assert reg.send(False, 0, 8, 1)
    assert reg.send(False, 8, 8, 2)
    # Tick 1 precedes the next edge at 1000; add ten access cycles.
    until(11001)
    assert [r[:2] for r in reg.getResponses()] == [[11000, 0], [11000, 8]]
    # A new zero-latency read must move ahead of a delayed write response.
    until(20000)
    assert reg.send(False, 16, 8, 3)
    assert reg.send(True, 24, 8, 0)
    until(20001)
    assert reg.getResponses()[-1][:2] == [20000, 24]
    assert len(reg.getResponses()) == 3
    until(30001)
    assert reg.getResponses()[-1][:2] == [30000, 16]
elif args.case == "spm-boundaries":
    # First request enters an empty queue and responds at its own ready tick.
    until(1000)
    assert reg.send(True, 0, 8, 0)
    until(1500)
    assert not reg.getResponses()
    until(2001)
    assert [r[:2] for r in reg.getResponses()] == [[2000, 0]]
    # Zero transfer duration permits equal-time requests on the same port.
    # These exercise singleton insertion and preserve acceptance order.
    until(3000)
    for address in (8, 16, 24):
        assert reg.send(True, address, 8, 0)
    until(4001)
    assert [r[:2] for r in reg.getResponses()[1:]] == [
        [4000, 8],
        [4000, 16],
        [4000, 24],
    ]
elif args.case == "spm-order":
    for i in range(8):
        until((i + 1) * 1000)
        assert reg.send(True, i * 8, 8, 0)
    until(100000)
    assert len(reg.getResponses()) == 8
elif args.case == "spm-blocked":
    reg.blockResponsesUntil(20000)
    until(1000)
    assert reg.send(True, 0, 8, 0)
    until(3000)
    assert reg.getResponseRejections() == [2000]
    assert load.send(True, 8, 8, 0)
    until(10000)
    assert not load.getResponses()  # must not bypass the blocked head
    assert not reg.getResponses()
    until(21000)
    assert reg.getResponses()[0][0] == 20000
    assert load.getResponses()[0][0] == 20000
elif args.case == "register-blocked":
    reg.functionalWrite(0, 8, 0x1122334455667788)
    reg.blockResponsesUntil(20000)
    assert reg.send(True, 0, 8, 0)
    assert reg.send(True, 8, 8, 0)
    load.functionalWrite(2, 2, 0xBBAA)
    until(10000)
    assert not reg.getResponses()
    assert reg.getResponseRejections() == [1]
    until(21000)
    assert [r[0] for r in reg.getResponses()] == [20000, 20000]
    assert reg.getResponses()[0][3] == 0x11223344BBAA7788
elif args.case == "stream-empty":
    assert not reg.send(True, 0, 8, 0)
    until(1000)
    assert load.send(False, 0, 4, 0x04030201)
    until(2000)
    assert not reg.getRequestRetries()  # only half the requested bytes
    assert load.send(False, 0, 4, 0x08070605)
    assert reg.getRequestRetries() == [2000]  # retry in the write's stack
    until(4000)
    assert reg.getResponses()[0][3] == 0x0807060504030201
    assert len(reg.getAttempts()) == 2
elif args.case == "stream-full":
    load.functionalWrite(0, 8, 0x0807060504030201)
    load.functionalWrite(0, 8, 0x100F0E0D0C0B0A09)
    assert not load.send(False, 0, 8, 0x1817161514131211)
    until(1000)
    assert reg.send(True, 0, 4, 0)
    until(2000)
    assert not load.getRequestRetries()  # only four free bytes
    assert reg.send(True, 0, 4, 0)
    assert load.getRequestRetries() == [2000]
    until(4000)
    assert len(load.getResponses()) == 1
    assert len(load.getAttempts()) == 2
    assert reg.functionalRead(0, 8) == 0x100F0E0D0C0B0A09
    assert reg.functionalRead(0, 8) == 0x1817161514131211
else:
    raise AssertionError(args.case)

records = {
    "case": args.case,
    "responses": [t.getResponses() for t in system.testers],
    "attempts": [t.getAttempts() for t in system.testers],
    "retries": [t.getRequestRetries() for t in system.testers],
    "response_rejections": [t.getResponseRejections() for t in system.testers],
}
with open(os.path.join(m5.options.outdir, "device-results.json"), "w") as f:
    json.dump(records, f, indent=2)
print("DEVICE CHECK PASSED", args.case)
