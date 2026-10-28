# Copyright (c) 2026 ZhangJian
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

import os

import m5
from m5.objects import *

# Requestor runs on event queue 0.
try:
    cpu = TrafficGen(
        config_file=os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "tgen-simple-mem.cfg",
        ),
        eventq_index=0,
    )
except NameError:
    m5.fatal("protobuf required for timing thread bridge test")

# Responder runs on event queue 1.
physmem = SimpleMemory(eventq_index=1)

# Bridge response side (in_port) runs on queue 0, request side (out_port)
# runs on queue 1. At 1GHz one cycle equals the 1ns simulation quantum.
bridge = TimingThreadBridge(eventq_index=0, peer_queue=1, delay=1)

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
exit_event = m5.simulate(1000000000)
if exit_event.getCause() != "simulate() limit reached":
    exit(1)
