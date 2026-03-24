# Copyright (c) 2025 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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


def check_value(name, result, expected):
    if result != expected:
        print(f"Test {name} FAILED")
        exit(1)
    else:
        print(f"Test {name} SUCCESS")


parser = argparse.ArgumentParser(description="Simple PyPort tester")

args = parser.parse_args()

# even if this is only a traffic generator, call it cpu to make sure
# the scripts are happy

# system simulated
system = System(
    physmem=SimpleMemory(range=AddrRange("512MiB")),
    membus=SystemXBar(),
    clk_domain=SrcClockDomain(clock="1GHz", voltage_domain=VoltageDomain()),
)

# connect the system port even if it is not used in this example
system.system_port = system.membus.cpu_side_ports

# connect memory to the membus
system.physmem.port = system.membus.mem_side_ports

# -----------------------
# run simulation
# -----------------------

root = Root(full_system=False, system=system)

m5.instantiate()

# Get a system PyPort
port = root.system.physProxy

# Test a bytearray as write argument
address = 0
ba = bytearray(b"\xaa\xbb\xcc")
port.write(address, ba)
result = port.read(address, len(ba))
check_value("bytearray test", result, ba)

# Test a byte literal as write argument
address = 0
bl = b"\xaa\xbb\xcc"
port.write(address, bl)
result = port.read(address, len(bl))
check_value("byte literal test", result, bl)

# Test a integer converted in byte format
address = 64
value = 35
port.write(address, value.to_bytes())
result = port.read(address, 1)
check_value("byte from integer test", int.from_bytes(result), value)

sys.exit(0)
