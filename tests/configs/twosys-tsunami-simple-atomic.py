# Copyright (c) 2006 The Regents of The University of Michigan
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
#
# Authors: Lisa Hsu

import m5
from m5.objects import *
m5.util.addToPath('../configs/common')
from FSConfig import *
from Benchmarks import *

test_sys = makeLinuxAlphaSystem('atomic',
                                 SysConfig('netperf-stream-client.rcS'))
test_sys.cpu = AtomicSimpleCPU(cpu_id=0)
# create the interrupt controller
test_sys.cpu.createInterruptController()
test_sys.cpu.connectAllPorts(test_sys.membus)
# In contrast to the other (one-system) Tsunami configurations we do
# not have an IO cache but instead rely on an IO bridge for accesses
# from masters on the IO bus to the memory bus
test_sys.iobridge = Bridge(delay='50ns', nack_delay='4ns',
                           ranges = [AddrRange(0, '8GB')])
test_sys.iobridge.slave = test_sys.iobus.master
test_sys.iobridge.master = test_sys.membus.slave

drive_sys = makeLinuxAlphaSystem('atomic',
                                 SysConfig('netperf-server.rcS'))
drive_sys.cpu = AtomicSimpleCPU(cpu_id=0)
# create the interrupt controller
drive_sys.cpu.createInterruptController()
drive_sys.cpu.connectAllPorts(drive_sys.membus)
drive_sys.iobridge = Bridge(delay='50ns', nack_delay='4ns',
                            ranges = [AddrRange(0, '8GB')])
drive_sys.iobridge.slave = drive_sys.iobus.master
drive_sys.iobridge.master = drive_sys.membus.slave

root = makeDualRoot(True, test_sys, drive_sys, "ethertrace")

maxtick = 199999999
