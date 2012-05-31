# Copyright (c) 2011 ARM Limited
# All rights reserved
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
#
# Authors: Geoffrey Blake

import m5
from m5.objects import *
m5.util.addToPath('../configs/common')
import FSConfig


# --------------------
# Base L1 Cache
# ====================

class L1(BaseCache):
    latency = '1ns'
    block_size = 64
    mshrs = 4
    tgts_per_mshr = 20
    is_top_level = True

# ----------------------
# Base L2 Cache
# ----------------------

class L2(BaseCache):
    block_size = 64
    latency = '10ns'
    mshrs = 92
    tgts_per_mshr = 16
    write_buffers = 8

# ---------------------
# I/O Cache
# ---------------------
class IOCache(BaseCache):
    assoc = 8
    block_size = 64
    latency = '50ns'
    mshrs = 20
    size = '1kB'
    tgts_per_mshr = 12
    addr_ranges = [AddrRange(0, size='256MB')]
    forward_snoops = False

#cpu
cpu = DerivO3CPU(cpu_id=0)
#the system
system = FSConfig.makeArmSystem('timing', "RealView_PBX", None, False)

system.cpu = cpu
#create the l1/l2 bus
system.toL2Bus = CoherentBus()
system.iocache = IOCache()
system.iocache.cpu_side = system.iobus.master
system.iocache.mem_side = system.membus.slave


#connect up the l2 cache
system.l2c = L2(size='4MB', assoc=8)
system.l2c.cpu_side = system.toL2Bus.master
system.l2c.mem_side = system.membus.slave

#connect up the checker
cpu.addCheckerCpu()
#connect up the cpu and l1s
cpu.createInterruptController()
cpu.addPrivateSplitL1Caches(L1(size = '32kB', assoc = 1),
                            L1(size = '32kB', assoc = 4))
# connect cpu level-1 caches to shared level-2 cache
cpu.connectAllPorts(system.toL2Bus, system.membus)
cpu.clock = '2GHz'

root = Root(full_system=True, system=system)
m5.ticks.setGlobalFrequency('1THz')

