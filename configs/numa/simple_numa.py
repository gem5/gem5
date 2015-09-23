# Copyright (c) 2015 Min Cai
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
# Authors: Min Cai

import m5
from m5.objects import *

num_domains = 2

num_cpus_per_domain = 2

isa = str(m5.defines.buildEnv['TARGET_ISA']).lower()

binary = 'tests/test-progs/hello/bin/' + isa + '/linux/hello'

class DomainXBar(SystemXBar):
    pass

class SystemXBar(SystemXBar):
    pass

class NUMACache(Cache):
    assoc = 8
    hit_latency = 50
    response_latency = 50
    mshrs = 20
    tgts_per_mshr = 12

class Domain:
    def __init__(self, id = -1):
        self.id = id

        self.mem_ranges = AddrRange(Addr(str(self.id * 512) + 'MB'), size = '512MB')

        self.mem_bus = DomainXBar()

        self.mem_ctrl = DDR3_1600_x64(range = self.mem_ranges, port = self.mem_bus.master)

        self.cpus = [TimingSimpleCPU(cpu_id = num_cpus_per_domain * self.id + i, icache_port = self.mem_bus.slave, dcache_port = self.mem_bus.slave) for i in range(num_cpus_per_domain)]

        for cpu in self.cpus:
            cpu.createInterruptController()

            if isa == 'x86':
                cpu.interrupts.pio = self.mem_bus.master
                cpu.interrupts.int_master = self.mem_bus.slave
                cpu.interrupts.int_slave = self.mem_bus.master

            cpu.workload = LiveProcess(cmd = [binary])
            cpu.createThreads()

system = System()

system.clk_domain = SrcClockDomain(clock = '1GHz', voltage_domain = VoltageDomain())

system.mem_mode = 'timing'

domains = [Domain(id = i) for i in range(num_domains)]

mem_ctrls = []
mem_buses = []
cpus = []

for domain in domains:
    for mem_ctrl in domain.mem_ctrl:
        mem_ctrls.append(mem_ctrl)

    for mem_bus in domain.mem_bus:
        mem_buses.append(mem_bus)

    for cpu in domain.cpus:
        cpus.append(cpu)

system.mem_ctrls = mem_ctrls
system.mem_buses = mem_buses
system.cpus = cpus

system.system_port = domains[0].mem_bus.slave

root = Root(full_system = False, system = system)

m5.instantiate()

exit_event = m5.simulate()
print 'Exiting @ tick ' + str(m5.curTick()) + ' because ' + exit_event.getCause()