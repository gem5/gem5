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
from m5.util import addToPath, fatal

addToPath('../common')

from Caches import *

from optparse import OptionParser

parser = OptionParser()
parser.add_option('--l1i_size', type='string', default='32kB', help = 'L1 instruction cache size')
parser.add_option('--l1d_size', type='string', default='64kB', help = 'L1 data cache size')
parser.add_option('--l2_size', type='string', default='512kB', help = 'L2 cache size')
parser.add_option('--num_domains', type='int', default=2, help = 'Number of NUMA domains')
parser.add_option('--num_cpus_per_domain', type='int', default=2, help = 'Number of CPUs per NUMA domain')
parser.add_option('--mem_size_per_domain', type='string', default='512MB', help = 'Memory size per NUMA domain')

(option, arg) = parser.parse_args()

num_domains = option.num_domains

num_cpus_per_domain = option.num_cpus_per_domain

isa = str(m5.defines.buildEnv['TARGET_ISA']).lower()

binary = 'tests/test-progs/hello/bin/' + isa + '/linux/hello' #TODO: binary should not be hardcoded

class Domain:
    def __init__(self, id = -1):
        self.id = id

        self.mem_ranges = AddrRange(Addr(str(self.id * 512) + 'MB'), size = option.mem_size_per_domain) # TODO: 512 should not be hardcoded

        self.membus = SystemXBar()

        self.mem_ctrl = DDR3_1600_x64(range = self.mem_ranges, port = self.membus.master)

        self.l2bus = L2XBar()

        self.l2cache = L2Cache(size = option.l2_size)
        self.l2cache.cpu_side = self.l2bus.master
        self.l2cache.mem_side = self.membus.slave

        self.cpu = [TimingSimpleCPU(cpu_id = num_cpus_per_domain * self.id + i,
                                     icache = L1_ICache(size = option.l1i_size), dcache = L1_DCache(size = option.l1d_size))
                     for i in range(num_cpus_per_domain)]

        for cpu in self.cpu:
            cpu.icache.cpu_side = cpu.icache_port
            cpu.icache.mem_side = self.l2bus.slave

            cpu.dcache.cpu_side = cpu.dcache_port
            cpu.dcache.mem_side = self.l2bus.slave

            cpu.createInterruptController()

            if isa == 'x86':
                cpu.interrupts.pio = self.membus.master
                cpu.interrupts.int_master = self.membus.slave
                cpu.interrupts.int_slave = self.membus.master

            cpu.workload = LiveProcess(cmd = [binary])
            cpu.createThreads()

system = System()

system.clk_domain = SrcClockDomain(clock = '1GHz', voltage_domain = VoltageDomain())

system.mem_mode = 'timing'

domains = [Domain(id = i) for i in range(num_domains)]

system.systembus = SystemXBar()

numa_cache_downward = []
numa_cache_upward = []
mem_ctrl = []
membus = []
l2bus = []
l2cache = []
cpu = []

for domain in domains:
    domain.numa_cache_downward = IOCache()
    domain.numa_cache_upward = IOCache()

    domain.numa_cache_downward.addr_ranges = []
    domain.numa_cache_upward.addr_ranges = []

    for r in range(num_domains):
        addr_range = AddrRange(Addr(str(r * 512) + 'MB'), size = option.mem_size_per_domain) # TODO: 512 should not be hardcoded
        if r != domain.id:
            domain.numa_cache_downward.addr_ranges.append(addr_range)
        else:
            domain.numa_cache_upward.addr_ranges.append(addr_range)

    domain.numa_cache_downward.cpu_side = domain.membus.master
    domain.numa_cache_downward.mem_side = system.systembus.slave

    domain.numa_cache_upward.cpu_side = system.systembus.master
    domain.numa_cache_upward.mem_side = domain.membus.slave

    numa_cache_downward.append(domain.numa_cache_downward)
    numa_cache_upward.append(domain.numa_cache_upward)

    system.mem_ranges.append(domain.mem_ranges)

    mem_ctrl.append(domain.mem_ctrl)

    membus.append(domain.membus)

    l2bus.append(domain.l2bus)

    l2cache.append(domain.l2cache)

    cpu += domain.cpu

system.numa_cache_downward = numa_cache_downward
system.numa_cache_upward = numa_cache_upward
system.mem_ctrl = mem_ctrl
system.membus = membus
system.l2bus = l2bus
system.l2cache = l2cache
system.cpu = cpu

system.system_port = system.systembus.slave

root = Root(full_system = False, system = system)

m5.instantiate()

exit_event = m5.simulate()
print 'Exiting @ tick ' + str(m5.curTick()) + ' because ' + exit_event.getCause()