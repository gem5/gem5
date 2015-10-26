# Copyright (c) 2015 Min Cai
# Copyright (c) 2010-2013 ARM Limited
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
# Copyright (c) 2012-2014 Mark D. Hill and David A. Wood
# Copyright (c) 2009-2011 Advanced Micro Devices, Inc.
# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Ali Saidi
#          Brad Beckmann
#          Min Cai

import optparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')

from ccnuma_FSConfig import *
from SysPaths import *
from Benchmarks import *
import Simulation
import ccnuma_CacheConfig
import ccnuma_MemConfig
from Caches import *
import Options
from optparse import OptionParser

def cmd_line_template():
    if options.command_line and options.command_line_file:
        print "Error: --command-line and --command-line-file are " \
              "mutually exclusive"
        sys.exit(1)
    if options.command_line:
        return options.command_line
    if options.command_line_file:
        return open(options.command_line_file).read().strip()
    return None

class Domain:
    def __init__(self, id = -1):
        self.id = id

        self.mem_ranges = AddrRange(Addr(str(self.id * 512) + 'MB'), size = options.mem_size_per_domain) # TODO: 512 should not be hardcoded

        self.membus = SystemXBar()

        self.mem_ctrl = DDR3_1600_x64(range = self.mem_ranges, port = self.membus.master)

        self.l2bus = L2XBar()

        self.l2cache = L2Cache(size = options.l2_size)
        self.l2cache.cpu_side = self.l2bus.master
        self.l2cache.mem_side = self.membus.slave

        self.system_port = self.membus.slave

        self.cpu = [TimingSimpleCPU(cpu_id = num_cpus_per_domain * self.id + i,
                                     icache = L1_ICache(size = options.l1i_size), dcache = L1_DCache(size = options.l1d_size))
                     for i in range(num_cpus_per_domain)]

        for cpu in self.cpu:
            cpu.icache.cpu_side = cpu.icache_port
            cpu.icache.mem_side = self.l2bus.slave

            cpu.dcache.cpu_side = cpu.dcache_port
            cpu.dcache.mem_side = self.l2bus.slave

            cpu.createThreads()
            cpu.createInterruptController()

def build_test_system(np):
    cmdline = cmd_line_template()
    if buildEnv['TARGET_ISA'] == "alpha":
        test_sys = makeLinuxAlphaSystem(test_mem_mode, bm[0], options.ruby,
                                        cmdline=cmdline)
    else:
        fatal("Incapable of building %s full system!", buildEnv['TARGET_ISA'])

    # Set the cache line size for the entire system
    test_sys.cache_line_size = options.cacheline_size

    # Create a top-level voltage domain
    test_sys.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

    # Create a source clock for the system and set the clock period
    test_sys.clk_domain = SrcClockDomain(clock =  options.sys_clock,
            voltage_domain = test_sys.voltage_domain)

    # Create a CPU voltage domain
    test_sys.cpu_voltage_domain = VoltageDomain()

    # Create a source clock for the CPUs and set the clock period
    test_sys.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                             voltage_domain =
                                             test_sys.cpu_voltage_domain)

    domains = [Domain(id = i) for i in range(num_domains)]

    test_sys.systembus = SystemXBar()

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
            addr_range = AddrRange(Addr(str(r * 512) + 'MB'), size = options.mem_size_per_domain) # TODO: 512 should not be hardcoded
            if r != domain.id:
                domain.numa_cache_downward.addr_ranges.append(addr_range)
            else:
                domain.numa_cache_upward.addr_ranges.append(addr_range)

    #     domain.numa_cache_downward.cpu_side = domain.membus.master
    #     domain.numa_cache_downward.mem_side = test_sys.systembus.slave
    #
    #     domain.numa_cache_upward.cpu_side = test_sys.systembus.master
    #     domain.numa_cache_upward.mem_side = domain.membus.slave
    #
    #     numa_cache_downward.append(domain.numa_cache_downward)
    #     numa_cache_upward.append(domain.numa_cache_upward)

        test_sys.mem_ranges.append(domain.mem_ranges)

        mem_ctrl.append(domain.mem_ctrl)

        membus.append(domain.membus)

        l2bus.append(domain.l2bus)

        l2cache.append(domain.l2cache)

        cpu += domain.cpu

    # test_sys.numa_cache_downward = numa_cache_downward
    # test_sys.numa_cache_upward = numa_cache_upward
    test_sys.mem_ctrl = mem_ctrl
    test_sys.membus = membus
    test_sys.l2bus = l2bus
    test_sys.l2cache = l2cache
    test_sys.cpu = cpu
    
    if options.kernel is not None:
        test_sys.kernel = binary(options.kernel)

    if options.script is not None:
        test_sys.readfile = options.script

    if options.lpae:
        test_sys.have_lpae = True

    if options.virtualisation:
        test_sys.have_virtualization = True

    test_sys.init_param = options.init_param

    # For now, assign all the CPUs to the same clock domain
    test_sys.cpu = [TestCPUClass(clk_domain=test_sys.cpu_clk_domain, cpu_id=i)
                    for i in xrange(np)] #TODO:NUMA

    # By default the IOCache runs at the system clock
    test_sys.iocache = IOCache(addr_ranges = test_sys.mem_ranges) #TODO:NUMA
    test_sys.iocache.cpu_side = test_sys.iobus.master
    test_sys.iocache.mem_side = test_sys.membus.slave

    for i in xrange(np):
        test_sys.cpu[i].createThreads()

    ccnuma_CacheConfig.config_cache(options, test_sys)
    ccnuma_MemConfig.config_mem(options, test_sys)

    return test_sys

# Add options
parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

# parser.add_option('--l1i_size', type='string', default='32kB', help = 'L1 instruction cache size')
# parser.add_option('--l1d_size', type='string', default='64kB', help = 'L1 data cache size')
# parser.add_option('--l2_size', type='string', default='512kB', help = 'L2 cache size')
parser.add_option('--num_domains', type='int', default=2, help = 'Number of NUMA domains')
parser.add_option('--num_cpus_per_domain', type='int', default=2, help = 'Number of CPUs per NUMA domain')
parser.add_option('--mem_size_per_domain', type='string', default='512MB', help = 'Memory size per NUMA domain')

(options, args) = parser.parse_args()

num_domains = options.num_domains
num_cpus_per_domain = options.num_cpus_per_domain

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

# Match the memories with the CPUs, based on the options for the test system
TestMemClass = Simulation.setMemClass(options)

bm = [SysConfig(disk=options.disk_image, rootdev=options.root_device,
                mem=options.mem_size, os_type=options.os_type)]

np = options.num_cpus

test_sys = build_test_system(np)
root = Root(full_system=True, system=test_sys)

if options.timesync:
    root.time_sync_enable = True

if options.frame_capture:
    VncServer.frame_capture = True

Simulation.setWorkCountOptions(test_sys, options)
Simulation.run(options, root, test_sys, FutureClass)
