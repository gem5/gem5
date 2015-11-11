# Copyright (c) 2010-2013 ARM Limited
# Copyright (c) 2015 Min Cai
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
from m5.util import *

addToPath('../common')

from ccnuma_FSConfig import *
from SysPaths import *
from Benchmarks import *
import Simulation
import ccnuma_CacheConfig
import ccnuma_MemConfig
from Caches import *
import Options


# Check if KVM support has been enabled, we might need to do VM
# configuration if that's the case.
have_kvm_support = 'BaseKvmCPU' in globals()
def is_kvm_cpu(cpu_class):
    return have_kvm_support and cpu_class != None and \
        issubclass(cpu_class, BaseKvmCPU)

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

class NUMACache(Cache):
    assoc = 8
    hit_latency = 50
    response_latency = 50
    mshrs = 20
    size = '1kB'
    tgts_per_mshr = 12

class Domain:
    def __init__(self, id = -1):
        self.id = id

        self.mem_ranges = [AddrRange(Addr(str(self.id * 512) + 'MB'), size = options.mem_size_per_domain)] # TODO: 512 should not be hardcoded

        self.membus = MemBus()

def build_test_system(np):
    cmdline = cmd_line_template()
    if buildEnv['TARGET_ISA'] == "alpha":
        test_sys = makeLinuxAlphaSystem(test_mem_mode, options, bm[0],
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

    if options.kernel is not None:
        test_sys.kernel = binary(options.kernel)

    if options.script is not None:
        test_sys.readfile = options.script

    if options.lpae:
        test_sys.have_lpae = True

    if options.virtualisation:
        test_sys.have_virtualization = True

    test_sys.init_param = options.init_param

    domains = [Domain(id = i) for i in range(options.num_domains)]

    # For now, assign all the CPUs to the same clock domain
    for domain in domains:
        domain.cpu = [TestCPUClass(cpu_id = options.num_cpus_per_domain * domain.id + i)
                    for i in range(options.num_cpus_per_domain)]

    numa_caches = []

    mem_ctrls = []
    membus = []

    l2bus = []
    l2cache = []

    cpus = []

    for domain in domains:
        ccnuma_CacheConfig.config_cache(options, test_sys, domain)
        ccnuma_MemConfig.config_mem(options, test_sys, domain)

        test_sys.mem_ranges += domain.mem_ranges
        mem_ctrls += domain.mem_ctrls
        membus.append(domain.membus)

        domain.tol2bus.clk_domain = test_sys.cpu_clk_domain
        l2bus.append(domain.tol2bus)

        domain.l2.clk_domain = test_sys.cpu_clk_domain
        l2cache.append(domain.l2)

        for cpu in domain.cpu:
            cpu.clk_domain = test_sys.cpu_clk_domain
        cpus += domain.cpu

    for domainX in domains:
        for domainY in domains:
            if domainX.id != domainY.id:
                numa_cache = NUMACache(addr_ranges = domainY.mem_ranges)
                numa_cache.cpu_side = domainX.membus.master
                numa_cache.mem_side = domainY.membus.slave

                if domainY.id == 0:
                    numa_cache.addr_ranges += test_sys.bridge.ranges

                numa_caches.append(numa_cache)

    # By default the IOCache runs at the system clock
    test_sys.iocache = IOCache(addr_ranges = test_sys.mem_ranges)
    test_sys.iocache.cpu_side = test_sys.iobus.master

    test_sys.numa_caches = numa_caches

    test_sys.mem_ctrls = mem_ctrls
    test_sys.membus = membus

    test_sys.iocache.mem_side = test_sys.membus[0].slave
    test_sys.bridge.slave = test_sys.membus[0].master
    test_sys.system_port = test_sys.membus[0].slave

    test_sys.l2bus = l2bus
    test_sys.l2cache = l2cache

    test_sys.cpu = cpus

    for i in xrange(np):
        test_sys.cpu[i].createThreads()

    return test_sys, domains

# Add options
parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

parser.add_option('--num_domains', type='int', default=2, help = 'Number of NUMA domains')
parser.add_option('--num_cpus_per_domain', type='int', default=2, help = 'Number of CPUs per NUMA domain')
parser.add_option('--mem_size_per_domain', type='string', default='512MB', help = 'Memory size per NUMA domain')

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

# Match the memories with the CPUs, based on the options for the test system
TestMemClass = Simulation.setMemClass(options)

if options.benchmark or options.dual:
    print "options.benchmark or options.dual are not supported. "
    sys.exit(1)
else:
    bm = [SysConfig(disk=options.disk_image, rootdev=options.root_device,
                mem=-1, os_type=options.os_type)]

np = options.num_cpus

test_sys, domains = build_test_system(np)

def addr_range_to_str(range):
    return '[0x%x:0x%x]' % (range.start, range.end)

def print_addr_ranges(label, ranges):
    print label
    for range in ranges:
        print addr_range_to_str(range)
    print ''

for domain in domains:
    for cpu in domain.cpu:
        print_addr_ranges('cpu#' + str(cpu.cpu_id) + '.icache.addr_ranges: ', cpu.icache.addr_ranges)
        print_addr_ranges('cpu#' + str(cpu.cpu_id) + '.dcache.addr_ranges: ', cpu.dcache.addr_ranges)
    print_addr_ranges('domain#' + str(domain.id) + '.l2.addr_ranges: ', domain.l2.addr_ranges)

    print 'domain#' + str(domain.id) + '.mem_ctrls.ranges: '
    for mem_ctrl in domain.mem_ctrls:
        print 'mem_ctrl.range: ' + addr_range_to_str(mem_ctrl.range)
    print ''

for numa_cache in test_sys.numa_caches:
    print_addr_ranges('numa_cache#' + numa_cache.get_name() + '.addr_ranges: ', numa_cache.addr_ranges)

print_addr_ranges('test_sys.bridge.ranges: ', test_sys.bridge.ranges)

print_addr_ranges('test_sys.iocache.addr_ranges: ', test_sys.iocache.addr_ranges)

if len(bm) != 1:
    print "Error I only know how to create one system."
    sys.exit(1)
else:
    root = Root(full_system=True, system=test_sys)

if options.timesync:
    root.time_sync_enable = True

if options.frame_capture:
    VncServer.frame_capture = True

Simulation.setWorkCountOptions(test_sys, options)
Simulation.run(options, root, test_sys, FutureClass)
