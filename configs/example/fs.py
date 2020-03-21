# Copyright (c) 2010-2013, 2016, 2019 ARM Limited
# Copyright (c) 2020 Barkhausen Institut
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

from __future__ import print_function
from __future__ import absolute_import

import optparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *

addToPath('../')

from ruby import Ruby

from common.FSConfig import *
from common.SysPaths import *
from common.Benchmarks import *
from common import Simulation
from common import CacheConfig
from common import CpuConfig
from common import MemConfig
from common import ObjectList
from common.Caches import *
from common import Options

def cmd_line_template():
    if options.command_line and options.command_line_file:
        print("Error: --command-line and --command-line-file are "
              "mutually exclusive")
        sys.exit(1)
    if options.command_line:
        return options.command_line
    if options.command_line_file:
        return open(options.command_line_file).read().strip()
    return None

def build_test_system(np):
    cmdline = cmd_line_template()
    if buildEnv['TARGET_ISA'] == "mips":
        test_sys = makeLinuxMipsSystem(test_mem_mode, bm[0], cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == "sparc":
        test_sys = makeSparcSystem(test_mem_mode, bm[0], cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == "riscv":
        test_sys = makeBareMetalRiscvSystem(test_mem_mode, bm[0],
                                            cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == "x86":
        test_sys = makeLinuxX86System(test_mem_mode, np, bm[0], options.ruby,
                                      cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == "arm":
        test_sys = makeArmSystem(
            test_mem_mode,
            options.machine_type,
            np,
            bm[0],
            options.dtb_filename,
            bare_metal=options.bare_metal,
            cmdline=cmdline,
            external_memory=options.external_memory_system,
            ruby=options.ruby,
            security=options.enable_security_extensions,
            vio_9p=options.vio_9p,
            bootloader=options.bootloader,
        )
        if options.enable_context_switch_stats_dump:
            test_sys.enable_context_switch_stats_dump = True
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

    if buildEnv['TARGET_ISA'] == 'riscv':
        test_sys.workload.bootloader = options.kernel
    elif options.kernel is not None:
        test_sys.workload.object_file = binary(options.kernel)

    if options.script is not None:
        test_sys.readfile = options.script

    if options.lpae:
        test_sys.have_lpae = True

    if options.virtualisation:
        test_sys.have_virtualization = True

    test_sys.init_param = options.init_param

    # For now, assign all the CPUs to the same clock domain
    test_sys.cpu = [TestCPUClass(clk_domain=test_sys.cpu_clk_domain, cpu_id=i)
                    for i in range(np)]

    if ObjectList.is_kvm_cpu(TestCPUClass) or \
        ObjectList.is_kvm_cpu(FutureClass):
        test_sys.kvm_vm = KvmVM()

    if options.ruby:
        bootmem = getattr(test_sys, '_bootmem', None)
        Ruby.create_system(options, True, test_sys, test_sys.iobus,
                           test_sys._dma_ports, bootmem)

        # Create a seperate clock domain for Ruby
        test_sys.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                        voltage_domain = test_sys.voltage_domain)

        # Connect the ruby io port to the PIO bus,
        # assuming that there is just one such port.
        test_sys.iobus.master = test_sys.ruby._io_port.slave

        for (i, cpu) in enumerate(test_sys.cpu):
            #
            # Tie the cpu ports to the correct ruby system ports
            #
            cpu.clk_domain = test_sys.cpu_clk_domain
            cpu.createThreads()
            cpu.createInterruptController()

            cpu.icache_port = test_sys.ruby._cpu_ports[i].slave
            cpu.dcache_port = test_sys.ruby._cpu_ports[i].slave

            if buildEnv['TARGET_ISA'] in ("x86", "arm"):
                cpu.itb.walker.port = test_sys.ruby._cpu_ports[i].slave
                cpu.dtb.walker.port = test_sys.ruby._cpu_ports[i].slave

            if buildEnv['TARGET_ISA'] in "x86":
                cpu.interrupts[0].pio = test_sys.ruby._cpu_ports[i].master
                cpu.interrupts[0].int_master = test_sys.ruby._cpu_ports[i].slave
                cpu.interrupts[0].int_slave = test_sys.ruby._cpu_ports[i].master

    else:
        if options.caches or options.l2cache:
            # By default the IOCache runs at the system clock
            test_sys.iocache = IOCache(addr_ranges = test_sys.mem_ranges)
            test_sys.iocache.cpu_side = test_sys.iobus.master
            test_sys.iocache.mem_side = test_sys.membus.slave
        elif not options.external_memory_system:
            test_sys.iobridge = Bridge(delay='50ns', ranges = test_sys.mem_ranges)
            test_sys.iobridge.slave = test_sys.iobus.master
            test_sys.iobridge.master = test_sys.membus.slave

        # Sanity check
        if options.simpoint_profile:
            if not ObjectList.is_noncaching_cpu(TestCPUClass):
                fatal("SimPoint generation should be done with atomic cpu")
            if np > 1:
                fatal("SimPoint generation not supported with more than one CPUs")

        for i in range(np):
            if options.simpoint_profile:
                test_sys.cpu[i].addSimPointProbe(options.simpoint_interval)
            if options.checker:
                test_sys.cpu[i].addCheckerCpu()
            if not ObjectList.is_kvm_cpu(TestCPUClass):
                if options.bp_type:
                    bpClass = ObjectList.bp_list.get(options.bp_type)
                    test_sys.cpu[i].branchPred = bpClass()
                if options.indirect_bp_type:
                    IndirectBPClass = ObjectList.indirect_bp_list.get(
                        options.indirect_bp_type)
                    test_sys.cpu[i].branchPred.indirectBranchPred = \
                        IndirectBPClass()
            test_sys.cpu[i].createThreads()

        # If elastic tracing is enabled when not restoring from checkpoint and
        # when not fast forwarding using the atomic cpu, then check that the
        # TestCPUClass is DerivO3CPU or inherits from DerivO3CPU. If the check
        # passes then attach the elastic trace probe.
        # If restoring from checkpoint or fast forwarding, the code that does this for
        # FutureCPUClass is in the Simulation module. If the check passes then the
        # elastic trace probe is attached to the switch CPUs.
        if options.elastic_trace_en and options.checkpoint_restore == None and \
            not options.fast_forward:
            CpuConfig.config_etrace(TestCPUClass, test_sys.cpu, options)

        CacheConfig.config_cache(options, test_sys)

        MemConfig.config_mem(options, test_sys)

    return test_sys

def build_drive_system(np):
    # driver system CPU is always simple, so is the memory
    # Note this is an assignment of a class, not an instance.
    DriveCPUClass = AtomicSimpleCPU
    drive_mem_mode = 'atomic'
    DriveMemClass = SimpleMemory

    cmdline = cmd_line_template()
    if buildEnv['TARGET_ISA'] == 'mips':
        drive_sys = makeLinuxMipsSystem(drive_mem_mode, bm[1], cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == 'sparc':
        drive_sys = makeSparcSystem(drive_mem_mode, bm[1], cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == 'x86':
        drive_sys = makeLinuxX86System(drive_mem_mode, np, bm[1],
                                       cmdline=cmdline)
    elif buildEnv['TARGET_ISA'] == 'arm':
        drive_sys = makeArmSystem(drive_mem_mode, options.machine_type, np,
                                  bm[1], options.dtb_filename, cmdline=cmdline)

    # Create a top-level voltage domain
    drive_sys.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

    # Create a source clock for the system and set the clock period
    drive_sys.clk_domain = SrcClockDomain(clock =  options.sys_clock,
            voltage_domain = drive_sys.voltage_domain)

    # Create a CPU voltage domain
    drive_sys.cpu_voltage_domain = VoltageDomain()

    # Create a source clock for the CPUs and set the clock period
    drive_sys.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                              voltage_domain =
                                              drive_sys.cpu_voltage_domain)

    drive_sys.cpu = DriveCPUClass(clk_domain=drive_sys.cpu_clk_domain,
                                  cpu_id=0)
    drive_sys.cpu.createThreads()
    drive_sys.cpu.createInterruptController()
    drive_sys.cpu.connectAllPorts(drive_sys.membus)
    if options.kernel is not None:
        drive_sys.workload.object_file = binary(options.kernel)

    if ObjectList.is_kvm_cpu(DriveCPUClass):
        drive_sys.kvm_vm = KvmVM()

    drive_sys.iobridge = Bridge(delay='50ns',
                                ranges = drive_sys.mem_ranges)
    drive_sys.iobridge.slave = drive_sys.iobus.master
    drive_sys.iobridge.master = drive_sys.membus.slave

    # Create the appropriate memory controllers and connect them to the
    # memory bus
    drive_sys.mem_ctrls = [DriveMemClass(range = r)
                           for r in drive_sys.mem_ranges]
    for i in range(len(drive_sys.mem_ctrls)):
        drive_sys.mem_ctrls[i].port = drive_sys.membus.master

    drive_sys.init_param = options.init_param

    return drive_sys

# Add options
parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

# Add the ruby specific and protocol specific options
if '--ruby' in sys.argv:
    Ruby.define_options(parser)

(options, args) = parser.parse_args()

if args:
    print("Error: script doesn't take any positional arguments")
    sys.exit(1)

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

# Match the memories with the CPUs, based on the options for the test system
TestMemClass = Simulation.setMemClass(options)

if options.benchmark:
    try:
        bm = Benchmarks[options.benchmark]
    except KeyError:
        print("Error benchmark %s has not been defined." % options.benchmark)
        print("Valid benchmarks are: %s" % DefinedBenchmarks)
        sys.exit(1)
else:
    if options.dual:
        bm = [SysConfig(disks=options.disk_image, rootdev=options.root_device,
                        mem=options.mem_size, os_type=options.os_type),
              SysConfig(disks=options.disk_image, rootdev=options.root_device,
                        mem=options.mem_size, os_type=options.os_type)]
    else:
        bm = [SysConfig(disks=options.disk_image, rootdev=options.root_device,
                        mem=options.mem_size, os_type=options.os_type)]

np = options.num_cpus

test_sys = build_test_system(np)
if len(bm) == 2:
    drive_sys = build_drive_system(np)
    root = makeDualRoot(True, test_sys, drive_sys, options.etherdump)
elif len(bm) == 1 and options.dist:
    # This system is part of a dist-gem5 simulation
    root = makeDistRoot(test_sys,
                        options.dist_rank,
                        options.dist_size,
                        options.dist_server_name,
                        options.dist_server_port,
                        options.dist_sync_repeat,
                        options.dist_sync_start,
                        options.ethernet_linkspeed,
                        options.ethernet_linkdelay,
                        options.etherdump);
elif len(bm) == 1:
    root = Root(full_system=True, system=test_sys)
else:
    print("Error I don't know how to create more than 2 systems.")
    sys.exit(1)

if options.timesync:
    root.time_sync_enable = True

if options.frame_capture:
    VncServer.frame_capture = True

if buildEnv['TARGET_ISA'] == "arm" and not options.bare_metal \
        and not options.dtb_filename:
    if options.machine_type not in ["VExpress_GEM5", "VExpress_GEM5_V1"]:
        warn("Can only correctly generate a dtb for VExpress_GEM5_V1 " \
             "platforms, unless custom hardware models have been equipped "\
             "with generation functionality.")

    # Generate a Device Tree
    for sysname in ('system', 'testsys', 'drivesys'):
        if hasattr(root, sysname):
            sys = getattr(root, sysname)
            sys.workload.dtb_filename = \
                os.path.join(m5.options.outdir, '%s.dtb' % sysname)
            sys.generateDtb(sys.workload.dtb_filename)

Simulation.setWorkCountOptions(test_sys, options)
Simulation.run(options, root, test_sys, FutureClass)
