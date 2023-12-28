# Copyright (c) 2010-2013, 2016, 2019-2020 ARM Limited
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

import argparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import (
    addToPath,
    fatal,
    warn,
)
from m5.util.fdthelper import *

from gem5.isas import ISA

addToPath("../../")

from common import (
    CacheConfig,
    CpuConfig,
    MemConfig,
    ObjectList,
    Options,
    Simulation,
)
from common.Benchmarks import *
from common.Caches import *
from common.FSConfig import *
from common.SysPaths import *
from ruby import Ruby


def cmd_line_template():
    if args.command_line and args.command_line_file:
        print(
            "Error: --command-line and --command-line-file are "
            "mutually exclusive"
        )
        sys.exit(1)
    if args.command_line:
        return args.command_line
    if args.command_line_file:
        return open(args.command_line_file).read().strip()
    return None


def build_test_system(np, isa: ISA):
    cmdline = cmd_line_template()
    if isa == ISA.MIPS:
        test_sys = makeLinuxMipsSystem(test_mem_mode, bm[0], cmdline=cmdline)
    elif isa == ISA.SPARC:
        test_sys = makeSparcSystem(test_mem_mode, bm[0], cmdline=cmdline)
    elif isa == ISA.RISCV:
        test_sys = makeBareMetalRiscvSystem(
            test_mem_mode, bm[0], cmdline=cmdline
        )
    elif isa == ISA.X86:
        test_sys = makeLinuxX86System(
            test_mem_mode, np, bm[0], args.ruby, cmdline=cmdline
        )
    elif isa == ISA.ARM:
        test_sys = makeArmSystem(
            test_mem_mode,
            args.machine_type,
            np,
            bm[0],
            args.dtb_filename,
            bare_metal=args.bare_metal,
            cmdline=cmdline,
            external_memory=args.external_memory_system,
            ruby=args.ruby,
            vio_9p=args.vio_9p,
            bootloader=args.bootloader,
        )
        if args.enable_context_switch_stats_dump:
            test_sys.enable_context_switch_stats_dump = True
    else:
        fatal("Incapable of building %s full system!", isa.name)

    # Set the cache line size for the entire system
    test_sys.cache_line_size = args.cacheline_size

    # Create a top-level voltage domain
    test_sys.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

    # Create a source clock for the system and set the clock period
    test_sys.clk_domain = SrcClockDomain(
        clock=args.sys_clock, voltage_domain=test_sys.voltage_domain
    )

    # Create a CPU voltage domain
    test_sys.cpu_voltage_domain = VoltageDomain()

    # Create a source clock for the CPUs and set the clock period
    test_sys.cpu_clk_domain = SrcClockDomain(
        clock=args.cpu_clock, voltage_domain=test_sys.cpu_voltage_domain
    )

    if buildEnv["USE_RISCV_ISA"]:
        test_sys.workload.bootloader = args.kernel
    elif args.kernel is not None:
        test_sys.workload.object_file = binary(args.kernel)

    if args.script is not None:
        test_sys.readfile = args.script

    test_sys.init_param = args.init_param

    # For now, assign all the CPUs to the same clock domain
    test_sys.cpu = [
        TestCPUClass(clk_domain=test_sys.cpu_clk_domain, cpu_id=i)
        for i in range(np)
    ]

    if args.ruby:
        bootmem = getattr(test_sys, "_bootmem", None)
        Ruby.create_system(
            args, True, test_sys, test_sys.iobus, test_sys._dma_ports, bootmem
        )

        # Create a seperate clock domain for Ruby
        test_sys.ruby.clk_domain = SrcClockDomain(
            clock=args.ruby_clock, voltage_domain=test_sys.voltage_domain
        )

        # Connect the ruby io port to the PIO bus,
        # assuming that there is just one such port.
        test_sys.iobus.mem_side_ports = test_sys.ruby._io_port.in_ports

        for i, cpu in enumerate(test_sys.cpu):
            #
            # Tie the cpu ports to the correct ruby system ports
            #
            cpu.clk_domain = test_sys.cpu_clk_domain
            cpu.createThreads()
            cpu.createInterruptController()

            test_sys.ruby._cpu_ports[i].connectCpuPorts(cpu)

    else:
        if args.caches or args.l2cache:
            # By default the IOCache runs at the system clock
            test_sys.iocache = IOCache(addr_ranges=test_sys.mem_ranges)
            test_sys.iocache.cpu_side = test_sys.iobus.mem_side_ports
            test_sys.iocache.mem_side = test_sys.membus.cpu_side_ports
        elif not args.external_memory_system:
            test_sys.iobridge = Bridge(
                delay="50ns", ranges=test_sys.mem_ranges
            )
            test_sys.iobridge.cpu_side_port = test_sys.iobus.mem_side_ports
            test_sys.iobridge.mem_side_port = test_sys.membus.cpu_side_ports

        # Sanity check
        if args.simpoint_profile:
            if not ObjectList.is_noncaching_cpu(TestCPUClass):
                fatal("SimPoint generation should be done with atomic cpu")
            if np > 1:
                fatal(
                    "SimPoint generation not supported with more than one CPUs"
                )

        for i in range(np):
            if args.simpoint_profile:
                test_sys.cpu[i].addSimPointProbe(args.simpoint_interval)
            if args.checker:
                test_sys.cpu[i].addCheckerCpu()
            if not ObjectList.is_kvm_cpu(TestCPUClass):
                if args.bp_type:
                    bpClass = ObjectList.bp_list.get(args.bp_type)
                    test_sys.cpu[i].branchPred = bpClass()
                if args.indirect_bp_type:
                    IndirectBPClass = ObjectList.indirect_bp_list.get(
                        args.indirect_bp_type
                    )
                    test_sys.cpu[
                        i
                    ].branchPred.indirectBranchPred = IndirectBPClass()
            test_sys.cpu[i].createThreads()

        # If elastic tracing is enabled when not restoring from checkpoint and
        # when not fast forwarding using the atomic cpu, then check that the
        # TestCPUClass is DerivO3CPU or inherits from DerivO3CPU. If the check
        # passes then attach the elastic trace probe.
        # If restoring from checkpoint or fast forwarding, the code that does this for
        # FutureCPUClass is in the Simulation module. If the check passes then the
        # elastic trace probe is attached to the switch CPUs.
        if (
            args.elastic_trace_en
            and args.checkpoint_restore == None
            and not args.fast_forward
        ):
            CpuConfig.config_etrace(TestCPUClass, test_sys.cpu, args)

        CacheConfig.config_cache(args, test_sys)

        MemConfig.config_mem(args, test_sys)

    if ObjectList.is_kvm_cpu(TestCPUClass) or ObjectList.is_kvm_cpu(
        FutureClass
    ):
        # Assign KVM CPUs to their own event queues / threads. This
        # has to be done after creating caches and other child objects
        # since these mustn't inherit the CPU event queue.
        for i, cpu in enumerate(test_sys.cpu):
            # Child objects usually inherit the parent's event
            # queue. Override that and use the same event queue for
            # all devices.
            for obj in cpu.descendants():
                obj.eventq_index = 0
            cpu.eventq_index = i + 1
        test_sys.kvm_vm = KvmVM()

    return test_sys


def build_drive_system(np):
    # driver system CPU is always simple, so is the memory
    # Note this is an assignment of a class, not an instance.
    DriveCPUClass = AtomicSimpleCPU
    drive_mem_mode = "atomic"
    DriveMemClass = SimpleMemory

    cmdline = cmd_line_template()
    if buildEnv["USE_MIPS_ISA"]:
        drive_sys = makeLinuxMipsSystem(drive_mem_mode, bm[1], cmdline=cmdline)
    elif buildEnv["USE_SPARC_ISA"]:
        drive_sys = makeSparcSystem(drive_mem_mode, bm[1], cmdline=cmdline)
    elif buildEnv["USE_X86_ISA"]:
        drive_sys = makeLinuxX86System(
            drive_mem_mode, np, bm[1], cmdline=cmdline
        )
    elif buildEnv["USE_ARM_ISA"]:
        drive_sys = makeArmSystem(
            drive_mem_mode,
            args.machine_type,
            np,
            bm[1],
            args.dtb_filename,
            cmdline=cmdline,
        )

    # Create a top-level voltage domain
    drive_sys.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

    # Create a source clock for the system and set the clock period
    drive_sys.clk_domain = SrcClockDomain(
        clock=args.sys_clock, voltage_domain=drive_sys.voltage_domain
    )

    # Create a CPU voltage domain
    drive_sys.cpu_voltage_domain = VoltageDomain()

    # Create a source clock for the CPUs and set the clock period
    drive_sys.cpu_clk_domain = SrcClockDomain(
        clock=args.cpu_clock, voltage_domain=drive_sys.cpu_voltage_domain
    )

    drive_sys.cpu = DriveCPUClass(
        clk_domain=drive_sys.cpu_clk_domain, cpu_id=0
    )
    drive_sys.cpu.createThreads()
    drive_sys.cpu.createInterruptController()
    drive_sys.cpu.connectBus(drive_sys.membus)
    if args.kernel is not None:
        drive_sys.workload.object_file = binary(args.kernel)

    if ObjectList.is_kvm_cpu(DriveCPUClass):
        drive_sys.kvm_vm = KvmVM()

    drive_sys.iobridge = Bridge(delay="50ns", ranges=drive_sys.mem_ranges)
    drive_sys.iobridge.cpu_side_port = drive_sys.iobus.mem_side_ports
    drive_sys.iobridge.mem_side_port = drive_sys.membus.cpu_side_ports

    # Create the appropriate memory controllers and connect them to the
    # memory bus
    drive_sys.mem_ctrls = [
        DriveMemClass(range=r) for r in drive_sys.mem_ranges
    ]
    for i in range(len(drive_sys.mem_ctrls)):
        drive_sys.mem_ctrls[i].port = drive_sys.membus.mem_side_ports

    drive_sys.init_param = args.init_param

    return drive_sys


warn(
    "The fs.py script is deprecated. It will be removed in future releases of "
    " gem5."
)

# Add args
parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

# Add the ruby specific and protocol specific args
if "--ruby" in sys.argv:
    Ruby.define_options(parser)

args = parser.parse_args()

# system under test can be any CPU
(TestCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(args)

# Match the memories with the CPUs, based on the options for the test system
TestMemClass = Simulation.setMemClass(args)

if args.benchmark:
    try:
        bm = Benchmarks[args.benchmark]
    except KeyError:
        print(f"Error benchmark {args.benchmark} has not been defined.")
        print(f"Valid benchmarks are: {DefinedBenchmarks}")
        sys.exit(1)
else:
    if args.dual:
        bm = [
            SysConfig(
                disks=args.disk_image,
                rootdev=args.root_device,
                mem=args.mem_size,
                os_type=args.os_type,
            ),
            SysConfig(
                disks=args.disk_image,
                rootdev=args.root_device,
                mem=args.mem_size,
                os_type=args.os_type,
            ),
        ]
    else:
        bm = [
            SysConfig(
                disks=args.disk_image,
                rootdev=args.root_device,
                mem=args.mem_size,
                os_type=args.os_type,
            )
        ]

np = args.num_cpus

isa = ObjectList.cpu_list.get_isa(args.cpu_type)
test_sys = build_test_system(np, isa)

if len(bm) == 2:
    drive_sys = build_drive_system(np)
    root = makeDualRoot(True, test_sys, drive_sys, args.etherdump)
elif len(bm) == 1 and args.dist:
    # This system is part of a dist-gem5 simulation
    root = makeDistRoot(
        test_sys,
        args.dist_rank,
        args.dist_size,
        args.dist_server_name,
        args.dist_server_port,
        args.dist_sync_repeat,
        args.dist_sync_start,
        args.ethernet_linkspeed,
        args.ethernet_linkdelay,
        args.etherdump,
    )
elif len(bm) == 1:
    root = Root(full_system=True, system=test_sys)
else:
    print("Error I don't know how to create more than 2 systems.")
    sys.exit(1)

if ObjectList.is_kvm_cpu(TestCPUClass) or ObjectList.is_kvm_cpu(FutureClass):
    # Required for running kvm on multiple host cores.
    # Uses gem5's parallel event queue feature
    # Note: The simulator is quite picky about this number!
    root.sim_quantum = int(1e9)  # 1 ms

if args.timesync:
    root.time_sync_enable = True

if args.frame_capture:
    VncServer.frame_capture = True

if buildEnv["USE_ARM_ISA"] and not args.bare_metal and not args.dtb_filename:
    if args.machine_type not in [
        "VExpress_GEM5",
        "VExpress_GEM5_V1",
        "VExpress_GEM5_V2",
        "VExpress_GEM5_Foundation",
    ]:
        warn(
            "Can only correctly generate a dtb for VExpress_GEM5_* "
            "platforms, unless custom hardware models have been equipped "
            "with generation functionality."
        )

    # Generate a Device Tree
    for sysname in ("system", "testsys", "drivesys"):
        if hasattr(root, sysname):
            sys = getattr(root, sysname)
            sys.workload.dtb_filename = os.path.join(
                m5.options.outdir, f"{sysname}.dtb"
            )
            sys.generateDtb(sys.workload.dtb_filename)

if args.wait_gdb:
    test_sys.workload.wait_for_remote_gdb = True

Simulation.setWorkCountOptions(test_sys, args)
Simulation.run(args, root, test_sys, FutureClass)
