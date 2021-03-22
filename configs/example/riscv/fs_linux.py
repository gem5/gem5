# Copyright (c) 2021 Huawei International
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

import optparse
import sys
from os import path

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *

addToPath('../../')

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


def generateMemNode(state, mem_range):
    node = FdtNode("memory@%x" % int(mem_range.start))
    node.append(FdtPropertyStrings("device_type", ["memory"]))
    node.append(FdtPropertyWords("reg",
        state.addrCells(mem_range.start) +
        state.sizeCells(mem_range.size()) ))
    return node

def generateDtb(system):
    """
    Autogenerate DTB. Arguments are the folder where the DTB
    will be stored, and the name of the DTB file.
    """
    state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
    root = FdtNode('/')
    root.append(state.addrCellsProperty())
    root.append(state.sizeCellsProperty())
    root.appendCompatible(["riscv-virtio"])

    for mem_range in system.mem_ranges:
        root.append(generateMemNode(state, mem_range))

    sections = [*system.cpu, system.platform]

    for section in sections:
        for node in section.generateDeviceTree(state):
            if node.get_name() == root.get_name():
                root.merge(node)
            else:
                root.append(node)

    fdt = Fdt()
    fdt.add_rootnode(root)
    fdt.writeDtsFile(path.join(m5.options.outdir, 'device.dts'))
    fdt.writeDtbFile(path.join(m5.options.outdir, 'device.dtb'))

# ----------------------------- Add Options ---------------------------- #
parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

# NOTE: Ruby in FS Linux has not been tested yet
if '--ruby' in sys.argv:
    Ruby.define_options(parser)

# ---------------------------- Parse Options --------------------------- #
(options, args) = parser.parse_args()

if args:
    print("Error: script doesn't take any positional arguments")
    sys.exit(1)

# CPU and Memory
(CPUClass, mem_mode, FutureClass) = Simulation.setCPUClass(options)
MemClass = Simulation.setMemClass(options)

np = options.num_cpus

# ---------------------------- Setup System ---------------------------- #
# Edit this section to customize peripherals and system settings
system = System()
mdesc = SysConfig(disks=options.disk_image, rootdev=options.root_device,
                        mem=options.mem_size, os_type=options.os_type)
system.mem_mode = mem_mode
system.mem_ranges = [AddrRange(start=0x80000000, size=mdesc.mem())]

system.workload = RiscvLinux()

system.iobus = IOXBar()
system.membus = MemBus()

system.system_port = system.membus.cpu_side_ports

# HiFive platform
system.platform = HiFive()

# RTCCLK (Set to 100MHz for faster simulation)
system.platform.rtc = RiscvRTC(frequency=Frequency("100MHz"))
system.platform.clint.int_pin = system.platform.rtc.int_pin

# VirtIOMMIO
image = CowDiskImage(child=RawDiskImage(read_only=True), read_only=False)
image.child.image_file = mdesc.disks()[0]
system.platform.disk = MmioVirtIO(
    vio=VirtIOBlock(image=image),
    interrupt_id=0x8,
    pio_size=4096,
    pio_addr=0x10008000
)

system.bridge = Bridge(delay='50ns')
system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports
system.bridge.ranges = system.platform._off_chip_ranges()

system.platform.attachOnChipIO(system.membus)
system.platform.attachOffChipIO(system.iobus)
system.platform.attachPlic()

# ---------------------------- Default Setup --------------------------- #

# Set the cache line size for the entire system
system.cache_line_size = options.cacheline_size

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
        voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a source clock for the CPUs and set the clock period
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                            voltage_domain =
                                            system.cpu_voltage_domain)

system.workload.object_file = options.kernel

# NOTE: Not yet tested
if options.script is not None:
    system.readfile = options.script

system.init_param = options.init_param

system.cpu = [CPUClass(clk_domain=system.cpu_clk_domain, cpu_id=i)
                for i in range(np)]

if options.caches or options.l2cache:
    # By default the IOCache runs at the system clock
    system.iocache = IOCache(addr_ranges = system.mem_ranges)
    system.iocache.cpu_side = system.iobus.mem_side_ports
    system.iocache.mem_side = system.membus.cpu_side_ports
elif not options.external_memory_system:
    system.iobridge = Bridge(delay='50ns', ranges = system.mem_ranges)
    system.iobridge.cpu_side_ports = system.iobus.mem_side_ports
    system.iobridge.mem_side_ports = system.membus.cpu_side_ports

# Sanity check
if options.simpoint_profile:
    if not ObjectList.is_noncaching_cpu(CPUClass):
        fatal("SimPoint generation should be done with atomic cpu")
    if np > 1:
        fatal("SimPoint generation not supported with more than one CPUs")

for i in range(np):
    if options.simpoint_profile:
        system.cpu[i].addSimPointProbe(options.simpoint_interval)
    if options.checker:
        system.cpu[i].addCheckerCpu()
    if not ObjectList.is_kvm_cpu(CPUClass):
        if options.bp_type:
            bpClass = ObjectList.bp_list.get(options.bp_type)
            system.cpu[i].branchPred = bpClass()
        if options.indirect_bp_type:
            IndirectBPClass = ObjectList.indirect_bp_list.get(
                options.indirect_bp_type)
            system.cpu[i].branchPred.indirectBranchPred = \
                IndirectBPClass()
    system.cpu[i].createThreads()

# ----------------------------- PMA Checker ---------------------------- #

uncacheable_range = [
    *system.platform._on_chip_ranges(),
    *system.platform._off_chip_ranges()
]

# PMA checker can be defined at system-level (system.pma_checker)
# or MMU-level (system.cpu[0].mmu.pma_checker). It will be resolved
# by RiscvTLB's Parent.any proxy
for cpu in system.cpu:
    cpu.mmu.pma_checker = PMAChecker(uncacheable=uncacheable_range)

# --------------------------- DTB Generation --------------------------- #

generateDtb(system)
system.workload.dtb_filename = path.join(m5.options.outdir, 'device.dtb')
# Default DTB address if bbl is bulit with --with-dts option
system.workload.dtb_addr = 0x87e00000

# Linux boot command flags
kernel_cmd = [
    "console=ttyS0",
    "root=/dev/vda",
    "ro"
]
system.workload.command_line = " ".join(kernel_cmd)

# ---------------------------- Default Setup --------------------------- #

if options.elastic_trace_en and options.checkpoint_restore == None and \
    not options.fast_forward:
    CpuConfig.config_etrace(CPUClass, system.cpu, options)

CacheConfig.config_cache(options, system)

MemConfig.config_mem(options, system)

root = Root(full_system=True, system=system)

Simulation.setWorkCountOptions(system, options)
Simulation.run(options, root, system, FutureClass)
