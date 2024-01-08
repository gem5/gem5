# Copyright (c) 2021 Arm Limited
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
# Copyright (c) 2021 The Regents of the University of California
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
from os import path

import m5
from m5.objects import *

m5.util.addToPath("../..")
from common import SysPaths


class ArmSstSystem(ArmSystem):
    def __init__(self, cpu_clock_rate, **kwargs):
        super(ArmSstSystem, self).__init__(**kwargs)

        self.voltage_domain = VoltageDomain(voltage="1.0V")
        self.clk_domain = SrcClockDomain(
            clock=cpu_clock_rate, voltage_domain=Parent.voltage_domain
        )

        self.terminal = Terminal()
        self.vncserver = VncServer()

        self.iobus = IOXBar()

        # Since the latency from CPU to the bus was set in SST,
        # additional latency is undesirable.
        self.membus = NoncoherentXBar(
            frontend_latency=0,
            forward_latency=0,
            response_latency=0,
            header_latency=0,
            width=64,
        )

        self.membus.badaddr_responder = BadAddr()
        self.membus.default = self.membus.badaddr_responder.pio

        _my_ranges = [
            AddrRange(0, size="64MiB"),
            AddrRange(0x80000000, size="16GiB"),
        ]
        self.memory_outgoing_bridge = OutgoingRequestBridge(
            physical_address_ranges=_my_ranges
        )

        self.memory_outgoing_bridge.port = self.membus.mem_side_ports

        self.cpu = [TimingSimpleCPU(cpu_id=0)]
        self.mem_mode = "timing"

        for cpu in self.cpu:
            cpu.createThreads()
            cpu.icache_port = self.membus.cpu_side_ports
            cpu.dcache_port = self.membus.cpu_side_ports

            cpu.mmu.connectWalkerPorts(
                self.membus.cpu_side_ports, self.membus.cpu_side_ports
            )

        self.bridge = Bridge(delay="50ns")
        self.bridge.mem_side_port = self.iobus.cpu_side_ports
        self.bridge.cpu_side_port = self.membus.mem_side_ports

    def getMemRanges(self, mem_size):
        """
        Define system memory ranges. This depends on the physical
        memory map provided by the realview platform and by the memory
        size provided by the user (mem_size argument).
        The method is iterating over all platform ranges until they cover
        the entire user's memory requirements.
        """
        mem_ranges = []
        for mem_range in self.platform._mem_regions:
            size_in_range = min(mem_size, mem_range.size())

            mem_ranges.append(
                AddrRange(start=mem_range.start, size=size_in_range)
            )

            mem_size -= size_in_range
            if mem_size == 0:
                return mem_ranges

        raise ValueError("memory size too big for platform capabilities")


def createArmPlatform(system):
    class VExpress_GEM5_V1_SST(VExpress_GEM5_V1):
        bootmem = SubSystem()

    system.platform = VExpress_GEM5_V1_SST()

    if hasattr(system.platform.gic, "cpu_addr"):
        system.gic_cpu_addr = system.platform.gic.cpu_addr

    system.platform.attachOnChipIO(system.membus, system.bridge)
    system.platform.attachIO(system.iobus)

    system.platform.setupBootLoader(system, SysPaths.binary)


parser = argparse.ArgumentParser()
parser.add_argument("--kernel", help="Path to the Kernel")
parser.add_argument(
    "--cpu-clock-rate", type=str, help="CPU clock rate, e.g. 3GHz"
)
parser.add_argument("--memory-size", type=str, help="Memory size, e.g. 4GiB")
parser.add_argument("--root-device", type=str, default="/dev/vda")
args = parser.parse_args()

system = ArmSstSystem(args.cpu_clock_rate)

# Setup Linux workload
system.workload = ArmFsLinux()
system.workload.object_file = args.kernel
system.workload.dtb_filename = path.join(m5.options.outdir, "system.dtb")
system.workload.addr_check = False

# Create RealView platform
createArmPlatform(system)

system.mem_ranges = system.getMemRanges(int(Addr(args.memory_size)))

system.system_outgoing_bridge = OutgoingRequestBridge()
system.system_port = system.system_outgoing_bridge.port
system.generateDtb(system.workload.dtb_filename)

# Linux boot command flags
kernel_cmd = [
    # Tell Linux to use the simulated serial port as a console
    "console=ttyAMA0",
    # Hard-code timi
    "lpj=19988480",
    # Disable address space randomisation to get a consistent
    # memory layout.
    "norandmaps",
    # Tell Linux where to find the root disk image.
    f"root={args.root_device}",
    # Mount the root disk read-write by default.
    "rw",
    # Tell Linux about the amount of physical memory present.
    f"mem={args.memory_size}",
]
system.workload.command_line = " ".join(kernel_cmd)

for cpu in system.cpu:
    cpu.createInterruptController()

root = Root(full_system=True, system=system)
