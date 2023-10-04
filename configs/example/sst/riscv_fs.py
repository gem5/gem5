# Copyright (c) 2023 The Regents of the University of California
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

import m5
from m5.objects import *
from os import path

# For downloading the disk image
from gem5.resources.resource import obtain_resource

import argparse


def generateMemNode(state, mem_range):
    node = FdtNode(f"memory@{int(mem_range.start):x}")
    node.append(FdtPropertyStrings("device_type", ["memory"]))
    node.append(
        FdtPropertyWords(
            "reg",
            state.addrCells(mem_range.start)
            + state.sizeCells(mem_range.size()),
        )
    )
    return node


def generateDtb(system):
    """
    Autogenerate DTB. Arguments are the folder where the DTB
    will be stored, and the name of the DTB file.
    """
    state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
    root = FdtNode("/")
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
    fdt.writeDtsFile(path.join(m5.options.outdir, "device.dts"))
    fdt.writeDtbFile(path.join(m5.options.outdir, "device.dtb"))


def createHiFivePlatform(system):
    # Since the latency from CPU to the bus was set in SST, additional latency
    # is undesirable.
    system.membus = NoncoherentXBar(
        frontend_latency=0,
        forward_latency=0,
        response_latency=0,
        header_latency=0,
        width=64,
    )
    system.membus.badaddr_responder = BadAddr()
    system.membus.default = system.membus.badaddr_responder.pio

    system.memory_outgoing_bridge = OutgoingRequestBridge()
    system.memory_outgoing_bridge.port = system.membus.mem_side_ports
    for cpu in system.cpu:
        cpu.createThreads()
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports

        cpu.mmu.connectWalkerPorts(
            system.membus.cpu_side_ports, system.membus.cpu_side_ports
        )

    system.platform = HiFive()

    system.platform.pci_host.pio = system.membus.mem_side_ports

    system.platform.rtc = RiscvRTC(frequency=Frequency("10MHz"))
    system.platform.clint.int_pin = system.platform.rtc.int_pin

    system.pma_checker = PMAChecker(
        uncacheable=[
            *system.platform._on_chip_ranges(),
            *system.platform._off_chip_ranges(),
        ]
    )

    system.iobus = IOXBar()
    system.bridge = Bridge(delay="50ns")
    system.bridge.mem_side_port = system.iobus.cpu_side_ports
    system.bridge.cpu_side_port = system.membus.mem_side_ports
    system.bridge.ranges = system.platform._off_chip_ranges()

    system.platform.setNumCores(1)

    system.platform.attachOnChipIO(system.membus)
    system.platform.attachOffChipIO(system.iobus)

    system.platform.attachPlic()


parser = argparse.ArgumentParser()
parser.add_argument(
    "--cpu-clock-rate", type=str, help="CPU clock rate, e.g. 3GHz"
)
parser.add_argument("--memory-size", type=str, help="Memory size, e.g. 4GiB")

args = parser.parse_args()
cpu_clock_rate = args.cpu_clock_rate
memory_size = args.memory_size

# Try downloading the Resource
bbl_resource = obtain_resource("riscv-boot-exit-nodisk")
bbl_path = bbl_resource.get_local_path()

system = System()

system.clk_domain = SrcClockDomain(
    clock=cpu_clock_rate, voltage_domain=VoltageDomain()
)

system.mem_ranges = [AddrRange(start=0x80000000, size=memory_size)]

system.cpu = [TimingSimpleCPU(cpu_id=i) for i in range(1)]
system.mem_mode = "timing"

createHiFivePlatform(system)

system.system_outgoing_bridge = OutgoingRequestBridge()
system.system_port = system.system_outgoing_bridge.port
generateDtb(system)
system.workload = RiscvLinux()
system.workload.addr_check = False
system.workload.object_file = bbl_path
system.workload.dtb_filename = path.join(m5.options.outdir, "device.dtb")
system.workload.dtb_addr = 0x87E00000
kernel_cmd = [
    # specifying Linux kernel boot options
    "console=ttyS0"
]
system.workload.command_line = " ".join(kernel_cmd)

for cpu in system.cpu:
    cpu.createInterruptController()

root = Root(full_system=True, system=system)
