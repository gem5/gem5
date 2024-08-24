# Copyright (c) 2010-2012, 2015-2019 ARM Limited
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
# Copyright (c) 2010-2011 Advanced Micro Devices, Inc.
# Copyright (c) 2006-2008 The Regents of The University of Michigan
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

from common import ObjectList
from common.Benchmarks import *

import m5
import m5.defines
from m5.objects import *
from m5.util import *

# Populate to reflect supported os types per target ISA
os_types = set()
if m5.defines.buildEnv["USE_ARM_ISA"]:
    os_types.update(
        [
            "linux",
            "android-gingerbread",
            "android-ics",
            "android-jellybean",
            "android-kitkat",
            "android-nougat",
        ]
    )
if m5.defines.buildEnv["USE_MIPS_ISA"]:
    os_types.add("linux")
if m5.defines.buildEnv["USE_POWER_ISA"]:
    os_types.add("linux")
if m5.defines.buildEnv["USE_RISCV_ISA"]:
    os_types.add("linux")  # TODO that's a lie
if m5.defines.buildEnv["USE_SPARC_ISA"]:
    os_types.add("linux")
if m5.defines.buildEnv["USE_X86_ISA"]:
    os_types.add("linux")


class CowIdeDisk(IdeDisk):
    image = CowDiskImage(child=RawDiskImage(read_only=True), read_only=False)

    def childImage(self, ci):
        self.image.child.image_file = ci


class MemBus(SystemXBar):
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio


def attach_9p(parent, bus):
    viopci = PciVirtIO()
    viopci.vio = VirtIO9PDiod()
    viodir = os.path.realpath(os.path.join(m5.options.outdir, "9p"))
    viopci.vio.root = os.path.join(viodir, "share")
    viopci.vio.socketPath = os.path.join(viodir, "socket")
    os.makedirs(viopci.vio.root, exist_ok=True)
    if os.path.exists(viopci.vio.socketPath):
        os.remove(viopci.vio.socketPath)
    parent.viopci = viopci
    parent.attachPciDevice(viopci, bus)


def fillInCmdline(mdesc, template, **kwargs):
    kwargs.setdefault("rootdev", mdesc.rootdev())
    kwargs.setdefault("mem", mdesc.mem())
    kwargs.setdefault("script", mdesc.script())
    return template % kwargs


def makeCowDisks(disk_paths):
    disks = []
    for disk_path in disk_paths:
        disk = CowIdeDisk(driveID="device0")
        disk.childImage(disk_path)
        disks.append(disk)
    return disks


def makeSparcSystem(mem_mode, mdesc=None, cmdline=None):
    # Constants from iob.cc and uart8250.cc
    iob_man_addr = 0x9800000000
    uart_pio_size = 8

    class CowMmDisk(MmDisk):
        image = CowDiskImage(
            child=RawDiskImage(read_only=True), read_only=False
        )

        def childImage(self, ci):
            self.image.child.image_file = ci

    self = System()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()
    self.iobus = IOXBar()
    self.membus = MemBus()
    self.bridge = Bridge(delay="50ns")
    self.t1000 = T1000()
    self.t1000.attachOnChipIO(self.membus)
    self.t1000.attachIO(self.iobus)
    self.mem_ranges = [
        AddrRange(Addr("1MiB"), size="64MiB"),
        AddrRange(Addr("2GiB"), size="256MiB"),
    ]
    self.bridge.mem_side_port = self.iobus.cpu_side_ports
    self.bridge.cpu_side_port = self.membus.mem_side_ports
    self.disk0 = CowMmDisk()
    self.disk0.childImage(mdesc.disks()[0])
    self.disk0.pio = self.iobus.mem_side_ports

    # The puart0 and hvuart are placed on the IO bus, so create ranges
    # for them. The remaining IO range is rather fragmented, so poke
    # holes for the iob and partition descriptors etc.
    self.bridge.ranges = [
        AddrRange(
            self.t1000.puart0.pio_addr,
            self.t1000.puart0.pio_addr + uart_pio_size - 1,
        ),
        AddrRange(
            self.disk0.pio_addr,
            self.t1000.fake_jbi.pio_addr + self.t1000.fake_jbi.pio_size - 1,
        ),
        AddrRange(self.t1000.fake_clk.pio_addr, iob_man_addr - 1),
        AddrRange(
            self.t1000.fake_l2_1.pio_addr,
            self.t1000.fake_ssi.pio_addr + self.t1000.fake_ssi.pio_size - 1,
        ),
        AddrRange(
            self.t1000.hvuart.pio_addr,
            self.t1000.hvuart.pio_addr + uart_pio_size - 1,
        ),
    ]

    workload = SparcFsWorkload()

    # ROM for OBP/Reset/Hypervisor
    self.rom = SimpleMemory(
        image_file=binary("t1000_rom.bin"),
        range=AddrRange(0xFFF0000000, size="8MiB"),
    )
    # nvram
    self.nvram = SimpleMemory(
        image_file=binary("nvram1"), range=AddrRange(0x1F11000000, size="8KiB")
    )
    # hypervisor description
    self.hypervisor_desc = SimpleMemory(
        image_file=binary("1up-hv.bin"),
        range=AddrRange(0x1F12080000, size="8KiB"),
    )
    # partition description
    self.partition_desc = SimpleMemory(
        image_file=binary("1up-md.bin"),
        range=AddrRange(0x1F12000000, size="8KiB"),
    )

    self.rom.port = self.membus.mem_side_ports
    self.nvram.port = self.membus.mem_side_ports
    self.hypervisor_desc.port = self.membus.mem_side_ports
    self.partition_desc.port = self.membus.mem_side_ports

    self.system_port = self.membus.cpu_side_ports

    self.workload = workload

    return self


def makeArmSystem(
    mem_mode,
    machine_type,
    num_cpus=1,
    mdesc=None,
    dtb_filename=None,
    bare_metal=False,
    cmdline=None,
    external_memory="",
    ruby=False,
    vio_9p=None,
    bootloader=None,
):
    assert machine_type

    pci_devices = []

    self = ArmSystem()

    if not mdesc:
        # generic system
        mdesc = SysConfig()

    self.readfile = mdesc.script()
    self.iobus = IOXBar()
    if not ruby:
        self.bridge = Bridge(delay="50ns")
        self.bridge.mem_side_port = self.iobus.cpu_side_ports
        self.membus = MemBus()
        self.membus.badaddr_responder.warn_access = "warn"
        self.bridge.cpu_side_port = self.membus.mem_side_ports

    self.mem_mode = mem_mode

    platform_class = ObjectList.platform_list.get(machine_type)
    # Resolve the real platform name, the original machine_type
    # variable might have been an alias.
    machine_type = platform_class.__name__
    self.realview = platform_class()
    self._bootmem = self.realview.bootmem

    # Attach any PCI devices this platform supports
    self.realview.attachPciDevices()

    disks = makeCowDisks(mdesc.disks())
    # Old platforms have a built-in IDE or CF controller. Default to
    # the IDE controller if both exist. New platforms expect the
    # storage controller to be added from the config script.
    if hasattr(self.realview, "ide"):
        self.realview.ide.disks = disks
    elif hasattr(self.realview, "cf_ctrl"):
        self.realview.cf_ctrl.disks = disks
    else:
        self.pci_ide = IdeController(disks=disks)
        pci_devices.append(self.pci_ide)

    self.mem_ranges = []
    size_remain = int(Addr(mdesc.mem()))
    for region in self.realview._mem_regions:
        if size_remain > int(region.size()):
            self.mem_ranges.append(region)
            size_remain = size_remain - int(region.size())
        else:
            self.mem_ranges.append(AddrRange(region.start, size=size_remain))
            size_remain = 0
            break
        warn(
            "Memory size specified spans more than one region. Creating"
            " another memory controller for that range."
        )

    if size_remain > 0:
        fatal(
            "The currently selected ARM platforms doesn't support"
            " the amount of DRAM you've selected. Please try"
            " another platform"
        )

    if bare_metal:
        # EOT character on UART will end the simulation
        self.realview.uart[0].end_on_eot = True
        self.workload = ArmFsWorkload(dtb_addr=0)
    else:
        workload = ArmFsLinux()

        if dtb_filename:
            workload.dtb_filename = binary(dtb_filename)

        workload.machine_type = (
            machine_type if machine_type in ArmMachineType.map else "DTOnly"
        )

        # Ensure that writes to the UART actually go out early in the boot
        if not cmdline:
            cmdline = (
                "earlyprintk=pl011,0x1c090000 console=ttyAMA0 "
                + "lpj=19988480 norandmaps rw loglevel=8 "
                + "mem=%(mem)s root=%(rootdev)s"
            )

        if hasattr(self.realview.gic, "cpu_addr"):
            self.gic_cpu_addr = self.realview.gic.cpu_addr

        # This check is for users who have previously put 'android' in
        # the disk image filename to tell the config scripts to
        # prepare the kernel with android-specific boot options. That
        # behavior has been replaced with a more explicit option per
        # the error message below. The disk can have any name now and
        # doesn't need to include 'android' substring.
        if mdesc.disks() and os.path.split(mdesc.disks()[0])[-1].lower().count(
            "android"
        ):
            if "android" not in mdesc.os_type():
                fatal(
                    "It looks like you are trying to boot an Android "
                    "platform.  To boot Android, you must specify "
                    "--os-type with an appropriate Android release on "
                    "the command line."
                )

        # android-specific tweaks
        if "android" in mdesc.os_type():
            # generic tweaks
            cmdline += " init=/init"

            # release-specific tweaks
            if "kitkat" in mdesc.os_type():
                cmdline += (
                    " androidboot.hardware=gem5 qemu=1 qemu.gles=0 "
                    + "android.bootanim=0 "
                )
            elif "nougat" in mdesc.os_type():
                cmdline += (
                    " androidboot.hardware=gem5 qemu=1 qemu.gles=0 "
                    + "android.bootanim=0 "
                    + "vmalloc=640MB "
                    + "android.early.fstab=/fstab.gem5 "
                    + "androidboot.selinux=permissive "
                    + "video=Virtual-1:1920x1080-16"
                )

        workload.command_line = fillInCmdline(mdesc, cmdline)

        self.workload = workload

        self.realview.setupBootLoader(self, binary, bootloader)

    if external_memory:
        # I/O traffic enters iobus
        self.external_io = ExternalMaster(
            port_data="external_io", port_type=external_memory
        )
        self.external_io.port = self.iobus.cpu_side_ports

        # Ensure iocache only receives traffic destined for (actual) memory.
        self.iocache = ExternalSlave(
            port_data="iocache",
            port_type=external_memory,
            addr_ranges=self.mem_ranges,
        )
        self.iocache.port = self.iobus.mem_side_ports

        # Let system_port get to nvmem and nothing else.
        self.bridge.ranges = [self.realview.nvmem.range]

        self.realview.attachOnChipIO(self.iobus)
        # Attach off-chip devices
        self.realview.attachIO(self.iobus)
    elif ruby:
        self._dma_ports = []
        self._mem_ports = []
        self.realview.attachOnChipIO(
            self.iobus, dma_ports=self._dma_ports, mem_ports=self._mem_ports
        )
        self.realview.attachIO(self.iobus, dma_ports=self._dma_ports)
    else:
        self.realview.attachOnChipIO(self.membus, self.bridge)
        # Attach off-chip devices
        self.realview.attachIO(self.iobus)

    for dev in pci_devices:
        self.realview.attachPciDevice(
            dev, self.iobus, dma_ports=self._dma_ports if ruby else None
        )

    self.terminal = Terminal()
    self.vncserver = VncServer()

    if vio_9p:
        attach_9p(self.realview, self.iobus)

    if not ruby:
        self.system_port = self.membus.cpu_side_ports

    if ruby:
        if buildEnv["PROTOCOL"] == "MI_example" and num_cpus > 1:
            fatal(
                "The MI_example protocol cannot implement Load/Store "
                "Exclusive operations. Multicore ARM systems configured "
                "with the MI_example protocol will not work properly."
            )

    return self


def makeLinuxMipsSystem(mem_mode, mdesc=None, cmdline=None):
    class BaseMalta(Malta):
        ethernet = NSGigE(pci_bus=0, pci_dev=1, pci_func=0)
        ide = IdeController(
            disks=Parent.disks, pci_func=0, pci_dev=0, pci_bus=0
        )

    self = System()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()
    self.iobus = IOXBar()
    self.membus = MemBus()
    self.bridge = Bridge(delay="50ns")
    self.mem_ranges = [AddrRange("1GiB")]
    self.bridge.mem_side_port = self.iobus.cpu_side_ports
    self.bridge.cpu_side_port = self.membus.mem_side_ports
    self.disks = makeCowDisks(mdesc.disks())
    self.malta = BaseMalta()
    self.malta.attachIO(self.iobus)
    self.malta.ide.pio = self.iobus.mem_side_ports
    self.malta.ide.dma = self.iobus.cpu_side_ports
    self.malta.ethernet.pio = self.iobus.mem_side_ports
    self.malta.ethernet.dma = self.iobus.cpu_side_ports
    self.simple_disk = SimpleDisk(
        disk=RawDiskImage(image_file=mdesc.disks()[0], read_only=True)
    )
    self.mem_mode = mem_mode
    self.terminal = Terminal()
    self.console = binary("mips/console")
    if not cmdline:
        cmdline = "root=/dev/hda1 console=ttyS0"
    self.workload = KernelWorkload(command_line=fillInCmdline(mdesc, cmdline))

    self.system_port = self.membus.cpu_side_ports

    return self


def x86IOAddress(port):
    IO_address_space_base = 0x8000000000000000
    return IO_address_space_base + port


def connectX86ClassicSystem(x86_sys, numCPUs):
    # Constants similar to x86_traits.hh
    IO_address_space_base = 0x8000000000000000
    pci_config_address_space_base = 0xC000000000000000
    interrupts_address_space_base = 0xA000000000000000
    APIC_range_size = 1 << 12

    x86_sys.membus = MemBus()

    # North Bridge
    x86_sys.iobus = IOXBar()
    x86_sys.bridge = Bridge(delay="50ns")
    x86_sys.bridge.mem_side_port = x86_sys.iobus.cpu_side_ports
    x86_sys.bridge.cpu_side_port = x86_sys.membus.mem_side_ports
    # Allow the bridge to pass through:
    #  1) kernel configured PCI device memory map address: address range
    #     [0xC0000000, 0xFFFF0000). (The upper 64KiB are reserved for m5ops.)
    #  2) the bridge to pass through the IO APIC (two pages, already contained in 1),
    #  3) everything in the IO address range up to the local APIC, and
    #  4) then the entire PCI address space and beyond.
    x86_sys.bridge.ranges = [
        AddrRange(0xC0000000, 0xFFFF0000),
        AddrRange(IO_address_space_base, interrupts_address_space_base - 1),
        AddrRange(pci_config_address_space_base, Addr.max),
    ]

    # Create a bridge from the IO bus to the memory bus to allow access to
    # the local APIC (two pages)
    x86_sys.apicbridge = Bridge(delay="50ns")
    x86_sys.apicbridge.cpu_side_port = x86_sys.iobus.mem_side_ports
    x86_sys.apicbridge.mem_side_port = x86_sys.membus.cpu_side_ports
    x86_sys.apicbridge.ranges = [
        AddrRange(
            interrupts_address_space_base,
            interrupts_address_space_base + numCPUs * APIC_range_size - 1,
        )
    ]

    # connect the io bus
    x86_sys.pc.attachIO(x86_sys.iobus)

    x86_sys.system_port = x86_sys.membus.cpu_side_ports


def connectX86RubySystem(x86_sys):
    # North Bridge
    x86_sys.iobus = IOXBar()

    # add the ide to the list of dma devices that later need to attach to
    # dma controllers
    x86_sys._dma_ports = [x86_sys.pc.south_bridge.ide.dma]
    x86_sys.pc.attachIO(x86_sys.iobus, x86_sys._dma_ports)


def makeX86System(mem_mode, numCPUs=1, mdesc=None, workload=None, Ruby=False):
    self = System()

    self.m5ops_base = 0xFFFF0000

    if workload is None:
        workload = X86FsWorkload()
    self.workload = workload

    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()

    self.mem_mode = mem_mode

    # Physical memory
    # On the PC platform, the memory region 0xC0000000-0xFFFFFFFF is reserved
    # for various devices.  Hence, if the physical memory size is greater than
    # 3GiB, we need to split it into two parts.
    excess_mem_size = convert.toMemorySize(mdesc.mem()) - convert.toMemorySize(
        "3GiB"
    )
    if excess_mem_size <= 0:
        self.mem_ranges = [AddrRange(mdesc.mem())]
    else:
        warn(
            "Physical memory size specified is %s which is greater than "
            "3GiB.  Twice the number of memory controllers would be "
            "created." % (mdesc.mem())
        )

        self.mem_ranges = [
            AddrRange("3GiB"),
            AddrRange(Addr("4GiB"), size=excess_mem_size),
        ]

    # Platform
    self.pc = Pc()

    # Create and connect the busses required by each memory system
    if Ruby:
        connectX86RubySystem(self)
    else:
        connectX86ClassicSystem(self, numCPUs)

    # Disks
    disks = makeCowDisks(mdesc.disks())
    self.pc.south_bridge.ide.disks = disks

    # Add in a Bios information structure.
    structures = [X86SMBiosBiosInformation()]
    workload.smbios_table.structures = structures

    # Set up the Intel MP table
    base_entries = []
    ext_entries = []
    madt_records = []
    for i in range(numCPUs):
        bp = X86IntelMPProcessor(
            local_apic_id=i,
            local_apic_version=0x14,
            enable=True,
            bootstrap=(i == 0),
        )
        base_entries.append(bp)
        lapic = X86ACPIMadtLAPIC(acpi_processor_id=i, apic_id=i, flags=1)
        madt_records.append(lapic)
    io_apic = X86IntelMPIOAPIC(
        id=numCPUs, version=0x11, enable=True, address=0xFEC00000
    )
    self.pc.south_bridge.io_apic.apic_id = io_apic.id
    base_entries.append(io_apic)
    madt_records.append(
        X86ACPIMadtIOAPIC(id=io_apic.id, address=io_apic.address, int_base=0)
    )
    # In gem5 Pc::calcPciConfigAddr(), it required "assert(bus==0)",
    # but linux kernel cannot config PCI device if it was not connected to
    # PCI bus, so we fix PCI bus id to 0, and ISA bus id to 1.
    pci_bus = X86IntelMPBus(bus_id=0, bus_type="PCI   ")
    base_entries.append(pci_bus)
    isa_bus = X86IntelMPBus(bus_id=1, bus_type="ISA   ")
    base_entries.append(isa_bus)
    connect_busses = X86IntelMPBusHierarchy(
        bus_id=1, subtractive_decode=True, parent_bus=0
    )
    ext_entries.append(connect_busses)
    pci_dev4_inta = X86IntelMPIOIntAssignment(
        interrupt_type="INT",
        polarity="ConformPolarity",
        trigger="ConformTrigger",
        source_bus_id=0,
        source_bus_irq=0 + (4 << 2),
        dest_io_apic_id=io_apic.id,
        dest_io_apic_intin=16,
    )
    base_entries.append(pci_dev4_inta)
    pci_dev4_inta_madt = X86ACPIMadtIntSourceOverride(
        bus_source=pci_dev4_inta.source_bus_id,
        irq_source=pci_dev4_inta.source_bus_irq,
        sys_int=pci_dev4_inta.dest_io_apic_intin,
        flags=0,
    )
    madt_records.append(pci_dev4_inta_madt)

    def assignISAInt(irq, apicPin):
        assign_8259_to_apic = X86IntelMPIOIntAssignment(
            interrupt_type="ExtInt",
            polarity="ConformPolarity",
            trigger="ConformTrigger",
            source_bus_id=1,
            source_bus_irq=irq,
            dest_io_apic_id=io_apic.id,
            dest_io_apic_intin=0,
        )
        base_entries.append(assign_8259_to_apic)
        assign_to_apic = X86IntelMPIOIntAssignment(
            interrupt_type="INT",
            polarity="ConformPolarity",
            trigger="ConformTrigger",
            source_bus_id=1,
            source_bus_irq=irq,
            dest_io_apic_id=io_apic.id,
            dest_io_apic_intin=apicPin,
        )
        base_entries.append(assign_to_apic)
        # acpi
        assign_to_apic_acpi = X86ACPIMadtIntSourceOverride(
            bus_source=1, irq_source=irq, sys_int=apicPin, flags=0
        )
        madt_records.append(assign_to_apic_acpi)

    assignISAInt(0, 2)
    assignISAInt(1, 1)
    for i in range(3, 15):
        assignISAInt(i, i)
    workload.intel_mp_table.base_entries = base_entries
    workload.intel_mp_table.ext_entries = ext_entries

    madt = X86ACPIMadt(
        local_apic_address=0, records=madt_records, oem_id="madt"
    )
    workload.acpi_description_table_pointer.rsdt.entries.append(madt)
    workload.acpi_description_table_pointer.xsdt.entries.append(madt)
    workload.acpi_description_table_pointer.oem_id = "gem5"
    workload.acpi_description_table_pointer.rsdt.oem_id = "gem5"
    workload.acpi_description_table_pointer.xsdt.oem_id = "gem5"
    return self


def makeLinuxX86System(
    mem_mode, numCPUs=1, mdesc=None, Ruby=False, cmdline=None
):
    # Build up the x86 system and then specialize it for Linux
    self = makeX86System(mem_mode, numCPUs, mdesc, X86FsLinux(), Ruby)

    # We assume below that there's at least 1MiB of memory. We'll require 2
    # just to avoid corner cases.
    phys_mem_size = sum([r.size() for r in self.mem_ranges])
    assert phys_mem_size >= 0x200000
    assert len(self.mem_ranges) <= 2

    entries = [
        # Mark the first mibibyte of memory as reserved
        X86E820Entry(addr=0, size="639KiB", range_type=1),
        X86E820Entry(addr=0x9FC00, size="385KiB", range_type=2),
        # Mark the rest of physical memory as available
        X86E820Entry(
            addr=0x100000,
            size="%dB" % (self.mem_ranges[0].size() - 0x100000),
            range_type=1,
        ),
    ]

    # Mark [mem_size, 3iB) as reserved if memory less than 3GiB, which force
    # IO devices to be mapped to [0xC0000000, 0xFFFF0000). Requests to this
    # specific range can pass though bridge to iobus.
    if len(self.mem_ranges) == 1:
        entries.append(
            X86E820Entry(
                addr=self.mem_ranges[0].size(),
                size="%dB" % (0xC0000000 - self.mem_ranges[0].size()),
                range_type=2,
            )
        )

    # Reserve the last 16KiB of the 32-bit address space for the m5op interface
    entries.append(X86E820Entry(addr=0xFFFF0000, size="64KiB", range_type=2))

    # In case the physical memory is greater than 3GiB, we split it into two
    # parts and add a separate e820 entry for the second part.  This entry
    # starts at 0x100000000,  which is the first address after the space
    # reserved for devices.
    if len(self.mem_ranges) == 2:
        entries.append(
            X86E820Entry(
                addr=0x100000000,
                size="%dB" % (self.mem_ranges[1].size()),
                range_type=1,
            )
        )

    self.workload.e820_table.entries = entries

    # Command line
    if not cmdline:
        cmdline = "earlyprintk=ttyS0 console=ttyS0 lpj=7999923 root=/dev/hda1"
    self.workload.command_line = fillInCmdline(mdesc, cmdline)
    return self


def makeBareMetalRiscvSystem(mem_mode, mdesc=None, cmdline=None):
    self = System()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.mem_mode = mem_mode
    self.mem_ranges = [AddrRange(mdesc.mem())]

    self.workload = RiscvBareMetal()

    self.iobus = IOXBar()
    self.membus = MemBus()

    self.bridge = Bridge(delay="50ns")
    self.bridge.mem_side_port = self.iobus.cpu_side_ports
    self.bridge.cpu_side_port = self.membus.mem_side_ports
    # Sv39 has 56 bit physical addresses; use the upper 8 bit for the IO space
    IO_address_space_base = 0x00FF000000000000
    self.bridge.ranges = [AddrRange(IO_address_space_base, Addr.max)]

    self.system_port = self.membus.cpu_side_ports
    return self


def makeDualRoot(full_system, testSystem, driveSystem, dumpfile):
    self = Root(full_system=full_system)
    self.testsys = testSystem
    self.drivesys = driveSystem
    self.etherlink = EtherLink()

    if hasattr(testSystem, "realview"):
        self.etherlink.int0 = Parent.testsys.realview.ethernet.interface
        self.etherlink.int1 = Parent.drivesys.realview.ethernet.interface
    elif hasattr(testSystem, "tsunami"):
        self.etherlink.int0 = Parent.testsys.tsunami.ethernet.interface
        self.etherlink.int1 = Parent.drivesys.tsunami.ethernet.interface
    else:
        fatal("Don't know how to connect these system together")

    if dumpfile:
        self.etherdump = EtherDump(file=dumpfile)
        self.etherlink.dump = Parent.etherdump

    return self


def makeDistRoot(
    testSystem,
    rank,
    size,
    server_name,
    server_port,
    sync_repeat,
    sync_start,
    linkspeed,
    linkdelay,
    dumpfile,
):
    self = Root(full_system=True)
    self.testsys = testSystem

    self.etherlink = DistEtherLink(
        speed=linkspeed,
        delay=linkdelay,
        dist_rank=rank,
        dist_size=size,
        server_name=server_name,
        server_port=server_port,
        sync_start=sync_start,
        sync_repeat=sync_repeat,
    )

    if hasattr(testSystem, "realview"):
        self.etherlink.int0 = Parent.testsys.realview.ethernet.interface
    elif hasattr(testSystem, "tsunami"):
        self.etherlink.int0 = Parent.testsys.tsunami.ethernet.interface
    else:
        fatal("Don't know how to connect DistEtherLink to this system")

    if dumpfile:
        self.etherdump = EtherDump(file=dumpfile)
        self.etherlink.dump = Parent.etherdump

    return self
