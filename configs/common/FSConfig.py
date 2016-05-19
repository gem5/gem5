# Copyright (c) 2010-2012, 2015 ARM Limited
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
#
# Authors: Kevin Lim

from m5.objects import *
from Benchmarks import *
from m5.util import *
import PlatformConfig

# Populate to reflect supported os types per target ISA
os_types = { 'alpha' : [ 'linux' ],
             'mips'  : [ 'linux' ],
             'sparc' : [ 'linux' ],
             'x86'   : [ 'linux' ],
             'arm'   : [ 'linux',
                         'android-gingerbread',
                         'android-ics',
                         'android-jellybean',
                         'android-kitkat' ],
           }

class CowIdeDisk(IdeDisk):
    image = CowDiskImage(child=RawDiskImage(read_only=True),
                         read_only=False)

    def childImage(self, ci):
        self.image.child.image_file = ci

class MemBus(SystemXBar):
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio

def fillInCmdline(mdesc, template, **kwargs):
    kwargs.setdefault('disk', mdesc.disk())
    kwargs.setdefault('rootdev', mdesc.rootdev())
    kwargs.setdefault('mem', mdesc.mem())
    kwargs.setdefault('script', mdesc.script())
    return template % kwargs

def makeLinuxAlphaSystem(mem_mode, mdesc=None, ruby=False, cmdline=None):

    class BaseTsunami(Tsunami):
        ethernet = NSGigE(pci_bus=0, pci_dev=1, pci_func=0)
        ide = IdeController(disks=[Parent.disk0, Parent.disk2],
                            pci_func=0, pci_dev=0, pci_bus=0)

    self = LinuxAlphaSystem()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()

    self.tsunami = BaseTsunami()

    # Create the io bus to connect all device ports
    self.iobus = IOXBar()
    self.tsunami.attachIO(self.iobus)

    self.tsunami.ide.pio = self.iobus.master

    self.tsunami.ethernet.pio = self.iobus.master

    if ruby:
        # Store the dma devices for later connection to dma ruby ports.
        # Append an underscore to dma_ports to avoid the SimObjectVector check.
        self._dma_ports = [self.tsunami.ide.dma, self.tsunami.ethernet.dma]
    else:
        self.membus = MemBus()

        # By default the bridge responds to all addresses above the I/O
        # base address (including the PCI config space)
        IO_address_space_base = 0x80000000000
        self.bridge = Bridge(delay='50ns',
                         ranges = [AddrRange(IO_address_space_base, Addr.max)])
        self.bridge.master = self.iobus.slave
        self.bridge.slave = self.membus.master

        self.tsunami.ide.dma = self.iobus.slave
        self.tsunami.ethernet.dma = self.iobus.slave

        self.system_port = self.membus.slave

    self.mem_ranges = [AddrRange(mdesc.mem())]
    self.disk0 = CowIdeDisk(driveID='master')
    self.disk2 = CowIdeDisk(driveID='master')
    self.disk0.childImage(mdesc.disk())
    self.disk2.childImage(disk('linux-bigswap2.img'))
    self.simple_disk = SimpleDisk(disk=RawDiskImage(image_file = mdesc.disk(),
                                               read_only = True))
    self.intrctrl = IntrControl()
    self.mem_mode = mem_mode
    self.terminal = Terminal()
    self.kernel = binary('vmlinux')
    self.pal = binary('ts_osfpal')
    self.console = binary('console')
    if not cmdline:
        cmdline = 'root=/dev/hda1 console=ttyS0'
    self.boot_osflags = fillInCmdline(mdesc, cmdline)

    return self

def makeSparcSystem(mem_mode, mdesc=None, cmdline=None):
    # Constants from iob.cc and uart8250.cc
    iob_man_addr = 0x9800000000
    uart_pio_size = 8

    class CowMmDisk(MmDisk):
        image = CowDiskImage(child=RawDiskImage(read_only=True),
                             read_only=False)

        def childImage(self, ci):
            self.image.child.image_file = ci

    self = SparcSystem()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()
    self.iobus = IOXBar()
    self.membus = MemBus()
    self.bridge = Bridge(delay='50ns')
    self.t1000 = T1000()
    self.t1000.attachOnChipIO(self.membus)
    self.t1000.attachIO(self.iobus)
    self.mem_ranges = [AddrRange(Addr('1MB'), size = '64MB'),
                       AddrRange(Addr('2GB'), size ='256MB')]
    self.bridge.master = self.iobus.slave
    self.bridge.slave = self.membus.master
    self.rom.port = self.membus.master
    self.nvram.port = self.membus.master
    self.hypervisor_desc.port = self.membus.master
    self.partition_desc.port = self.membus.master
    self.intrctrl = IntrControl()
    self.disk0 = CowMmDisk()
    self.disk0.childImage(disk('disk.s10hw2'))
    self.disk0.pio = self.iobus.master

    # The puart0 and hvuart are placed on the IO bus, so create ranges
    # for them. The remaining IO range is rather fragmented, so poke
    # holes for the iob and partition descriptors etc.
    self.bridge.ranges = \
        [
        AddrRange(self.t1000.puart0.pio_addr,
                  self.t1000.puart0.pio_addr + uart_pio_size - 1),
        AddrRange(self.disk0.pio_addr,
                  self.t1000.fake_jbi.pio_addr +
                  self.t1000.fake_jbi.pio_size - 1),
        AddrRange(self.t1000.fake_clk.pio_addr,
                  iob_man_addr - 1),
        AddrRange(self.t1000.fake_l2_1.pio_addr,
                  self.t1000.fake_ssi.pio_addr +
                  self.t1000.fake_ssi.pio_size - 1),
        AddrRange(self.t1000.hvuart.pio_addr,
                  self.t1000.hvuart.pio_addr + uart_pio_size - 1)
        ]
    self.reset_bin = binary('reset_new.bin')
    self.hypervisor_bin = binary('q_new.bin')
    self.openboot_bin = binary('openboot_new.bin')
    self.nvram_bin = binary('nvram1')
    self.hypervisor_desc_bin = binary('1up-hv.bin')
    self.partition_desc_bin = binary('1up-md.bin')

    self.system_port = self.membus.slave

    return self

def makeArmSystem(mem_mode, machine_type, num_cpus=1, mdesc=None,
                  dtb_filename=None, bare_metal=False, cmdline=None,
                  external_memory=""):
    assert machine_type

    default_dtbs = {
        "RealViewEB": None,
        "RealViewPBX": None,
        "VExpress_EMM": "vexpress.aarch32.ll_20131205.0-gem5.%dcpu.dtb" % num_cpus,
        "VExpress_EMM64": "vexpress.aarch64.20140821.dtb",
    }

    default_kernels = {
        "RealViewEB": "vmlinux.arm.smp.fb.2.6.38.8",
        "RealViewPBX": "vmlinux.arm.smp.fb.2.6.38.8",
        "VExpress_EMM": "vmlinux.aarch32.ll_20131205.0-gem5",
        "VExpress_EMM64": "vmlinux.aarch64.20140821",
    }

    pci_devices = []

    if bare_metal:
        self = ArmSystem()
    else:
        self = LinuxArmSystem()

    if not mdesc:
        # generic system
        mdesc = SysConfig()

    self.readfile = mdesc.script()
    self.iobus = IOXBar()
    self.membus = MemBus()
    self.membus.badaddr_responder.warn_access = "warn"
    self.bridge = Bridge(delay='50ns')
    self.bridge.master = self.iobus.slave
    self.bridge.slave = self.membus.master

    self.mem_mode = mem_mode

    platform_class = PlatformConfig.get(machine_type)
    # Resolve the real platform name, the original machine_type
    # variable might have been an alias.
    machine_type = platform_class.__name__
    self.realview = platform_class()

    if not dtb_filename and not bare_metal:
        try:
            dtb_filename = default_dtbs[machine_type]
        except KeyError:
            fatal("No DTB specified and no default DTB known for '%s'" % \
                  machine_type)

    if isinstance(self.realview, VExpress_EMM64):
        if os.path.split(mdesc.disk())[-1] == 'linux-aarch32-ael.img':
            print "Selected 64-bit ARM architecture, updating default disk image..."
            mdesc.diskname = 'linaro-minimal-aarch64.img'


    # Attach any PCI devices this platform supports
    self.realview.attachPciDevices()

    self.cf0 = CowIdeDisk(driveID='master')
    self.cf0.childImage(mdesc.disk())
    # Old platforms have a built-in IDE or CF controller. Default to
    # the IDE controller if both exist. New platforms expect the
    # storage controller to be added from the config script.
    if hasattr(self.realview, "ide"):
        self.realview.ide.disks = [self.cf0]
    elif hasattr(self.realview, "cf_ctrl"):
        self.realview.cf_ctrl.disks = [self.cf0]
    else:
        self.pci_ide = IdeController(disks=[self.cf0])
        pci_devices.append(self.pci_ide)

    self.mem_ranges = []
    size_remain = long(Addr(mdesc.mem()))
    for region in self.realview._mem_regions:
        if size_remain > long(region[1]):
            self.mem_ranges.append(AddrRange(region[0], size=region[1]))
            size_remain = size_remain - long(region[1])
        else:
            self.mem_ranges.append(AddrRange(region[0], size=size_remain))
            size_remain = 0
            break
        warn("Memory size specified spans more than one region. Creating" \
             " another memory controller for that range.")

    if size_remain > 0:
        fatal("The currently selected ARM platforms doesn't support" \
              " the amount of DRAM you've selected. Please try" \
              " another platform")

    if bare_metal:
        # EOT character on UART will end the simulation
        self.realview.uart.end_on_eot = True
    else:
        if machine_type in default_kernels:
            self.kernel = binary(default_kernels[machine_type])

        if dtb_filename:
            self.dtb_filename = binary(dtb_filename)

        self.machine_type = machine_type if machine_type in ArmMachineType.map \
                            else "DTOnly"

        # Ensure that writes to the UART actually go out early in the boot
        if not cmdline:
            cmdline = 'earlyprintk=pl011,0x1c090000 console=ttyAMA0 ' + \
                      'lpj=19988480 norandmaps rw loglevel=8 ' + \
                      'mem=%(mem)s root=%(rootdev)s'

        # When using external memory, gem5 writes the boot loader to nvmem
        # and then SST will read from it, but SST can only get to nvmem from
        # iobus, as gem5's membus is only used for initialization and
        # SST doesn't use it.  Attaching nvmem to iobus solves this issue.
        # During initialization, system_port -> membus -> iobus -> nvmem.
        if external_memory:
            self.realview.setupBootLoader(self.iobus,  self, binary)
        else:
            self.realview.setupBootLoader(self.membus, self, binary)
        self.gic_cpu_addr = self.realview.gic.cpu_addr
        self.flags_addr = self.realview.realview_io.pio_addr + 0x30

        # This check is for users who have previously put 'android' in
        # the disk image filename to tell the config scripts to
        # prepare the kernel with android-specific boot options. That
        # behavior has been replaced with a more explicit option per
        # the error message below. The disk can have any name now and
        # doesn't need to include 'android' substring.
        if (os.path.split(mdesc.disk())[-1]).lower().count('android'):
            if 'android' not in mdesc.os_type():
                fatal("It looks like you are trying to boot an Android " \
                      "platform.  To boot Android, you must specify " \
                      "--os-type with an appropriate Android release on " \
                      "the command line.")

        # android-specific tweaks
        if 'android' in mdesc.os_type():
            # generic tweaks
            cmdline += " init=/init"

            # release-specific tweaks
            if 'kitkat' in mdesc.os_type():
                cmdline += " androidboot.hardware=gem5 qemu=1 qemu.gles=0 " + \
                           "android.bootanim=0"

        self.boot_osflags = fillInCmdline(mdesc, cmdline)

    if external_memory:
        # I/O traffic enters iobus
        self.external_io = ExternalMaster(port_data="external_io",
                                          port_type=external_memory)
        self.external_io.port = self.iobus.slave

        # Ensure iocache only receives traffic destined for (actual) memory.
        self.iocache = ExternalSlave(port_data="iocache",
                                     port_type=external_memory,
                                     addr_ranges=self.mem_ranges)
        self.iocache.port = self.iobus.master

        # Let system_port get to nvmem and nothing else.
        self.bridge.ranges = [self.realview.nvmem.range]

        self.realview.attachOnChipIO(self.iobus)
    else:
        self.realview.attachOnChipIO(self.membus, self.bridge)

    # Attach off-chip devices
    self.realview.attachIO(self.iobus)
    for dev_id, dev in enumerate(pci_devices):
        dev.pci_bus, dev.pci_dev, dev.pci_func = (0, dev_id + 1, 0)
        self.realview.attachPciDevice(dev, self.iobus)

    self.intrctrl = IntrControl()
    self.terminal = Terminal()
    self.vncserver = VncServer()

    self.system_port = self.membus.slave

    return self


def makeLinuxMipsSystem(mem_mode, mdesc=None, cmdline=None):
    class BaseMalta(Malta):
        ethernet = NSGigE(pci_bus=0, pci_dev=1, pci_func=0)
        ide = IdeController(disks=[Parent.disk0, Parent.disk2],
                            pci_func=0, pci_dev=0, pci_bus=0)

    self = LinuxMipsSystem()
    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()
    self.iobus = IOXBar()
    self.membus = MemBus()
    self.bridge = Bridge(delay='50ns')
    self.mem_ranges = [AddrRange('1GB')]
    self.bridge.master = self.iobus.slave
    self.bridge.slave = self.membus.master
    self.disk0 = CowIdeDisk(driveID='master')
    self.disk2 = CowIdeDisk(driveID='master')
    self.disk0.childImage(mdesc.disk())
    self.disk2.childImage(disk('linux-bigswap2.img'))
    self.malta = BaseMalta()
    self.malta.attachIO(self.iobus)
    self.malta.ide.pio = self.iobus.master
    self.malta.ide.dma = self.iobus.slave
    self.malta.ethernet.pio = self.iobus.master
    self.malta.ethernet.dma = self.iobus.slave
    self.simple_disk = SimpleDisk(disk=RawDiskImage(image_file = mdesc.disk(),
                                               read_only = True))
    self.intrctrl = IntrControl()
    self.mem_mode = mem_mode
    self.terminal = Terminal()
    self.kernel = binary('mips/vmlinux')
    self.console = binary('mips/console')
    if not cmdline:
        cmdline = 'root=/dev/hda1 console=ttyS0'
    self.boot_osflags = fillInCmdline(mdesc, cmdline)

    self.system_port = self.membus.slave

    return self

def x86IOAddress(port):
    IO_address_space_base = 0x8000000000000000
    return IO_address_space_base + port

def connectX86ClassicSystem(x86_sys, numCPUs):
    # Constants similar to x86_traits.hh
    IO_address_space_base = 0x8000000000000000
    pci_config_address_space_base = 0xc000000000000000
    interrupts_address_space_base = 0xa000000000000000
    APIC_range_size = 1 << 12;

    x86_sys.membus = MemBus()

    # North Bridge
    x86_sys.iobus = IOXBar()
    x86_sys.bridge = Bridge(delay='50ns')
    x86_sys.bridge.master = x86_sys.iobus.slave
    x86_sys.bridge.slave = x86_sys.membus.master
    # Allow the bridge to pass through:
    #  1) kernel configured PCI device memory map address: address range
    #     [0xC0000000, 0xFFFF0000). (The upper 64kB are reserved for m5ops.)
    #  2) the bridge to pass through the IO APIC (two pages, already contained in 1),
    #  3) everything in the IO address range up to the local APIC, and
    #  4) then the entire PCI address space and beyond.
    x86_sys.bridge.ranges = \
        [
        AddrRange(0xC0000000, 0xFFFF0000),
        AddrRange(IO_address_space_base,
                  interrupts_address_space_base - 1),
        AddrRange(pci_config_address_space_base,
                  Addr.max)
        ]

    # Create a bridge from the IO bus to the memory bus to allow access to
    # the local APIC (two pages)
    x86_sys.apicbridge = Bridge(delay='50ns')
    x86_sys.apicbridge.slave = x86_sys.iobus.master
    x86_sys.apicbridge.master = x86_sys.membus.slave
    x86_sys.apicbridge.ranges = [AddrRange(interrupts_address_space_base,
                                           interrupts_address_space_base +
                                           numCPUs * APIC_range_size
                                           - 1)]

    # connect the io bus
    x86_sys.pc.attachIO(x86_sys.iobus)

    x86_sys.system_port = x86_sys.membus.slave

def connectX86RubySystem(x86_sys):
    # North Bridge
    x86_sys.iobus = IOXBar()

    # add the ide to the list of dma devices that later need to attach to
    # dma controllers
    x86_sys._dma_ports = [x86_sys.pc.south_bridge.ide.dma]
    x86_sys.pc.attachIO(x86_sys.iobus, x86_sys._dma_ports)


def makeX86System(mem_mode, numCPUs=1, mdesc=None, self=None, Ruby=False):
    if self == None:
        self = X86System()

    if not mdesc:
        # generic system
        mdesc = SysConfig()
    self.readfile = mdesc.script()

    self.mem_mode = mem_mode

    # Physical memory
    # On the PC platform, the memory region 0xC0000000-0xFFFFFFFF is reserved
    # for various devices.  Hence, if the physical memory size is greater than
    # 3GB, we need to split it into two parts.
    excess_mem_size = \
        convert.toMemorySize(mdesc.mem()) - convert.toMemorySize('3GB')
    if excess_mem_size <= 0:
        self.mem_ranges = [AddrRange(mdesc.mem())]
    else:
        warn("Physical memory size specified is %s which is greater than " \
             "3GB.  Twice the number of memory controllers would be " \
             "created."  % (mdesc.mem()))

        self.mem_ranges = [AddrRange('3GB'),
            AddrRange(Addr('4GB'), size = excess_mem_size)]

    # Platform
    self.pc = Pc()

    # Create and connect the busses required by each memory system
    if Ruby:
        connectX86RubySystem(self)
    else:
        connectX86ClassicSystem(self, numCPUs)

    self.intrctrl = IntrControl()

    # Disks
    disk0 = CowIdeDisk(driveID='master')
    disk2 = CowIdeDisk(driveID='master')
    disk0.childImage(mdesc.disk())
    disk2.childImage(disk('linux-bigswap2.img'))
    self.pc.south_bridge.ide.disks = [disk0, disk2]

    # Add in a Bios information structure.
    structures = [X86SMBiosBiosInformation()]
    self.smbios_table.structures = structures

    # Set up the Intel MP table
    base_entries = []
    ext_entries = []
    for i in xrange(numCPUs):
        bp = X86IntelMPProcessor(
                local_apic_id = i,
                local_apic_version = 0x14,
                enable = True,
                bootstrap = (i == 0))
        base_entries.append(bp)
    io_apic = X86IntelMPIOAPIC(
            id = numCPUs,
            version = 0x11,
            enable = True,
            address = 0xfec00000)
    self.pc.south_bridge.io_apic.apic_id = io_apic.id
    base_entries.append(io_apic)
    # In gem5 Pc::calcPciConfigAddr(), it required "assert(bus==0)",
    # but linux kernel cannot config PCI device if it was not connected to PCI bus,
    # so we fix PCI bus id to 0, and ISA bus id to 1.
    pci_bus = X86IntelMPBus(bus_id = 0, bus_type='PCI   ')
    base_entries.append(pci_bus)
    isa_bus = X86IntelMPBus(bus_id = 1, bus_type='ISA   ')
    base_entries.append(isa_bus)
    connect_busses = X86IntelMPBusHierarchy(bus_id=1,
            subtractive_decode=True, parent_bus=0)
    ext_entries.append(connect_busses)
    pci_dev4_inta = X86IntelMPIOIntAssignment(
            interrupt_type = 'INT',
            polarity = 'ConformPolarity',
            trigger = 'ConformTrigger',
            source_bus_id = 0,
            source_bus_irq = 0 + (4 << 2),
            dest_io_apic_id = io_apic.id,
            dest_io_apic_intin = 16)
    base_entries.append(pci_dev4_inta)
    def assignISAInt(irq, apicPin):
        assign_8259_to_apic = X86IntelMPIOIntAssignment(
                interrupt_type = 'ExtInt',
                polarity = 'ConformPolarity',
                trigger = 'ConformTrigger',
                source_bus_id = 1,
                source_bus_irq = irq,
                dest_io_apic_id = io_apic.id,
                dest_io_apic_intin = 0)
        base_entries.append(assign_8259_to_apic)
        assign_to_apic = X86IntelMPIOIntAssignment(
                interrupt_type = 'INT',
                polarity = 'ConformPolarity',
                trigger = 'ConformTrigger',
                source_bus_id = 1,
                source_bus_irq = irq,
                dest_io_apic_id = io_apic.id,
                dest_io_apic_intin = apicPin)
        base_entries.append(assign_to_apic)
    assignISAInt(0, 2)
    assignISAInt(1, 1)
    for i in range(3, 15):
        assignISAInt(i, i)
    self.intel_mp_table.base_entries = base_entries
    self.intel_mp_table.ext_entries = ext_entries

def makeLinuxX86System(mem_mode, numCPUs=1, mdesc=None, Ruby=False,
                       cmdline=None):
    self = LinuxX86System()

    # Build up the x86 system and then specialize it for Linux
    makeX86System(mem_mode, numCPUs, mdesc, self, Ruby)

    # We assume below that there's at least 1MB of memory. We'll require 2
    # just to avoid corner cases.
    phys_mem_size = sum(map(lambda r: r.size(), self.mem_ranges))
    assert(phys_mem_size >= 0x200000)
    assert(len(self.mem_ranges) <= 2)

    entries = \
       [
        # Mark the first megabyte of memory as reserved
        X86E820Entry(addr = 0, size = '639kB', range_type = 1),
        X86E820Entry(addr = 0x9fc00, size = '385kB', range_type = 2),
        # Mark the rest of physical memory as available
        X86E820Entry(addr = 0x100000,
                size = '%dB' % (self.mem_ranges[0].size() - 0x100000),
                range_type = 1),
        ]

    # Mark [mem_size, 3GB) as reserved if memory less than 3GB, which force
    # IO devices to be mapped to [0xC0000000, 0xFFFF0000). Requests to this
    # specific range can pass though bridge to iobus.
    if len(self.mem_ranges) == 1:
        entries.append(X86E820Entry(addr = self.mem_ranges[0].size(),
            size='%dB' % (0xC0000000 - self.mem_ranges[0].size()),
            range_type=2))

    # Reserve the last 16kB of the 32-bit address space for the m5op interface
    entries.append(X86E820Entry(addr=0xFFFF0000, size='64kB', range_type=2))

    # In case the physical memory is greater than 3GB, we split it into two
    # parts and add a separate e820 entry for the second part.  This entry
    # starts at 0x100000000,  which is the first address after the space
    # reserved for devices.
    if len(self.mem_ranges) == 2:
        entries.append(X86E820Entry(addr = 0x100000000,
            size = '%dB' % (self.mem_ranges[1].size()), range_type = 1))

    self.e820_table.entries = entries

    # Command line
    if not cmdline:
        cmdline = 'earlyprintk=ttyS0 console=ttyS0 lpj=7999923 root=/dev/hda1'
    self.boot_osflags = fillInCmdline(mdesc, cmdline)
    self.kernel = binary('x86_64-vmlinux-2.6.22.9')
    return self


def makeDualRoot(full_system, testSystem, driveSystem, dumpfile):
    self = Root(full_system = full_system)
    self.testsys = testSystem
    self.drivesys = driveSystem
    self.etherlink = EtherLink()

    if hasattr(testSystem, 'realview'):
        self.etherlink.int0 = Parent.testsys.realview.ethernet.interface
        self.etherlink.int1 = Parent.drivesys.realview.ethernet.interface
    elif hasattr(testSystem, 'tsunami'):
        self.etherlink.int0 = Parent.testsys.tsunami.ethernet.interface
        self.etherlink.int1 = Parent.drivesys.tsunami.ethernet.interface
    else:
        fatal("Don't know how to connect these system together")

    if dumpfile:
        self.etherdump = EtherDump(file=dumpfile)
        self.etherlink.dump = Parent.etherdump

    return self


def makeDistRoot(testSystem,
                 rank,
                 size,
                 server_name,
                 server_port,
                 sync_repeat,
                 sync_start,
                 linkspeed,
                 linkdelay,
                 dumpfile):
    self = Root(full_system = True)
    self.testsys = testSystem

    self.etherlink = DistEtherLink(speed = linkspeed,
                                   delay = linkdelay,
                                   dist_rank = rank,
                                   dist_size = size,
                                   server_name = server_name,
                                   server_port = server_port,
                                   sync_start = sync_start,
                                   sync_repeat = sync_repeat)

    if hasattr(testSystem, 'realview'):
        self.etherlink.int0 = Parent.testsys.realview.ethernet.interface
    elif hasattr(testSystem, 'tsunami'):
        self.etherlink.int0 = Parent.testsys.tsunami.ethernet.interface
    else:
        fatal("Don't know how to connect DistEtherLink to this system")

    if dumpfile:
        self.etherdump = EtherDump(file=dumpfile)
        self.etherlink.dump = Parent.etherdump

    return self
