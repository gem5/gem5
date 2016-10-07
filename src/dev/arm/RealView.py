# Copyright (c) 2009-2015 ARM Limited
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
#          Gabe Black
#          William Wang

from m5.params import *
from m5.proxy import *
from ClockDomain import ClockDomain
from VoltageDomain import VoltageDomain
from Device import BasicPioDevice, PioDevice, IsaFake, BadAddr, DmaDevice
from PciHost import *
from Ethernet import NSGigE, IGbE_igb, IGbE_e1000
from Ide import *
from Platform import Platform
from Terminal import Terminal
from Uart import Uart
from SimpleMemory import SimpleMemory
from Gic import *
from EnergyCtrl import EnergyCtrl
from ClockDomain import SrcClockDomain
from SubSystem import SubSystem

class AmbaPioDevice(BasicPioDevice):
    type = 'AmbaPioDevice'
    abstract = True
    cxx_header = "dev/arm/amba_device.hh"
    amba_id = Param.UInt32("ID of AMBA device for kernel detection")

class AmbaIntDevice(AmbaPioDevice):
    type = 'AmbaIntDevice'
    abstract = True
    cxx_header = "dev/arm/amba_device.hh"
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    int_num = Param.UInt32("Interrupt number that connects to GIC")
    int_delay = Param.Latency("100ns",
            "Time between action and interrupt generation by device")

class AmbaDmaDevice(DmaDevice):
    type = 'AmbaDmaDevice'
    abstract = True
    cxx_header = "dev/arm/amba_device.hh"
    pio_addr = Param.Addr("Address for AMBA slave interface")
    pio_latency = Param.Latency("10ns", "Time between action and write/read result by AMBA DMA Device")
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    int_num = Param.UInt32("Interrupt number that connects to GIC")
    amba_id = Param.UInt32("ID of AMBA device for kernel detection")

class A9SCU(BasicPioDevice):
    type = 'A9SCU'
    cxx_header = "dev/arm/a9scu.hh"

class ArmPciIntRouting(Enum): vals = [
    'ARM_PCI_INT_STATIC',
    'ARM_PCI_INT_DEV',
    'ARM_PCI_INT_PIN',
    ]

class GenericArmPciHost(GenericPciHost):
    type = 'GenericArmPciHost'
    cxx_header = "dev/arm/pci_host.hh"

    int_policy = Param.ArmPciIntRouting("PCI interrupt routing policy")
    int_base = Param.Unsigned("PCI interrupt base")
    int_count = Param.Unsigned("Maximum number of interrupts used by this host")

class RealViewCtrl(BasicPioDevice):
    type = 'RealViewCtrl'
    cxx_header = "dev/arm/rv_ctrl.hh"
    proc_id0 = Param.UInt32(0x0C000000, "Processor ID, SYS_PROCID")
    proc_id1 = Param.UInt32(0x0C000222, "Processor ID, SYS_PROCID1")
    idreg = Param.UInt32(0x00000000, "ID Register, SYS_ID")

class RealViewOsc(ClockDomain):
    type = 'RealViewOsc'
    cxx_header = "dev/arm/rv_ctrl.hh"

    parent = Param.RealViewCtrl(Parent.any, "RealView controller")

    # TODO: We currently don't have the notion of a clock source,
    # which means we have to associate oscillators with a voltage
    # source.
    voltage_domain = Param.VoltageDomain(Parent.voltage_domain,
                                         "Voltage domain")

    # See ARM DUI 0447J (ARM Motherboard Express uATX -- V2M-P1) and
    # the individual core/logic tile reference manuals for details
    # about the site/position/dcc/device allocation.
    site = Param.UInt8("Board Site")
    position = Param.UInt8("Position in device stack")
    dcc = Param.UInt8("Daughterboard Configuration Controller")
    device = Param.UInt8("Device ID")

    freq = Param.Clock("Default frequency")

class RealViewTemperatureSensor(SimObject):
    type = 'RealViewTemperatureSensor'
    cxx_header = "dev/arm/rv_ctrl.hh"

    parent = Param.RealViewCtrl(Parent.any, "RealView controller")

    system = Param.System(Parent.any, "system")

    # See ARM DUI 0447J (ARM Motherboard Express uATX -- V2M-P1) and
    # the individual core/logic tile reference manuals for details
    # about the site/position/dcc/device allocation.
    site = Param.UInt8("Board Site")
    position = Param.UInt8("Position in device stack")
    dcc = Param.UInt8("Daughterboard Configuration Controller")
    device = Param.UInt8("Device ID")

class VExpressMCC(SubSystem):
    """ARM V2M-P1 Motherboard Configuration Controller

This subsystem describes a subset of the devices that sit behind the
motherboard configuration controller on the the ARM Motherboard
Express (V2M-P1) motherboard. See ARM DUI 0447J for details.
    """

    class Osc(RealViewOsc):
        site, position, dcc = (0, 0, 0)

    class Temperature(RealViewTemperatureSensor):
        site, position, dcc = (0, 0, 0)

    osc_mcc = Osc(device=0, freq="50MHz")
    osc_clcd = Osc(device=1, freq="23.75MHz")
    osc_peripheral = Osc(device=2, freq="24MHz")
    osc_system_bus = Osc(device=4, freq="24MHz")

    # See Table 4.19 in ARM DUI 0447J (Motherboard Express uATX TRM).
    temp_crtl = Temperature(device=0)

class CoreTile2A15DCC(SubSystem):
    """ARM CoreTile Express A15x2 Daughterboard Configuration Controller

This subsystem describes a subset of the devices that sit behind the
daughterboard configuration controller on a CoreTile Express A15x2. See
ARM DUI 0604E for details.
    """

    class Osc(RealViewOsc):
        site, position, dcc = (1, 0, 0)

    # See Table 2.8 in ARM DUI 0604E (CoreTile Express A15x2 TRM)
    osc_cpu = Osc(device=0, freq="60MHz")
    osc_hsbm = Osc(device=4, freq="40MHz")
    osc_pxl = Osc(device=5, freq="23.75MHz")
    osc_smb = Osc(device=6, freq="50MHz")
    osc_sys = Osc(device=7, freq="60MHz")
    osc_ddr = Osc(device=8, freq="40MHz")

class VGic(PioDevice):
    type = 'VGic'
    cxx_header = "dev/arm/vgic.hh"
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    vcpu_addr = Param.Addr(0, "Address for vcpu interfaces")
    hv_addr = Param.Addr(0, "Address for hv control")
    pio_delay = Param.Latency('10ns', "Delay for PIO r/w")
   # The number of list registers is not currently configurable at runtime.
    ppint = Param.UInt32("HV maintenance interrupt number")

class AmbaFake(AmbaPioDevice):
    type = 'AmbaFake'
    cxx_header = "dev/arm/amba_fake.hh"
    ignore_access = Param.Bool(False, "Ignore reads/writes to this device, (e.g. IsaFake + AMBA)")
    amba_id = 0;

class Pl011(Uart):
    type = 'Pl011'
    cxx_header = "dev/arm/pl011.hh"
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    int_num = Param.UInt32("Interrupt number that connects to GIC")
    end_on_eot = Param.Bool(False, "End the simulation when a EOT is received on the UART")
    int_delay = Param.Latency("100ns", "Time between action and interrupt generation by UART")

class Sp804(AmbaPioDevice):
    type = 'Sp804'
    cxx_header = "dev/arm/timer_sp804.hh"
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    int_num0 = Param.UInt32("Interrupt number that connects to GIC")
    clock0 = Param.Clock('1MHz', "Clock speed of the input")
    int_num1 = Param.UInt32("Interrupt number that connects to GIC")
    clock1 = Param.Clock('1MHz', "Clock speed of the input")
    amba_id = 0x00141804

class CpuLocalTimer(BasicPioDevice):
    type = 'CpuLocalTimer'
    cxx_header = "dev/arm/timer_cpulocal.hh"
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    int_num_timer = Param.UInt32("Interrrupt number used per-cpu to GIC")
    int_num_watchdog = Param.UInt32("Interrupt number for per-cpu watchdog to GIC")

class GenericTimer(SimObject):
    type = 'GenericTimer'
    cxx_header = "dev/arm/generic_timer.hh"
    system = Param.ArmSystem(Parent.any, "system")
    gic = Param.BaseGic(Parent.any, "GIC to use for interrupting")
    # @todo: for now only two timers per CPU is supported, which is the
    # normal behaviour when security extensions are disabled.
    int_phys = Param.UInt32("Physical timer interrupt number")
    int_virt = Param.UInt32("Virtual timer interrupt number")

class GenericTimerMem(PioDevice):
    type = 'GenericTimerMem'
    cxx_header = "dev/arm/generic_timer.hh"
    gic = Param.BaseGic(Parent.any, "GIC to use for interrupting")

    base = Param.Addr(0, "Base address")

    int_phys = Param.UInt32("Interrupt number")
    int_virt = Param.UInt32("Interrupt number")

class PL031(AmbaIntDevice):
    type = 'PL031'
    cxx_header = "dev/arm/rtc_pl031.hh"
    time = Param.Time('01/01/2009', "System time to use ('Now' for actual time)")
    amba_id = 0x00341031

class Pl050(AmbaIntDevice):
    type = 'Pl050'
    cxx_header = "dev/arm/kmi.hh"
    vnc = Param.VncInput(Parent.any, "Vnc server for remote frame buffer display")
    is_mouse = Param.Bool(False, "Is this interface a mouse, if not a keyboard")
    int_delay = '1us'
    amba_id = 0x00141050

class Pl111(AmbaDmaDevice):
    type = 'Pl111'
    cxx_header = "dev/arm/pl111.hh"
    pixel_clock = Param.Clock('24MHz', "Pixel clock")
    vnc   = Param.VncInput(Parent.any, "Vnc server for remote frame buffer display")
    amba_id = 0x00141111
    enable_capture = Param.Bool(True, "capture frame to system.framebuffer.bmp")

class HDLcd(AmbaDmaDevice):
    type = 'HDLcd'
    cxx_header = "dev/arm/hdlcd.hh"
    vnc = Param.VncInput(Parent.any, "Vnc server for remote frame buffer "
                                     "display")
    amba_id = 0x00141000
    workaround_swap_rb = Param.Bool(False, "Workaround incorrect color "
                                    "selector order in some kernels")
    workaround_dma_line_count = Param.Bool(True, "Workaround incorrect "
                                           "DMA line count (off by 1)")
    enable_capture = Param.Bool(True, "capture frame to system.framebuffer.bmp")

    pixel_buffer_size = Param.MemorySize32("2kB", "Size of address range")

    pxl_clk = Param.ClockDomain("Pixel clock source")
    pixel_chunk = Param.Unsigned(32, "Number of pixels to handle in one batch")

class RealView(Platform):
    type = 'RealView'
    cxx_header = "dev/arm/realview.hh"
    system = Param.System(Parent.any, "system")
    _mem_regions = [(Addr(0), Addr('256MB'))]

    def _on_chip_devices(self):
        return []

    def _off_chip_devices(self):
        return []

    _off_chip_ranges = []

    def _attach_device(self, device, bus, dma_ports=None):
        if hasattr(device, "pio"):
            device.pio = bus.master
        if hasattr(device, "dma"):
            if dma_ports is None:
                device.dma = bus.slave
            else:
                dma_ports.append(device.dma)

    def _attach_io(self, devices, *args, **kwargs):
        for d in devices:
            self._attach_device(d, *args, **kwargs)

    def _attach_clk(self, devices, clkdomain):
        for d in devices:
            if hasattr(d, "clk_domain"):
                d.clk_domain = clkdomain

    def attachPciDevices(self):
        pass

    def enableMSIX(self):
        pass

    def onChipIOClkDomain(self, clkdomain):
        self._attach_clk(self._on_chip_devices(), clkdomain)

    def offChipIOClkDomain(self, clkdomain):
        self._attach_clk(self._off_chip_devices(), clkdomain)

    def attachOnChipIO(self, bus, bridge=None, **kwargs):
        self._attach_io(self._on_chip_devices(), bus, **kwargs)
        if bridge:
            bridge.ranges = self._off_chip_ranges

    def attachIO(self, *args, **kwargs):
        self._attach_io(self._off_chip_devices(), *args, **kwargs)


    def setupBootLoader(self, mem_bus, cur_sys, loc):
        self.nvmem = SimpleMemory(range = AddrRange('2GB', size = '64MB'),
                                  conf_table_reported = False)
        self.nvmem.port = mem_bus.master
        cur_sys.boot_loader = loc('boot.arm')
        cur_sys.atags_addr = 0x100
        cur_sys.load_addr_mask = 0xfffffff
        cur_sys.load_offset = 0


# Reference for memory map and interrupt number
# RealView Platform Baseboard Explore for Cortex-A9 User Guide(ARM DUI 0440A)
# Chapter 4: Programmer's Reference
class RealViewPBX(RealView):
    uart = Pl011(pio_addr=0x10009000, int_num=44)
    realview_io = RealViewCtrl(pio_addr=0x10000000)
    mcc = VExpressMCC()
    dcc = CoreTile2A15DCC()
    gic = Pl390()
    pci_host = GenericPciHost(
        conf_base=0x30000000, conf_size='256MB', conf_device_bits=16,
        pci_pio_base=0)
    timer0 = Sp804(int_num0=36, int_num1=36, pio_addr=0x10011000)
    timer1 = Sp804(int_num0=37, int_num1=37, pio_addr=0x10012000)
    local_cpu_timer = CpuLocalTimer(int_num_timer=29, int_num_watchdog=30, pio_addr=0x1f000600)
    clcd = Pl111(pio_addr=0x10020000, int_num=55)
    kmi0   = Pl050(pio_addr=0x10006000, int_num=52)
    kmi1   = Pl050(pio_addr=0x10007000, int_num=53, is_mouse=True)
    a9scu  = A9SCU(pio_addr=0x1f000000)
    cf_ctrl = IdeController(disks=[], pci_func=0, pci_dev=7, pci_bus=2,
                            io_shift = 1, ctrl_offset = 2, Command = 0x1,
                            BAR0 = 0x18000000, BAR0Size = '16B',
                            BAR1 = 0x18000100, BAR1Size = '1B',
                            BAR0LegacyIO = True, BAR1LegacyIO = True)


    l2x0_fake     = IsaFake(pio_addr=0x1f002000, pio_size=0xfff)
    flash_fake    = IsaFake(pio_addr=0x40000000, pio_size=0x20000000,
                            fake_mem=True)
    dmac_fake     = AmbaFake(pio_addr=0x10030000)
    uart1_fake    = AmbaFake(pio_addr=0x1000a000)
    uart2_fake    = AmbaFake(pio_addr=0x1000b000)
    uart3_fake    = AmbaFake(pio_addr=0x1000c000)
    smc_fake      = AmbaFake(pio_addr=0x100e1000)
    sp810_fake    = AmbaFake(pio_addr=0x10001000, ignore_access=True)
    watchdog_fake = AmbaFake(pio_addr=0x10010000)
    gpio0_fake    = AmbaFake(pio_addr=0x10013000)
    gpio1_fake    = AmbaFake(pio_addr=0x10014000)
    gpio2_fake    = AmbaFake(pio_addr=0x10015000)
    ssp_fake      = AmbaFake(pio_addr=0x1000d000)
    sci_fake      = AmbaFake(pio_addr=0x1000e000)
    aaci_fake     = AmbaFake(pio_addr=0x10004000)
    mmc_fake      = AmbaFake(pio_addr=0x10005000)
    rtc           = PL031(pio_addr=0x10017000, int_num=42)
    energy_ctrl   = EnergyCtrl(pio_addr=0x1000f000)


    # Attach I/O devices that are on chip and also set the appropriate
    # ranges for the bridge
    def attachOnChipIO(self, bus, bridge):
       self.gic.pio = bus.master
       self.l2x0_fake.pio = bus.master
       self.a9scu.pio = bus.master
       self.local_cpu_timer.pio = bus.master
       # Bridge ranges based on excluding what is part of on-chip I/O
       # (gic, l2x0, a9scu, local_cpu_timer)
       bridge.ranges = [AddrRange(self.realview_io.pio_addr,
                                  self.a9scu.pio_addr - 1),
                        AddrRange(self.flash_fake.pio_addr,
                                  self.flash_fake.pio_addr + \
                                  self.flash_fake.pio_size - 1)]

    # Set the clock domain for IO objects that are considered
    # to be "close" to the cores.
    def onChipIOClkDomain(self, clkdomain):
        self.gic.clk_domain             = clkdomain
        self.l2x0_fake.clk_domain       = clkdomain
        self.a9scu.clkdomain            = clkdomain
        self.local_cpu_timer.clk_domain = clkdomain

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
       self.uart.pio          = bus.master
       self.realview_io.pio   = bus.master
       self.pci_host.pio      = bus.master
       self.timer0.pio        = bus.master
       self.timer1.pio        = bus.master
       self.clcd.pio          = bus.master
       self.clcd.dma          = bus.slave
       self.kmi0.pio          = bus.master
       self.kmi1.pio          = bus.master
       self.cf_ctrl.pio       = bus.master
       self.cf_ctrl.dma       = bus.slave
       self.dmac_fake.pio     = bus.master
       self.uart1_fake.pio    = bus.master
       self.uart2_fake.pio    = bus.master
       self.uart3_fake.pio    = bus.master
       self.smc_fake.pio      = bus.master
       self.sp810_fake.pio    = bus.master
       self.watchdog_fake.pio = bus.master
       self.gpio0_fake.pio    = bus.master
       self.gpio1_fake.pio    = bus.master
       self.gpio2_fake.pio    = bus.master
       self.ssp_fake.pio      = bus.master
       self.sci_fake.pio      = bus.master
       self.aaci_fake.pio     = bus.master
       self.mmc_fake.pio      = bus.master
       self.rtc.pio           = bus.master
       self.flash_fake.pio    = bus.master
       self.energy_ctrl.pio   = bus.master

    # Set the clock domain for IO objects that are considered
    # to be "far" away from the cores.
    def offChipIOClkDomain(self, clkdomain):
        self.uart.clk_domain          = clkdomain
        self.realview_io.clk_domain   = clkdomain
        self.timer0.clk_domain        = clkdomain
        self.timer1.clk_domain        = clkdomain
        self.clcd.clk_domain          = clkdomain
        self.kmi0.clk_domain          = clkdomain
        self.kmi1.clk_domain          = clkdomain
        self.cf_ctrl.clk_domain       = clkdomain
        self.dmac_fake.clk_domain     = clkdomain
        self.uart1_fake.clk_domain    = clkdomain
        self.uart2_fake.clk_domain    = clkdomain
        self.uart3_fake.clk_domain    = clkdomain
        self.smc_fake.clk_domain      = clkdomain
        self.sp810_fake.clk_domain    = clkdomain
        self.watchdog_fake.clk_domain = clkdomain
        self.gpio0_fake.clk_domain    = clkdomain
        self.gpio1_fake.clk_domain    = clkdomain
        self.gpio2_fake.clk_domain    = clkdomain
        self.ssp_fake.clk_domain      = clkdomain
        self.sci_fake.clk_domain      = clkdomain
        self.aaci_fake.clk_domain     = clkdomain
        self.mmc_fake.clk_domain      = clkdomain
        self.rtc.clk_domain           = clkdomain
        self.flash_fake.clk_domain    = clkdomain
        self.energy_ctrl.clk_domain   = clkdomain

# Reference for memory map and interrupt number
# RealView Emulation Baseboard User Guide (ARM DUI 0143B)
# Chapter 4: Programmer's Reference
class RealViewEB(RealView):
    uart = Pl011(pio_addr=0x10009000, int_num=44)
    realview_io = RealViewCtrl(pio_addr=0x10000000, idreg=0x01400500)
    mcc = VExpressMCC()
    dcc = CoreTile2A15DCC()
    gic = Pl390(dist_addr=0x10041000, cpu_addr=0x10040000)
    timer0 = Sp804(int_num0=36, int_num1=36, pio_addr=0x10011000)
    timer1 = Sp804(int_num0=37, int_num1=37, pio_addr=0x10012000)
    clcd   = Pl111(pio_addr=0x10020000, int_num=23)
    kmi0   = Pl050(pio_addr=0x10006000, int_num=20)
    kmi1   = Pl050(pio_addr=0x10007000, int_num=21, is_mouse=True)

    l2x0_fake     = IsaFake(pio_addr=0x1f002000, pio_size=0xfff, warn_access="1")
    flash_fake    = IsaFake(pio_addr=0x40000000, pio_size=0x20000000-1,
                            fake_mem=True)
    dmac_fake     = AmbaFake(pio_addr=0x10030000)
    uart1_fake    = AmbaFake(pio_addr=0x1000a000)
    uart2_fake    = AmbaFake(pio_addr=0x1000b000)
    uart3_fake    = AmbaFake(pio_addr=0x1000c000)
    smcreg_fake   = IsaFake(pio_addr=0x10080000, pio_size=0x10000-1)
    smc_fake      = AmbaFake(pio_addr=0x100e1000)
    sp810_fake    = AmbaFake(pio_addr=0x10001000, ignore_access=True)
    watchdog_fake = AmbaFake(pio_addr=0x10010000)
    gpio0_fake    = AmbaFake(pio_addr=0x10013000)
    gpio1_fake    = AmbaFake(pio_addr=0x10014000)
    gpio2_fake    = AmbaFake(pio_addr=0x10015000)
    ssp_fake      = AmbaFake(pio_addr=0x1000d000)
    sci_fake      = AmbaFake(pio_addr=0x1000e000)
    aaci_fake     = AmbaFake(pio_addr=0x10004000)
    mmc_fake      = AmbaFake(pio_addr=0x10005000)
    rtc_fake      = AmbaFake(pio_addr=0x10017000, amba_id=0x41031)
    energy_ctrl   = EnergyCtrl(pio_addr=0x1000f000)

    # Attach I/O devices that are on chip and also set the appropriate
    # ranges for the bridge
    def attachOnChipIO(self, bus, bridge):
       self.gic.pio = bus.master
       self.l2x0_fake.pio = bus.master
       # Bridge ranges based on excluding what is part of on-chip I/O
       # (gic, l2x0)
       bridge.ranges = [AddrRange(self.realview_io.pio_addr,
                                  self.gic.cpu_addr - 1),
                        AddrRange(self.flash_fake.pio_addr, Addr.max)]

    # Set the clock domain for IO objects that are considered
    # to be "close" to the cores.
    def onChipIOClkDomain(self, clkdomain):
        self.gic.clk_domain             = clkdomain
        self.l2x0_fake.clk_domain       = clkdomain

    # Attach I/O devices to specified bus object.  Can't do this
    # earlier, since the bus object itself is typically defined at the
    # System level.
    def attachIO(self, bus):
       self.uart.pio          = bus.master
       self.realview_io.pio   = bus.master
       self.pci_host.pio      = bus.master
       self.timer0.pio        = bus.master
       self.timer1.pio        = bus.master
       self.clcd.pio          = bus.master
       self.clcd.dma          = bus.slave
       self.kmi0.pio          = bus.master
       self.kmi1.pio          = bus.master
       self.dmac_fake.pio     = bus.master
       self.uart1_fake.pio    = bus.master
       self.uart2_fake.pio    = bus.master
       self.uart3_fake.pio    = bus.master
       self.smc_fake.pio      = bus.master
       self.sp810_fake.pio    = bus.master
       self.watchdog_fake.pio = bus.master
       self.gpio0_fake.pio    = bus.master
       self.gpio1_fake.pio    = bus.master
       self.gpio2_fake.pio    = bus.master
       self.ssp_fake.pio      = bus.master
       self.sci_fake.pio      = bus.master
       self.aaci_fake.pio     = bus.master
       self.mmc_fake.pio      = bus.master
       self.rtc_fake.pio      = bus.master
       self.flash_fake.pio    = bus.master
       self.smcreg_fake.pio   = bus.master
       self.energy_ctrl.pio   = bus.master

    # Set the clock domain for IO objects that are considered
    # to be "far" away from the cores.
    def offChipIOClkDomain(self, clkdomain):
        self.uart.clk_domain          = clkdomain
        self.realview_io.clk_domain   = clkdomain
        self.timer0.clk_domain        = clkdomain
        self.timer1.clk_domain        = clkdomain
        self.clcd.clk_domain          = clkdomain
        self.kmi0.clk_domain          = clkdomain
        self.kmi1.clk_domain          = clkdomain
        self.dmac_fake.clk_domain     = clkdomain
        self.uart1_fake.clk_domain    = clkdomain
        self.uart2_fake.clk_domain    = clkdomain
        self.uart3_fake.clk_domain    = clkdomain
        self.smc_fake.clk_domain      = clkdomain
        self.sp810_fake.clk_domain    = clkdomain
        self.watchdog_fake.clk_domain = clkdomain
        self.gpio0_fake.clk_domain    = clkdomain
        self.gpio1_fake.clk_domain    = clkdomain
        self.gpio2_fake.clk_domain    = clkdomain
        self.ssp_fake.clk_domain      = clkdomain
        self.sci_fake.clk_domain      = clkdomain
        self.aaci_fake.clk_domain     = clkdomain
        self.mmc_fake.clk_domain      = clkdomain
        self.rtc.clk_domain           = clkdomain
        self.flash_fake.clk_domain    = clkdomain
        self.smcreg_fake.clk_domain   = clkdomain
        self.energy_ctrl.clk_domain   = clkdomain

class VExpress_EMM(RealView):
    _mem_regions = [(Addr('2GB'), Addr('2GB'))]
    uart = Pl011(pio_addr=0x1c090000, int_num=37)
    realview_io = RealViewCtrl(
        proc_id0=0x14000000, proc_id1=0x14000000,
        idreg=0x02250000, pio_addr=0x1C010000)
    mcc = VExpressMCC()
    dcc = CoreTile2A15DCC()
    gic = Pl390(dist_addr=0x2C001000, cpu_addr=0x2C002000)
    pci_host = GenericPciHost(
        conf_base=0x30000000, conf_size='256MB', conf_device_bits=16,
        pci_pio_base=0)
    local_cpu_timer = CpuLocalTimer(int_num_timer=29, int_num_watchdog=30, pio_addr=0x2C080000)
    generic_timer = GenericTimer(int_phys=29, int_virt=27)
    timer0 = Sp804(int_num0=34, int_num1=34, pio_addr=0x1C110000, clock0='1MHz', clock1='1MHz')
    timer1 = Sp804(int_num0=35, int_num1=35, pio_addr=0x1C120000, clock0='1MHz', clock1='1MHz')
    clcd   = Pl111(pio_addr=0x1c1f0000, int_num=46)
    hdlcd  = HDLcd(pxl_clk=dcc.osc_pxl,
                   pio_addr=0x2b000000, int_num=117,
                   workaround_swap_rb=True)
    kmi0   = Pl050(pio_addr=0x1c060000, int_num=44)
    kmi1   = Pl050(pio_addr=0x1c070000, int_num=45, is_mouse=True)
    vgic   = VGic(vcpu_addr=0x2c006000, hv_addr=0x2c004000, ppint=25)
    cf_ctrl = IdeController(disks=[], pci_func=0, pci_dev=0, pci_bus=2,
                            io_shift = 2, ctrl_offset = 2, Command = 0x1,
                            BAR0 = 0x1C1A0000, BAR0Size = '256B',
                            BAR1 = 0x1C1A0100, BAR1Size = '4096B',
                            BAR0LegacyIO = True, BAR1LegacyIO = True)

    vram           = SimpleMemory(range = AddrRange(0x18000000, size='32MB'),
                                  conf_table_reported = False)
    rtc            = PL031(pio_addr=0x1C170000, int_num=36)

    l2x0_fake      = IsaFake(pio_addr=0x2C100000, pio_size=0xfff)
    uart1_fake     = AmbaFake(pio_addr=0x1C0A0000)
    uart2_fake     = AmbaFake(pio_addr=0x1C0B0000)
    uart3_fake     = AmbaFake(pio_addr=0x1C0C0000)
    sp810_fake     = AmbaFake(pio_addr=0x1C020000, ignore_access=True)
    watchdog_fake  = AmbaFake(pio_addr=0x1C0F0000)
    aaci_fake      = AmbaFake(pio_addr=0x1C040000)
    lan_fake       = IsaFake(pio_addr=0x1A000000, pio_size=0xffff)
    usb_fake       = IsaFake(pio_addr=0x1B000000, pio_size=0x1ffff)
    mmc_fake       = AmbaFake(pio_addr=0x1c050000)
    energy_ctrl    = EnergyCtrl(pio_addr=0x1c080000)

    # Attach any PCI devices that are supported
    def attachPciDevices(self):
        self.ethernet = IGbE_e1000(pci_bus=0, pci_dev=0, pci_func=0,
                                   InterruptLine=1, InterruptPin=1)
        self.ide = IdeController(disks = [], pci_bus=0, pci_dev=1, pci_func=0,
                                 InterruptLine=2, InterruptPin=2)

    def enableMSIX(self):
        self.gic = Pl390(dist_addr=0x2C001000, cpu_addr=0x2C002000, it_lines=512)
        self.gicv2m = Gicv2m()
        self.gicv2m.frames = [Gicv2mFrame(spi_base=256, spi_len=64, addr=0x2C1C0000)]

    def setupBootLoader(self, mem_bus, cur_sys, loc):
        self.nvmem = SimpleMemory(range = AddrRange('64MB'),
                                  conf_table_reported = False)
        self.nvmem.port = mem_bus.master
        cur_sys.boot_loader = loc('boot_emm.arm')
        cur_sys.atags_addr = 0x8000000
        cur_sys.load_addr_mask = 0xfffffff
        cur_sys.load_offset = 0x80000000

    # Attach I/O devices that are on chip and also set the appropriate
    # ranges for the bridge
    def attachOnChipIO(self, bus, bridge=None):
        self.gic.pio             = bus.master
        self.vgic.pio            = bus.master
        self.local_cpu_timer.pio = bus.master
        if hasattr(self, "gicv2m"):
            self.gicv2m.pio      = bus.master
        self.hdlcd.dma           = bus.slave
        if bridge:
            # Bridge ranges based on excluding what is part of on-chip I/O
            # (gic, a9scu)
            bridge.ranges = [AddrRange(0x2F000000, size='16MB'),
                             AddrRange(0x2B000000, size='4MB'),
                             AddrRange(0x30000000, size='256MB'),
                             AddrRange(0x40000000, size='512MB'),
                             AddrRange(0x18000000, size='64MB'),
                             AddrRange(0x1C000000, size='64MB')]


    # Set the clock domain for IO objects that are considered
    # to be "close" to the cores.
    def onChipIOClkDomain(self, clkdomain):
        self.gic.clk_domain             = clkdomain
        if hasattr(self, "gicv2m"):
            self.gicv2m.clk_domain      = clkdomain
        self.hdlcd.clk_domain           = clkdomain
        self.vgic.clk_domain            = clkdomain

    # Attach I/O devices to specified bus object.  Done here
    # as the specified bus to connect to may not always be fixed.
    def attachIO(self, bus):
       self.uart.pio            = bus.master
       self.realview_io.pio     = bus.master
       self.pci_host.pio        = bus.master
       self.timer0.pio          = bus.master
       self.timer1.pio          = bus.master
       self.clcd.pio            = bus.master
       self.clcd.dma            = bus.slave
       self.hdlcd.pio           = bus.master
       self.kmi0.pio            = bus.master
       self.kmi1.pio            = bus.master
       self.cf_ctrl.pio         = bus.master
       self.cf_ctrl.dma         = bus.slave
       self.rtc.pio             = bus.master
       self.vram.port           = bus.master

       self.l2x0_fake.pio       = bus.master
       self.uart1_fake.pio      = bus.master
       self.uart2_fake.pio      = bus.master
       self.uart3_fake.pio      = bus.master
       self.sp810_fake.pio      = bus.master
       self.watchdog_fake.pio   = bus.master
       self.aaci_fake.pio       = bus.master
       self.lan_fake.pio        = bus.master
       self.usb_fake.pio        = bus.master
       self.mmc_fake.pio        = bus.master
       self.energy_ctrl.pio     = bus.master

       # Try to attach the I/O if it exists
       try:
           self.ide.pio         = bus.master
           self.ide.dma         = bus.slave
           self.ethernet.pio    = bus.master
           self.ethernet.dma    = bus.slave
       except:
           pass

    # Set the clock domain for IO objects that are considered
    # to be "far" away from the cores.
    def offChipIOClkDomain(self, clkdomain):
        self.uart.clk_domain          = clkdomain
        self.realview_io.clk_domain   = clkdomain
        self.timer0.clk_domain        = clkdomain
        self.timer1.clk_domain        = clkdomain
        self.clcd.clk_domain          = clkdomain
        self.kmi0.clk_domain          = clkdomain
        self.kmi1.clk_domain          = clkdomain
        self.cf_ctrl.clk_domain       = clkdomain
        self.rtc.clk_domain           = clkdomain
        self.vram.clk_domain          = clkdomain

        self.l2x0_fake.clk_domain     = clkdomain
        self.uart1_fake.clk_domain    = clkdomain
        self.uart2_fake.clk_domain    = clkdomain
        self.uart3_fake.clk_domain    = clkdomain
        self.sp810_fake.clk_domain    = clkdomain
        self.watchdog_fake.clk_domain = clkdomain
        self.aaci_fake.clk_domain     = clkdomain
        self.lan_fake.clk_domain      = clkdomain
        self.usb_fake.clk_domain      = clkdomain
        self.mmc_fake.clk_domain      = clkdomain
        self.energy_ctrl.clk_domain   = clkdomain

class VExpress_EMM64(VExpress_EMM):
    # Three memory regions are specified totalling 512GB
    _mem_regions = [(Addr('2GB'), Addr('2GB')), (Addr('34GB'), Addr('30GB')),
                    (Addr('512GB'), Addr('480GB'))]
    pci_host = GenericPciHost(
        conf_base=0x30000000, conf_size='256MB', conf_device_bits=12,
        pci_pio_base=0x2f000000)

    def setupBootLoader(self, mem_bus, cur_sys, loc):
        self.nvmem = SimpleMemory(range=AddrRange(0, size='64MB'),
                                  conf_table_reported=False)
        self.nvmem.port = mem_bus.master
        cur_sys.boot_loader = loc('boot_emm.arm64')
        cur_sys.atags_addr = 0x8000000
        cur_sys.load_addr_mask = 0xfffffff
        cur_sys.load_offset = 0x80000000


class VExpress_GEM5_V1(RealView):
    """
The VExpress gem5 memory map is loosely based on a modified
Versatile Express RS1 memory map.

The gem5 platform has been designed to implement a subset of the
original Versatile Express RS1 memory map. Off-chip peripherals should,
when possible, adhere to the Versatile Express memory map. Non-PCI
off-chip devices that are gem5-specific should live in the CS5 memory
space to avoid conflicts with existing devices that we might want to
model in the future. Such devices should normally have interrupts in
the gem5-specific SPI range.

On-chip peripherals are loosely modeled after the ARM CoreTile Express
A15x2 A7x3 memory and interrupt map. In particular, the GIC and
Generic Timer have the same interrupt lines and base addresses. Other
on-chip devices are gem5 specific.

Unlike the original Versatile Express RS2 extended platform, gem5 implements a
large contigious DRAM space, without aliases or holes, starting at the
2GiB boundary. This means that PCI memory is limited to 1GiB.

Memory map:
   0x00000000-0x03ffffff: Boot memory (CS0)
   0x04000000-0x07ffffff: Reserved
   0x08000000-0x0bffffff: Reserved (CS0 alias)
   0x0c000000-0x0fffffff: Reserved (Off-chip, CS4)
   0x10000000-0x13ffffff: gem5-specific peripherals (Off-chip, CS5)
       0x10000000-0x1000ffff: gem5 energy controller

   0x14000000-0x17ffffff: Reserved (Off-chip, PSRAM, CS1)
   0x18000000-0x1bffffff: Reserved (Off-chip, Peripherals, CS2)
   0x1c000000-0x1fffffff: Peripheral block 1 (Off-chip, CS3):
       0x1c010000-0x1c01ffff: realview_io (VE system control regs.)
       0x1c060000-0x1c06ffff: KMI0 (keyboard)
       0x1c070000-0x1c07ffff: KMI1 (mouse)
       0x1c090000-0x1c09ffff: UART0
       0x1c0a0000-0x1c0affff: UART1 (reserved)
       0x1c0b0000-0x1c0bffff: UART2 (reserved)
       0x1c0c0000-0x1c0cffff: UART3 (reserved)
       0x1c170000-0x1c17ffff: RTC

   0x20000000-0x3fffffff: On-chip peripherals:
       0x2b000000-0x2b00ffff: HDLCD

       0x2c001000-0x2c001fff: GIC (distributor)
       0x2c002000-0x2c0020ff: GIC (CPU interface)
       0x2c004000-0x2c005fff: vGIC (HV)
       0x2c006000-0x2c007fff: vGIC (VCPU)
       0x2c1c0000-0x2c1cffff: GICv2m MSI frame 0

       0x2d000000-0x2d00ffff: GPU (reserved)

       0x2f000000-0x2fffffff: PCI IO space
       0x30000000-0x3fffffff: PCI config space

   0x40000000-0x7fffffff: Ext. AXI: Used as PCI memory

   0x80000000-X: DRAM

Interrupts:
      0- 15: Software generated interrupts (SGIs)
     16- 31: On-chip private peripherals (PPIs)
        25   : vgic
        26   : generic_timer (hyp)
        27   : generic_timer (virt)
        28   : Reserved (Legacy FIQ)
        29   : generic_timer (phys, sec)
        30   : generic_timer (phys, non-sec)
        31   : Reserved (Legacy IRQ)
    32- 95: Mother board peripherals (SPIs)
        32   : Reserved (SP805)
        33   : Reserved (IOFPGA SW int)
        34-35: Reserved (SP804)
        36   : RTC
        37-40: uart0-uart3
        41-42: Reserved (PL180)
        43   : Reserved (AACI)
        44-45: kmi0-kmi1
        46   : Reserved (CLCD)
        47   : Reserved (Ethernet)
        48   : Reserved (USB)
    95-255: On-chip interrupt sources (we use these for
            gem5-specific devices, SPIs)
         95    : HDLCD
         96- 98: GPU (reserved)
        100-103: PCI
   256-319: MSI frame 0 (gem5-specific, SPIs)
   320-511: Unused

    """

    # Everything above 2GiB is memory
    _mem_regions = [(Addr('2GB'), Addr('510GB'))]

    _off_chip_ranges = [
        # CS1-CS5
        AddrRange(0x0c000000, 0x1fffffff),
        # External AXI interface (PCI)
        AddrRange(0x2f000000, 0x7fffffff),
    ]

    # Platform control device (off-chip)
    realview_io = RealViewCtrl(proc_id0=0x14000000, proc_id1=0x14000000,
                               idreg=0x02250000, pio_addr=0x1c010000)
    mcc = VExpressMCC()
    dcc = CoreTile2A15DCC()

    ### On-chip devices ###
    gic = Pl390(dist_addr=0x2c001000, cpu_addr=0x2c002000, it_lines=512)
    vgic = VGic(vcpu_addr=0x2c006000, hv_addr=0x2c004000, ppint=25)
    gicv2m = Gicv2m()
    gicv2m.frames = [
        Gicv2mFrame(spi_base=256, spi_len=64, addr=0x2c1c0000),
    ]

    generic_timer = GenericTimer(int_phys=29, int_virt=27)

    hdlcd  = HDLcd(pxl_clk=dcc.osc_pxl,
                   pio_addr=0x2b000000, int_num=95)

    def _on_chip_devices(self):
        return [
            self.gic, self.vgic, self.gicv2m,
            self.hdlcd,
            self.generic_timer,
        ]

    ### Off-chip devices ###
    uart0 = Pl011(pio_addr=0x1c090000, int_num=37)

    kmi0 = Pl050(pio_addr=0x1c060000, int_num=44)
    kmi1 = Pl050(pio_addr=0x1c070000, int_num=45, is_mouse=True)

    rtc = PL031(pio_addr=0x1c170000, int_num=36)

    ### gem5-specific off-chip devices ###
    pci_host = GenericArmPciHost(
        conf_base=0x30000000, conf_size='256MB', conf_device_bits=12,
        pci_pio_base=0x2f000000,
        int_policy="ARM_PCI_INT_DEV", int_base=100, int_count=4)

    energy_ctrl = EnergyCtrl(pio_addr=0x10000000)


    def _off_chip_devices(self):
        return [
            self.realview_io,
            self.uart0,
            self.kmi0, self.kmi1,
            self.rtc,
            self.pci_host,
            self.energy_ctrl,
        ]

    def attachPciDevice(self, device, *args, **kwargs):
        device.host = self.pci_host
        self._attach_device(device, *args, **kwargs)

    def setupBootLoader(self, mem_bus, cur_sys, loc):
        self.nvmem = SimpleMemory(range=AddrRange(0, size='64MB'),
                                  conf_table_reported=False)
        self.nvmem.port = mem_bus.master
        cur_sys.boot_loader = [ loc('boot_emm.arm64'), loc('boot_emm.arm') ]
        cur_sys.atags_addr = 0x8000000
        cur_sys.load_addr_mask = 0xfffffff
        cur_sys.load_offset = 0x80000000
