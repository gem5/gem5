# Copyright (c) 2009-2022 Arm Limited
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

from m5.defines import buildEnv
from m5.objects.ArmSystem import ArmExtension
from m5.objects.CfiMemory import CfiMemory
from m5.objects.ClockDomain import (
    ClockDomain,
    SrcClockDomain,
)
from m5.objects.ClockedObject import ClockedObject
from m5.objects.Device import (
    BadAddr,
    BasicPioDevice,
    DmaDevice,
    IsaFake,
    PioDevice,
)
from m5.objects.Display import (
    Display,
    Display1080p,
)
from m5.objects.EnergyCtrl import EnergyCtrl
from m5.objects.Ethernet import (
    IGbE_e1000,
    IGbE_igb,
    NSGigE,
)
from m5.objects.GenericTimer import *
from m5.objects.Gic import *
from m5.objects.Graphics import ImageFormat
from m5.objects.Ide import *
from m5.objects.MHU import (
    MHU,
    Ap2ScpDoorbell,
    Scp2ApDoorbell,
)
from m5.objects.PciDevice import (
    PciIoBar,
    PciLegacyIoBar,
)
from m5.objects.PciHost import *
from m5.objects.Platform import Platform
from m5.objects.PS2 import *
from m5.objects.Scmi import *
from m5.objects.SimpleMemory import SimpleMemory
from m5.objects.SMMUv3 import SMMUv3
from m5.objects.SubSystem import SubSystem
from m5.objects.Terminal import Terminal
from m5.objects.Uart import Uart
from m5.objects.VirtIOMMIO import MmioVirtIO
from m5.objects.VoltageDomain import VoltageDomain
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *

# Platforms with KVM support should generally use in-kernel GIC
# emulation. Use a GIC model that automatically switches between
# gem5's GIC model and KVM's GIC model if KVM is available.
try:
    from m5.objects.KvmGic import (
        MuxingKvmGicV2,
        MuxingKvmGicV3,
    )

    kvm_gicv2_class = MuxingKvmGicV2
    kvm_gicv3_class = MuxingKvmGicV3
except ImportError:
    # KVM support wasn't compiled into gem5. Fallback to a
    # software-only GIC.
    kvm_gicv2_class = Gic400
    kvm_gicv3_class = Gicv3
    pass


class AmbaPioDevice(BasicPioDevice):
    type = "AmbaPioDevice"
    abstract = True
    cxx_header = "dev/arm/amba_device.hh"
    cxx_class = "gem5::AmbaPioDevice"
    amba_id = Param.UInt32("ID of AMBA device for kernel detection")


class AmbaIntDevice(AmbaPioDevice):
    type = "AmbaIntDevice"
    abstract = True
    cxx_header = "dev/arm/amba_device.hh"
    cxx_class = "gem5::AmbaIntDevice"
    interrupt = Param.ArmInterruptPin("Interrupt that connects to GIC")
    int_delay = Param.Latency(
        "100ns", "Time between action and interrupt generation by device"
    )


class AmbaDmaDevice(DmaDevice):
    type = "AmbaDmaDevice"
    abstract = True
    cxx_header = "dev/arm/amba_device.hh"
    cxx_class = "gem5::AmbaDmaDevice"
    pio_addr = Param.Addr("Address for AMBA responder interface")
    pio_latency = Param.Latency(
        "10ns",
        "Time between action and write/readresult by AMBA DMA Device",
    )
    interrupt = Param.ArmInterruptPin("Interrupt that connects to GIC")
    amba_id = Param.UInt32("ID of AMBA device for kernel detection")


class A9SCU(BasicPioDevice):
    type = "A9SCU"
    cxx_header = "dev/arm/a9scu.hh"
    cxx_class = "gem5::A9SCU"


class ArmPciIntRouting(Enum):
    vals = ["ARM_PCI_INT_STATIC", "ARM_PCI_INT_DEV", "ARM_PCI_INT_PIN"]


class GenericArmPciHost(GenericPciHost):
    type = "GenericArmPciHost"
    cxx_header = "dev/arm/pci_host.hh"
    cxx_class = "gem5::GenericArmPciHost"

    int_policy = Param.ArmPciIntRouting("PCI interrupt routing policy")
    int_base = Param.Unsigned("PCI interrupt base")
    int_count = Param.Unsigned(
        "Maximum number of interrupts used by this host"
    )

    # This python parameter can be used in configuration scripts to turn
    # on/off the fdt dma-coherent flag when doing dtb autogeneration
    _dma_coherent = True

    def generateDeviceTree(self, state):
        local_state = FdtState(
            addr_cells=3, size_cells=2, cpu_cells=1, interrupt_cells=1
        )

        node = FdtNode("pci")

        if int(self.conf_device_bits) == 8:
            node.appendCompatible("pci-host-cam-generic")
        elif int(self.conf_device_bits) == 12:
            node.appendCompatible("pci-host-ecam-generic")
        else:
            m5.fatal("No compatibility string for the set conf_device_width")

        node.append(FdtPropertyStrings("device_type", ["pci"]))

        # Cell sizes of child nodes/peripherals
        node.append(local_state.addrCellsProperty())
        node.append(local_state.sizeCellsProperty())
        node.append(local_state.interruptCellsProperty())
        # PCI address for CPU
        node.append(
            FdtPropertyWords(
                "reg",
                state.addrCells(self.conf_base)
                + state.sizeCells(self.conf_size),
            )
        )

        # Ranges mapping
        # For now some of this is hard coded, because the PCI module does not
        # have a proper full understanding of the memory map, but adapting the
        # PCI module is beyond the scope of what I'm trying to do here.
        # Values are taken from the VExpress_GEM5_V1 platform.
        ranges = []
        # Pio address range
        ranges += self.pciFdtAddr(space=1, addr=0)
        ranges += state.addrCells(self.pci_pio_base)
        ranges += local_state.sizeCells(0x10000)  # Fixed size

        # AXI memory address range
        ranges += self.pciFdtAddr(space=2, addr=0)
        ranges += state.addrCells(self.pci_mem_base)
        ranges += local_state.sizeCells(0x40000000)  # Fixed size
        node.append(FdtPropertyWords("ranges", ranges))

        if str(self.int_policy) == "ARM_PCI_INT_DEV":
            gic = self._parent.unproxy(self).gic
            int_phandle = state.phandle(gic)
            # Interrupt mapping
            interrupts = []

            # child interrupt specifier
            child_interrupt = local_state.interruptCells(0x0)

            # parent unit address
            parent_addr = gic._state.addrCells(0x0)

            for i in range(int(self.int_count)):
                parent_interrupt = gic.interruptCells(
                    0, int(self.int_base) - 32 + i, 1
                )

                interrupts += (
                    self.pciFdtAddr(device=i, addr=0)
                    + child_interrupt
                    + [int_phandle]
                    + parent_addr
                    + parent_interrupt
                )

            node.append(FdtPropertyWords("interrupt-map", interrupts))

            int_count = int(self.int_count)
            if int_count & (int_count - 1):
                fatal("PCI interrupt count should be power of 2")

            intmask = self.pciFdtAddr(device=int_count - 1, addr=0) + [0x0]
            node.append(FdtPropertyWords("interrupt-map-mask", intmask))
        else:
            m5.fatal(
                "Unsupported PCI interrupt policy "
                + "for Device Tree generation"
            )

        if self._dma_coherent:
            node.append(FdtProperty("dma-coherent"))

        yield node


class RealViewCtrl(BasicPioDevice):
    type = "RealViewCtrl"
    cxx_header = "dev/arm/rv_ctrl.hh"
    cxx_class = "gem5::RealViewCtrl"
    proc_id0 = Param.UInt32(0x0C000000, "Processor ID, SYS_PROCID")
    proc_id1 = Param.UInt32(0x0C000222, "Processor ID, SYS_PROCID1")
    idreg = Param.UInt32(0x00000000, "ID Register, SYS_ID")

    def generateDeviceTree(self, state):
        node = FdtNode(f"sysreg@{int(self.pio_addr):x}")
        node.appendCompatible("arm,vexpress-sysreg")
        node.append(
            FdtPropertyWords(
                "reg", state.addrCells(self.pio_addr) + state.sizeCells(0x1000)
            )
        )
        node.append(FdtProperty("gpio-controller"))
        node.append(FdtPropertyWords("#gpio-cells", [2]))
        node.appendPhandle(self)

        yield node


class RealViewOsc(ClockDomain):
    type = "RealViewOsc"
    cxx_header = "dev/arm/rv_ctrl.hh"
    cxx_class = "gem5::RealViewOsc"

    parent = Param.RealViewCtrl(Parent.any, "RealView controller")

    # TODO: We currently don't have the notion of a clock source,
    # which means we have to associate oscillators with a voltage
    # source.
    voltage_domain = Param.VoltageDomain(
        Parent.voltage_domain, "Voltage domain"
    )

    # See ARM DUI 0447J (ARM Motherboard Express uATX -- V2M-P1) and
    # the individual core/logic tile reference manuals for details
    # about the site/position/dcc/device allocation.
    site = Param.UInt8("Board Site")
    position = Param.UInt8("Position in device stack")
    dcc = Param.UInt8("Daughterboard Configuration Controller")
    device = Param.UInt8("Device ID")

    freq = Param.Clock("Default frequency")

    # These are currently only used for the device tree.
    min_freq = Param.Clock("0t", "Minimum frequency")
    max_freq = Param.Clock("0t", "Maximum frequency")

    def generateDeviceTree(self, state):
        phandle = state.phandle(self)
        node = FdtNode("osc@" + format(int(phandle), "x"))
        node.appendCompatible("arm,vexpress-osc")
        node.append(
            FdtPropertyWords(
                "arm,vexpress-sysreg,func", [0x1, int(self.device)]
            )
        )
        node.append(FdtPropertyWords("#clock-cells", [0]))

        minf = self.min_freq if self.min_freq.value else self.freq
        maxf = self.max_freq if self.max_freq.value else self.freq

        # Values are stored as a clock period.
        def to_freq(prop):
            return int(1.0 / prop.value)

        node.append(
            FdtPropertyWords("freq-range", [to_freq(minf), to_freq(maxf)])
        )
        node.append(
            FdtPropertyStrings("clock-output-names", ["oscclk" + str(phandle)])
        )
        node.appendPhandle(self)
        yield node


class RealViewTemperatureSensor(SimObject):
    type = "RealViewTemperatureSensor"
    cxx_header = "dev/arm/rv_ctrl.hh"
    cxx_class = "gem5::RealViewTemperatureSensor"

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

    osc_mcc = Osc(device=0, min_freq="25MHz", max_freq="60MHz", freq="50MHz")
    osc_clcd = Osc(
        device=1, min_freq="23.75MHz", max_freq="63.5MHz", freq="23.75MHz"
    )
    osc_peripheral = Osc(device=2, freq="24MHz")
    osc_system_bus = Osc(
        device=4, min_freq="2MHz", max_freq="230MHz", freq="24MHz"
    )

    # See Table 4.19 in ARM DUI 0447J (Motherboard Express uATX TRM).
    temp_crtl = Temperature(device=0)

    def generateDeviceTree(self, state):
        node = FdtNode("mcc")
        node.appendCompatible("arm,vexpress,config-bus")
        node.append(FdtPropertyWords("arm,vexpress,site", [0]))

        for obj in self._children.values():
            if issubclass(type(obj), SimObject):
                node.append(obj.generateDeviceTree(state))

        io_phandle = state.phandle(self.osc_mcc.parent.unproxy(self))
        node.append(FdtPropertyWords("arm,vexpress,config-bridge", io_phandle))

        yield node


class CoreTile2A15DCC(SubSystem):
    """ARM CoreTile Express A15x2 Daughterboard Configuration Controller

    This subsystem describes a subset of the devices that sit behind the
    daughterboard configuration controller on a CoreTile Express A15x2. See
    ARM DUI 0604E for details.
    """

    class Osc(RealViewOsc):
        site, position, dcc = (1, 0, 0)

    # See Table 2.8 in ARM DUI 0604E (CoreTile Express A15x2 TRM)
    osc_cpu = Osc(device=0, min_freq="20MHz", max_freq="60MHz", freq="60MHz")
    osc_hsbm = Osc(device=4, min_freq="20MHz", max_freq="40MHz", freq="40MHz")
    osc_pxl = Osc(
        device=5, min_freq="23.76MHz", max_freq="165MHz", freq="23.75MHz"
    )
    osc_smb = Osc(device=6, min_freq="20MHz", max_freq="50MHz", freq="50MHz")
    osc_sys = Osc(device=7, min_freq="20MHz", max_freq="60MHz", freq="60MHz")
    osc_ddr = Osc(device=8, freq="40MHz")

    def generateDeviceTree(self, state):
        node = FdtNode("dcc")
        node.appendCompatible("arm,vexpress,config-bus")

        for obj in self._children.values():
            if isinstance(obj, SimObject):
                node.append(obj.generateDeviceTree(state))

        io_phandle = state.phandle(self.osc_cpu.parent.unproxy(self))
        node.append(FdtPropertyWords("arm,vexpress,config-bridge", io_phandle))

        yield node


class SysSecCtrl(BasicPioDevice):
    """
    System Security Control registers. Taken from:
        Arm Neoverse N1 System Development Platform - TRM - Version 0.0
        Document ID: 101489_0000_02_en
    """

    type = "SysSecCtrl"
    cxx_header = "dev/arm/ssc.hh"
    cxx_class = "gem5::SysSecCtrl"

    ssc_dbgcfg_stat = Param.Unsigned(
        0x00010000, "Debug authentication configuration status"
    )
    ssc_version = Param.Unsigned(0x100417B0, "Version register")
    ssc_pid0 = Param.Unsigned(0x44, "Peripheral ID0 register")
    ssc_pid1 = Param.Unsigned(0xB8, "Peripheral ID1 register")
    ssc_pid2 = Param.Unsigned(0xB, "Peripheral ID2 register")
    ssc_pid4 = Param.Unsigned(0x4, "Peripheral ID4 register")

    compid0 = Param.Unsigned(0x0D, "Component ID0 register")
    compid1 = Param.Unsigned(0xF0, "Component ID1 register")
    compid2 = Param.Unsigned(0x5, "Component ID2 register")
    compid3 = Param.Unsigned(0xB1, "Component ID3 register")


class AmbaFake(AmbaPioDevice):
    type = "AmbaFake"
    cxx_header = "dev/arm/amba_fake.hh"
    cxx_class = "gem5::AmbaFake"
    ignore_access = Param.Bool(
        False, "Ignore reads/writes to this device, (e.g. IsaFake + AMBA)"
    )
    amba_id = 0


# Simple fixed-rate clock source. Intended to be instantiated in Platform
# instances for definition of clock bindings on DTB auto-generation
class FixedClock(SrcClockDomain):
    # Keep track of the number of FixedClock instances in the system
    # to provide unique names
    _index = 0

    def generateDeviceTree(self, state):
        if len(self.clock) > 1:
            fatal("FixedClock configured with multiple frequencies")
        node = FdtNode(f"clock{FixedClock._index}")
        node.appendCompatible("fixed-clock")
        node.append(FdtPropertyWords("#clock-cells", 0))
        node.append(
            FdtPropertyWords("clock-frequency", self.clock[0].frequency)
        )
        node.appendPhandle(self)
        FixedClock._index += 1

        yield node


class Pl011(Uart):
    type = "Pl011"
    cxx_header = "dev/arm/pl011.hh"
    cxx_class = "gem5::Pl011"
    interrupt = Param.ArmInterruptPin("Interrupt that connects to GIC")
    end_on_eot = Param.Bool(
        False, "End the simulation when a EOT is received on the UART"
    )
    int_delay = Param.Latency(
        "100ns", "Time between action and interrupt generation by UART"
    )

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "uart", self.pio_addr, 0x1000, [self.interrupt]
        )
        node.appendCompatible(["arm,pl011", "arm,primecell"])

        # Hardcoded reference to the realview platform clocks, because the
        # clk_domain can only store one clock (i.e. it is not a VectorParam)
        realview = self._parent.unproxy(self)
        node.append(
            FdtPropertyWords(
                "clocks",
                [
                    state.phandle(realview.mcc.osc_peripheral),
                    state.phandle(realview.dcc.osc_smb),
                ],
            )
        )
        node.append(FdtPropertyStrings("clock-names", ["uartclk", "apb_pclk"]))
        yield node


class Sp804(AmbaPioDevice):
    type = "Sp804"
    cxx_header = "dev/arm/timer_sp804.hh"
    cxx_class = "gem5::Sp804"
    int0 = Param.ArmSPI("Interrupt that connects to GIC")
    clock0 = Param.Clock("1MHz", "Clock speed of the input")
    int1 = Param.ArmSPI("Interrupt that connects to GIC")
    clock1 = Param.Clock("1MHz", "Clock speed of the input")
    amba_id = 0x00141804


class Sp805(AmbaIntDevice):
    """
    Arm Watchdog Module (SP805)
    Reference:
        Arm Watchdog Module (SP805) - Technical Reference Manual - rev. r1p0
        Doc. ID: ARM DDI 0270B
    """

    type = "Sp805"
    cxx_header = "dev/arm/watchdog_sp805.hh"
    cxx_class = "gem5::Sp805"

    amba_id = 0x00141805

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "watchdog", self.pio_addr, 0x1000, [self.interrupt]
        )
        node.appendCompatible(["arm,sp805", "arm,primecell"])
        clocks = [state.phandle(self.clk_domain.unproxy(self))]
        clock_names = ["wdogclk"]
        platform = self._parent.unproxy(self)
        if self in platform._off_chip_devices():
            clocks.append(state.phandle(platform.dcc.osc_smb))
            clock_names.append("apb_pclk")
        node.append(FdtPropertyWords("clocks", clocks))
        node.append(FdtPropertyStrings("clock-names", clock_names))

        yield node


class GenericWatchdog(PioDevice):
    type = "GenericWatchdog"
    cxx_header = "dev/arm/watchdog_generic.hh"
    cxx_class = "gem5::GenericWatchdog"

    refresh_start = Param.Addr("Start address for the refresh frame")
    control_start = Param.Addr("Start address for the control frame")
    pio_latency = Param.Latency("10ns", "Delay for PIO r/w")

    ws0 = Param.ArmInterruptPin("WS0 Signal")
    ws1 = Param.ArmInterruptPin("WS1 Signal")

    system_counter = Param.SystemCounter(
        Parent.any,
        "The Watchdog uses the Generic Timer system counter as the timebase "
        "against which the decision to trigger an interrupt is made.",
    )


class CpuLocalTimer(BasicPioDevice):
    type = "CpuLocalTimer"
    cxx_header = "dev/arm/timer_cpulocal.hh"
    cxx_class = "gem5::CpuLocalTimer"
    int_timer = Param.ArmPPI("Interrrupt used per-cpu to GIC")
    int_watchdog = Param.ArmPPI("Interrupt for per-cpu watchdog to GIC")


class PL031(AmbaIntDevice):
    type = "PL031"
    cxx_header = "dev/arm/rtc_pl031.hh"
    cxx_class = "gem5::PL031"
    time = Param.Time(
        "01/01/2009", "System time to use ('Now' for actual time)"
    )
    amba_id = 0x00041031

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "rtc", self.pio_addr, 0x1000, [self.interrupt]
        )

        node.appendCompatible(["arm,pl031", "arm,primecell"])
        clock = state.phandle(self.clk_domain.unproxy(self))
        node.append(FdtPropertyWords("clocks", clock))
        node.append(FdtPropertyStrings("clock-names", ["apb_pclk"]))

        yield node


class Pl050(AmbaIntDevice):
    type = "Pl050"
    cxx_header = "dev/arm/kmi.hh"
    cxx_class = "gem5::Pl050"
    amba_id = 0x00141050

    ps2 = Param.PS2Device("PS/2 device")

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "kmi", self.pio_addr, 0x1000, [self.interrupt]
        )

        node.appendCompatible(["arm,pl050", "arm,primecell"])
        clock = state.phandle(self.clk_domain.unproxy(self))
        node.append(FdtPropertyWords("clocks", clock))

        yield node


class Pl111(AmbaDmaDevice):
    type = "Pl111"
    cxx_header = "dev/arm/pl111.hh"
    cxx_class = "gem5::Pl111"
    pixel_clock = Param.Clock("24MHz", "Pixel clock")
    vnc = Param.VncInput(
        Parent.any, "Vnc server for remote frame buffer display"
    )
    amba_id = 0x00141111
    enable_capture = Param.Bool(
        True, "capture frame to system.framebuffer.bmp"
    )


class HDLcd(AmbaDmaDevice):
    type = "HDLcd"
    cxx_header = "dev/arm/hdlcd.hh"
    cxx_class = "gem5::HDLcd"
    vnc = Param.VncInput(
        Parent.any, "Vnc server for remote frame buffer display"
    )
    amba_id = 0x00141000
    workaround_swap_rb = Param.Bool(
        False, "Workaround incorrect color selector order in some kernels"
    )
    workaround_dma_line_count = Param.Bool(
        True, "Workaround incorrect DMA line count (off by 1)"
    )
    enable_capture = Param.Bool(
        True, "capture frame to system.framebuffer.{extension}"
    )
    frame_format = Param.ImageFormat(
        "Auto", "image format of the captured frame"
    )

    pixel_buffer_size = Param.MemorySize32("2KiB", "Size of address range")

    pxl_clk = Param.ClockDomain("Pixel clock source")
    pixel_chunk = Param.Unsigned(32, "Number of pixels to handle in one batch")
    virt_refresh_rate = Param.Frequency(
        "20Hz", "Frame refresh rate in KVM mode"
    )
    _status = "ok"

    encoder = Param.Display(Display1080p(), "Display encoder")

    def endpointPhandle(self):
        return "hdlcd_endpoint"

    def generateDeviceTree(self, state):
        endpoint_node = FdtNode("endpoint")
        endpoint_node.appendPhandle(self.endpointPhandle())

        for encoder_node in self.encoder.generateDeviceTree(state):
            encoder_endpoint = self.encoder.endpointNode()

            # Endpoint subnode
            endpoint_node.append(
                FdtPropertyWords(
                    "remote-endpoint",
                    [state.phandle(self.encoder.endpointPhandle())],
                )
            )
            encoder_endpoint.append(
                FdtPropertyWords(
                    "remote-endpoint", [state.phandle(self.endpointPhandle())]
                )
            )

            yield encoder_node

        port_node = FdtNode("port")
        port_node.append(endpoint_node)

        node = self.generateBasicPioDeviceNode(
            state, "hdlcd", self.pio_addr, 0x1000, [self.interrupt]
        )

        node.appendCompatible(["arm,hdlcd"])
        node.append(FdtPropertyWords("clocks", state.phandle(self.pxl_clk)))
        node.append(FdtPropertyStrings("clock-names", ["pxlclk"]))

        node.append(FdtPropertyStrings("status", [self._status]))

        self.addIommuProperty(state, node)

        node.append(port_node)

        yield node


class ParentMem(SimpleMemory):
    """
    This is a base abstract class for child node generation
    A memory willing to autogenerate child nodes can do that
    directly in the generateDeviceTree method.
    However sometimes portions of memory (child nodes) are tagged
    for specific applications. Hardcoding the child node in the
    parent memory class is not flexible, so we delegate this
    to the application model, which is registering the generator
    helper via the ParentMem interface.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._generators = []

    def addSubnodeGenerator(self, gen):
        """
        This is the method a client application would use to
        register a child generator in the memory object.
        """
        self._generators.append(gen)

    def generateSubnodes(self, node, state):
        """
        This is the method the memory would use to instantiate
        the child nodes via the previously registered generators.
        """
        for subnode_gen in self._generators:
            node.append(subnode_gen(state))


class MmioSRAM(ParentMem):
    def __init__(self, *args, **kwargs):
        super().__init__(**kwargs)

    def generateDeviceTree(self, state):
        node = FdtNode(f"sram@{int(self.range.start):x}")
        node.appendCompatible(["mmio-sram"])
        node.append(
            FdtPropertyWords(
                "reg",
                state.addrCells(self.range.start)
                + state.sizeCells(self.range.size()),
            )
        )

        local_state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
        node.append(local_state.addrCellsProperty())
        node.append(local_state.sizeCellsProperty())
        node.append(
            FdtPropertyWords(
                "ranges",
                local_state.addrCells(0)
                + state.addrCells(self.range.start)
                + state.sizeCells(self.range.size()),
            )
        )

        self.generateSubnodes(node, state)

        yield node


class FVPBasePwrCtrl(BasicPioDevice):
    """
    Based on Fast Models Base_PowerController v11.8
    Reference:
        Fast Models Reference Manual - Section 7.7.2 - Version 11.8
        Document ID: 100964_1180_00_en
    """

    type = "FVPBasePwrCtrl"
    cxx_header = "dev/arm/fvp_base_pwr_ctrl.hh"
    cxx_class = "gem5::FVPBasePwrCtrl"


class GenericMHU(MHU):
    lowp_scp2ap = Scp2ApDoorbell(
        set_address=0x10020008,
        clear_address=0x10020010,
        interrupt=ArmSPI(num=68),
    )
    highp_scp2ap = Scp2ApDoorbell(
        set_address=0x10020028,
        clear_address=0x10020030,
        interrupt=ArmSPI(num=67),
    )
    sec_scp2ap = Scp2ApDoorbell(
        set_address=0x10020208,
        clear_address=0x10020210,
        interrupt=ArmSPI(num=69),
    )
    lowp_ap2scp = Ap2ScpDoorbell(
        set_address=0x10020108, clear_address=0x10020110
    )
    highp_ap2scp = Ap2ScpDoorbell(
        set_address=0x10020128, clear_address=0x10020130
    )
    sec_ap2scp = Ap2ScpDoorbell(
        set_address=0x10020308, clear_address=0x10020310
    )


class RealView(Platform):
    type = "RealView"
    cxx_header = "dev/arm/realview.hh"
    cxx_class = "gem5::RealView"
    _mem_regions = [AddrRange(0, size="256MiB")]
    _num_pci_dev = 0

    def _on_chip_devices(self):
        return []

    def _off_chip_devices(self):
        return []

    def _on_chip_memory(self):
        return []

    def _off_chip_memory(self):
        return []

    _off_chip_ranges = []

    def _attach_memory(self, mem, bus, mem_ports=None):
        if hasattr(mem, "port"):
            if mem_ports is None:
                mem.port = bus.mem_side_ports
            else:
                mem_ports.append(mem.port)

    def _attach_device(self, device, bus, dma_ports=None):
        if hasattr(device, "pio"):
            device.pio = bus.mem_side_ports
        if hasattr(device, "dma"):
            if dma_ports is None:
                device.dma = bus.cpu_side_ports
            else:
                dma_ports.append(device.dma)

    def _attach_io(self, devices, *args, **kwargs):
        for d in devices:
            self._attach_device(d, *args, **kwargs)

    def _attach_mem(self, memories, *args, **kwargs):
        for mem in memories:
            self._attach_memory(mem, *args, **kwargs)

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

    def attachOnChipIO(self, bus, bridge=None, dma_ports=None, mem_ports=None):
        self._attach_mem(self._on_chip_memory(), bus, mem_ports)
        self._attach_io(self._on_chip_devices(), bus, dma_ports)
        if bridge:
            bridge.ranges = self._off_chip_ranges

    def attachIO(self, bus, dma_ports=None, mem_ports=None):
        self._attach_mem(self._off_chip_memory(), bus, mem_ports)
        self._attach_io(self._off_chip_devices(), bus, dma_ports)

    def setupBootLoader(self, cur_sys, boot_loader, dtb_addr, load_offset):
        cur_sys.workload.boot_loader = boot_loader
        cur_sys.workload.load_addr_offset = load_offset
        cur_sys.workload.dtb_addr = load_offset + dtb_addr
        # Use 0x200000 as this is the maximum size allowed for a DTB
        cur_sys.workload.initrd_addr = cur_sys.workload.dtb_addr + 0x200000
        cur_sys.workload.cpu_release_addr = cur_sys.workload.dtb_addr - 8

    def generateDeviceTree(self, state):
        node = FdtNode("/")  # Things in this module need to end up in the root
        node.append(
            FdtPropertyWords("interrupt-parent", state.phandle(self.gic))
        )

        for subnode in self.recurseDeviceTree(state):
            node.append(subnode)

        yield node

    def annotateCpuDeviceNode(self, cpu, state):
        system = self.system.unproxy(self)
        if system._have_psci:
            cpu.append(FdtPropertyStrings("enable-method", "psci"))
        else:
            cpu.append(FdtPropertyStrings("enable-method", "spin-table"))
            # The kernel writes the entry addres of secondary CPUs to this
            # address before waking up secondary CPUs.
            # The gem5 bootloader then makes secondary CPUs jump to it.
            cpu.append(
                FdtPropertyWords(
                    "cpu-release-addr",
                    state.addrCells(system.workload.cpu_release_addr),
                )
            )


class VExpress_EMM(RealView):
    _mem_regions = [AddrRange("2GiB", size="2GiB")]

    # Ranges based on excluding what is part of on-chip I/O (gic,
    # a9scu)
    _off_chip_ranges = [
        AddrRange(0x2F000000, size="16MiB"),
        AddrRange(0x30000000, size="256MiB"),
        AddrRange(0x40000000, size="512MiB"),
        AddrRange(0x18000000, size="64MiB"),
        AddrRange(0x1C000000, size="64MiB"),
    ]

    # Platform control device (off-chip)
    realview_io = RealViewCtrl(
        proc_id0=0x14000000,
        proc_id1=0x14000000,
        idreg=0x02250000,
        pio_addr=0x1C010000,
    )

    mcc = VExpressMCC()
    dcc = CoreTile2A15DCC()

    ### On-chip devices ###
    gic = Gic400(dist_addr=0x2C001000, cpu_addr=0x2C002000)
    vgic = VGic(vcpu_addr=0x2C006000, hv_addr=0x2C004000, maint_int=25)

    local_cpu_timer = CpuLocalTimer(
        int_timer=ArmPPI(num=29),
        int_watchdog=ArmPPI(num=30),
        pio_addr=0x2C080000,
    )

    hdlcd = HDLcd(
        pxl_clk=dcc.osc_pxl,
        pio_addr=0x2B000000,
        interrupt=ArmSPI(num=117),
        workaround_swap_rb=True,
    )

    def _on_chip_devices(self):
        devices = [self.gic, self.vgic, self.local_cpu_timer]
        if hasattr(self, "gicv2m"):
            devices.append(self.gicv2m)
        devices.append(self.hdlcd)
        return devices

    def _on_chip_memory(self):
        memories = [self.bootmem]
        return memories

    ### Off-chip devices ###
    uart = Pl011(pio_addr=0x1C090000, interrupt=ArmSPI(num=37))
    pci_host = GenericPciHost(
        conf_base=0x30000000,
        conf_size="256MiB",
        conf_device_bits=16,
        pci_pio_base=0,
    )

    sys_counter = SystemCounter()
    generic_timer = GenericTimer(
        int_el3_phys=ArmPPI(num=29, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el1_phys=ArmPPI(num=30, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el1_virt=ArmPPI(num=27, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_ns_phys=ArmPPI(num=26, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_ns_virt=ArmPPI(num=28, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_s_phys=ArmPPI(num=20, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_s_virt=ArmPPI(num=19, int_type="IRQ_TYPE_LEVEL_LOW"),
    )

    timer0 = Sp804(
        int0=ArmSPI(num=34),
        int1=ArmSPI(num=34),
        pio_addr=0x1C110000,
        clock0="1MHz",
        clock1="1MHz",
    )
    timer1 = Sp804(
        int0=ArmSPI(num=35),
        int1=ArmSPI(num=35),
        pio_addr=0x1C120000,
        clock0="1MHz",
        clock1="1MHz",
    )
    clcd = Pl111(pio_addr=0x1C1F0000, interrupt=ArmSPI(num=46))
    kmi0 = Pl050(
        pio_addr=0x1C060000, interrupt=ArmSPI(num=44), ps2=PS2Keyboard()
    )
    kmi1 = Pl050(
        pio_addr=0x1C070000, interrupt=ArmSPI(num=45), ps2=PS2TouchKit()
    )
    cf_ctrl = IdeController(
        disks=[],
        pci_func=0,
        pci_dev=0,
        pci_bus=2,
        io_shift=2,
        ctrl_offset=2,
        Command=0x1,
    )
    cf_ctrl.BAR0 = PciLegacyIoBar(addr="0x1C1A0000", size="256B")
    cf_ctrl.BAR1 = PciLegacyIoBar(addr="0x1C1A0100", size="4096B")

    bootmem = SimpleMemory(range=AddrRange("64MiB"), conf_table_reported=False)
    vram = SimpleMemory(
        range=AddrRange(0x18000000, size="32MiB"), conf_table_reported=False
    )
    rtc = PL031(pio_addr=0x1C170000, interrupt=ArmSPI(num=36))

    l2x0_fake = IsaFake(pio_addr=0x2C100000, pio_size=0xFFF)
    uart1_fake = AmbaFake(pio_addr=0x1C0A0000)
    uart2_fake = AmbaFake(pio_addr=0x1C0B0000)
    uart3_fake = AmbaFake(pio_addr=0x1C0C0000)
    sp810_fake = AmbaFake(pio_addr=0x1C020000, ignore_access=True)
    watchdog_fake = AmbaFake(pio_addr=0x1C0F0000)
    aaci_fake = AmbaFake(pio_addr=0x1C040000)
    lan_fake = IsaFake(pio_addr=0x1A000000, pio_size=0xFFFF)
    usb_fake = IsaFake(pio_addr=0x1B000000, pio_size=0x1FFFF)
    mmc_fake = AmbaFake(pio_addr=0x1C050000)
    energy_ctrl = EnergyCtrl(pio_addr=0x1C080000)

    def _off_chip_devices(self):
        devices = [
            self.uart,
            self.realview_io,
            self.pci_host,
            self.timer0,
            self.timer1,
            self.clcd,
            self.kmi0,
            self.kmi1,
            self.cf_ctrl,
            self.rtc,
            self.vram,
            self.l2x0_fake,
            self.uart1_fake,
            self.uart2_fake,
            self.uart3_fake,
            self.sp810_fake,
            self.watchdog_fake,
            self.aaci_fake,
            self.lan_fake,
            self.usb_fake,
            self.mmc_fake,
            self.energy_ctrl,
        ]
        # Try to attach the I/O if it exists
        if hasattr(self, "ide"):
            devices.append(self.ide)
        if hasattr(self, "ethernet"):
            devices.append(self.ethernet)
        return devices

    # Attach any PCI devices that are supported
    def attachPciDevices(self):
        self.ethernet = IGbE_e1000(
            pci_bus=0, pci_dev=0, pci_func=0, InterruptLine=1, InterruptPin=1
        )
        self.ide = IdeController(
            disks=[],
            pci_bus=0,
            pci_dev=1,
            pci_func=0,
            InterruptLine=2,
            InterruptPin=2,
        )

    def enableMSIX(self):
        self.gic = Gic400(
            dist_addr=0x2C001000, cpu_addr=0x2C002000, it_lines=512
        )
        self.gicv2m = Gicv2m()
        self.gicv2m.frames = [
            Gicv2mFrame(spi_base=256, spi_len=64, addr=0x2C1C0000)
        ]

    def setupBootLoader(self, cur_sys, loc, boot_loader=None):
        if boot_loader is None:
            boot_loader = loc("boot_emm.arm")
        super().setupBootLoader(cur_sys, boot_loader, 0x8000000, 0x80000000)


class VExpress_EMM64(VExpress_EMM):
    # Three memory regions are specified totalling 512GiB
    _mem_regions = [
        AddrRange("2GiB", size="2GiB"),
        AddrRange("34GiB", size="30GiB"),
        AddrRange("512GiB", size="480GiB"),
    ]
    pci_host = GenericPciHost(
        conf_base=0x30000000,
        conf_size="256MiB",
        conf_device_bits=12,
        pci_pio_base=0x2F000000,
    )

    def setupBootLoader(self, cur_sys, loc, boot_loader=None):
        if boot_loader is None:
            boot_loader = loc("boot_emm.arm64")
        RealView.setupBootLoader(
            self, cur_sys, boot_loader, 0x8000000, 0x80000000
        )


class VExpress_GEM5_Base(RealView):
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
    A15x2 memory and interrupt map. In particular, the GIC and
    Generic Timer have the same interrupt lines and base addresses. Other
    on-chip devices are gem5 specific.

    Unlike the original Versatile Express RS2 extended platform, gem5 implements a
    large contigious DRAM space, without aliases or holes, starting at the
    2GiB boundary. This means that PCI memory is limited to 1GiB.

    References:

        Technical Reference Manuals:
            Arm Motherboard Express uATX (V2M-P1) - ARM DUI 0447J
            Arm CoreTile Express A15x2 (V2P-CA15) - ARM DUI 0604E

        Official Linux device tree specifications:
            V2M-P1   - arch/arm/boot/dts/vexpress-v2m-rs1.dtsi
            V2P-CA15 - arch/arm/boot/dts/vexpress-v2p-ca15-tc1.dts

        Memory map:
            Arm CoreTile Express A15x2 (V2P-CA15) - ARM DUI 0604E
            Daughterboard (global)
                Section 3.2.1 - Table 3-1 - Daughterboard memory map
            On-chip
                Section 3.2.3 - Table 3-2 - Cortex-A15 MPCore on-chip peripheral
                                            memory map

        Interrupts:
            Armv8-A Foundation Platform - User Guide - Version 11.8
            Document ID: 100961_1180_00_en

    Memory map:
       0x00000000-0x03ffffff: Boot memory (CS0)
       0x04000000-0x07ffffff: Trusted Memory/Reserved
            0x04000000-0x0403FFFF: 256kB Trusted SRAM
            0x06000000-0x07ffffff: 32MB Trusted DRAM
       0x08000000-0x0bffffff: NOR FLASH0 (CS0 alias)
       0x0c000000-0x0fffffff: NOR FLASH1 (Off-chip, CS4)
       0x10000000-0x13ffffff: gem5-specific peripherals (Off-chip, CS5)
           0x10000000-0x1000ffff: gem5 energy controller
           0x10010000-0x1001ffff: gem5 pseudo-ops
           0x10020000-0x1002ffff: gem5 MHU

       0x14000000-0x17ffffff: Reserved (Off-chip, PSRAM, CS1)

       0x18000000-0x1bffffff: Off-chip, Peripherals, CS2
           0x18000000-0x19ffffff: VRAM
           0x1a000000-0x1bffffff: Reserved

       0x1c000000-0x1fffffff: Peripheral block 1 (Off-chip, CS3):
           0x1c010000-0x1c01ffff: realview_io (VE system control regs.)
           0x1c060000-0x1c06ffff: KMI0 (keyboard)
           0x1c070000-0x1c07ffff: KMI1 (mouse)
           0x1c090000-0x1c09ffff: UART0
           0x1c0a0000-0x1c0affff: UART1
           0x1c0b0000-0x1c0bffff: UART2
           0x1c0c0000-0x1c0cffff: UART3
           0x1c0f0000-0x1c0fffff: Watchdog (SP805)
           0x1c130000-0x1c13ffff: VirtIO (gem5/FM extension)
           0x1c140000-0x1c14ffff: VirtIO (gem5/FM extension)
           0x1c170000-0x1c17ffff: RTC

       0x20000000-0x3fffffff: On-chip peripherals:
           0x2a430000-0x2a43ffff: System Counter (control)
           0x2a490000-0x2a49ffff: Trusted Watchdog (SP805)
           0x2a800000-0x2a800fff: System Counter (read)
           0x2a810000-0x2a810fff: System Timer (control)

           0x2a820000-0x2a820fff: System Timer (frame 0)
           0x2a830000-0x2a830fff: System Timer (frame 1)

           0x2b000000-0x2b00ffff: HDLCD

           0x2b060000-0x2b060fff: System Watchdog (SP805)

           0x2b400000-0x2b41ffff: SMMUv3

           0x2c001000-0x2c001fff: GIC (distributor)
           0x2c002000-0x2c003fff: GIC (CPU interface)
           0x2c004000-0x2c005fff: vGIC (HV)
           0x2c006000-0x2c007fff: vGIC (VCPU)
           0x2c1c0000-0x2c1cffff: GICv2m MSI frame 0

           0x2d000000-0x2d00ffff: GPU (reserved)

           0x2e000000-0x2e007fff: Non-trusted SRAM

           0x2f000000-0x2fffffff: PCI IO space
           0x30000000-0x3fffffff: PCI config space

       0x40000000-0x7fffffff: Ext. AXI: Used as PCI memory

       0x80000000-X: DRAM

    Interrupts:
          0- 15: Software generated interrupts (SGIs)
         16- 31: On-chip private peripherals (PPIs)
            19   : generic_timer (virt sec EL2)
            20   : generic_timer (phys sec EL2)
            25   : vgic
            26   : generic_timer (phys non-sec EL2)
            27   : generic_timer (virt EL1)
            28   : generic_timer (virt non-sec EL2)
            29   : generic_timer (phys EL3)
            30   : generic_timer (phys EL1)
            31   : Reserved (Legacy IRQ)
        32- 95: Mother board peripherals (SPIs)
            32   : Watchdog (SP805)
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
            56   : Trusted Watchdog (SP805)
            57   : System timer0 (phys)
            58   : System timer1 (phys)
        95-255: On-chip interrupt sources (we use these for
                gem5-specific devices, SPIs)
             74    : VirtIO (gem5/FM extension)
             75    : VirtIO (gem5/FM extension)
             95    : HDLCD
             96- 98: GPU (reserved)
            100-103: PCI
            130    : System Watchdog (SP805)
       256-319: MSI frame 0 (gem5-specific, SPIs)
       320-511: Unused

    """

    # Everything above 2GiB is memory
    _mem_regions = [AddrRange("2GiB", size="510GiB")]

    _off_chip_ranges = [
        # CS1-CS5
        AddrRange(0x0C000000, 0x20000000),
        # External AXI interface (PCI)
        AddrRange(0x2F000000, 0x80000000),
    ]

    bootmem = SimpleMemory(
        range=AddrRange(0, size="64MiB"), conf_table_reported=False
    )

    # NOR flash, flash0
    flash0 = SimpleMemory(
        range=AddrRange(0x08000000, size="64MiB"), conf_table_reported=False
    )

    # Trusted SRAM
    trusted_sram = SimpleMemory(
        range=AddrRange(0x04000000, size="256KiB"), conf_table_reported=False
    )
    # Trusted DRAM
    # TODO: preventing access from unsecure world to the trusted RAM
    trusted_dram = SimpleMemory(
        range=AddrRange(0x06000000, size="32MB"), conf_table_reported=False
    )
    # Non-Trusted SRAM
    non_trusted_sram = MmioSRAM(
        range=AddrRange(0x2E000000, size=0x8000), conf_table_reported=False
    )

    # Platform control device (off-chip)
    realview_io = RealViewCtrl(
        proc_id0=0x14000000,
        proc_id1=0x14000000,
        idreg=0x30101100,
        pio_addr=0x1C010000,
    )
    mcc = VExpressMCC()
    dcc = CoreTile2A15DCC()

    ### On-chip devices ###

    el2_watchdog = GenericWatchdog(
        control_start=0x2A440000,
        refresh_start=0x2A450000,
        ws0=ArmSPI(num=59),
        ws1=ArmSPI(num=60),
    )

    # Trusted Watchdog, SP805
    trusted_watchdog = Sp805(pio_addr=0x2A490000, interrupt=ArmSPI(num=56))

    sys_counter = SystemCounter()
    generic_timer = GenericTimer(
        int_el3_phys=ArmPPI(num=29, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el1_phys=ArmPPI(num=30, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el1_virt=ArmPPI(num=27, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_ns_phys=ArmPPI(num=26, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_ns_virt=ArmPPI(num=28, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_s_phys=ArmPPI(num=20, int_type="IRQ_TYPE_LEVEL_LOW"),
        int_el2_s_virt=ArmPPI(num=19, int_type="IRQ_TYPE_LEVEL_LOW"),
    )
    generic_timer_mem = GenericTimerMem(
        cnt_control_base=0x2A430000,
        cnt_read_base=0x2A800000,
        cnt_ctl_base=0x2A810000,
        frames=[
            GenericTimerFrame(
                cnt_base=0x2A820000,
                int_phys=ArmSPI(num=57),
                int_virt=ArmSPI(num=133),
            ),
            GenericTimerFrame(
                cnt_base=0x2A830000,
                int_phys=ArmSPI(num=58),
                int_virt=ArmSPI(num=134),
            ),
        ],
    )

    system_watchdog = Sp805(pio_addr=0x2B060000, interrupt=ArmSPI(num=130))

    def _on_chip_devices(self):
        return [
            self.generic_timer_mem,
            self.el2_watchdog,
            self.trusted_watchdog,
            self.system_watchdog,
        ] + self.generic_timer_mem.frames

    def _on_chip_memory(self):
        memories = [
            self.bootmem,
            self.trusted_sram,
            self.trusted_dram,
            self.non_trusted_sram,
            self.flash0,
        ]
        return memories

    ### Off-chip devices ###
    io_voltage = VoltageDomain(voltage="3.3V")
    clock32KHz = SrcClockDomain(clock="32kHz")
    clock24MHz = SrcClockDomain(clock="24MHz")

    uart = [
        Pl011(pio_addr=0x1C090000, interrupt=ArmSPI(num=37)),
        Pl011(
            pio_addr=0x1C0A0000, interrupt=ArmSPI(num=38), device=Terminal()
        ),
        Pl011(
            pio_addr=0x1C0B0000, interrupt=ArmSPI(num=39), device=Terminal()
        ),
        Pl011(
            pio_addr=0x1C0C0000, interrupt=ArmSPI(num=40), device=Terminal()
        ),
    ]

    kmi0 = Pl050(
        pio_addr=0x1C060000, interrupt=ArmSPI(num=44), ps2=PS2Keyboard()
    )
    kmi1 = Pl050(
        pio_addr=0x1C070000, interrupt=ArmSPI(num=45), ps2=PS2TouchKit()
    )

    watchdog = Sp805(pio_addr=0x1C0F0000, interrupt=ArmSPI(num=32))

    rtc = PL031(pio_addr=0x1C170000, interrupt=ArmSPI(num=36))

    ### gem5-specific off-chip devices ###
    pci_host = GenericArmPciHost(
        conf_base=0x30000000,
        conf_size="256MiB",
        conf_device_bits=12,
        pci_pio_base=0x2F000000,
        pci_mem_base=0x40000000,
        int_policy="ARM_PCI_INT_DEV",
        int_base=100,
        int_count=4,
    )

    energy_ctrl = EnergyCtrl(pio_addr=0x10000000)

    pwr_ctrl = FVPBasePwrCtrl(pio_addr=0x1C100000)

    vio = [
        MmioVirtIO(
            pio_addr=0x1C130000, pio_size=0x1000, interrupt=ArmSPI(num=74)
        ),
        MmioVirtIO(
            pio_addr=0x1C140000, pio_size=0x1000, interrupt=ArmSPI(num=75)
        ),
    ]

    # NOR flash, flash1
    flash1 = CfiMemory(
        range=AddrRange(0x0C000000, 0x10000000), conf_table_reported=False
    )

    # VRAM
    vram = SimpleMemory(
        range=AddrRange(0x18000000, size="32MB"), conf_table_reported=False
    )

    def _off_chip_devices(self):
        return [
            self.realview_io,
            self.kmi0,
            self.kmi1,
            self.watchdog,
            self.rtc,
            self.pci_host,
            self.energy_ctrl,
            self.pwr_ctrl,
            self.clock32KHz,
            self.clock24MHz,
            self.vio[0],
            self.vio[1],
        ] + self.uart

    def _off_chip_memory(self):
        return [self.flash1, self.vram]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.clock32KHz.voltage_domain = self.io_voltage
        self.clock24MHz.voltage_domain = self.io_voltage
        self.system_watchdog.clk_domain = self.dcc.osc_sys
        self.watchdog.clk_domain = self.clock32KHz

    def attachPciDevice(self, device, *args, **kwargs):
        device.host = self.pci_host
        self._num_pci_dev += 1
        device.pci_bus = 0
        device.pci_dev = self._num_pci_dev
        device.pci_func = 0
        self._attach_device(device, *args, **kwargs)

    def attachSmmu(self, devices, bus):
        """
        Instantiate a single SMMU and attach a group of client devices to it.
        The devices' dma port is wired to the SMMU and the SMMU's dma port
        is attached to the bus. In order to make it work, the list of clients
        shouldn't contain any device part of the _off_chip_devices or
        _on_chip_devices.
        This method should be called only once.

        Parameters:
            devices (list): List of devices which will be using the SMMU
            bus (Bus): The bus downstream of the SMMU. Its response port will
                       receive memory requests from the SMMU, and its request
                       port will forward accesses to the memory mapped devices
        """
        if hasattr(self, "smmu"):
            m5.fatal("A SMMU has already been instantiated\n")

        self.smmu = SMMUv3(reg_map=AddrRange(0x2B400000, size=0x00020000))

        self.smmu.request = bus.cpu_side_ports
        self.smmu.control = bus.mem_side_ports

        dma_ports = []
        for dev in devices:
            self._attach_device(dev, bus, dma_ports)
            self.smmu.connect(dev)

    def setupBootLoader(self, cur_sys, boot_loader):
        super().setupBootLoader(cur_sys, boot_loader, 0x8000000, 0x80000000)

        #  Setup m5ops. It's technically not a part of the boot
        #  loader, but this is the only place we can configure the
        #  system.
        cur_sys.m5ops_base = 0x10010000

    def attachScmi(self, bus):
        # Generate and attach the mailbox
        self.mailbox = GenericMHU(pio_addr=0x10020000)
        self._attach_device(self.mailbox, bus)

        # Generate and attach the SCMI platform
        _scmi_comm = ScmiCommunication(
            agent_channel=ScmiAgentChannel(
                shmem=self.non_trusted_sram,
                shmem_range=AddrRange(0x2E000000, size=0x200),
                doorbell=self.mailbox.highp_ap2scp,
            ),
            platform_channel=ScmiPlatformChannel(
                shmem=self.non_trusted_sram,
                shmem_range=AddrRange(0x2E000000, size=0x200),
                doorbell=self.mailbox.highp_scp2ap,
            ),
        )

        self.scmi = ScmiPlatform(comms=[_scmi_comm])
        self._attach_device(self.scmi, bus)

    def generateDeviceTree(self, state):
        # Generate using standard RealView function
        dt = list(super().generateDeviceTree(state))
        if len(dt) > 1:
            raise Exception("System returned too many DT nodes")
        node = dt[0]

        node.appendCompatible(["arm,vexpress"])
        node.append(FdtPropertyStrings("model", ["V2P-CA15"]))
        node.append(FdtPropertyWords("arm,hbi", [0x0]))
        node.append(FdtPropertyWords("arm,vexpress,site", [0xF]))

        system = self.system.unproxy(self)
        if system._have_psci:
            # PSCI functions exposed to the kernel
            if not system.release.has(ArmExtension("SECURITY")):
                raise AssertionError("PSCI requires EL3 (have_security)")

            psci_node = FdtNode("psci")
            psci_node.appendCompatible(
                ["arm,psci-1.0", "arm,psci-0.2", "arm,psci"]
            )
            method = "smc"
            psci_node.append(FdtPropertyStrings("method", method))
            psci_node.append(FdtPropertyWords("cpu_suspend", 0xC4000001))
            psci_node.append(FdtPropertyWords("cpu_off", 0x84000002))
            psci_node.append(FdtPropertyWords("cpu_on", 0xC4000003))
            psci_node.append(FdtPropertyWords("sys_poweroff", 0x84000008))
            psci_node.append(FdtPropertyWords("sys_reset", 0x84000009))
            node.append(psci_node)

        yield node


class VExpress_GEM5_V1_Base(VExpress_GEM5_Base):
    gic = kvm_gicv2_class(
        dist_addr=0x2C001000, cpu_addr=0x2C002000, it_lines=512
    )
    vgic = VGic(vcpu_addr=0x2C006000, hv_addr=0x2C004000, maint_int=25)
    gicv2m = Gicv2m()
    gicv2m.frames = [Gicv2mFrame(spi_base=256, spi_len=64, addr=0x2C1C0000)]

    def setupBootLoader(self, cur_sys, loc, boot_loader=None):
        if boot_loader is None:
            boot_loader = [loc("boot.arm64"), loc("boot.arm")]
        super().setupBootLoader(cur_sys, boot_loader)

    def _on_chip_devices(self):
        return super()._on_chip_devices() + [self.gic, self.vgic, self.gicv2m]


class VExpress_GEM5_V1(VExpress_GEM5_V1_Base):
    """
    We subclass VExpress_GEM5_V1_Base in order to alias it to
    VExpress_GEM5_V1, which is what gem5 scripts are currently using
    """

    pass


class VExpress_GEM5_V1_HDLcd(VExpress_GEM5_V1_Base):
    hdlcd = HDLcd(
        pxl_clk=VExpress_GEM5_V1_Base.dcc.osc_pxl,
        pio_addr=0x2B000000,
        interrupt=ArmSPI(num=95),
    )

    def _on_chip_devices(self):
        return super()._on_chip_devices() + [self.hdlcd]


class VExpress_GEM5_V2_Base(VExpress_GEM5_Base):
    gic = Gicv3(
        dist_addr=0x2C000000,
        redist_addr=0x2C010000,
        maint_int=ArmPPI(num=25),
        gicv4=True,
        its=Gicv3Its(pio_addr=0x2E010000),
    )

    # Limiting to 128 since it will otherwise overlap with PCI space
    gic.cpu_max = 128

    def _on_chip_devices(self):
        return super()._on_chip_devices() + [self.gic, self.gic.its]

    def setupBootLoader(self, cur_sys, loc, boot_loader=None):
        if boot_loader is None:
            boot_loader = [loc("boot_v2.arm64")]
        super().setupBootLoader(cur_sys, boot_loader)


class VExpress_GEM5_V2(VExpress_GEM5_V2_Base):
    """
    We subclass VExpress_GEM5_V2_Base in order to alias it to
    VExpress_GEM5_V2, which is what gem5 scripts are currently using
    """

    pass


class VExpress_GEM5_V2_HDLcd(VExpress_GEM5_V2_Base):
    hdlcd = HDLcd(
        pxl_clk=VExpress_GEM5_V2_Base.dcc.osc_pxl,
        pio_addr=0x2B000000,
        interrupt=ArmSPI(num=95),
    )

    def _on_chip_devices(self):
        return super()._on_chip_devices() + [self.hdlcd]


class VExpress_GEM5_Foundation(VExpress_GEM5_Base):
    """
    Based on Armv8-A FVP Foundation platform v11.8
    Reference for memory and interrupt map:
        Armv8-A Foundation Platform - User Guide - Version 11.8
        Document ID: 100961_1180_00_en

    We are adding PCI capabilities to the Armv8-A FVP Foundation
    Platform. We are enabling it by using the PCI memory map
    of the Armv8-A FVP Base Platform:
        Fast Models - Reference Manual - Version 11.8
        Document ID: 100964_1108_00_en
    """

    _off_chip_ranges = [
        # CS1-CS5
        AddrRange(0x0C000000, 0x20000000),
        # External AXI interface (PCI)
        AddrRange(0x40000000, 0x80000000),
        AddrRange(0x4000000000, 0x8000000000),
    ]

    sp810_fake = AmbaFake(pio_addr=0x1C020000, ignore_access=True)

    clcd = Pl111(pio_addr=0x1C1F0000, interrupt=ArmSPI(num=46))

    gic = kvm_gicv3_class(
        dist_addr=0x2F000000,
        redist_addr=0x2F100000,
        maint_int=ArmPPI(num=25),
        gicv4=False,
        its=NULL,
    )

    pci_host = GenericArmPciHost(
        conf_base=0x40000000,
        conf_size="256MiB",
        conf_device_bits=12,
        pci_pio_base=0x50000000,
        pci_mem_base=0x4000000000,
        int_policy="ARM_PCI_INT_DEV",
        int_base=100,
        int_count=4,
    )

    def _on_chip_devices(self):
        return super()._on_chip_devices() + [self.gic]

    def _off_chip_devices(self):
        return super()._off_chip_devices() + [self.clcd, self.sp810_fake]

    def setupBootLoader(self, cur_sys, loc, boot_loader=None):
        if boot_loader is None:
            boot_loader = [loc("boot_foundation.arm64")]
        super().setupBootLoader(cur_sys, boot_loader)
