# Copyright (c) 2012-2013, 2017-2020 ARM Limited
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
from m5.objects.Device import BasicPioDevice
from m5.objects.Device import PioDevice
from m5.objects.IntPin import IntSourcePin
from m5.objects.Platform import Platform
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util.fdthelper import *


class BaseGic(PioDevice):
    type = "BaseGic"
    abstract = True
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "gem5::BaseGic"

    # Used for DTB autogeneration
    _state = FdtState(addr_cells=0, interrupt_cells=3)

    platform = Param.Platform(Parent.any, "Platform this device is part of.")

    gicd_iidr = Param.UInt32(
        0, "Distributor Implementer Identification Register"
    )
    gicd_pidr = Param.UInt32(0, "Peripheral Identification Register")
    gicc_iidr = Param.UInt32(0, "CPU Interface Identification Register")
    gicv_iidr = Param.UInt32(0, "VM CPU Interface Identification Register")

    def interruptCells(self, int_type, int_num, int_trigger, partition=None):
        """
        Interupt cells generation helper:
        Following specifications described in

        Documentation/devicetree/bindings/interrupt-controller/arm,gic.txt
        """
        assert self._state.interrupt_cells == 3

        # Check for affinity in case of PPI. If there is no PPI
        # partitioning, set the affinity to target all CPUs
        # (affinity = 0xf00)
        if partition is None and int_type == ArmPPI._LINUX_ID:
            affinity = 0xF00
        else:
            affinity = 0

        return [int_type, int_num, affinity | int_trigger]


class ArmInterruptType(ScopedEnum):
    """
    The values of the scoped enum are matching Linux macroes
    defined in include/linux/irq.h. They are mainly meant
    to be used for DTB autogen
    """

    map = {
        "IRQ_TYPE_EDGE_RISING": 0x1,
        "IRQ_TYPE_EDGE_FALLING": 0x2,
        "IRQ_TYPE_LEVEL_HIGH": 0x4,
        "IRQ_TYPE_LEVEL_LOW": 0x8,
    }


class ArmInterruptPin(SimObject):
    type = "ArmInterruptPin"
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "gem5::ArmInterruptPinGen"
    abstract = True

    platform = Param.Platform(Parent.any, "Platform with interrupt controller")
    num = Param.UInt32("Interrupt number in GIC")
    int_type = Param.ArmInterruptType(
        "IRQ_TYPE_LEVEL_HIGH", "Interrupt type (level/edge triggered)"
    )


class ArmSPI(ArmInterruptPin):
    type = "ArmSPI"
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "gem5::ArmSPIGen"

    _LINUX_ID = 0

    def generateFdtProperty(self, gic):
        """
        Return a list used as an entry for an interrupt FdtProperty

        Subtracting 32 because Linux assumes that SPIs start at 0, while
        gem5 uses the internal GIC numbering (SPIs start at 32)
        """
        return gic.interruptCells(
            self._LINUX_ID, self.num - 32, int(self.int_type.getValue())
        )


class ArmPPI(ArmInterruptPin):
    type = "ArmPPI"
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "gem5::ArmPPIGen"

    _LINUX_ID = 1

    def generateFdtProperty(self, gic):
        """
        Return a list used as an entry for an interrupt FdtProperty

        Subtracting 16 because Linux assumes that PPIs start at 0, while
        gem5 uses the internal GIC numbering (PPIs start at 16)
        """
        return gic.interruptCells(
            self._LINUX_ID, self.num - 16, int(self.int_type.getValue())
        )


class ArmSigInterruptPin(ArmInterruptPin):
    type = "ArmSigInterruptPin"
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "gem5::ArmSigInterruptPinGen"

    irq = IntSourcePin("Interrupt pin")


class GicV2(BaseGic):
    type = "GicV2"
    cxx_header = "dev/arm/gic_v2.hh"
    cxx_class = "gem5::GicV2"

    dist_addr = Param.Addr("Address for distributor")
    cpu_addr = Param.Addr("Address for cpu")
    cpu_size = Param.Addr(0x2000, "Size of cpu register bank")
    dist_pio_delay = Param.Latency("10ns", "Delay for PIO r/w to distributor")
    cpu_pio_delay = Param.Latency("10ns", "Delay for PIO r/w to cpu interface")
    int_latency = Param.Latency("10ns", "Delay for interrupt to get to CPU")
    it_lines = Param.UInt32(
        128, "Number of interrupt lines supported (max = 1020)"
    )
    gem5_extensions = Param.Bool(False, "Enable gem5 extensions")


class Gic400(GicV2):
    """
    As defined in:
    "ARM Generic Interrupt Controller Architecture" version 2.0
    "CoreLink GIC-400 Generic Interrupt Controller" revision r0p1
    """

    gicd_pidr = 0x002BB490
    gicd_iidr = 0x0200143B
    gicc_iidr = 0x0202143B

    # gicv_iidr same as gicc_idr
    gicv_iidr = gicc_iidr


class Gicv2mFrame(SimObject):
    type = "Gicv2mFrame"
    cxx_header = "dev/arm/gic_v2m.hh"
    cxx_class = "gem5::Gicv2mFrame"
    spi_base = Param.UInt32(0x0, "Frame SPI base number")
    spi_len = Param.UInt32(0x0, "Frame SPI total number")
    addr = Param.Addr("Address for frame PIO")


class Gicv2m(PioDevice):
    type = "Gicv2m"
    cxx_header = "dev/arm/gic_v2m.hh"
    cxx_class = "gem5::Gicv2m"

    pio_delay = Param.Latency("10ns", "Delay for PIO r/w")
    gic = Param.BaseGic(Parent.any, "Gic on which to trigger interrupts")
    frames = VectorParam.Gicv2mFrame([], "Power of two number of frames")


class VGic(PioDevice):
    type = "VGic"
    cxx_header = "dev/arm/vgic.hh"
    cxx_class = "gem5::VGic"
    gic = Param.BaseGic(Parent.any, "Gic to use for interrupting")
    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    vcpu_addr = Param.Addr(0, "Address for vcpu interfaces")
    hv_addr = Param.Addr(0, "Address for hv control")
    pio_delay = Param.Latency("10ns", "Delay for PIO r/w")
    # The number of list registers is not currently configurable at runtime.
    maint_int = Param.UInt32("HV maintenance interrupt number")

    # gicv_iidr same as gicc_idr
    gicv_iidr = Param.UInt32(
        Self.gic.gicc_iidr, "VM CPU Interface Identification Register"
    )

    def generateDeviceTree(self, state):
        gic = self.gic.unproxy(self)

        node = FdtNode("interrupt-controller")
        node.appendCompatible(
            ["gem5,gic", "arm,cortex-a15-gic", "arm,cortex-a9-gic"]
        )
        node.append(gic._state.interruptCellsProperty())
        node.append(gic._state.addrCellsProperty())
        node.append(FdtProperty("interrupt-controller"))

        regs = (
            state.addrCells(gic.dist_addr)
            + state.sizeCells(0x1000)
            + state.addrCells(gic.cpu_addr)
            + state.sizeCells(0x1000)
            + state.addrCells(self.hv_addr)
            + state.sizeCells(0x2000)
            + state.addrCells(self.vcpu_addr)
            + state.sizeCells(0x2000)
        )

        node.append(FdtPropertyWords("reg", regs))
        node.append(
            FdtPropertyWords(
                "interrupts", [1, int(self.maint_int) - 16, 0xF04]
            )
        )

        node.appendPhandle(gic)

        yield node


class Gicv3Its(BasicPioDevice):
    type = "Gicv3Its"
    cxx_header = "dev/arm/gic_v3_its.hh"
    cxx_class = "gem5::Gicv3Its"

    dma = RequestPort("DMA port")
    pio_size = Param.Unsigned(0x20000, "Gicv3Its pio size")

    # CIL [36] = 0: ITS supports 16-bit CollectionID
    # Devbits [17:13] = 0b100011: ITS supports 23 DeviceID bits
    # ID_bits [12:8] = 0b11111: ITS supports 31 EventID bits
    gits_typer = Param.UInt64(0x30023F01, "GITS_TYPER RO value")

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "gic-its", self.pio_addr, self.pio_size
        )
        node.appendCompatible(["arm,gic-v3-its"])
        node.append(FdtProperty("msi-controller"))
        node.append(FdtPropertyWords("#msi-cells", [1]))

        return node


class Gicv3(BaseGic):
    type = "Gicv3"
    cxx_header = "dev/arm/gic_v3.hh"
    cxx_class = "gem5::Gicv3"

    # Used for DTB autogeneration
    _state = FdtState(addr_cells=2, size_cells=2, interrupt_cells=3)

    its = Param.Gicv3Its(NULL, "GICv3 Interrupt Translation Service")

    dist_addr = Param.Addr("Address for distributor")
    dist_pio_delay = Param.Latency("10ns", "Delay for PIO r/w to distributor")
    redist_addr = Param.Addr("Address for redistributors")
    redist_pio_delay = Param.Latency(
        "10ns", "Delay for PIO r/w to redistributors"
    )
    it_lines = Param.UInt32(
        1020, "Number of interrupt lines supported (max = 1020)"
    )

    maint_int = Param.ArmInterruptPin(
        "HV maintenance interrupt."
        "ARM strongly recommends that maintenance interrupts "
        "are configured to use INTID 25 (PPI Interrupt)."
    )

    cpu_max = Param.Unsigned(
        256,
        "Maximum number of PE. This is affecting the maximum number of "
        "redistributors",
    )

    gicv4 = Param.Bool(False, "GIC is GICv4 compatible")

    reserved_is_res0 = Param.Bool(
        True,
        "According to the GIC specification (IHI0069) "
        "reserved addresses in the GIC memory map are treated as RES0. "
        "We allow to disable this behaviour and panic instead "
        "(reserved_res0 = False) to catch development bugs "
        "(in gem5 and in the guest SW)",
    )

    def interruptCells(self, int_type, int_num, int_trigger, partition=None):
        """
        Interupt cells generation helper:
        Following specifications described in

        Documentation/devicetree/bindings/interrupt-controller/arm,gic-v3.txt
        """
        prop = self._state.interruptCells(0)
        assert len(prop) >= 3
        prop[0] = int_type
        prop[1] = int_num
        prop[2] = int_trigger
        return prop

    def generateDeviceTree(self, state):
        node = FdtNode("interrupt-controller")
        node.appendCompatible(["arm,gic-v3"])
        node.append(self._state.interruptCellsProperty())
        node.append(self._state.addrCellsProperty())
        node.append(self._state.sizeCellsProperty())
        node.append(FdtProperty("ranges"))
        node.append(FdtProperty("interrupt-controller"))

        redist_stride = 0x40000 if self.gicv4 else 0x20000
        node.append(
            FdtPropertyWords(
                "redistributor-stride", state.sizeCells(redist_stride)
            )
        )

        regs = (
            state.addrCells(self.dist_addr)
            + state.sizeCells(0x10000)
            + state.addrCells(self.redist_addr)
            + state.sizeCells(0x2000000)
        )

        node.append(FdtPropertyWords("reg", regs))
        node.append(
            FdtPropertyWords(
                "interrupts",
                self.interruptCells(1, int(self.maint_int.num) - 16, 0x4),
            )
        )

        node.appendPhandle(self)

        # Generate the ITS device tree if instantiated
        if self.its != NULL:
            node.append(self.its.generateDeviceTree(self._state))

        yield node
