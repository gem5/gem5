# Copyright (c) 2012-2013, 2017-2018 ARM Limited
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
#
# Authors: Andreas Sandberg

from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *
from m5.SimObject import SimObject

from m5.objects.Device import PioDevice
from m5.objects.Platform import Platform

class BaseGic(PioDevice):
    type = 'BaseGic'
    abstract = True
    cxx_header = "dev/arm/base_gic.hh"

    platform = Param.Platform(Parent.any, "Platform this device is part of.")

    gicd_iidr = Param.UInt32(0,
        "Distributor Implementer Identification Register")
    gicd_pidr = Param.UInt32(0,
        "Peripheral Identification Register")
    gicc_iidr = Param.UInt32(0,
        "CPU Interface Identification Register")
    gicv_iidr = Param.UInt32(0,
        "VM CPU Interface Identification Register")

class ArmInterruptPin(SimObject):
    type = 'ArmInterruptPin'
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "ArmInterruptPinGen"
    abstract = True

    platform = Param.Platform(Parent.any, "Platform with interrupt controller")
    num = Param.UInt32("Interrupt number in GIC")

class ArmSPI(ArmInterruptPin):
    type = 'ArmSPI'
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "ArmSPIGen"

class ArmPPI(ArmInterruptPin):
    type = 'ArmPPI'
    cxx_header = "dev/arm/base_gic.hh"
    cxx_class = "ArmPPIGen"

class GicV2(BaseGic):
    type = 'GicV2'
    cxx_header = "dev/arm/gic_v2.hh"

    dist_addr = Param.Addr("Address for distributor")
    cpu_addr = Param.Addr("Address for cpu")
    cpu_size = Param.Addr(0x2000, "Size of cpu register bank")
    dist_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to distributor")
    cpu_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to cpu interface")
    int_latency = Param.Latency('10ns', "Delay for interrupt to get to CPU")
    it_lines = Param.UInt32(128, "Number of interrupt lines supported (max = 1020)")
    gem5_extensions = Param.Bool(False, "Enable gem5 extensions")

class Gic400(GicV2):
    """
    As defined in:
    "ARM Generic Interrupt Controller Architecture" version 2.0
    "CoreLink GIC-400 Generic Interrupt Controller" revision r0p1
    """
    gicd_pidr = 0x002bb490
    gicd_iidr = 0x0200143B
    gicc_iidr = 0x0202143B

    # gicv_iidr same as gicc_idr
    gicv_iidr = gicc_iidr

class Gicv2mFrame(SimObject):
    type = 'Gicv2mFrame'
    cxx_header = "dev/arm/gic_v2m.hh"
    spi_base = Param.UInt32(0x0, "Frame SPI base number");
    spi_len = Param.UInt32(0x0, "Frame SPI total number");
    addr = Param.Addr("Address for frame PIO")

class Gicv2m(PioDevice):
    type = 'Gicv2m'
    cxx_header = "dev/arm/gic_v2m.hh"

    pio_delay = Param.Latency('10ns', "Delay for PIO r/w")
    gic = Param.BaseGic(Parent.any, "Gic on which to trigger interrupts")
    frames = VectorParam.Gicv2mFrame([], "Power of two number of frames")

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

    # gicv_iidr same as gicc_idr
    gicv_iidr = Param.UInt32(Self.gic.gicc_iidr,
        "VM CPU Interface Identification Register")

    def generateDeviceTree(self, state):
        gic = self.gic.unproxy(self)

        node = FdtNode("interrupt-controller")
        node.appendCompatible(["gem5,gic", "arm,cortex-a15-gic",
                               "arm,cortex-a9-gic"])
        node.append(FdtPropertyWords("#interrupt-cells", [3]))
        node.append(FdtPropertyWords("#address-cells", [0]))
        node.append(FdtProperty("interrupt-controller"))

        regs = (
            state.addrCells(gic.dist_addr) +
            state.sizeCells(0x1000) +
            state.addrCells(gic.cpu_addr) +
            state.sizeCells(0x1000) +
            state.addrCells(self.hv_addr) +
            state.sizeCells(0x2000) +
            state.addrCells(self.vcpu_addr) +
            state.sizeCells(0x2000) )

        node.append(FdtPropertyWords("reg", regs))
        node.append(FdtPropertyWords("interrupts",
                                     [1, int(self.ppint)-16, 0xf04]))

        node.appendPhandle(gic)

        yield node

class Gicv3(BaseGic):
    type = 'Gicv3'
    cxx_header = "dev/arm/gic_v3.hh"

    dist_addr = Param.Addr(0x2c000000, "Address for distributor")
    dist_pio_delay = Param.Latency('10ns', "Delay for PIO r/w to distributor")
    redist_addr = Param.Addr(0x2c010000, "Address for redistributors")
    redist_pio_delay = Param.Latency('10ns',
            "Delay for PIO r/w to redistributors")
    it_lines = Param.UInt32(1020,
            "Number of interrupt lines supported (max = 1020)")
