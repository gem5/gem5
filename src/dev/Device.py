# Copyright (c) 2012-2016,2019-2020 ARM Limited
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
# Copyright (c) 2005-2007 The Regents of The University of Michigan
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

from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *

from m5.objects.ClockedObject import ClockedObject


class PioDevice(ClockedObject):
    type = "PioDevice"
    cxx_header = "dev/io_device.hh"
    cxx_class = "gem5::PioDevice"
    abstract = True

    pio = ResponsePort("Programmed I/O port")
    system = Param.System(Parent.any, "System this device is part of")

    def generateBasicPioDeviceNode(
        self, state, name, pio_addr, size, interrupts=None
    ):
        node = FdtNode(f"{name}@{int(pio_addr):x}")
        node.append(
            FdtPropertyWords(
                "reg", state.addrCells(pio_addr) + state.sizeCells(size)
            )
        )

        if interrupts:
            if any([i.num < 32 for i in interrupts]):
                raise (
                    ("Interrupt number smaller than 32 " + " in PioDevice %s")
                    % name
                )

            gic = self._parent.unproxy(self).gic

            node.append(
                FdtPropertyWords(
                    "interrupts",
                    sum([i.generateFdtProperty(gic) for i in interrupts], []),
                )
            )

        return node


class BasicPioDevice(PioDevice):
    type = "BasicPioDevice"
    cxx_header = "dev/io_device.hh"
    cxx_class = "gem5::BasicPioDevice"
    abstract = True

    pio_addr = Param.Addr("Device Address")
    pio_latency = Param.Latency("100ns", "Programmed IO latency")


class DmaDevice(PioDevice):
    type = "DmaDevice"
    cxx_header = "dev/dma_device.hh"
    cxx_class = "gem5::DmaDevice"
    abstract = True

    dma = RequestPort("DMA port")

    _iommu = None

    sid = Param.Unsigned(
        0,
        "Stream identifier used by an IOMMU to distinguish amongst "
        "several devices attached to it",
    )
    ssid = Param.Unsigned(
        0,
        "Substream identifier used by an IOMMU to distinguish amongst "
        "several devices attached to it",
    )

    def addIommuProperty(self, state, node):
        """
        This method takes an FdtState and a FdtNode as parameters, and
        it is appending a "iommus = <>" property in case the DmaDevice
        is attached to an IOMMU.
        This method is necessary for autogenerating a binding between
        a dma device and the iommu.
        """
        if self._iommu is not None:
            node.append(
                FdtPropertyWords(
                    "iommus", [state.phandle(self._iommu), self.sid]
                )
            )


class DmaVirtDevice(DmaDevice):
    type = "DmaVirtDevice"
    cxx_header = "dev/dma_virt_device.hh"
    cxx_class = "gem5::DmaVirtDevice"
    abstract = True


class IsaFake(BasicPioDevice):
    type = "IsaFake"
    cxx_header = "dev/isa_fake.hh"
    cxx_class = "gem5::IsaFake"

    pio_size = Param.Addr(0x8, "Size of address range")
    ret_data8 = Param.UInt8(0xFF, "Default data to return")
    ret_data16 = Param.UInt16(0xFFFF, "Default data to return")
    ret_data32 = Param.UInt32(0xFFFFFFFF, "Default data to return")
    ret_data64 = Param.UInt64(0xFFFFFFFFFFFFFFFF, "Default data to return")
    ret_bad_addr = Param.Bool(False, "Return pkt status bad address on access")
    update_data = Param.Bool(
        False, "Update the data that is returned on writes"
    )
    warn_access = Param.String("", "String to print when device is accessed")
    fake_mem = Param.Bool(
        False,
        "Is this device acting like a memory and thus may get a cache line sized req",
    )


class BadAddr(IsaFake):
    pio_addr = 0
    ret_bad_addr = Param.Bool(True, "Return pkt status bad address on access")
