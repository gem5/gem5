# Copyright (c) 2025 Nikita Proshkin
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

from m5.objects.Device import PioDevice
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *


class SfPDMA(PioDevice):
    type = "SfPDMA"
    cxx_header = "dev/riscv/sf_pdma.hh"
    cxx_class = "gem5::SfPDMA"

    dma = VectorRequestPort("DMA ports")

    pio_addr = Param.Addr("Device base address")
    pio_size = Param.Addr("Size of address range")
    chan_cnt = Param.UInt8(4, "Number of DMA channels that exist on device")
    platform = Param.Platform(Parent.any, "Platform")

    sid = Param.UInt8(0, "StreamID for iommu")

    done_irq = VectorParam.Int("Done IRQ for channels")
    error_irq = VectorParam.Int("Error IRQ for channels")

    pkts_each_dir = Param.UInt8(
        5,
        "Max number of in-flight packets in each direction "
        "(e.g. 1 means there might be 2 dma packets from "
        "SfPDMA in memory system - one for read and one for write)",
    )

    chan_reg_rd_wr_delay = Param.Cycles(
        1, "Latency to access MMIO channels registers"
    )
    lat_before_begin = Param.Cycles(
        1, "Latency after a DMA command is seen before it's processed"
    )
    lat_before_completion = Param.Cycles(
        1,
        "Latency between DMA completion and displaying it with registers and irq",
    )
    dma_ports_width = Param.Unsigned(
        64, "Width in bytes of the channels dma ports"
    )

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.pio_size = 0x1000 * int(self.chan_cnt)

    # fdt generation
    _dma_coherent = True
    _iommu = None

    def generateDeviceTree(self, state):
        node = FdtNode(f"dma@{int(self.pio_addr):x}")
        node.appendCompatible(["microchip,mpfs-pdma", "sifive,pdma0"])
        node.append(
            FdtPropertyWords(
                "reg",
                state.addrCells(self.pio_addr)
                + state.sizeCells(0x1000 * int(self.chan_cnt)),
            )
        )
        node.append(FdtPropertyWords("#dma-cells", [1]))
        node.append(FdtPropertyWords("dma-channels", [self.chan_cnt]))

        platform = self.platform.unproxy(self)
        plic = platform.plic

        if plic is not None:
            node.append(
                FdtPropertyWords("interrupt-parent", state.phandle(plic))
            )

            # driver awaits interrupts list in form [done_irq0, err_irq0, ...]
            driver_irq = []
            for done, err in zip(self.done_irq, self.error_irq):
                driver_irq.append(done)
                driver_irq.append(err)

            node.append(FdtPropertyWords("interrupts", driver_irq))
        else:
            print("SfPDMA: Failed to find plic")

        if self._dma_coherent:
            node.append(FdtProperty("dma-coherent"))

        if self._iommu is not None:
            node.append(
                FdtPropertyWords(
                    "iommus", [state.phandle(self._iommu), self.sid]
                )
            )

        yield node
