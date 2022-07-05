# Copyright (c) 2021 Huawei International
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
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *


class Plic(BasicPioDevice):
    """
    This implementation of PLIC is based on
    the SiFive U54MC datasheet:
    https://sifive.cdn.prismic.io/sifive/fab000f6-
    0e07-48d0-9602-e437d5367806_sifive_U54MC_rtl_
    full_20G1.03.00_manual.pdf
    """

    type = "Plic"
    cxx_header = "dev/riscv/plic.hh"
    cxx_class = "gem5::Plic"
    pio_size = Param.Addr(0x4000000, "PIO Size")
    n_src = Param.Int("Number of interrupt sources")
    n_contexts = Param.Int(
        "Number of interrupt contexts. Usually the number "
        "of threads * 2. One for M mode, one for S mode"
    )

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "plic", self.pio_addr, self.pio_size
        )

        int_state = FdtState(addr_cells=0, interrupt_cells=1)
        node.append(int_state.addrCellsProperty())
        node.append(int_state.interruptCellsProperty())

        phandle = int_state.phandle(self)
        node.append(FdtPropertyWords("phandle", [phandle]))
        node.append(FdtPropertyWords("riscv,ndev", [self.n_src - 1]))

        cpus = self.system.unproxy(self).cpu
        int_extended = list()
        for cpu in cpus:
            phandle = int_state.phandle(cpu)
            int_extended.append(phandle)
            int_extended.append(0xB)
            int_extended.append(phandle)
            int_extended.append(0x9)

        node.append(FdtPropertyWords("interrupts-extended", int_extended))
        node.append(FdtProperty("interrupt-controller"))
        node.appendCompatible(["riscv,plic0"])

        yield node
