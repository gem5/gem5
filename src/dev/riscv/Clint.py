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
from m5.objects.IntPin import IntSinkPin
from m5.objects.ResetPort import ResetResponsePort
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *


class Clint(BasicPioDevice):
    """
    This implementation of CLINT is based on
    the SiFive U54MC datasheet:
    https://sifive.cdn.prismic.io/sifive/fab000f6-
    0e07-48d0-9602-e437d5367806_sifive_U54MC_rtl_
    full_20G1.03.00_manual.pdf
    """

    type = "Clint"
    cxx_header = "dev/riscv/clint.hh"
    cxx_class = "gem5::Clint"
    int_pin = IntSinkPin("Pin to receive RTC signal")
    pio_size = Param.Addr(0xC000, "PIO Size")
    num_threads = Param.Int("Number of threads in the system.")
    reset = ResetResponsePort("Reset")
    reset_mtimecmp = Param.Bool(
        False, "Change mtimecmp to `mtimecmp_reset_value` when reset"
    )
    mtimecmp_reset_value = Param.UInt64(
        0xFFFFFFFFFFFFFFFF, "mtimecmp reset value"
    )

    def generateDeviceTree(self, state):
        node = self.generateBasicPioDeviceNode(
            state, "clint", self.pio_addr, self.pio_size
        )

        cpus = self.system.unproxy(self).cpu
        int_extended = list()
        for cpu in cpus:
            phandle = state.phandle(cpu)
            int_extended.append(phandle)
            int_extended.append(0x3)
            int_extended.append(phandle)
            int_extended.append(0x7)

        node.append(FdtPropertyWords("interrupts-extended", int_extended))
        node.appendCompatible(["riscv,clint0"])

        yield node
