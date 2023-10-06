# Copyright (c) 2020 ARM Limited
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
from m5.objects.Doorbell import Doorbell
from m5.params import *
from m5.proxy import *
from m5.util.fdthelper import *


class MhuDoorbell(Doorbell):
    type = "MhuDoorbell"
    abstract = True
    cxx_header = "dev/arm/css/mhu.hh"
    cxx_class = "gem5::MhuDoorbell"


class Scp2ApDoorbell(MhuDoorbell):
    type = "Scp2ApDoorbell"
    cxx_header = "dev/arm/css/mhu.hh"
    cxx_class = "gem5::Scp2ApDoorbell"

    interrupt = Param.ArmInterruptPin("Interrupt Pin")


class Ap2ScpDoorbell(MhuDoorbell):
    type = "Ap2ScpDoorbell"
    cxx_header = "dev/arm/css/mhu.hh"
    cxx_class = "gem5::Ap2ScpDoorbell"


# Message Handling Unit
class MHU(BasicPioDevice):
    type = "MHU"
    cxx_header = "dev/arm/css/mhu.hh"
    cxx_class = "gem5::MHU"
    pio_size = Param.Unsigned(0x1000, "MHU pio size")

    lowp_scp2ap = Param.Scp2ApDoorbell(
        "Low Priority doorbell channel for communications "
        "from the System Control Processor (SCP) to the "
        "Application Processor (AP)",
    )
    highp_scp2ap = Param.Scp2ApDoorbell(
        "High Priority doorbell channel for communications "
        "from the System Control Processor (SCP) to the "
        "Application Processor (AP)",
    )
    sec_scp2ap = Param.Scp2ApDoorbell(
        "Secure doorbell channel for communications "
        "from the System Control Processor (SCP) to the "
        "Application Processor (AP)",
    )

    lowp_ap2scp = Param.Ap2ScpDoorbell(
        "Low Priority doorbell channel for communications "
        "from the Application Processor (AP) to the "
        "System Control Processor (SCP)",
    )
    highp_ap2scp = Param.Ap2ScpDoorbell(
        "High Priority doorbell channel for communications "
        "from the Application Processor (AP) to the "
        "System Control Processor (SCP)",
    )
    sec_ap2scp = Param.Ap2ScpDoorbell(
        "Secure doorbell channel for communications "
        "from the Application Processor (AP) to the "
        "System Control Processor (SCP)",
    )

    scp = Param.Scp(Parent.any, "System Control Processor")

    def generateDeviceTree(self, state):
        node = FdtNode(f"mailbox@{int(self.pio_addr):x}")
        node.appendCompatible(["arm,mhu", "arm,primecell"])
        node.append(
            FdtPropertyWords(
                "reg",
                state.addrCells(self.pio_addr)
                + state.sizeCells(self.pio_size),
            ),
        )
        node.append(FdtPropertyWords("#mbox-cells", 1))

        node.append(
            FdtPropertyWords(
                "interrupts",
                [
                    0,
                    int(self.lowp_scp2ap.interrupt.num) - 32,
                    1,
                    0,
                    int(self.highp_scp2ap.interrupt.num) - 32,
                    1,
                    0,
                    int(self.sec_scp2ap.interrupt.num) - 32,
                    1,
                ],
            ),
        )

        realview = self._parent.unproxy(self)
        node.append(
            FdtPropertyWords(
                "clocks",
                [
                    state.phandle(realview.mcc.osc_peripheral),
                    state.phandle(realview.dcc.osc_smb),
                ],
            ),
        )
        node.append(FdtPropertyStrings("clock-names", ["apb_pclk"]))

        node.appendPhandle(self)

        yield node
