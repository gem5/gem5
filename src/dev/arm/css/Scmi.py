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
from m5.objects.Doorbell import Doorbell
from m5.objects.Scp import Scp
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from m5.util.fdthelper import *


class ScmiChannel(SimObject):
    """
    Unidirectional channel
    """

    type = "ScmiChannel"
    cxx_header = "dev/arm/css/scmi_platform.hh"
    cxx_class = "gem5::scmi::VirtualChannel"
    shmem_range = Param.AddrRange(
        "Virtual channel's shared memory address range"
    )
    phys_id = Param.Unsigned(4, "Physical slot of the channel")
    virt_id = Param.Unsigned(
        0, "Virtual slot of the channel (within the physical)"
    )
    doorbell = Param.Doorbell(
        "This is the doorbell used to notify the SCMI platform"
    )

    def __init__(self, shmem, *args, **kwargs):
        super().__init__(**kwargs)

        def shmemGenerator(state):
            shmem_node = FdtNode("scp-shmem@%x" % 0)
            shmem_node.appendCompatible(["arm,scmi-shmem"])
            shmem_node.append(
                FdtPropertyWords(
                    "reg", state.addrCells(0) + state.sizeCells(0x200)
                )
            )
            # shmem_node.appendPhandle(self._parent.unproxy(self).channel)
            shmem_node.appendPhandle("scmi_virt" + str(self.virt_id))
            return shmem_node

        self._shmem = shmem
        self._shmem.addSubnodeGenerator(shmemGenerator)


class ScmiAgentChannel(ScmiChannel):
    """
    This is a Agent to Platform channel (The agent is the initiator)
    """

    type = "ScmiAgentChannel"
    cxx_header = "dev/arm/css/scmi_platform.hh"
    cxx_class = "gem5::scmi::AgentChannel"


class ScmiPlatformChannel(ScmiChannel):
    """
    This is a Platform to Agent channel (The platform is the initiator)
    """

    type = "ScmiPlatformChannel"
    cxx_header = "dev/arm/css/scmi_platform.hh"
    cxx_class = "gem5::scmi::PlatformChannel"


class ScmiCommunication(SimObject):
    """
    The SCMI Communication class models a bidirectional
    communication between the SCMI platform and the agent.
    As such it has a ScmiAgentChannel and a ScmiPlatformChannel
    object as members.
    """

    type = "ScmiCommunication"
    cxx_header = "dev/arm/css/scmi_platform.hh"
    cxx_class = "gem5::scmi::Communication"

    agent_channel = Param.ScmiAgentChannel("Agent to Platform channel")
    platform_channel = Param.ScmiPlatformChannel("Platform to Agent channel")


class ScmiPlatform(Scp):
    type = "ScmiPlatform"
    cxx_header = "dev/arm/css/scmi_platform.hh"
    cxx_class = "gem5::scmi::Platform"

    comms = VectorParam.ScmiCommunication([], "SCMI Communications")
    agents = VectorParam.String(
        ["OSPM"], "Vector of SCMI agents (names) in the system"
    )

    sys = Param.System(Parent.any, "System object parameter")
    dma = MasterPort("DMA port")

    # Protocol params
    base_vendor = Param.String(
        "arm", "Return string for the Base protocol DISCOVER_VENDOR command"
    )
    base_subvendor = Param.String(
        "gem5",
        "Return string for the Base protocol DISCOVER_SUBVENDOR command",
    )
    base_impl_version = Param.Unsigned(
        0,
        "Return value for the Base protocol "
        "DISCOVER_IMPLEMENTATION_VERSION command",
    )

    def generateDeviceTree(self, state):
        scmi_node = self.generateScmiNode(state)

        fw_node = FdtNode("firmware")
        fw_node.append(scmi_node)
        yield fw_node

    def generateScmiNode(self, state):
        node = FdtNode("scmi")
        node.appendCompatible(["arm,scmi"])

        mbox_phandle = state.phandle(self._parent.unproxy(self).mailbox)
        shmem_phandles = []
        for comm in self.unproxy(self).comms:
            shmem_phandles.append(
                state.phandle("scmi_virt" + str(comm.agent_channel.virt_id))
            )
            shmem_phandles.append(
                state.phandle("scmi_virt" + str(comm.platform_channel.virt_id))
            )

        phys_channel = 1  # HP-NonSecure
        node.append(FdtPropertyWords("mboxes", [mbox_phandle, phys_channel]))
        node.append(FdtPropertyWords("shmem", shmem_phandles))
        return node
