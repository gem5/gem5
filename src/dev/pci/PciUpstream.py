# Copyright (c) 2025 REDS institute of the HEIG-VD
#  All rights reserved
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

from m5.objects.Bridge import (
    Bridge,
    BridgeBase,
)
from m5.objects.ClockedObject import ClockedObject
from m5.objects.Device import IsaFake
from m5.objects.XBar import NoncoherentXBar
from m5.params import *
from m5.proxy import *


class PciBus(NoncoherentXBar):
    type = "PciBus"
    cxx_class = "gem5::PciBus"
    cxx_header = "dev/pci/bus.hh"

    config_error_port = RequestPort("Port to send config errors to")

    # Set some default values bases on IOXBar
    width = 16
    frontend_latency = 2
    forward_latency = 1
    response_latency = 2


class PciConfigError(IsaFake):
    type = "PciConfigError"
    cxx_class = "gem5::PciConfigError"
    cxx_header = "dev/pci/upstream.hh"

    # Those will be set by the PciUpstream
    pio_addr = Param.Addr(0x0, "Size of address range")
    pio_size = Param.Addr(0x0, "Size of address range")


class PciUpDownBridge(BridgeBase):
    type = "PciUpDownBridge"
    cxx_class = "gem5::PciUpDownBridge"
    cxx_header = "dev/pci/up_down_bridge.hh"


class PciUpstream(ClockedObject):
    type = "PciUpstream"
    cxx_class = "gem5::PciUpstream"
    cxx_header = "dev/pci/upstream.hh"
    abstract = True

    up_to_down = Param.PciUpDownBridge(
        PciUpDownBridge(), "Bridge upstream -> downstream"
    )
    down_to_up = Param.BridgeBase(Bridge(), "Bridge downstream -> upstream")

    config_error = Param.PciConfigError(
        PciConfigError(), "Device to handle config errors"
    )

    def down_response_port(self):
        return self.down_to_up.cpu_side_port

    def down_request_port(self):
        return self.up_to_down.mem_side_port

    def up_response_port(self):
        return self.up_to_down.cpu_side_port

    def up_request_port(self):
        return self.down_to_up.mem_side_port
