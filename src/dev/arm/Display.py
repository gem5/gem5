# Copyright (c) 2019 ARM Limited
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

from m5.params import *
from m5.SimObject import SimObject
from m5.util.fdthelper import *

class Display(SimObject):
    type = 'Display'
    cxx_header = "dev/arm/display.hh"
    clock_frequency = Param.Unsigned("clock-frequency property")
    hactive = Param.Unsigned("hactive property")
    vactive = Param.Unsigned("vactive property")
    hfront_porch = Param.Unsigned("hfront-porch property")
    hback_porch = Param.Unsigned("hback-porch property")
    hsync_len = Param.Unsigned("hsync-len property")
    vfront_porch = Param.Unsigned("vfront-porch property")
    vback_porch = Param.Unsigned("vback-porch property")
    vsync_len = Param.Unsigned("vsync-len property")

    _endpoint_node = None

    def endpointPhandle(self):
        return "encoder_endpoint"

    def endpointNode(self):
        assert self._endpoint_node is not None
        return self._endpoint_node

    def generateDeviceTree(self, state):
        # timing node
        timing_node = FdtNode(self.timingNode())

        timing_node.append(FdtPropertyWords(
            "clock-frequency", [self.clock_frequency]))
        timing_node.append(FdtPropertyWords(
            "hactive", [self.hactive]))
        timing_node.append(FdtPropertyWords(
            "vactive", [self.vactive]))
        timing_node.append(FdtPropertyWords(
            "hfront-porch", [self.hfront_porch]))
        timing_node.append(FdtPropertyWords(
            "hback-porch", [self.hback_porch]))
        timing_node.append(FdtPropertyWords(
            "hsync-len", [self.hsync_len]))
        timing_node.append(FdtPropertyWords(
            "vfront-porch", [self.vfront_porch]))
        timing_node.append(FdtPropertyWords(
            "vback-porch", [self.vback_porch]))
        timing_node.append(FdtPropertyWords(
            "vsync-len", [self.vsync_len]))

        timing_node.appendPhandle(self.timingNode())

        # display timing node
        dispt_node = FdtNode("display-timings")
        dispt_node.append(FdtPropertyWords("native-mode",
            state.phandle(self.timingNode())))
        dispt_node.append(timing_node)

        # endpoint node
        endpoint_node = FdtNode("endpoint")
        endpoint_node.appendPhandle(
            self.endpointPhandle())

        # Assign node so that it can be retrieved
        self._endpoint_node = endpoint_node

        # port node
        port_node = FdtNode("port")
        port_node.append(endpoint_node)

        # Virt-encoder
        node = FdtNode("virt-encoder")
        node.appendCompatible(["drm,virtual-encoder"])
        node.append(dispt_node)
        node.append(port_node)

        yield node

class Display1080p(Display):
    clock_frequency = 148500000
    hactive = 1920
    vactive = 1080
    hfront_porch = 148
    hback_porch = 88
    hsync_len = 44
    vfront_porch = 36
    vback_porch = 4
    vsync_len = 5

    def timingNode(self):
        return "timing_1080p60"
