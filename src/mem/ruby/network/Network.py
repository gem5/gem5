# Copyright (c) 2009 Advanced Micro Devices, Inc.
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

from m5.objects.BasicLink import BasicLink
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class RubyNetwork(ClockedObject):
    type = "RubyNetwork"
    cxx_class = "gem5::ruby::Network"
    cxx_header = "mem/ruby/network/Network.hh"
    abstract = True

    topology = Param.String(
        "Not Specified", "the name of the imported topology module"
    )

    number_of_virtual_networks = Param.Unsigned(
        "Number of virtual networks "
        "used by the coherence protocol in use.  The on-chip network "
        "assumes the protocol numbers vnets starting from 0.  Therefore, "
        "the number of virtual networks should be one more than the "
        "highest numbered vnet in use."
    )
    control_msg_size = Param.Int(8, "")
    ruby_system = Param.RubySystem("")

    routers = VectorParam.BasicRouter("Network routers")
    netifs = VectorParam.ClockedObject("Network Interfaces")
    ext_links = VectorParam.BasicExtLink("Links to external nodes")
    int_links = VectorParam.BasicIntLink("Links between internal nodes")

    in_port = VectorResponsePort("CPU input port")
    slave = DeprecatedParam(in_port, "`slave` is now called `in_port`")
    out_port = VectorRequestPort("CPU output port")
    master = DeprecatedParam(out_port, "`master` is now called `out_port`")

    data_msg_size = Param.Int(
        Parent.block_size_bytes,
        "Size of data messages. Defaults to the parent "
        "RubySystem cache line size.",
    )
