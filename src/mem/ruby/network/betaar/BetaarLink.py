# Copyright (c) 2008 Princeton University
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

from m5.objects.BasicLink import (
    BasicExtLink,
    BasicIntLink,
)
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class BetaarCDCType(Enum):
    vals = ["LINK_OBJECT", "OBJECT_LINK"]


class BetaarNetworkLink(ClockedObject):
    type = "BetaarNetworkLink"
    cxx_header = "mem/ruby/network/betaar/BetaarNetworkLink.hh"
    cxx_class = "gem5::ruby::betaar::BetaarNetworkLink"

    link_id = Param.Int(Parent.link_id, "link id")
    link_latency = Param.Cycles(Parent.latency, "link latency")
    vcs_per_vnet = Param.Int(
        Parent.vcs_per_vnet, "virtual channels per virtual network"
    )
    virt_nets = Param.Int(
        Parent.number_of_virtual_networks, "number of virtual networks"
    )
    supported_vnets = VectorParam.Int(
        Parent.supported_vnets, "Vnets supported"
    )
    width = Param.UInt32(Parent.width, "bit-width of the link")


class BetaarCreditLink(BetaarNetworkLink):
    type = "BetaarCreditLink"
    cxx_header = "mem/ruby/network/betaar/BetaarCreditLink.hh"
    cxx_class = "gem5::ruby::betaar::BetaarCreditLink"


class BetaarNetworkBridge(BetaarCreditLink):
    type = "BetaarNetworkBridge"
    cxx_header = "mem/ruby/network/betaar/BetaarNetworkBridge.hh"
    cxx_class = "gem5::ruby::betaar::BetaarNetworkBridge"

    link = Param.BetaarNetworkLink("Associated Network Link")
    vtype = Param.BetaarCDCType(
        "LINK_OBJECT", "Direction of CDC LINK->OBJECT or OBJECT->LINK"
    )
    serdes_latency = Param.Cycles(1, "Latency of SerDes Unit")
    cdc_latency = Param.Cycles(1, "Latency of CDC Unit")


# Interior fixed pipeline links between routers
class BetaarIntLink(BasicIntLink):
    type = "BetaarIntLink"
    cxx_header = "mem/ruby/network/betaar/BetaarLink.hh"
    cxx_class = "gem5::ruby::betaar::BetaarIntLink"

    # The internal link includes one forward link (for flit)
    # and one backward flow-control link (for credit)
    network_link = Param.BetaarNetworkLink(BetaarNetworkLink(), "forward link")
    credit_link = Param.BetaarCreditLink(
        BetaarCreditLink(), "backward flow-control link"
    )

    # The src_cdc and dst_cdc flags are used to enable the
    # clock domain crossing(CDC) at the source and destination
    # end of the link respectively. This is required when the
    # link and the objected connected to the link are operating
    # at different clock domains. These flags should be set
    # in the network topology files.
    src_cdc = Param.Bool(False, "Enable Clock Domain Crossing")
    dst_cdc = Param.Bool(False, "Enable Clock Domain Crossing")

    # The src_serdes and dst_serdes flags are used to enable
    # the Serializer-Deserializer units at the source and
    # destination end of the link respectively. Enabling
    # these flags is necessary when the connecting object
    # supports a different flit width.
    src_serdes = Param.Bool(False, "Enable Serializer-Deserializer")
    dst_serdes = Param.Bool(False, "Enable Serializer-Deserializer")

    # The network bridge encapsulates both the CDC and Ser-Des
    # units in HeteroBetaar. This is automatically enabled when
    # either CDC or Ser-Des is enabled.
    src_net_bridge = Param.BetaarNetworkBridge(
        NULL, "Network Bridge at source"
    )
    dst_net_bridge = Param.BetaarNetworkBridge(NULL, "Network Bridge at dest")
    src_cred_bridge = Param.BetaarNetworkBridge(
        NULL, "Credit Bridge at source"
    )
    dst_cred_bridge = Param.BetaarNetworkBridge(NULL, "Credit Bridge at dest")

    width = Param.UInt32(
        Parent.ni_flit_size, "bit width supported by the router"
    )


# Exterior fixed pipeline links between a router and a controller
class BetaarExtLink(BasicExtLink):
    type = "BetaarExtLink"
    cxx_header = "mem/ruby/network/betaar/BetaarLink.hh"
    cxx_class = "gem5::ruby::betaar::BetaarExtLink"

    # The external link is bi-directional.
    # It includes two forward links (for flits)
    # and two backward flow-control links (for credits),
    # one per direction
    _nls = []
    # In uni-directional link
    _nls.append(BetaarNetworkLink())
    # Out uni-directional link
    _nls.append(BetaarNetworkLink())
    network_links = VectorParam.BetaarNetworkLink(_nls, "forward links")

    _cls = []
    # In uni-directional link
    _cls.append(BetaarCreditLink())
    # Out uni-directional link
    _cls.append(BetaarCreditLink())
    credit_links = VectorParam.BetaarCreditLink(
        _cls, "backward flow-control links"
    )

    # The ext_cdc and intt_cdc flags are used to enable the
    # clock domain crossing(CDC) at the external and internal
    # end of the link respectively. This is required when the
    # link and the objected connected to the link are operating
    # at different clock domains. These flags should be set
    # in the network topology files.
    ext_cdc = Param.Bool(False, "Enable Clock Domain Crossing")
    int_cdc = Param.Bool(False, "Enable Clock Domain Crossing")

    # The ext_serdes and int_serdes flags are used to enable
    # the Serializer-Deserializer units at the external and
    # internal end of the link respectively. Enabling
    # these flags is necessary when the connecting object
    # supports a different flit width.
    ext_serdes = Param.Bool(False, "Enable Serializer-Deserializer")
    int_serdes = Param.Bool(False, "Enable Serializer-Deserializer")

    # The network bridge encapsulates both the CDC and Ser-Des
    # units in HeteroBetaar. This is automatically enabled when
    # either CDC or Ser-Des is enabled.
    ext_net_bridge = VectorParam.BetaarNetworkBridge(
        [], "Network Bridge at external end"
    )
    ext_cred_bridge = VectorParam.BetaarNetworkBridge(
        [], "Credit Bridge at external end"
    )
    int_net_bridge = VectorParam.BetaarNetworkBridge(
        [], "Network Bridge at internal end"
    )
    int_cred_bridge = VectorParam.BetaarNetworkBridge(
        [], "Credit Bridge at internal end"
    )

    width = Param.UInt32(
        Parent.ni_flit_size, "bit width supported by the router"
    )
