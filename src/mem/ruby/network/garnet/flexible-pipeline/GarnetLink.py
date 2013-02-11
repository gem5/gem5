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
#
# Authors: Steve Reinhardt
#          Brad Beckmann

from m5.params import *
from m5.proxy import *
from ClockedObject import ClockedObject
from BasicLink import BasicIntLink, BasicExtLink

class NetworkLink(ClockedObject):
    type = 'NetworkLink'
    cxx_header = "mem/ruby/network/garnet/flexible-pipeline/NetworkLink.hh"
    link_id = Param.Int(Parent.link_id, "link id")
    link_latency = Param.Cycles(Parent.latency, "link latency")
    vcs_per_vnet = Param.Int(Parent.vcs_per_vnet,
                              "virtual channels per virtual network")
    virt_nets = Param.Int(Parent.number_of_virtual_networks,
                          "number of virtual networks")
    channel_width = Param.Int(Parent.bandwidth_factor,
                              "channel width == bw factor")

# Interior fixed pipeline links between routers
class GarnetIntLink(BasicIntLink):
    type = 'GarnetIntLink'
    cxx_header = "mem/ruby/network/garnet/flexible-pipeline/GarnetLink.hh"
    # The flexible pipeline bi-directional link only include two main
    # forward links and no backward flow-control links
    nls = []
    # In uni-directional link
    nls.append(NetworkLink()); 
    # Out uni-directional link
    nls.append(NetworkLink());
    network_links = VectorParam.NetworkLink(nls, "forward links")

# Exterior fixed pipeline links between a router and a controller
class GarnetExtLink(BasicExtLink):
    type = 'GarnetExtLink'
    cxx_header = "mem/ruby/network/garnet/flexible-pipeline/GarnetLink.hh"
    # The flexible pipeline bi-directional link only include two main
    # forward links and no backward flow-control links
    nls = []
    # In uni-directional link
    nls.append(NetworkLink());
    # Out uni-directional link
    nls.append(NetworkLink());
    network_links = VectorParam.NetworkLink(nls, "forward links")
