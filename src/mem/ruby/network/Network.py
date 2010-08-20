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
from m5.SimObject import SimObject

class Link(SimObject):
    type = 'Link'
    latency = Param.Int(1, "")
    bw_multiplier = Param.Int("")
    weight = Param.Int(1, "")

class ExtLink(Link):
    type = 'ExtLink'
    ext_node = Param.RubyController("External node")
    int_node = Param.Int("ID of internal node")
    bw_multiplier = 64

class IntLink(Link):
    type = 'IntLink'
    node_a = Param.Int("ID of internal node on one end")
    node_b = Param.Int("ID of internal node on other end")
    bw_multiplier = 16

class Topology(SimObject):
    type = 'Topology'
    description = Param.String("Not Specified",
                               "the name of the imported topology module")
    ext_links = VectorParam.ExtLink("Links to external nodes")
    int_links = VectorParam.IntLink("Links between internal nodes")
    num_int_nodes = Param.Int("Nunber of internal nodes")
    print_config = Param.Bool(False,
        "display topology config in the stats file")

class RubyNetwork(SimObject):
    type = 'RubyNetwork'
    cxx_class = 'Network'
    abstract = True
    number_of_virtual_networks = Param.Int(10, "");
    topology = Param.Topology("");
    buffer_size = Param.Int(0,
        "default buffer size; 0 indicates infinite buffering");
    endpoint_bandwidth = Param.Int(10000, "");
    adaptive_routing = Param.Bool(False, "enable adaptive routing");
    link_latency = Param.Int(1,
        "local memory latency ?? NetworkLinkLatency");
    control_msg_size = Param.Int(8, "");
