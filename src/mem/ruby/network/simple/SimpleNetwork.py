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
from Network import RubyNetwork
from BasicRouter import BasicRouter
from MessageBuffer import MessageBuffer

class SimpleNetwork(RubyNetwork):
    type = 'SimpleNetwork'
    cxx_header = "mem/ruby/network/simple/SimpleNetwork.hh"
    buffer_size = Param.Int(0,
        "default buffer size; 0 indicates infinite buffering");
    endpoint_bandwidth = Param.Int(1000, "bandwidth adjustment factor");
    adaptive_routing = Param.Bool(False, "enable adaptive routing");
    int_link_buffers = VectorParam.MessageBuffer("Buffers for int_links")

    def setup_buffers(self):
        # Note that all SimpleNetwork MessageBuffers are currently ordered
        network_buffers = []
        for link in self.int_links:
            # The network needs number_of_virtual_networks buffers per
            # int_link port
            for i in xrange(self.number_of_virtual_networks):
                network_buffers.append(MessageBuffer(ordered = True))
                network_buffers.append(MessageBuffer(ordered = True))
        self.int_link_buffers = network_buffers

        # Also add buffers for all router-link connections
        for router in self.routers:
            router_buffers = []
            # Add message buffers to routers for each internal link connection
            for link in self.int_links:
                if link.node_a == router:
                    for i in xrange(self.number_of_virtual_networks):
                        router_buffers.append(MessageBuffer(ordered = True))
                if link.node_b == router:
                    for i in xrange(self.number_of_virtual_networks):
                        router_buffers.append(MessageBuffer(ordered = True))

            # Add message buffers to routers for each external link connection
            for link in self.ext_links:
                # Routers can only be int_nodes on ext_links
                if link.int_node in self.routers:
                    for i in xrange(self.number_of_virtual_networks):
                        router_buffers.append(MessageBuffer(ordered = True))
            router.port_buffers = router_buffers

class Switch(BasicRouter):
    type = 'Switch'
    cxx_header = 'mem/ruby/network/simple/Switch.hh'
    virt_nets = Param.Int(Parent.number_of_virtual_networks,
                          "number of virtual networks")
    port_buffers = VectorParam.MessageBuffer("Port buffers")
