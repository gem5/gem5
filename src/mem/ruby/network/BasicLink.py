# Copyright (c) 2011 Advanced Micro Devices, Inc.
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

class BasicLink(SimObject):
    type = 'BasicLink'
    cxx_header = "mem/ruby/network/BasicLink.hh"
    link_id = Param.Int("ID in relation to other links")
    latency = Param.Cycles(1, "latency")
    # The following banwidth factor does not translate to the same value for
    # both the simple and Garnet models.  For the most part, the bandwidth
    # factor is the width of the link in bytes, expect for certain situations
    # with regard to the simple network.
    bandwidth_factor = Param.Int("generic bandwidth factor, usually in bytes")
    weight = Param.Int(1, "used to restrict routing in shortest path analysis")

class BasicExtLink(BasicLink):
    type = 'BasicExtLink'
    cxx_header = "mem/ruby/network/BasicLink.hh"
    ext_node = Param.RubyController("External node")
    int_node = Param.BasicRouter("ID of internal node")
    bandwidth_factor = 16

class BasicIntLink(BasicLink):
    type = 'BasicIntLink'
    cxx_header = "mem/ruby/network/BasicLink.hh"
    node_a = Param.BasicRouter("Router on one end")
    node_b = Param.BasicRouter("Router on other end")
    bandwidth_factor = 16
