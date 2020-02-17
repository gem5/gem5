# Copyright (c) 2017,2019 ARM Limited
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

from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject

class RubyController(ClockedObject):
    type = 'RubyController'
    cxx_class = 'AbstractController'
    cxx_header = "mem/ruby/slicc_interface/AbstractController.hh"
    abstract = True
    version = Param.Int("")
    addr_ranges = VectorParam.AddrRange([AllMemory], "Address range this "
                                        "controller responds to")
    cluster_id = Param.UInt32(0, "Id of this controller's cluster")

    transitions_per_cycle = \
        Param.Int(32, "no. of  SLICC state machine transitions per cycle")
    buffer_size = Param.UInt32(0, "max buffer size 0 means infinite")

    recycle_latency = Param.Cycles(10, "")
    number_of_TBEs = Param.Int(256, "")
    ruby_system = Param.RubySystem("")

    # This is typically a proxy to the icache/dcache hit latency.
    # If the latency depends on the request type or protocol-specific states,
    # the protocol may ignore this parameter by overriding the
    # mandatoryQueueLatency function
    mandatory_queue_latency = \
        Param.Cycles(1, "Default latency for requests added to the " \
                        "mandatory queue on top-level controllers")

    memory = MasterPort("Port for attaching a memory controller")
    system = Param.System(Parent.any, "system object parameter")
