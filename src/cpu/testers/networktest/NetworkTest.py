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
# Authors: Tushar Krishna

from MemObject import MemObject
from m5.params import *
from m5.proxy import *

class NetworkTest(MemObject):
    type = 'NetworkTest'
    cxx_header = "cpu/testers/networktest/networktest.hh"
    block_offset = Param.Int(6, "block offset in bits")
    num_memories = Param.Int(1, "Num Memories")
    memory_size = Param.Int(65536, "memory size")
    sim_cycles = Param.Int(1000, "Number of simulation cycles")
    fixed_pkts = Param.Bool(False, "Send fixed number of packets")
    max_packets = Param.Counter(0, "Number of packets to send when in fixed_pkts mode")
    traffic_type = Param.Counter(0, "Traffic type: uniform random, tornado, bit complement")
    inj_rate = Param.Float(0.1, "Packet injection rate")
    precision = Param.Int(3, "Number of digits of precision after decimal point")
    test = MasterPort("Port to the memory system to test")
    system = Param.System(Parent.any, "System we belong to")
