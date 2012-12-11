# Copyright (c) 2010 Advanced Micro Devices, Inc.
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
# Authors: Brad Beckmann

from m5.SimObject import SimObject
from MemObject import MemObject
from m5.params import *
from m5.proxy import *

class DirectedGenerator(SimObject):
    type = 'DirectedGenerator'
    abstract = True
    cxx_header = "cpu/testers/directedtest/DirectedGenerator.hh"
    num_cpus = Param.Int("num of cpus")
    system = Param.System(Parent.any, "System we belong to")

class SeriesRequestGenerator(DirectedGenerator):
    type = 'SeriesRequestGenerator'
    cxx_header = "cpu/testers/directedtest/SeriesRequestGenerator.hh"
    addr_increment_size = Param.Int(64, "address increment size")
    num_series = Param.UInt32(1,
        "number of different address streams to generate")
    percent_writes = Param.Percent(50, "percent of access that are writes")

class InvalidateGenerator(DirectedGenerator):
    type = 'InvalidateGenerator'
    cxx_header = "cpu/testers/directedtest/InvalidateGenerator.hh"
    addr_increment_size = Param.Int(64, "address increment size")

class RubyDirectedTester(MemObject):
    type = 'RubyDirectedTester'
    cxx_header = "cpu/testers/directedtest/RubyDirectedTester.hh"
    cpuPort = VectorMasterPort("the cpu ports")
    requests_to_complete = Param.Int("checks to complete")
    generator = Param.DirectedGenerator("the request generator")
