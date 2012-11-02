# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
# Authors: Nathan Binkert

from MemObject import MemObject
from m5.params import *
from m5.proxy import *

class MemTest(MemObject):
    type = 'MemTest'
    cxx_header = "cpu/testers/memtest/memtest.hh"
    max_loads = Param.Counter(0, "number of loads to execute")
    atomic = Param.Bool(False, "Execute tester in atomic mode? (or timing)\n")
    memory_size = Param.Int(65536, "memory size")
    percent_dest_unaligned = Param.Percent(50,
        "percent of copy dest address that are unaligned")
    percent_reads = Param.Percent(65, "target read percentage")
    issue_dmas = Param.Bool(False, "this memtester should issue dma requests")
    percent_source_unaligned = Param.Percent(50,
        "percent of copy source address that are unaligned")
    percent_functional = Param.Percent(50, "percent of access that are functional")
    percent_uncacheable = Param.Percent(10,
        "target uncacheable percentage")
    progress_interval = Param.Counter(1000000,
        "progress report interval (in accesses)")
    trace_addr = Param.Addr(0, "address to trace")

    test = MasterPort("Port to the memory system to test")
    functional = MasterPort("Port to the functional memory " \
                                "used for verification")
    suppress_func_warnings = Param.Bool(False,
        "suppress warnings when functional accesses fail.\n")
    sys = Param.System(Parent.any, "System Parameter")

