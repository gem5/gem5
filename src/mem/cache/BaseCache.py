# Copyright (c) 2012-2013 ARM Limited
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

from m5.params import *
from m5.proxy import *
from MemObject import MemObject
from Prefetcher import BasePrefetcher
from Tags import *

class BaseCache(MemObject):
    type = 'BaseCache'
    cxx_header = "mem/cache/base.hh"
    assoc = Param.Int("associativity")
    hit_latency = Param.Cycles("The hit latency for this cache")
    response_latency = Param.Cycles(
            "Additional cache latency for the return path to core on a miss");
    max_miss_count = Param.Counter(0,
        "number of misses to handle before calling exit")
    mshrs = Param.Int("number of MSHRs (max outstanding requests)")
    size = Param.MemorySize("capacity in bytes")
    forward_snoops = Param.Bool(True,
        "forward snoops from mem side to cpu side")
    is_top_level = Param.Bool(False, "Is this cache at the top level (e.g. L1)")
    tgts_per_mshr = Param.Int("max number of accesses per MSHR")
    two_queue = Param.Bool(False,
        "whether the lifo should have two queue replacement")
    write_buffers = Param.Int(8, "number of write buffers")
    prefetch_on_access = Param.Bool(False,
         "notify the hardware prefetcher on every access (not just misses)")
    prefetcher = Param.BasePrefetcher(NULL,"Prefetcher attached to cache")
    cpu_side = SlavePort("Port on side closer to CPU")
    mem_side = MasterPort("Port on side closer to MEM")
    addr_ranges = VectorParam.AddrRange([AllMemory], "The address range for the CPU-side port")
    system = Param.System(Parent.any, "System we belong to")
    sequential_access = Param.Bool(False,
        "Whether to access tags and data sequentially")
    tags = Param.BaseTags(LRU(), "Tag Store for LRU caches")
