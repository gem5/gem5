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


class BaseCache(MemObject):
    type = 'BaseCache'
    assoc = Param.Int("associativity")
    block_size = Param.Int("block size in bytes")
    latency = Param.Latency("Latency")
    hash_delay = Param.Int(1, "time in cycles of hash access")
    max_miss_count = Param.Counter(0,
        "number of misses to handle before calling exit")
    mshrs = Param.Int("number of MSHRs (max outstanding requests)")
    prioritizeRequests = Param.Bool(False,
        "always service demand misses first")
    repl = Param.Repl(NULL, "replacement policy")
    size = Param.MemorySize("capacity in bytes")
    forward_snoops = Param.Bool(True,
        "forward snoops from mem side to cpu side")
    is_top_level = Param.Bool(False, "Is this cache at the top level (e.g. L1)")
    subblock_size = Param.Int(0,
        "Size of subblock in IIC used for compression")
    tgts_per_mshr = Param.Int("max number of accesses per MSHR")
    trace_addr = Param.Addr(0, "address to trace")
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
