# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Brad Beckmann

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath


#
# Note: the L1 Cache latency is only used by the sequencer on fast path hits
#
class L1Cache(RubyCache):
    assoc = 2
    latency = 3
    size = 32768

#
# Note: the L2 Cache latency is not currently used
#
class L2Cache(RubyCache):
    assoc = 16
    latency = 15
    size = 1048576

def create_system(options, physmem):
    
    if buildEnv['PROTOCOL'] != 'MOESI_hammer':
        panic("This script requires the MOESI_hammer protocol to be built.")

    sequencers = []
    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.  Therefore the l1 controller nodes must be
    # listed before the directory nodes and directory nodes before dma nodes, etc.
    #
    l1_cntrl_nodes = []
    dir_cntrl_nodes = []
    dma_cntrl_nodes = []

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #
    for i in range(options.num_cpus):
        #
        # First create the Ruby objects associated with this cpu
        # Eventually this code should go in a python file specific to the
        # MOESI_hammer protocol
        #
        l1i_profiler = CacheProfiler(description = ("l1i_%s_profiler" % i))
        l1i_cache = L1Cache(cache_profiler = l1i_profiler)

        l1d_profiler = CacheProfiler(description = ("l1d_%s_profiler" % i))
        l1d_cache = L1Cache(cache_profiler = l1d_profiler)

        l2_profiler = CacheProfiler(description = ("l2_%s_profiler" % i))
        l2_cache = L2Cache(cache_profiler = l2_profiler)

        cpu_seq = RubySequencer(icache = l1i_cache,
                                dcache = l1d_cache,
                                funcmem_port = physmem.port)

        l1_cntrl = L1Cache_Controller(version = i,
                                      sequencer = cpu_seq,
                                      L1IcacheMemory = l1i_cache,
                                      L1DcacheMemory = l1d_cache,
                                      L2cacheMemory = l2_cache)

        mem_cntrl = RubyMemoryControl(version = i)

        dir_cntrl = Directory_Controller(version = i,
                                         directory = RubyDirectoryMemory(),
                                         memBuffer = mem_cntrl)

        dma_cntrl = DMA_Controller(version = i,
                                   dma_sequencer = DMASequencer())

        #
        # Add controllers and sequencers to the appropriate lists
        # As noted above: Independent list are track to maintain the order of
        # nodes/controllers assumed by the ruby network
        #
        sequencers.append(cpu_seq)
        l1_cntrl_nodes.append(l1_cntrl)
        dir_cntrl_nodes.append(dir_cntrl)
        dma_cntrl_nodes.append(dma_cntrl)

    all_cntrls = l1_cntrl_nodes + dir_cntrl_nodes + dma_cntrl_nodes

    return (sequencers, dir_cntrl_nodes, all_cntrls)
