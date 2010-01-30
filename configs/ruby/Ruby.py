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

import MOESI_hammer

def create_system(options, physmem, piobus = None, dma_devices = []):

    protocol = buildEnv['PROTOCOL']

    if protocol == "MOESI_hammer":
        (cpu_sequencers, dir_cntrls, all_cntrls) = \
            MOESI_hammer.create_system(options, \
                                       physmem, \
                                       piobus, \
                                       dma_devices)
    else:
         print "Error: unsupported ruby protocol"
         sys.exit(1)

    #
    # Important: the topology constructor must be called before the network
    # constructor.
    #
    network = SimpleNetwork(topology = makeCrossbar(all_cntrls))

    #
    # determine the total memory size of the ruby system and verify it is equal
    # to physmem
    #
    total_mem_size = MemorySize('0B')
    for dir_cntrl in dir_cntrls:
        total_mem_size.value += dir_cntrl.directory.size.value
    physmem_size = long(physmem.range.second) - long(physmem.range.first) + 1
    assert(total_mem_size.value == physmem_size)

    ruby_profiler = RubyProfiler(num_of_sequencers = len(cpu_sequencers))
    
    ruby = RubySystem(clock = options.clock,
                      network = network,
                      profiler = ruby_profiler,
                      tracer = RubyTracer(),
                      debug = RubyDebug(filter_string = 'none',
                                        verbosity_string = 'none',
                                        protocol_trace = False),
                      mem_size = total_mem_size)

    ruby.cpu_ruby_ports = cpu_sequencers

    return ruby
