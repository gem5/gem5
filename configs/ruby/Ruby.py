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

import math
import m5
from m5.objects import *
from m5.defines import buildEnv

def define_options(parser):
    # ruby network options
    parser.add_option("--topology", type="string", default="Crossbar",
                 help="check src/mem/ruby/network/topologies for complete set")
    parser.add_option("--mesh-rows", type="int", default=1,
                      help="the number of rows in the mesh topology")
    parser.add_option("--garnet-network", type="string", default=none,
                      help="'fixed'|'flexible'")

    # ruby mapping options
    parser.add_option("--numa-high-bit", type="int", default=none,
                      help="high order address bit to use for numa mapping. " \
                           "0 = highest bit, not specified = lowest bit")

    # ruby sparse memory options
    parser.add_option("--use-map", action="store_true", default=False)
    parser.add_option("--map-levels", type="int", default=4)

    # ruby debug cmd line options
    parser.add_option("--ruby-debug", action="store_true", default=False)
    parser.add_option("--ruby-debug-cycle", type="int", default=1)

    protocol = buildEnv['PROTOCOL']
    exec "import %s" % protocol
    eval("%s.define_options(parser)" % protocol)

def create_system(options, system, piobus = None, dma_devices = []):

    protocol = buildEnv['PROTOCOL']
    exec "import %s" % protocol
    try:
        (cpu_sequencers, dir_cntrls, all_cntrls) = \
             eval("%s.create_system(options, system, piobus, dma_devices)" \
                  % protocol)
    except:
        print "Error: could not create sytem for ruby protocol %s" % protocol
        raise
        
    #
    # Important: the topology must be created before the network and after the
    # controllers.
    #
    exec "import %s" % options.topology
    try:
        net_topology = eval("%s.makeTopology(all_cntrls, options)" \
                            % options.topology)
    except:
        print "Error: could not create topology %s" % options.topology
        raise
        
    if options.garnet_network == "fixed":
        network = GarnetNetwork_d(topology = net_topology)
    elif options.garnet_network == "flexible":
        network = GarnetNetwork(topology = net_topology)
    else:
        network = SimpleNetwork(topology = net_topology)

    #
    # Loop through the directory controlers.
    # Determine the total memory size of the ruby system and verify it is equal
    # to physmem.  However, if Ruby memory is using sparse memory in SE 
    # mode, then the system should not back-up the memory state with
    # the Memory Vector and thus the memory size bytes should stay at 0.
    # Also set the numa bits to the appropriate values.
    #
    total_mem_size = MemorySize('0B')

    dir_bits = int(math.log(options.num_dirs, 2))

    if options.numa_high_bit:
        numa_bit = options.numa_high_bit
    else:
        # if not specified, use the lowest bits above the block offest
        if options.numa_high_bit == 0:
            if dir_bits > 0:
                # add 5 because bits 0-5 are the block offset
                numa_bit = dir_bits + 5
            else:
                numa_bit = 6
        
    for dir_cntrl in dir_cntrls:
        total_mem_size.value += dir_cntrl.directory.size.value
        dir_cntrl.directory.numa_high_bit = numa_bit
        
    physmem_size = long(system.physmem.range.second) - \
                     long(system.physmem.range.first) + 1
    assert(total_mem_size.value == physmem_size)

    ruby_profiler = RubyProfiler(num_of_sequencers = len(cpu_sequencers))
    
    ruby = RubySystem(clock = options.clock,
                      network = network,
                      profiler = ruby_profiler,
                      tracer = RubyTracer(),
                      debug = RubyDebug(filter_string = 'none',
                                        verbosity_string = 'none',
                                        protocol_trace = options.ruby_debug,
                                        start_time = options.ruby_debug_cycle),
                      mem_size = total_mem_size)

    ruby.cpu_ruby_ports = cpu_sequencers

    return ruby
