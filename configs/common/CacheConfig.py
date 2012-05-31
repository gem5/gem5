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
# Authors: Lisa Hsu

# Configure the M5 cache hierarchy config in one place
#

import m5
from m5.objects import *
from Caches import *
from O3_ARM_v7a import *

def config_cache(options, system):
    if options.l2cache:
        if options.cpu_type == "arm_detailed":
            system.l2 = O3_ARM_v7aL2(size = options.l2_size, assoc = options.l2_assoc,
                                block_size=options.cacheline_size)
        else:
            system.l2 = L2Cache(size = options.l2_size, assoc = options.l2_assoc,
                                block_size=options.cacheline_size)

        system.tol2bus = CoherentBus()
        system.l2.cpu_side = system.tol2bus.master
        system.l2.mem_side = system.membus.slave

    for i in xrange(options.num_cpus):
        if options.caches:
            if options.cpu_type == "arm_detailed":
                icache = O3_ARM_v7a_ICache(size = options.l1i_size,
                                     assoc = options.l1i_assoc,
                                     block_size=options.cacheline_size)
                dcache = O3_ARM_v7a_DCache(size = options.l1d_size,
                                     assoc = options.l1d_assoc,
                                     block_size=options.cacheline_size)
            else:
                icache = L1Cache(size = options.l1i_size,
                                 assoc = options.l1i_assoc,
                                 block_size=options.cacheline_size)
                dcache = L1Cache(size = options.l1d_size,
                                 assoc = options.l1d_assoc,
                                 block_size=options.cacheline_size)

            if buildEnv['TARGET_ISA'] == 'x86':
                system.cpu[i].addPrivateSplitL1Caches(icache, dcache,
                                                      PageTableWalkerCache(),
                                                      PageTableWalkerCache())
            else:
                system.cpu[i].addPrivateSplitL1Caches(icache, dcache)
        system.cpu[i].createInterruptController()
        if options.l2cache:
            system.cpu[i].connectAllPorts(system.tol2bus, system.membus)
        else:
            system.cpu[i].connectAllPorts(system.membus)

    return system
