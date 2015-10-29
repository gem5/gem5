# Copyright (c) 2012-2013, 2015 ARM Limited
# All rights reserved
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

def config_cache(options, system, domain):
    if options.external_memory_system:
        print "external memory system is not currently supported."
        sys.exit(-1)

    dcache_class, icache_class, l2_cache_class = L1_DCache, L1_ICache, L2Cache

    # if not options.l2cache:
    #     print "l2cache should be present"
    #     sys.exit(-1)

    # Provide a clock for the L2 and the L1-to-L2 bus here as they
    # are not connected using addTwoLevelCacheHierarchy. Use the
    # same clock as the CPUs.
    domain.l2 = l2_cache_class(size=options.l2_size,
                               assoc=options.l2_assoc)

    domain.tol2bus = L2XBar()
    domain.l2.cpu_side = domain.tol2bus.master
    domain.l2.mem_side = domain.membus.slave

    for cpu in domain.cpu:
        icache = icache_class(size=options.l1i_size,
                              assoc=options.l1i_assoc)
        dcache = dcache_class(size=options.l1d_size,
                              assoc=options.l1d_assoc)

        cpu.addPrivateSplitL1Caches(icache, dcache)

        cpu.createInterruptController()
        cpu.connectAllPorts(domain.tol2bus, domain.membus)

    return domain
