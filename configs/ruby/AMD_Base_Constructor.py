# Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath, convert
from .CntrlBase import *

addToPath("../")

from topologies.Cluster import Cluster


#
# Note: the L1 Cache latency is only used by the sequencer on fast path hits
#
class L1Cache(RubyCache):
    latency = 1
    resourceStalls = False

    def create(self, size, assoc, options):
        self.size = MemorySize(size)
        self.assoc = assoc
        self.replacement_policy = TreePLRURP()


#
# Note: the L2 Cache latency is not currently used
#
class L2Cache(RubyCache):
    latency = 10
    resourceStalls = False

    def create(self, size, assoc, options):
        self.size = MemorySize(size)
        self.assoc = assoc
        self.replacement_policy = TreePLRURP()


class CPCntrl(AMD_Base_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.cntrl_id = self.cntrlCount()

        self.L1Icache = L1Cache()
        self.L1Icache.create(options.l1i_size, options.l1i_assoc, options)
        self.L1D0cache = L1Cache()
        self.L1D0cache.create(options.l1d_size, options.l1d_assoc, options)
        self.L1D1cache = L1Cache()
        self.L1D1cache.create(options.l1d_size, options.l1d_assoc, options)
        self.L2cache = L2Cache()
        self.L2cache.create(options.l2_size, options.l2_assoc, options)

        self.sequencer = RubySequencer()
        self.sequencer.version = self.seqCount()
        self.sequencer.dcache = self.L1D0cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.coreid = 0
        self.sequencer.is_cpu_sequencer = True

        self.sequencer1 = RubySequencer()
        self.sequencer1.version = self.seqCount()
        self.sequencer1.dcache = self.L1D1cache
        self.sequencer1.ruby_system = ruby_system
        self.sequencer1.coreid = 1
        self.sequencer1.is_cpu_sequencer = True

        self.issue_latency = options.cpu_to_dir_latency
        self.send_evictions = send_evicts(options)

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency


def define_options(parser):
    parser.add_argument("--cpu-to-dir-latency", type=int, default=15)


def construct(options, system, ruby_system):
    if buildEnv["PROTOCOL"] != "GPU_VIPER":
        panic(
            "This script requires VIPER based protocols \
        to be built."
        )
    cpu_sequencers = []
    cpuCluster = None
    cpuCluster = Cluster(name="CPU Cluster", extBW=8, intBW=8)  # 16 GB/s
    for i in range((options.num_cpus + 1) // 2):
        cp_cntrl = CPCntrl()
        cp_cntrl.create(options, ruby_system, system)

        # Connect the CP controllers to the ruby network
        cp_cntrl.requestFromCore = ruby_system.network.in_port
        cp_cntrl.responseFromCore = ruby_system.network.in_port
        cp_cntrl.unblockFromCore = ruby_system.network.in_port
        cp_cntrl.probeToCore = ruby_system.network.out_port
        cp_cntrl.responseToCore = ruby_system.network.out_port

        exec("system.cp_cntrl%d = cp_cntrl" % i)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.extend([cp_cntrl.sequencer, cp_cntrl.sequencer1])
        cpuCluster.add(cp_cntrl)
    return cpu_sequencers, cpuCluster
