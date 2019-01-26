# Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
# All rights reserved.
#
# For use for simulation and test purposes only
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
#
# Authors: Lisa Hsu

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
from Ruby import create_topology
from Ruby import send_evicts

addToPath('../')

from topologies.Cluster import Cluster
from topologies.Crossbar import Crossbar

class CntrlBase:
    _seqs = 0
    @classmethod
    def seqCount(cls):
        # Use SeqCount not class since we need global count
        CntrlBase._seqs += 1
        return CntrlBase._seqs - 1

    _cntrls = 0
    @classmethod
    def cntrlCount(cls):
        # Use CntlCount not class since we need global count
        CntrlBase._cntrls += 1
        return CntrlBase._cntrls - 1

    _version = 0
    @classmethod
    def versionCount(cls):
        cls._version += 1 # Use count for this particular type
        return cls._version - 1

class TccDirCache(RubyCache):
    size = "512kB"
    assoc = 16
    resourceStalls = False
    def create(self, options):
        self.size = MemorySize(options.tcc_size)
        self.size.value += (options.num_compute_units *
                            (MemorySize(options.tcp_size).value) *
                            options.tcc_dir_factor) / long(options.num_tccs)
        self.start_index_bit = math.log(options.cacheline_size, 2) + \
                               math.log(options.num_tccs, 2)
        self.replacement_policy = PseudoLRUReplacementPolicy()

class L1DCache(RubyCache):
    resourceStalls = False
    def create(self, options):
        self.size = MemorySize(options.l1d_size)
        self.assoc = options.l1d_assoc
        self.replacement_policy = PseudoLRUReplacementPolicy()

class L1ICache(RubyCache):
    resourceStalls = False
    def create(self, options):
        self.size = MemorySize(options.l1i_size)
        self.assoc = options.l1i_assoc
        self.replacement_policy = PseudoLRUReplacementPolicy()

class L2Cache(RubyCache):
    resourceStalls = False
    def create(self, options):
        self.size = MemorySize(options.l2_size)
        self.assoc = options.l2_assoc
        self.replacement_policy = PseudoLRUReplacementPolicy()


class CPCntrl(CorePair_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1Icache = L1ICache()
        self.L1Icache.create(options)
        self.L1D0cache = L1DCache()
        self.L1D0cache.create(options)
        self.L1D1cache = L1DCache()
        self.L1D1cache.create(options)
        self.L2cache = L2Cache()
        self.L2cache.create(options)

        self.sequencer = RubySequencer()
        self.sequencer.icache_hit_latency = 2
        self.sequencer.dcache_hit_latency = 2
        self.sequencer.version = self.seqCount()
        self.sequencer.icache = self.L1Icache
        self.sequencer.dcache = self.L1D0cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.coreid = 0
        self.sequencer.is_cpu_sequencer = True

        self.sequencer1 = RubySequencer()
        self.sequencer1.version = self.seqCount()
        self.sequencer1.icache = self.L1Icache
        self.sequencer1.dcache = self.L1D1cache
        self.sequencer1.icache_hit_latency = 2
        self.sequencer1.dcache_hit_latency = 2
        self.sequencer1.ruby_system = ruby_system
        self.sequencer1.coreid = 1
        self.sequencer1.is_cpu_sequencer = True

        self.issue_latency = options.cpu_to_dir_latency
        self.send_evictions = send_evicts(options)

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

class TCPCache(RubyCache):
    assoc = 8
    dataArrayBanks = 16
    tagArrayBanks = 4
    dataAccessLatency = 4
    tagAccessLatency = 1
    def create(self, options):
        self.size = MemorySize(options.tcp_size)
        self.replacement_policy = PseudoLRUReplacementPolicy()

class TCPCntrl(TCP_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = TCPCache(tagAccessLatency = options.TCP_latency)
        self.L1cache.resourceStalls = options.no_resource_stalls
        self.L1cache.create(options)

        self.coalescer = RubyGPUCoalescer()
        self.coalescer.version = self.seqCount()
        self.coalescer.icache = self.L1cache
        self.coalescer.dcache = self.L1cache
        self.coalescer.ruby_system = ruby_system
        self.coalescer.support_inst_reqs = False
        self.coalescer.is_cpu_sequencer = False
        self.coalescer.max_outstanding_requests = options.simds_per_cu * \
                                                  options.wfs_per_simd * \
                                                  options.wf_size

        self.sequencer = RubySequencer()
        self.sequencer.version = self.seqCount()
        self.sequencer.icache = self.L1cache
        self.sequencer.dcache = self.L1cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.is_cpu_sequencer = True

        self.use_seq_not_coal = False

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def createCP(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = TCPCache(tagAccessLatency = options.TCP_latency)
        self.L1cache.resourceStalls = options.no_resource_stalls
        self.L1cache.create(options)

        self.coalescer = RubyGPUCoalescer()
        self.coalescer.version = self.seqCount()
        self.coalescer.icache = self.L1cache
        self.coalescer.dcache = self.L1cache
        self.coalescer.ruby_system = ruby_system
        self.coalescer.support_inst_reqs = False
        self.coalescer.is_cpu_sequencer = False

        self.sequencer = RubySequencer()
        self.sequencer.version = self.seqCount()
        self.sequencer.icache = self.L1cache
        self.sequencer.dcache = self.L1cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.is_cpu_sequencer = True

        self.use_seq_not_coal = True

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

class SQCCache(RubyCache):
    size = "32kB"
    assoc = 8
    dataArrayBanks = 16
    tagArrayBanks = 4
    dataAccessLatency = 4
    tagAccessLatency = 1
    def create(self, options):
        self.replacement_policy = PseudoLRUReplacementPolicy()

class SQCCntrl(SQC_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = SQCCache()
        self.L1cache.create(options)
        self.L1cache.resourceStalls = options.no_resource_stalls

        self.sequencer = RubySequencer()

        self.sequencer.version = self.seqCount()
        self.sequencer.icache = self.L1cache
        self.sequencer.dcache = self.L1cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.support_data_reqs = False
        self.sequencer.is_cpu_sequencer = False

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def createCP(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = SQCCache()
        self.L1cache.create(options)
        self.L1cache.resourceStalls = options.no_resource_stalls

        self.sequencer = RubySequencer()

        self.sequencer.version = self.seqCount()
        self.sequencer.icache = self.L1cache
        self.sequencer.dcache = self.L1cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.support_data_reqs = False

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency


class TCC(RubyCache):
    assoc = 16
    dataAccessLatency = 8
    tagAccessLatency = 2
    resourceStalls = True
    def create(self, options):
        self.size = MemorySize(options.tcc_size)
        self.size = self.size / options.num_tccs
        self.dataArrayBanks = 256 / options.num_tccs #number of data banks
        self.tagArrayBanks = 256 / options.num_tccs #number of tag banks
        if ((self.size.value / long(self.assoc)) < 128):
            self.size.value = long(128 * self.assoc)
        self.start_index_bit = math.log(options.cacheline_size, 2) + \
                               math.log(options.num_tccs, 2)
        self.replacement_policy = PseudoLRUReplacementPolicy()

class TCCCntrl(TCC_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L2cache = TCC()
        self.L2cache.create(options)
        self.l2_response_latency = options.TCC_latency

        self.number_of_TBEs = 2048

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def connectWireBuffers(self, req_to_tccdir, resp_to_tccdir,
                           tcc_unblock_to_tccdir, req_to_tcc,
                           probe_to_tcc, resp_to_tcc):
        self.w_reqToTCCDir = req_to_tccdir
        self.w_respToTCCDir = resp_to_tccdir
        self.w_TCCUnblockToTCCDir = tcc_unblock_to_tccdir
        self.w_reqToTCC = req_to_tcc
        self.w_probeToTCC = probe_to_tcc
        self.w_respToTCC = resp_to_tcc

class TCCDirCntrl(TCCdir_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.directory = TccDirCache()
        self.directory.create(options)

        self.number_of_TBEs = 1024

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def connectWireBuffers(self, req_to_tccdir, resp_to_tccdir,
                           tcc_unblock_to_tccdir, req_to_tcc,
                           probe_to_tcc, resp_to_tcc):
        self.w_reqToTCCDir = req_to_tccdir
        self.w_respToTCCDir = resp_to_tccdir
        self.w_TCCUnblockToTCCDir = tcc_unblock_to_tccdir
        self.w_reqToTCC = req_to_tcc
        self.w_probeToTCC = probe_to_tcc
        self.w_respToTCC = resp_to_tcc

class L3Cache(RubyCache):
    assoc = 8
    dataArrayBanks = 256
    tagArrayBanks = 256

    def create(self, options, ruby_system, system):
        self.size = MemorySize(options.l3_size)
        self.size.value /= options.num_dirs
        self.dataArrayBanks /= options.num_dirs
        self.tagArrayBanks /= options.num_dirs
        self.dataArrayBanks /= options.num_dirs
        self.tagArrayBanks /= options.num_dirs
        self.dataAccessLatency = options.l3_data_latency
        self.tagAccessLatency = options.l3_tag_latency
        self.resourceStalls = options.no_resource_stalls
        self.replacement_policy = PseudoLRUReplacementPolicy()

class L3Cntrl(L3Cache_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L3cache = L3Cache()
        self.L3cache.create(options, ruby_system, system)

        self.l3_response_latency = max(self.L3cache.dataAccessLatency,
                                       self.L3cache.tagAccessLatency)
        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def connectWireBuffers(self, req_to_dir, resp_to_dir, l3_unblock_to_dir,
                           req_to_l3, probe_to_l3, resp_to_l3):
        self.reqToDir = req_to_dir
        self.respToDir = resp_to_dir
        self.l3UnblockToDir = l3_unblock_to_dir
        self.reqToL3 = req_to_l3
        self.probeToL3 = probe_to_l3
        self.respToL3 = resp_to_l3

class DirCntrl(Directory_Controller, CntrlBase):
    def create(self, options, dir_ranges, ruby_system, system):
        self.version = self.versionCount()

        self.response_latency = 30

        self.addr_ranges = dir_ranges
        self.directory = RubyDirectoryMemory()

        self.L3CacheMemory = L3Cache()
        self.L3CacheMemory.create(options, ruby_system, system)

        self.l3_hit_latency = max(self.L3CacheMemory.dataAccessLatency,
                                  self.L3CacheMemory.tagAccessLatency)

        self.number_of_TBEs = options.num_tbes

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def connectWireBuffers(self, req_to_dir, resp_to_dir, l3_unblock_to_dir,
                           req_to_l3, probe_to_l3, resp_to_l3):
        self.reqToDir = req_to_dir
        self.respToDir = resp_to_dir
        self.l3UnblockToDir = l3_unblock_to_dir
        self.reqToL3 = req_to_l3
        self.probeToL3 = probe_to_l3
        self.respToL3 = resp_to_l3



def define_options(parser):
    parser.add_option("--num-subcaches", type="int", default=4)
    parser.add_option("--l3-data-latency", type="int", default=20)
    parser.add_option("--l3-tag-latency", type="int", default=15)
    parser.add_option("--cpu-to-dir-latency", type="int", default=15)
    parser.add_option("--gpu-to-dir-latency", type="int", default=160)
    parser.add_option("--no-resource-stalls", action="store_false",
                      default=True)
    parser.add_option("--num-tbes", type="int", default=256)
    parser.add_option("--l2-latency", type="int", default=50) # load to use
    parser.add_option("--num-tccs", type="int", default=1,
                      help="number of TCC directories and banks in the GPU")
    parser.add_option("--TCP_latency", type="int", default=4,
                      help="TCP latency")
    parser.add_option("--TCC_latency", type="int", default=16,
                      help="TCC latency")
    parser.add_option("--tcc-size", type='string', default='256kB',
                      help="agregate tcc size")
    parser.add_option("--tcp-size", type='string', default='16kB',
                      help="tcp size")
    parser.add_option("--tcc-dir-factor", type='int', default=4,
                      help="TCCdir size = factor *(TCPs + TCC)")

def create_system(options, full_system, system, dma_devices, bootmem,
                  ruby_system):
    if buildEnv['PROTOCOL'] != 'GPU_RfO':
        panic("This script requires the GPU_RfO protocol to be built.")

    cpu_sequencers = []

    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.  Therefore the l1 controller nodes
    # must be listed before the directory nodes and directory nodes before
    # dma nodes, etc.
    #
    cp_cntrl_nodes = []
    tcp_cntrl_nodes = []
    sqc_cntrl_nodes = []
    tcc_cntrl_nodes = []
    tccdir_cntrl_nodes = []
    dir_cntrl_nodes = []
    l3_cntrl_nodes = []

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #

    TCC_bits = int(math.log(options.num_tccs, 2))

    # This is the base crossbar that connects the L3s, Dirs, and cpu/gpu
    # Clusters
    mainCluster = Cluster(extBW = 512, intBW = 512) # 1 TB/s

    if options.numa_high_bit:
        numa_bit = options.numa_high_bit
    else:
        # if the numa_bit is not specified, set the directory bits as the
        # lowest bits above the block offset bits, and the numa_bit as the
        # highest of those directory bits
        dir_bits = int(math.log(options.num_dirs, 2))
        block_size_bits = int(math.log(options.cacheline_size, 2))
        numa_bit = block_size_bits + dir_bits - 1

    for i in range(options.num_dirs):
        dir_ranges = []
        for r in system.mem_ranges:
            addr_range = m5.objects.AddrRange(r.start, size = r.size(),
                                              intlvHighBit = numa_bit,
                                              intlvBits = dir_bits,
                                              intlvMatch = i)
            dir_ranges.append(addr_range)

        dir_cntrl = DirCntrl(TCC_select_num_bits = TCC_bits)
        dir_cntrl.create(options, dir_ranges, ruby_system, system)
        dir_cntrl.number_of_TBEs = 2560 * options.num_compute_units
        #Enough TBEs for all TCP TBEs

        # Connect the Directory controller to the ruby network
        dir_cntrl.requestFromCores = MessageBuffer(ordered = True)
        dir_cntrl.requestFromCores.slave = ruby_system.network.master

        dir_cntrl.responseFromCores = MessageBuffer()
        dir_cntrl.responseFromCores.slave = ruby_system.network.master

        dir_cntrl.unblockFromCores = MessageBuffer()
        dir_cntrl.unblockFromCores.slave = ruby_system.network.master

        dir_cntrl.probeToCore = MessageBuffer()
        dir_cntrl.probeToCore.master = ruby_system.network.slave

        dir_cntrl.responseToCore = MessageBuffer()
        dir_cntrl.responseToCore.master = ruby_system.network.slave

        dir_cntrl.triggerQueue = MessageBuffer(ordered = True)
        dir_cntrl.L3triggerQueue = MessageBuffer(ordered = True)
        dir_cntrl.responseFromMemory = MessageBuffer()

        exec("system.dir_cntrl%d = dir_cntrl" % i)
        dir_cntrl_nodes.append(dir_cntrl)

        mainCluster.add(dir_cntrl)

    # For an odd number of CPUs, still create the right number of controllers
    cpuCluster = Cluster(extBW = 512, intBW = 512)  # 1 TB/s
    for i in range((options.num_cpus + 1) // 2):

        cp_cntrl = CPCntrl()
        cp_cntrl.create(options, ruby_system, system)

        exec("system.cp_cntrl%d = cp_cntrl" % i)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.extend([cp_cntrl.sequencer, cp_cntrl.sequencer1])

        # Connect the CP controllers and the network
        cp_cntrl.requestFromCore = MessageBuffer()
        cp_cntrl.requestFromCore.master = ruby_system.network.slave

        cp_cntrl.responseFromCore = MessageBuffer()
        cp_cntrl.responseFromCore.master = ruby_system.network.slave

        cp_cntrl.unblockFromCore = MessageBuffer()
        cp_cntrl.unblockFromCore.master = ruby_system.network.slave

        cp_cntrl.probeToCore = MessageBuffer()
        cp_cntrl.probeToCore.slave = ruby_system.network.master

        cp_cntrl.responseToCore = MessageBuffer()
        cp_cntrl.responseToCore.slave = ruby_system.network.master

        cp_cntrl.mandatoryQueue = MessageBuffer()
        cp_cntrl.triggerQueue = MessageBuffer(ordered = True)

        cpuCluster.add(cp_cntrl)

    gpuCluster = Cluster(extBW = 512, intBW = 512)  # 1 TB/s

    for i in range(options.num_compute_units):

        tcp_cntrl = TCPCntrl(TCC_select_num_bits = TCC_bits,
                             number_of_TBEs = 2560) # max outstanding requests
        tcp_cntrl.create(options, ruby_system, system)

        exec("system.tcp_cntrl%d = tcp_cntrl" % i)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(tcp_cntrl.coalescer)
        tcp_cntrl_nodes.append(tcp_cntrl)

        # Connect the TCP controller to the ruby network
        tcp_cntrl.requestFromTCP = MessageBuffer(ordered = True)
        tcp_cntrl.requestFromTCP.master = ruby_system.network.slave

        tcp_cntrl.responseFromTCP = MessageBuffer(ordered = True)
        tcp_cntrl.responseFromTCP.master = ruby_system.network.slave

        tcp_cntrl.unblockFromCore = MessageBuffer(ordered = True)
        tcp_cntrl.unblockFromCore.master = ruby_system.network.slave

        tcp_cntrl.probeToTCP = MessageBuffer(ordered = True)
        tcp_cntrl.probeToTCP.slave = ruby_system.network.master

        tcp_cntrl.responseToTCP = MessageBuffer(ordered = True)
        tcp_cntrl.responseToTCP.slave = ruby_system.network.master

        tcp_cntrl.mandatoryQueue = MessageBuffer()

        gpuCluster.add(tcp_cntrl)

    for i in range(options.num_sqc):

        sqc_cntrl = SQCCntrl(TCC_select_num_bits = TCC_bits)
        sqc_cntrl.create(options, ruby_system, system)

        exec("system.sqc_cntrl%d = sqc_cntrl" % i)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(sqc_cntrl.sequencer)

        # Connect the SQC controller to the ruby network
        sqc_cntrl.requestFromSQC = MessageBuffer(ordered = True)
        sqc_cntrl.requestFromSQC.master = ruby_system.network.slave

        sqc_cntrl.responseFromSQC = MessageBuffer(ordered = True)
        sqc_cntrl.responseFromSQC.master = ruby_system.network.slave

        sqc_cntrl.unblockFromCore = MessageBuffer(ordered = True)
        sqc_cntrl.unblockFromCore.master = ruby_system.network.slave

        sqc_cntrl.probeToSQC = MessageBuffer(ordered = True)
        sqc_cntrl.probeToSQC.slave = ruby_system.network.master

        sqc_cntrl.responseToSQC = MessageBuffer(ordered = True)
        sqc_cntrl.responseToSQC.slave = ruby_system.network.master

        sqc_cntrl.mandatoryQueue = MessageBuffer()

        # SQC also in GPU cluster
        gpuCluster.add(sqc_cntrl)

    for i in range(options.num_cp):

        tcp_cntrl = TCPCntrl(TCC_select_num_bits = TCC_bits,
                             number_of_TBEs = 2560) # max outstanding requests
        tcp_cntrl.createCP(options, ruby_system, system)

        exec("system.tcp_cntrl%d = tcp_cntrl" % (options.num_compute_units + i))
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(tcp_cntrl.sequencer)
        tcp_cntrl_nodes.append(tcp_cntrl)

        # Connect the TCP controller to the ruby network
        tcp_cntrl.requestFromTCP = MessageBuffer(ordered = True)
        tcp_cntrl.requestFromTCP.master = ruby_system.network.slave

        tcp_cntrl.responseFromTCP = MessageBuffer(ordered = True)
        tcp_cntrl.responseFromTCP.master = ruby_system.network.slave

        tcp_cntrl.unblockFromCore = MessageBuffer(ordered = True)
        tcp_cntrl.unblockFromCore.master = ruby_system.network.slave

        tcp_cntrl.probeToTCP = MessageBuffer(ordered = True)
        tcp_cntrl.probeToTCP.slave = ruby_system.network.master

        tcp_cntrl.responseToTCP = MessageBuffer(ordered = True)
        tcp_cntrl.responseToTCP.slave = ruby_system.network.master

        tcp_cntrl.mandatoryQueue = MessageBuffer()

        gpuCluster.add(tcp_cntrl)

        sqc_cntrl = SQCCntrl(TCC_select_num_bits = TCC_bits)
        sqc_cntrl.createCP(options, ruby_system, system)

        exec("system.sqc_cntrl%d = sqc_cntrl" % (options.num_compute_units + i))
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(sqc_cntrl.sequencer)

        # Connect the SQC controller to the ruby network
        sqc_cntrl.requestFromSQC = MessageBuffer(ordered = True)
        sqc_cntrl.requestFromSQC.master = ruby_system.network.slave

        sqc_cntrl.responseFromSQC = MessageBuffer(ordered = True)
        sqc_cntrl.responseFromSQC.master = ruby_system.network.slave

        sqc_cntrl.unblockFromCore = MessageBuffer(ordered = True)
        sqc_cntrl.unblockFromCore.master = ruby_system.network.slave

        sqc_cntrl.probeToSQC = MessageBuffer(ordered = True)
        sqc_cntrl.probeToSQC.slave = ruby_system.network.master

        sqc_cntrl.responseToSQC = MessageBuffer(ordered = True)
        sqc_cntrl.responseToSQC.slave = ruby_system.network.master

        sqc_cntrl.mandatoryQueue = MessageBuffer()

        # SQC also in GPU cluster
        gpuCluster.add(sqc_cntrl)

    for i in range(options.num_tccs):

        tcc_cntrl = TCCCntrl(TCC_select_num_bits = TCC_bits,
                             number_of_TBEs = options.num_compute_units * 2560)
        #Enough TBEs for all TCP TBEs
        tcc_cntrl.create(options, ruby_system, system)
        tcc_cntrl_nodes.append(tcc_cntrl)

        tccdir_cntrl = TCCDirCntrl(TCC_select_num_bits = TCC_bits,
                              number_of_TBEs = options.num_compute_units * 2560)
        #Enough TBEs for all TCP TBEs
        tccdir_cntrl.create(options, ruby_system, system)
        tccdir_cntrl_nodes.append(tccdir_cntrl)

        exec("system.tcc_cntrl%d = tcc_cntrl" % i)
        exec("system.tccdir_cntrl%d = tccdir_cntrl" % i)

        # connect all of the wire buffers between L3 and dirs up
        req_to_tccdir = RubyWireBuffer()
        resp_to_tccdir = RubyWireBuffer()
        tcc_unblock_to_tccdir = RubyWireBuffer()
        req_to_tcc = RubyWireBuffer()
        probe_to_tcc = RubyWireBuffer()
        resp_to_tcc = RubyWireBuffer()

        tcc_cntrl.connectWireBuffers(req_to_tccdir, resp_to_tccdir,
                                     tcc_unblock_to_tccdir, req_to_tcc,
                                     probe_to_tcc, resp_to_tcc)
        tccdir_cntrl.connectWireBuffers(req_to_tccdir, resp_to_tccdir,
                                        tcc_unblock_to_tccdir, req_to_tcc,
                                        probe_to_tcc, resp_to_tcc)

        # Connect the TCC controller to the ruby network
        tcc_cntrl.responseFromTCC = MessageBuffer(ordered = True)
        tcc_cntrl.responseFromTCC.master = ruby_system.network.slave

        tcc_cntrl.responseToTCC = MessageBuffer(ordered = True)
        tcc_cntrl.responseToTCC.slave = ruby_system.network.master

        # Connect the TCC Dir controller to the ruby network
        tccdir_cntrl.requestFromTCP = MessageBuffer(ordered = True)
        tccdir_cntrl.requestFromTCP.slave = ruby_system.network.master

        tccdir_cntrl.responseFromTCP = MessageBuffer(ordered = True)
        tccdir_cntrl.responseFromTCP.slave = ruby_system.network.master

        tccdir_cntrl.unblockFromTCP = MessageBuffer(ordered = True)
        tccdir_cntrl.unblockFromTCP.slave = ruby_system.network.master

        tccdir_cntrl.probeToCore = MessageBuffer(ordered = True)
        tccdir_cntrl.probeToCore.master = ruby_system.network.slave

        tccdir_cntrl.responseToCore = MessageBuffer(ordered = True)
        tccdir_cntrl.responseToCore.master = ruby_system.network.slave

        tccdir_cntrl.probeFromNB = MessageBuffer()
        tccdir_cntrl.probeFromNB.slave = ruby_system.network.master

        tccdir_cntrl.responseFromNB = MessageBuffer()
        tccdir_cntrl.responseFromNB.slave = ruby_system.network.master

        tccdir_cntrl.requestToNB = MessageBuffer()
        tccdir_cntrl.requestToNB.master = ruby_system.network.slave

        tccdir_cntrl.responseToNB = MessageBuffer()
        tccdir_cntrl.responseToNB.master = ruby_system.network.slave

        tccdir_cntrl.unblockToNB = MessageBuffer()
        tccdir_cntrl.unblockToNB.master = ruby_system.network.slave

        tccdir_cntrl.triggerQueue = MessageBuffer(ordered = True)

        # TCC cntrls added to the GPU cluster
        gpuCluster.add(tcc_cntrl)
        gpuCluster.add(tccdir_cntrl)

    # Assuming no DMA devices
    assert(len(dma_devices) == 0)

    # Add cpu/gpu clusters to main cluster
    mainCluster.add(cpuCluster)
    mainCluster.add(gpuCluster)

    ruby_system.network.number_of_virtual_networks = 10

    return (cpu_sequencers, dir_cntrl_nodes, mainCluster)
