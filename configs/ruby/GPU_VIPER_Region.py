#
#  Copyright (c) 2015 Advanced Micro Devices, Inc.
#  All rights reserved.
#
#  For use for simulation and test purposes only
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors
#  may be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Sooraj Puthoor
#

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from Ruby import send_evicts

from Cluster import Cluster

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

#
# Note: the L1 Cache latency is only used by the sequencer on fast path hits
#
class L1Cache(RubyCache):
    resourceStalls = False
    dataArrayBanks = 2
    tagArrayBanks = 2
    dataAccessLatency = 1
    tagAccessLatency = 1
    def create(self, size, assoc, options):
        self.size = MemorySize(size)
        self.assoc = assoc
        self.replacement_policy = PseudoLRUReplacementPolicy()

class L2Cache(RubyCache):
    resourceStalls = False
    assoc = 16
    dataArrayBanks = 16
    tagArrayBanks = 16
    def create(self, size, assoc, options):
        self.size = MemorySize(size)
        self.assoc = assoc
        self.replacement_policy = PseudoLRUReplacementPolicy()

class CPCntrl(CorePair_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

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
        self.sequencer.icache = self.L1Icache
        self.sequencer.dcache = self.L1D0cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.coreid = 0
        self.sequencer.is_cpu_sequencer = True

        self.sequencer1 = RubySequencer()
        self.sequencer1.version = self.seqCount()
        self.sequencer1.icache = self.L1Icache
        self.sequencer1.dcache = self.L1D1cache
        self.sequencer1.ruby_system = ruby_system
        self.sequencer1.coreid = 1
        self.sequencer1.is_cpu_sequencer = True

        self.issue_latency = 1
        self.send_evictions = send_evicts(options)

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

class TCPCache(RubyCache):
    size = "16kB"
    assoc = 16
    dataArrayBanks = 16
    tagArrayBanks = 16
    dataAccessLatency = 4
    tagAccessLatency = 1
    def create(self, options):
        self.size = MemorySize(options.tcp_size)
        self.dataArrayBanks = 16
        self.tagArrayBanks = 16
        self.dataAccessLatency = 4
        self.tagAccessLatency = 1
        self.resourceStalls = options.no_tcc_resource_stalls
        self.replacement_policy = PseudoLRUReplacementPolicy(assoc = self.assoc)

class TCPCntrl(TCP_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L1cache = TCPCache(dataAccessLatency = options.TCP_latency)
        self.L1cache.create(options)
        self.issue_latency = 1

        self.coalescer = VIPERCoalescer()
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

        self.use_seq_not_coal = False

        self.ruby_system = ruby_system
        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

class SQCCache(RubyCache):
    dataArrayBanks = 8
    tagArrayBanks = 8
    dataAccessLatency = 1
    tagAccessLatency = 1

    def create(self, options):
        self.size = MemorySize(options.sqc_size)
        self.assoc = options.sqc_assoc
        self.replacement_policy = PseudoLRUReplacementPolicy(assoc = self.assoc)

class SQCCntrl(SQC_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L1cache = SQCCache()
        self.L1cache.create(options)
        self.L1cache.resourceStalls = False
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

class TCC(RubyCache):
    size = MemorySize("256kB")
    assoc = 16
    dataAccessLatency = 8
    tagAccessLatency = 2
    resourceStalls = False
    def create(self, options):
        self.assoc = options.tcc_assoc
        if hasattr(options, 'bw_scalor') and options.bw_scalor > 0:
          s = options.num_compute_units
          tcc_size = s * 128
          tcc_size = str(tcc_size)+'kB'
          self.size = MemorySize(tcc_size)
          self.dataArrayBanks = 64
          self.tagArrayBanks = 64
        else:
          self.size = MemorySize(options.tcc_size)
          self.dataArrayBanks = 256 / options.num_tccs #number of data banks
          self.tagArrayBanks = 256 / options.num_tccs #number of tag banks
        self.size.value = self.size.value / options.num_tccs
        if ((self.size.value / long(self.assoc)) < 128):
            self.size.value = long(128 * self.assoc)
        self.start_index_bit = math.log(options.cacheline_size, 2) + \
                               math.log(options.num_tccs, 2)
        self.replacement_policy = PseudoLRUReplacementPolicy(assoc = self.assoc)

class TCCCntrl(TCC_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L2cache = TCC()
        self.L2cache.create(options)
        self.ruby_system = ruby_system
        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

class L3Cache(RubyCache):
    dataArrayBanks = 16
    tagArrayBanks = 16

    def create(self, options, ruby_system, system):
        self.size = MemorySize(options.l3_size)
        self.size.value /= options.num_dirs
        self.assoc = options.l3_assoc
        self.dataArrayBanks /= options.num_dirs
        self.tagArrayBanks /= options.num_dirs
        self.dataArrayBanks /= options.num_dirs
        self.tagArrayBanks /= options.num_dirs
        self.dataAccessLatency = options.l3_data_latency
        self.tagAccessLatency = options.l3_tag_latency
        self.resourceStalls = False
        self.replacement_policy = PseudoLRUReplacementPolicy(assoc = self.assoc)

class L3Cntrl(L3Cache_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L3cache = L3Cache()
        self.L3cache.create(options, ruby_system, system)
        self.l3_response_latency = \
            max(self.L3cache.dataAccessLatency, self.L3cache.tagAccessLatency)
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

# Directory memory: Directory memory of infinite size which is
# used by directory controller to store the "states" of the
# state machine. The state machine is implemented per cache block
class DirMem(RubyDirectoryMemory, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        phys_mem_size = AddrRange(options.mem_size).size()
        mem_module_size = phys_mem_size / options.num_dirs
        dir_size = MemorySize('0B')
        dir_size.value = mem_module_size
        self.size = dir_size

# Directory controller: Contains directory memory, L3 cache and associated state
# machine which is used to accurately redirect a data request to L3 cache or to
# memory. The permissions requests do not come to this directory for region
# based protocols as they are handled exclusively by the region directory.
# However, region directory controller uses this directory controller for
# sending probe requests and receiving probe responses.
class DirCntrl(Directory_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.response_latency = 25
        self.response_latency_regionDir = 1
        self.directory = DirMem()
        self.directory.create(options, ruby_system, system)
        self.L3CacheMemory = L3Cache()
        self.L3CacheMemory.create(options, ruby_system, system)
        self.l3_hit_latency = \
            max(self.L3CacheMemory.dataAccessLatency,
            self.L3CacheMemory.tagAccessLatency)

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

# Region directory : Stores region permissions
class RegionDir(RubyCache):

    def create(self, options, ruby_system, system):
        self.block_size = "%dB" % (64 * options.blocks_per_region)
        self.size = options.region_dir_entries * \
            self.block_size * options.num_compute_units
        self.assoc = 8
        self.tagArrayBanks = 8
        self.tagAccessLatency = options.dir_tag_latency
        self.dataAccessLatency = 1
        self.resourceStalls = options.no_resource_stalls
        self.start_index_bit = 6 + int(math.log(options.blocks_per_region, 2))
        self.replacement_policy = PseudoLRUReplacementPolicy(assoc = self.assoc)
# Region directory controller : Contains region directory and associated state
# machine for dealing with region coherence requests.
class RegionCntrl(RegionDir_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.cacheMemory = RegionDir()
        self.cacheMemory.create(options, ruby_system, system)
        self.blocksPerRegion = options.blocks_per_region
        self.toDirLatency = \
            max(self.cacheMemory.dataAccessLatency,
            self.cacheMemory.tagAccessLatency)
        self.ruby_system = ruby_system
        self.always_migrate = options.always_migrate
        self.sym_migrate = options.symmetric_migrate
        self.asym_migrate = options.asymmetric_migrate
        if self.always_migrate:
            assert(not self.asym_migrate and not self.sym_migrate)
        if self.sym_migrate:
            assert(not self.always_migrate and not self.asym_migrate)
        if self.asym_migrate:
            assert(not self.always_migrate and not self.sym_migrate)
        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

# Region Buffer: A region directory cache which avoids some potential
# long latency lookup of region directory for getting region permissions
class RegionBuffer(RubyCache):
    assoc = 4
    dataArrayBanks = 256
    tagArrayBanks = 256
    dataAccessLatency = 1
    tagAccessLatency = 1
    resourceStalls = True

class RBCntrl(RegionBuffer_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.cacheMemory = RegionBuffer()
        self.cacheMemory.resourceStalls = options.no_tcc_resource_stalls
        self.cacheMemory.dataArrayBanks = 64
        self.cacheMemory.tagArrayBanks = 64
        self.blocksPerRegion = options.blocks_per_region
        self.cacheMemory.block_size = "%dB" % (64 * self.blocksPerRegion)
        self.cacheMemory.start_index_bit = \
            6 + int(math.log(self.blocksPerRegion, 2))
        self.cacheMemory.size = options.region_buffer_entries * \
            self.cacheMemory.block_size * options.num_compute_units
        self.toDirLatency = options.gpu_to_dir_latency
        self.toRegionDirLatency = options.cpu_to_dir_latency
        self.noTCCdir = True
        TCC_bits = int(math.log(options.num_tccs, 2))
        self.TCC_select_num_bits = TCC_bits
        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency
        self.cacheMemory.replacement_policy = \
            PseudoLRUReplacementPolicy(assoc = self.cacheMemory.assoc)

def define_options(parser):
    parser.add_option("--num-subcaches", type="int", default=4)
    parser.add_option("--l3-data-latency", type="int", default=20)
    parser.add_option("--l3-tag-latency", type="int", default=15)
    parser.add_option("--cpu-to-dir-latency", type="int", default=120)
    parser.add_option("--gpu-to-dir-latency", type="int", default=60)
    parser.add_option("--no-resource-stalls", action="store_false",
                      default=True)
    parser.add_option("--no-tcc-resource-stalls", action="store_false",
                      default=True)
    parser.add_option("--num-tbes", type="int", default=32)
    parser.add_option("--l2-latency", type="int", default=50) # load to use
    parser.add_option("--num-tccs", type="int", default=1,
                      help="number of TCC banks in the GPU")

    parser.add_option("--sqc-size", type='string', default='32kB',
                      help="SQC cache size")
    parser.add_option("--sqc-assoc", type='int', default=8,
                      help="SQC cache assoc")

    parser.add_option("--WB_L1", action="store_true",
        default=False, help="L2 Writeback Cache")
    parser.add_option("--WB_L2", action="store_true",
        default=False, help="L2 Writeback Cache")
    parser.add_option("--TCP_latency",
        type="int", default=4, help="TCP latency")
    parser.add_option("--TCC_latency",
        type="int", default=16, help="TCC latency")
    parser.add_option("--tcc-size", type='string', default='2MB',
                      help="agregate tcc size")
    parser.add_option("--tcc-assoc", type='int', default=16,
                      help="tcc assoc")
    parser.add_option("--tcp-size", type='string', default='16kB',
                      help="tcp size")

    parser.add_option("--dir-tag-latency", type="int", default=4)
    parser.add_option("--dir-tag-banks", type="int", default=4)
    parser.add_option("--blocks-per-region", type="int", default=16)
    parser.add_option("--dir-entries", type="int", default=8192)

    # Region buffer is a cache of region directory. Hence region
    # directory is inclusive with respect to region directory.
    # However, region directory is non-inclusive with respect to
    # the caches in the system
    parser.add_option("--region-dir-entries", type="int", default=1024)
    parser.add_option("--region-buffer-entries", type="int", default=512)

    parser.add_option("--always-migrate",
        action="store_true", default=False)
    parser.add_option("--symmetric-migrate",
        action="store_true", default=False)
    parser.add_option("--asymmetric-migrate",
        action="store_true", default=False)
    parser.add_option("--use-L3-on-WT", action="store_true", default=False)

def create_system(options, full_system, system, dma_devices, ruby_system):
    if buildEnv['PROTOCOL'] != 'GPU_VIPER_Region':
        panic("This script requires the GPU_VIPER_Region protocol to be built.")

    cpu_sequencers = []

    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.  Therefore the l1 controller nodes
    # must be listed before the directory nodes and directory nodes before
    # dma nodes, etc.
    #
    dir_cntrl_nodes = []

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #

    # For an odd number of CPUs, still create the right number of controllers
    crossbar_bw = 16 * options.num_compute_units #Assuming a 2GHz clock
    cpuCluster = Cluster(extBW = (crossbar_bw), intBW=crossbar_bw)
    for i in xrange((options.num_cpus + 1) / 2):

        cp_cntrl = CPCntrl()
        cp_cntrl.create(options, ruby_system, system)

        rb_cntrl = RBCntrl()
        rb_cntrl.create(options, ruby_system, system)
        rb_cntrl.number_of_TBEs = 256
        rb_cntrl.isOnCPU = True

        cp_cntrl.regionBufferNum = rb_cntrl.version

        exec("system.cp_cntrl%d = cp_cntrl" % i)
        exec("system.rb_cntrl%d = rb_cntrl" % i)
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

        # Connect the RB controllers to the ruby network
        rb_cntrl.requestFromCore = MessageBuffer(ordered = True)
        rb_cntrl.requestFromCore.slave = ruby_system.network.master

        rb_cntrl.responseFromCore = MessageBuffer()
        rb_cntrl.responseFromCore.slave = ruby_system.network.master

        rb_cntrl.requestToNetwork = MessageBuffer()
        rb_cntrl.requestToNetwork.master = ruby_system.network.slave

        rb_cntrl.notifyFromRegionDir = MessageBuffer()
        rb_cntrl.notifyFromRegionDir.slave = ruby_system.network.master

        rb_cntrl.probeFromRegionDir = MessageBuffer()
        rb_cntrl.probeFromRegionDir.slave = ruby_system.network.master

        rb_cntrl.unblockFromDir = MessageBuffer()
        rb_cntrl.unblockFromDir.slave = ruby_system.network.master

        rb_cntrl.responseToRegDir = MessageBuffer()
        rb_cntrl.responseToRegDir.master = ruby_system.network.slave

        rb_cntrl.triggerQueue = MessageBuffer(ordered = True)

        cpuCluster.add(cp_cntrl)
        cpuCluster.add(rb_cntrl)

    gpuCluster = Cluster(extBW = (crossbar_bw), intBW = crossbar_bw)
    for i in xrange(options.num_compute_units):

        tcp_cntrl = TCPCntrl(TCC_select_num_bits = TCC_bits,
                             issue_latency = 1,
                             number_of_TBEs = 2560)
        # TBEs set to max outstanding requests
        tcp_cntrl.create(options, ruby_system, system)
        tcp_cntrl.WB = options.WB_L1
        tcp_cntrl.disableL1 = False

        exec("system.tcp_cntrl%d = tcp_cntrl" % i)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(tcp_cntrl.coalescer)

        # Connect the CP (TCP) controllers to the ruby network
        tcp_cntrl.requestFromTCP = MessageBuffer(ordered = True)
        tcp_cntrl.requestFromTCP.master = ruby_system.network.slave

        tcp_cntrl.responseFromTCP = MessageBuffer(ordered = True)
        tcp_cntrl.responseFromTCP.master = ruby_system.network.slave

        tcp_cntrl.unblockFromCore = MessageBuffer()
        tcp_cntrl.unblockFromCore.master = ruby_system.network.slave

        tcp_cntrl.probeToTCP = MessageBuffer(ordered = True)
        tcp_cntrl.probeToTCP.slave = ruby_system.network.master

        tcp_cntrl.responseToTCP = MessageBuffer(ordered = True)
        tcp_cntrl.responseToTCP.slave = ruby_system.network.master

        tcp_cntrl.mandatoryQueue = MessageBuffer()

        gpuCluster.add(tcp_cntrl)

    for i in xrange(options.num_sqc):

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

        sqc_cntrl.probeToSQC = MessageBuffer(ordered = True)
        sqc_cntrl.probeToSQC.slave = ruby_system.network.master

        sqc_cntrl.responseToSQC = MessageBuffer(ordered = True)
        sqc_cntrl.responseToSQC.slave = ruby_system.network.master

        sqc_cntrl.mandatoryQueue = MessageBuffer()

        # SQC also in GPU cluster
        gpuCluster.add(sqc_cntrl)

    numa_bit = 6

    for i in xrange(options.num_tccs):

        tcc_cntrl = TCCCntrl()
        tcc_cntrl.create(options, ruby_system, system)
        tcc_cntrl.l2_request_latency = 1
        tcc_cntrl.l2_response_latency = options.TCC_latency
        tcc_cntrl.WB = options.WB_L2
        tcc_cntrl.number_of_TBEs = 2560 * options.num_compute_units

        # Connect the TCC controllers to the ruby network
        tcc_cntrl.requestFromTCP = MessageBuffer(ordered = True)
        tcc_cntrl.requestFromTCP.slave = ruby_system.network.master

        tcc_cntrl.responseToCore = MessageBuffer(ordered = True)
        tcc_cntrl.responseToCore.master = ruby_system.network.slave

        tcc_cntrl.probeFromNB = MessageBuffer()
        tcc_cntrl.probeFromNB.slave = ruby_system.network.master

        tcc_cntrl.responseFromNB = MessageBuffer()
        tcc_cntrl.responseFromNB.slave = ruby_system.network.master

        tcc_cntrl.requestToNB = MessageBuffer(ordered = True)
        tcc_cntrl.requestToNB.master = ruby_system.network.slave

        tcc_cntrl.responseToNB = MessageBuffer()
        tcc_cntrl.responseToNB.master = ruby_system.network.slave

        tcc_cntrl.unblockToNB = MessageBuffer()
        tcc_cntrl.unblockToNB.master = ruby_system.network.slave

        tcc_cntrl.triggerQueue = MessageBuffer(ordered = True)

        rb_cntrl = RBCntrl()
        rb_cntrl.create(options, ruby_system, system)
        rb_cntrl.number_of_TBEs = 2560 * options.num_compute_units
        rb_cntrl.isOnCPU = False

        # Connect the RB controllers to the ruby network
        rb_cntrl.requestFromCore = MessageBuffer(ordered = True)
        rb_cntrl.requestFromCore.slave = ruby_system.network.master

        rb_cntrl.responseFromCore = MessageBuffer()
        rb_cntrl.responseFromCore.slave = ruby_system.network.master

        rb_cntrl.requestToNetwork = MessageBuffer()
        rb_cntrl.requestToNetwork.master = ruby_system.network.slave

        rb_cntrl.notifyFromRegionDir = MessageBuffer()
        rb_cntrl.notifyFromRegionDir.slave = ruby_system.network.master

        rb_cntrl.probeFromRegionDir = MessageBuffer()
        rb_cntrl.probeFromRegionDir.slave = ruby_system.network.master

        rb_cntrl.unblockFromDir = MessageBuffer()
        rb_cntrl.unblockFromDir.slave = ruby_system.network.master

        rb_cntrl.responseToRegDir = MessageBuffer()
        rb_cntrl.responseToRegDir.master = ruby_system.network.slave

        rb_cntrl.triggerQueue = MessageBuffer(ordered = True)

        tcc_cntrl.regionBufferNum = rb_cntrl.version

        exec("system.tcc_cntrl%d = tcc_cntrl" % i)
        exec("system.tcc_rb_cntrl%d = rb_cntrl" % i)

        # TCC cntrls added to the GPU cluster
        gpuCluster.add(tcc_cntrl)
        gpuCluster.add(rb_cntrl)

    # Because of wire buffers, num_l3caches must equal num_dirs
    # Region coherence only works with 1 dir
    assert(options.num_l3caches == options.num_dirs == 1)

    # This is the base crossbar that connects the L3s, Dirs, and cpu/gpu
    # Clusters
    mainCluster = Cluster(intBW = crossbar_bw)

    dir_cntrl = DirCntrl()
    dir_cntrl.create(options, ruby_system, system)
    dir_cntrl.number_of_TBEs = 2560 * options.num_compute_units
    dir_cntrl.useL3OnWT = options.use_L3_on_WT

    # Connect the Directory controller to the ruby network
    dir_cntrl.requestFromCores = MessageBuffer()
    dir_cntrl.requestFromCores.slave = ruby_system.network.master

    dir_cntrl.responseFromCores = MessageBuffer()
    dir_cntrl.responseFromCores.slave = ruby_system.network.master

    dir_cntrl.unblockFromCores = MessageBuffer()
    dir_cntrl.unblockFromCores.slave = ruby_system.network.master

    dir_cntrl.probeToCore = MessageBuffer()
    dir_cntrl.probeToCore.master = ruby_system.network.slave

    dir_cntrl.responseToCore = MessageBuffer()
    dir_cntrl.responseToCore.master = ruby_system.network.slave

    dir_cntrl.reqFromRegBuf = MessageBuffer()
    dir_cntrl.reqFromRegBuf.slave = ruby_system.network.master

    dir_cntrl.reqToRegDir = MessageBuffer(ordered = True)
    dir_cntrl.reqToRegDir.master = ruby_system.network.slave

    dir_cntrl.reqFromRegDir = MessageBuffer(ordered = True)
    dir_cntrl.reqFromRegDir.slave = ruby_system.network.master

    dir_cntrl.unblockToRegDir = MessageBuffer()
    dir_cntrl.unblockToRegDir.master = ruby_system.network.slave

    dir_cntrl.triggerQueue = MessageBuffer(ordered = True)
    dir_cntrl.L3triggerQueue = MessageBuffer(ordered = True)
    dir_cntrl.responseFromMemory = MessageBuffer()

    exec("system.dir_cntrl%d = dir_cntrl" % i)
    dir_cntrl_nodes.append(dir_cntrl)

    mainCluster.add(dir_cntrl)

    reg_cntrl = RegionCntrl(noTCCdir=True,TCC_select_num_bits = TCC_bits)
    reg_cntrl.create(options, ruby_system, system)
    reg_cntrl.number_of_TBEs = options.num_tbes
    reg_cntrl.cpuRegionBufferNum = system.rb_cntrl0.version
    reg_cntrl.gpuRegionBufferNum = system.tcc_rb_cntrl0.version

    # Connect the Region Dir controllers to the ruby network
    reg_cntrl.requestToDir = MessageBuffer(ordered = True)
    reg_cntrl.requestToDir.master = ruby_system.network.slave

    reg_cntrl.notifyToRBuffer = MessageBuffer()
    reg_cntrl.notifyToRBuffer.master = ruby_system.network.slave

    reg_cntrl.probeToRBuffer = MessageBuffer()
    reg_cntrl.probeToRBuffer.master = ruby_system.network.slave

    reg_cntrl.responseFromRBuffer = MessageBuffer()
    reg_cntrl.responseFromRBuffer.slave = ruby_system.network.master

    reg_cntrl.requestFromRegBuf = MessageBuffer()
    reg_cntrl.requestFromRegBuf.slave = ruby_system.network.master

    reg_cntrl.triggerQueue = MessageBuffer(ordered = True)

    exec("system.reg_cntrl%d = reg_cntrl" % i)

    mainCluster.add(reg_cntrl)

    # Assuming no DMA devices
    assert(len(dma_devices) == 0)

    # Add cpu/gpu clusters to main cluster
    mainCluster.add(cpuCluster)
    mainCluster.add(gpuCluster)

    ruby_system.network.number_of_virtual_networks = 10

    return (cpu_sequencers, dir_cntrl_nodes, mainCluster)
