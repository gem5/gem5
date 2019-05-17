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

        self.issue_latency = options.cpu_to_dir_latency
        self.send_evictions = send_evicts(options)

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

class TCPCache(RubyCache):
    size = "16kB"
    assoc = 16
    dataArrayBanks = 16 #number of data banks
    tagArrayBanks = 16  #number of tag banks
    dataAccessLatency = 4
    tagAccessLatency = 1
    def create(self, options):
        self.size = MemorySize(options.tcp_size)
        self.assoc = options.tcp_assoc
        self.resourceStalls = options.no_tcc_resource_stalls
        self.replacement_policy = PseudoLRUReplacementPolicy()

class TCPCntrl(TCP_Controller, CntrlBase):

    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = TCPCache(tagAccessLatency = options.TCP_latency,
                                dataAccessLatency = options.TCP_latency)
        self.L1cache.resourceStalls = options.no_resource_stalls
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

    def createCP(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = TCPCache(tagAccessLatency = options.TCP_latency,
                                dataAccessLatency = options.TCP_latency)
        self.L1cache.resourceStalls = options.no_resource_stalls
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

        self.use_seq_not_coal = True

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

class TCC(RubyCache):
    size = MemorySize("256kB")
    assoc = 16
    dataAccessLatency = 8
    tagAccessLatency = 2
    resourceStalls = True
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
        self.replacement_policy = PseudoLRUReplacementPolicy()


class TCCCntrl(TCC_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L2cache = TCC()
        self.L2cache.create(options)
        self.L2cache.resourceStalls = options.no_tcc_resource_stalls

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
        self.replacement_policy = PseudoLRUReplacementPolicy()

class L3Cntrl(L3Cache_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L3cache = L3Cache()
        self.L3cache.create(options, ruby_system, system)

        self.l3_response_latency = max(self.L3cache.dataAccessLatency, self.L3cache.tagAccessLatency)
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

class DirMem(RubyDirectoryMemory, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        phys_mem_size = AddrRange(options.mem_size).size()
        mem_module_size = phys_mem_size / options.num_dirs
        dir_size = MemorySize('0B')
        dir_size.value = mem_module_size
        self.size = dir_size

class DirCntrl(Directory_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.response_latency = 30

        self.directory = DirMem()
        self.directory.create(options, ruby_system, system)

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
    parser.add_option("--num-subcaches", type = "int", default = 4)
    parser.add_option("--l3-data-latency", type = "int", default = 20)
    parser.add_option("--l3-tag-latency", type = "int", default = 15)
    parser.add_option("--cpu-to-dir-latency", type = "int", default = 120)
    parser.add_option("--gpu-to-dir-latency", type = "int", default = 120)
    parser.add_option("--no-resource-stalls", action = "store_false",
                      default = True)
    parser.add_option("--no-tcc-resource-stalls", action = "store_false",
                      default = True)
    parser.add_option("--use-L3-on-WT", action = "store_true", default = False)
    parser.add_option("--num-tbes", type = "int", default = 256)
    parser.add_option("--l2-latency", type = "int", default = 50)  # load to use
    parser.add_option("--num-tccs", type = "int", default = 1,
                      help = "number of TCC banks in the GPU")
    parser.add_option("--sqc-size", type = 'string', default = '32kB',
                      help = "SQC cache size")
    parser.add_option("--sqc-assoc", type = 'int', default = 8,
                      help = "SQC cache assoc")
    parser.add_option("--WB_L1", action = "store_true", default = False,
                      help = "writeback L1")
    parser.add_option("--WB_L2", action = "store_true", default = False,
                      help = "writeback L2")
    parser.add_option("--TCP_latency", type = "int", default = 4,
                      help = "TCP latency")
    parser.add_option("--TCC_latency", type = "int", default = 16,
                      help = "TCC latency")
    parser.add_option("--tcc-size", type = 'string', default = '256kB',
                      help = "agregate tcc size")
    parser.add_option("--tcc-assoc", type = 'int', default = 16,
                      help = "tcc assoc")
    parser.add_option("--tcp-size", type = 'string', default = '16kB',
                      help = "tcp size")
    parser.add_option("--tcp-assoc", type = 'int', default = 16,
                      help = "tcp assoc")
    parser.add_option("--noL1", action = "store_true", default = False,
                      help = "bypassL1")

def create_system(options, full_system, system, dma_devices, bootmem,
                  ruby_system):
    if buildEnv['PROTOCOL'] != 'GPU_VIPER':
        panic("This script requires the GPU_VIPER protocol to be built.")

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
    dir_cntrl_nodes = []
    l3_cntrl_nodes = []

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    # This is the base crossbar that connects the L3s, Dirs, and cpu/gpu
    # Clusters
    crossbar_bw = None
    mainCluster = None
    if hasattr(options, 'bw_scalor') and options.bw_scalor > 0:
        #Assuming a 2GHz clock
        crossbar_bw = 16 * options.num_compute_units * options.bw_scalor
        mainCluster = Cluster(intBW=crossbar_bw)
    else:
        mainCluster = Cluster(intBW=8) # 16 GB/s
    for i in range(options.num_dirs):

        dir_cntrl = DirCntrl(noTCCdir = True, TCC_select_num_bits = TCC_bits)
        dir_cntrl.create(options, ruby_system, system)
        dir_cntrl.number_of_TBEs = options.num_tbes
        dir_cntrl.useL3OnWT = options.use_L3_on_WT
        # the number_of_TBEs is inclusive of TBEs below

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

        exec("ruby_system.dir_cntrl%d = dir_cntrl" % i)
        dir_cntrl_nodes.append(dir_cntrl)

        mainCluster.add(dir_cntrl)

    cpuCluster = None
    if hasattr(options, 'bw_scalor') and options.bw_scalor > 0:
        cpuCluster = Cluster(extBW = crossbar_bw, intBW = crossbar_bw)
    else:
        cpuCluster = Cluster(extBW = 8, intBW = 8) # 16 GB/s
    for i in range((options.num_cpus + 1) // 2):

        cp_cntrl = CPCntrl()
        cp_cntrl.create(options, ruby_system, system)

        exec("ruby_system.cp_cntrl%d = cp_cntrl" % i)
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

    # Register CPUs and caches for each CorePair and directory (SE mode only)
    if not full_system:
        for i in xrange((options.num_cpus + 1) // 2):
            FileSystemConfig.register_cpu(physical_package_id = 0,
                                          core_siblings = \
                                            xrange(options.num_cpus),
                                          core_id = i*2,
                                          thread_siblings = [])

            FileSystemConfig.register_cpu(physical_package_id = 0,
                                          core_siblings = \
                                            xrange(options.num_cpus),
                                          core_id = i*2+1,
                                          thread_siblings = [])

            FileSystemConfig.register_cache(level = 0,
                                            idu_type = 'Instruction',
                                            size = options.l1i_size,
                                            line_size = options.cacheline_size,
                                            assoc = options.l1i_assoc,
                                            cpus = [i*2, i*2+1])

            FileSystemConfig.register_cache(level = 0,
                                            idu_type = 'Data',
                                            size = options.l1d_size,
                                            line_size = options.cacheline_size,
                                            assoc = options.l1d_assoc,
                                            cpus = [i*2])

            FileSystemConfig.register_cache(level = 0,
                                            idu_type = 'Data',
                                            size = options.l1d_size,
                                            line_size = options.cacheline_size,
                                            assoc = options.l1d_assoc,
                                            cpus = [i*2+1])

            FileSystemConfig.register_cache(level = 1,
                                            idu_type = 'Unified',
                                            size = options.l2_size,
                                            line_size = options.cacheline_size,
                                            assoc = options.l2_assoc,
                                            cpus = [i*2, i*2+1])

        for i in range(options.num_dirs):
            FileSystemConfig.register_cache(level = 2,
                                            idu_type = 'Unified',
                                            size = options.l3_size,
                                            line_size = options.cacheline_size,
                                            assoc = options.l3_assoc,
                                            cpus = [n for n in
                                                xrange(options.num_cpus)])

    gpuCluster = None
    if hasattr(options, 'bw_scalor') and options.bw_scalor > 0:
      gpuCluster = Cluster(extBW = crossbar_bw, intBW = crossbar_bw)
    else:
      gpuCluster = Cluster(extBW = 8, intBW = 8) # 16 GB/s
    for i in range(options.num_compute_units):

        tcp_cntrl = TCPCntrl(TCC_select_num_bits = TCC_bits,
                             issue_latency = 1,
                             number_of_TBEs = 2560)
        # TBEs set to max outstanding requests
        tcp_cntrl.create(options, ruby_system, system)
        tcp_cntrl.WB = options.WB_L1
        tcp_cntrl.disableL1 = options.noL1
        tcp_cntrl.L1cache.tagAccessLatency = options.TCP_latency
        tcp_cntrl.L1cache.dataAccessLatency = options.TCP_latency

        exec("ruby_system.tcp_cntrl%d = tcp_cntrl" % i)
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

        tcp_cntrl.unblockFromCore = MessageBuffer()
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

        exec("ruby_system.sqc_cntrl%d = sqc_cntrl" % i)
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

    for i in range(options.num_cp):

        tcp_ID = options.num_compute_units + i
        sqc_ID = options.num_sqc + i

        tcp_cntrl = TCPCntrl(TCC_select_num_bits = TCC_bits,
                             issue_latency = 1,
                             number_of_TBEs = 2560)
        # TBEs set to max outstanding requests
        tcp_cntrl.createCP(options, ruby_system, system)
        tcp_cntrl.WB = options.WB_L1
        tcp_cntrl.disableL1 = options.noL1
        tcp_cntrl.L1cache.tagAccessLatency = options.TCP_latency
        tcp_cntrl.L1cache.dataAccessLatency = options.TCP_latency

        exec("ruby_system.tcp_cntrl%d = tcp_cntrl" % tcp_ID)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(tcp_cntrl.sequencer)
        tcp_cntrl_nodes.append(tcp_cntrl)

        # Connect the CP (TCP) controllers to the ruby network
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
        sqc_cntrl.create(options, ruby_system, system)

        exec("ruby_system.sqc_cntrl%d = sqc_cntrl" % sqc_ID)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cpu_sequencers.append(sqc_cntrl.sequencer)

        # SQC also in GPU cluster
        gpuCluster.add(sqc_cntrl)

    for i in range(options.num_tccs):

        tcc_cntrl = TCCCntrl(l2_response_latency = options.TCC_latency)
        tcc_cntrl.create(options, ruby_system, system)
        tcc_cntrl.l2_request_latency = options.gpu_to_dir_latency
        tcc_cntrl.l2_response_latency = options.TCC_latency
        tcc_cntrl_nodes.append(tcc_cntrl)
        tcc_cntrl.WB = options.WB_L2
        tcc_cntrl.number_of_TBEs = 2560 * options.num_compute_units
        # the number_of_TBEs is inclusive of TBEs below

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

        exec("ruby_system.tcc_cntrl%d = tcc_cntrl" % i)

        # connect all of the wire buffers between L3 and dirs up
        # TCC cntrls added to the GPU cluster
        gpuCluster.add(tcc_cntrl)

    # Assuming no DMA devices
    assert(len(dma_devices) == 0)

    # Add cpu/gpu clusters to main cluster
    mainCluster.add(cpuCluster)
    mainCluster.add(gpuCluster)

    ruby_system.network.number_of_virtual_networks = 10

    return (cpu_sequencers, dir_cntrl_nodes, mainCluster)
