# Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
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

from common import (
    FileSystemConfig,
    MemConfig,
    ObjectList,
)

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

from .Ruby import (
    create_topology,
    send_evicts,
)

addToPath("../")

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
        cls._version += 1  # Use count for this particular type
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
        self.replacement_policy = TreePLRURP()


class L2Cache(RubyCache):
    resourceStalls = False
    assoc = 16
    dataArrayBanks = 16
    tagArrayBanks = 16

    def create(self, size, assoc, options):
        self.size = MemorySize(size)
        self.assoc = assoc
        self.replacement_policy = TreePLRURP()


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


class TCPCache(RubyCache):
    size = "16kB"
    assoc = 16
    dataArrayBanks = 16  # number of data banks
    tagArrayBanks = 16  # number of tag banks
    dataAccessLatency = 4
    tagAccessLatency = 1

    def create(self, options):
        self.size = MemorySize(options.tcp_size)
        self.assoc = options.tcp_assoc
        self.resourceStalls = options.no_tcc_resource_stalls
        self.replacement_policy = TreePLRURP()


class TCPCntrl(TCP_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = TCPCache(
            tagAccessLatency=options.TCP_latency,
            dataAccessLatency=options.TCP_latency,
        )
        self.L1cache.resourceStalls = options.no_resource_stalls
        self.L1cache.dataArrayBanks = options.tcp_num_banks
        self.L1cache.tagArrayBanks = options.tcp_num_banks
        self.L1cache.create(options)
        self.issue_latency = 1
        # TCP_Controller inherits this from RubyController
        self.mandatory_queue_latency = options.mandatory_queue_latency

        self.coalescer = VIPERCoalescer()
        self.coalescer.version = self.seqCount()
        self.coalescer.icache = self.L1cache
        self.coalescer.dcache = self.L1cache
        self.coalescer.ruby_system = ruby_system
        self.coalescer.support_inst_reqs = False
        self.coalescer.is_cpu_sequencer = False
        if options.tcp_deadlock_threshold:
            self.coalescer.deadlock_threshold = options.tcp_deadlock_threshold
        self.coalescer.max_coalesces_per_cycle = (
            options.max_coalesces_per_cycle
        )

        self.sequencer = RubySequencer()
        self.sequencer.version = self.seqCount()
        self.sequencer.dcache = self.L1cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.is_cpu_sequencer = True

        self.use_seq_not_coal = False

        self.ruby_system = ruby_system
        if hasattr(options, "gpu_clock") and hasattr(options, "gpu_voltage"):
            self.clk_domain = SrcClockDomain(
                clock=options.gpu_clock,
                voltage_domain=VoltageDomain(voltage=options.gpu_voltage),
            )

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def createCP(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = TCPCache(
            tagAccessLatency=options.TCP_latency,
            dataAccessLatency=options.TCP_latency,
        )
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
        self.replacement_policy = TreePLRURP()


class SQCCntrl(SQC_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()

        self.L1cache = SQCCache()
        self.L1cache.create(options)
        self.L1cache.resourceStalls = options.no_resource_stalls

        self.sequencer = RubySequencer()

        self.sequencer.version = self.seqCount()
        self.sequencer.dcache = self.L1cache
        self.sequencer.ruby_system = ruby_system
        self.sequencer.support_data_reqs = False
        self.sequencer.is_cpu_sequencer = False
        if options.sqc_deadlock_threshold:
            self.sequencer.deadlock_threshold = options.sqc_deadlock_threshold

        self.ruby_system = ruby_system
        if hasattr(options, "gpu_clock") and hasattr(options, "gpu_voltage"):
            self.clk_domain = SrcClockDomain(
                clock=options.gpu_clock,
                voltage_domain=VoltageDomain(voltage=options.gpu_voltage),
            )

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
        self.atomicLatency = options.atomic_alu_latency
        self.atomicALUs = options.tcc_num_atomic_alus
        if hasattr(options, "bw_scalor") and options.bw_scalor > 0:
            s = options.num_compute_units
            tcc_size = s * 128
            tcc_size = str(tcc_size) + "kB"
            self.size = MemorySize(tcc_size)
            self.dataArrayBanks = 64
            self.tagArrayBanks = 64
        else:
            self.size = MemorySize(options.tcc_size)
            self.dataArrayBanks = (
                256 / options.num_tccs
            )  # number of data banks
            self.tagArrayBanks = 256 / options.num_tccs  # number of tag banks
        self.size.value = self.size.value / options.num_tccs
        if (self.size.value / int(self.assoc)) < 128:
            self.size.value = int(128 * self.assoc)
        self.start_index_bit = math.log(options.cacheline_size, 2) + math.log(
            options.num_tccs, 2
        )
        self.replacement_policy = TreePLRURP()


class TCCCntrl(TCC_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L2cache = TCC(
            tagAccessLatency=options.tcc_tag_access_latency,
            dataAccessLatency=options.tcc_data_access_latency,
        )
        self.L2cache.create(options)
        self.L2cache.resourceStalls = options.no_tcc_resource_stalls

        self.ruby_system = ruby_system
        if hasattr(options, "gpu_clock") and hasattr(options, "gpu_voltage"):
            self.clk_domain = SrcClockDomain(
                clock=options.gpu_clock,
                voltage_domain=VoltageDomain(voltage=options.gpu_voltage),
            )

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
        self.replacement_policy = TreePLRURP()


# unused in GPU_VIPER; see git blame for discussion
class L3Cntrl(L3Cache_Controller, CntrlBase):
    def create(self, options, ruby_system, system):
        self.version = self.versionCount()
        self.L3cache = L3Cache()
        self.L3cache.create(options, ruby_system, system)

        self.l3_response_latency = max(
            self.L3cache.dataAccessLatency, self.L3cache.tagAccessLatency
        )
        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def connectWireBuffers(
        self,
        req_to_dir,
        resp_to_dir,
        l3_unblock_to_dir,
        req_to_l3,
        probe_to_l3,
        resp_to_l3,
    ):
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

        self.l3_hit_latency = max(
            self.L3CacheMemory.dataAccessLatency,
            self.L3CacheMemory.tagAccessLatency,
        )

        self.number_of_TBEs = options.num_tbes

        self.ruby_system = ruby_system

        if options.recycle_latency:
            self.recycle_latency = options.recycle_latency

    def connectWireBuffers(
        self,
        req_to_dir,
        resp_to_dir,
        l3_unblock_to_dir,
        req_to_l3,
        probe_to_l3,
        resp_to_l3,
    ):
        self.reqToDir = req_to_dir
        self.respToDir = resp_to_dir
        self.l3UnblockToDir = l3_unblock_to_dir
        self.reqToL3 = req_to_l3
        self.probeToL3 = probe_to_l3
        self.respToL3 = resp_to_l3


def define_options(parser):
    parser.add_argument("--num-subcaches", type=int, default=4)
    parser.add_argument("--l3-data-latency", type=int, default=20)
    parser.add_argument("--l3-tag-latency", type=int, default=15)
    parser.add_argument("--cpu-to-dir-latency", type=int, default=120)
    parser.add_argument("--gpu-to-dir-latency", type=int, default=120)
    parser.add_argument(
        "--no-resource-stalls", action="store_false", default=True
    )
    parser.add_argument(
        "--no-tcc-resource-stalls", action="store_false", default=True
    )
    parser.add_argument("--use-L3-on-WT", action="store_true", default=False)
    parser.add_argument("--num-tbes", type=int, default=256)
    parser.add_argument("--l2-latency", type=int, default=50)  # load to use
    parser.add_argument(
        "--num-tccs",
        type=int,
        default=1,
        help="number of TCC banks in the GPU",
    )
    parser.add_argument(
        "--sqc-size", type=str, default="32kB", help="SQC cache size"
    )
    parser.add_argument(
        "--sqc-assoc", type=int, default=8, help="SQC cache assoc"
    )
    parser.add_argument(
        "--sqc-deadlock-threshold",
        type=int,
        help="Set the SQC deadlock threshold to some value",
    )

    parser.add_argument(
        "--WB_L1", action="store_true", default=False, help="writeback L1"
    )
    parser.add_argument(
        "--WB_L2", action="store_true", default=False, help="writeback L2"
    )
    parser.add_argument(
        "--TCP_latency",
        type=int,
        default=4,
        help="In combination with the number of banks for the "
        "TCP, this determines how many requests can happen "
        "per cycle (i.e., the bandwidth)",
    )
    parser.add_argument(
        "--mandatory_queue_latency",
        type=int,
        default=1,
        help="Hit latency for TCP",
    )
    parser.add_argument(
        "--TCC_latency", type=int, default=16, help="TCC latency"
    )
    parser.add_argument(
        "--tcc-size", type=str, default="256kB", help="agregate tcc size"
    )
    parser.add_argument("--tcc-assoc", type=int, default=16, help="tcc assoc")
    parser.add_argument(
        "--tcp-size", type=str, default="16kB", help="tcp size"
    )
    parser.add_argument("--tcp-assoc", type=int, default=16, help="tcp assoc")
    parser.add_argument(
        "--tcp-deadlock-threshold",
        type=int,
        help="Set the TCP deadlock threshold to some value",
    )
    parser.add_argument(
        "--max-coalesces-per-cycle",
        type=int,
        default=1,
        help="Maximum insts that may coalesce in a cycle",
    )

    parser.add_argument(
        "--noL1", action="store_true", default=False, help="bypassL1"
    )
    parser.add_argument(
        "--scalar-buffer-size",
        type=int,
        default=128,
        help="Size of the mandatory queue in the GPU scalar "
        "cache controller",
    )
    parser.add_argument(
        "--glc-atomic-latency", type=int, default=1, help="GLC Atomic Latency"
    )
    parser.add_argument(
        "--atomic-alu-latency", type=int, default=0, help="Atomic ALU Latency"
    )
    parser.add_argument(
        "--tcc-num-atomic-alus",
        type=int,
        default=64,
        help="Number of atomic ALUs in the TCC",
    )
    parser.add_argument(
        "--tcp-num-banks",
        type=int,
        default="16",
        help="Num of banks in L1 cache",
    )
    parser.add_argument(
        "--tcc-num-banks",
        type=int,
        default="16",
        help="Num of banks in L2 cache",
    )
    parser.add_argument(
        "--tcc-tag-access-latency",
        type=int,
        default="2",
        help="Tag access latency in L2 cache",
    )
    parser.add_argument(
        "--tcc-data-access-latency",
        type=int,
        default="8",
        help="Data access latency in L2 cache",
    )


def construct_dirs(options, system, ruby_system, network):
    dir_cntrl_nodes = []

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

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
            addr_range = m5.objects.AddrRange(
                r.start,
                size=r.size(),
                intlvHighBit=numa_bit,
                intlvBits=dir_bits,
                intlvMatch=i,
            )
            dir_ranges.append(addr_range)

        dir_cntrl = DirCntrl(noTCCdir=True, TCC_select_num_bits=TCC_bits)
        dir_cntrl.create(options, dir_ranges, ruby_system, system)
        dir_cntrl.number_of_TBEs = options.num_tbes
        dir_cntrl.useL3OnWT = options.use_L3_on_WT
        dir_cntrl.L2isWB = options.WB_L2
        # the number_of_TBEs is inclusive of TBEs below

        # Connect the Directory controller to the ruby network
        dir_cntrl.requestFromCores = MessageBuffer(ordered=True)
        dir_cntrl.requestFromCores.in_port = network.out_port

        dir_cntrl.responseFromCores = MessageBuffer()
        dir_cntrl.responseFromCores.in_port = network.out_port

        dir_cntrl.unblockFromCores = MessageBuffer()
        dir_cntrl.unblockFromCores.in_port = network.out_port

        dir_cntrl.probeToCore = MessageBuffer()
        dir_cntrl.probeToCore.out_port = network.in_port

        dir_cntrl.responseToCore = MessageBuffer()
        dir_cntrl.responseToCore.out_port = network.in_port

        dir_cntrl.triggerQueue = MessageBuffer(ordered=True)
        dir_cntrl.L3triggerQueue = MessageBuffer(ordered=True)
        dir_cntrl.requestToMemory = MessageBuffer(ordered=True)
        dir_cntrl.responseFromMemory = MessageBuffer(ordered=True)

        dir_cntrl.requestFromDMA = MessageBuffer(ordered=True)
        dir_cntrl.requestFromDMA.in_port = network.out_port

        dir_cntrl.responseToDMA = MessageBuffer()
        dir_cntrl.responseToDMA.out_port = network.in_port

        exec("ruby_system.dir_cntrl%d = dir_cntrl" % i)
        dir_cntrl_nodes.append(dir_cntrl)

    return dir_cntrl_nodes


def construct_gpudirs(options, system, ruby_system, network):
    dir_cntrl_nodes = []
    mem_ctrls = []

    xor_low_bit = 0

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    dir_bits = int(math.log(options.dgpu_num_dirs, 2))
    block_size_bits = int(math.log(options.cacheline_size, 2))
    numa_bit = block_size_bits + dir_bits - 1

    gpu_mem_range = AddrRange(0, size=options.dgpu_mem_size)
    for i in range(options.dgpu_num_dirs):
        addr_range = m5.objects.AddrRange(
            gpu_mem_range.start,
            size=gpu_mem_range.size(),
            intlvHighBit=numa_bit,
            intlvBits=dir_bits,
            intlvMatch=i,
            xorHighBit=xor_low_bit,
        )

        dir_cntrl = DirCntrl(noTCCdir=True, TCC_select_num_bits=TCC_bits)
        dir_cntrl.create(options, [addr_range], ruby_system, system)
        dir_cntrl.number_of_TBEs = options.num_tbes
        dir_cntrl.useL3OnWT = False
        dir_cntrl.L2isWB = options.WB_L2

        # Connect the Directory controller to the ruby network
        dir_cntrl.requestFromCores = MessageBuffer(ordered=True)
        dir_cntrl.requestFromCores.in_port = network.out_port

        dir_cntrl.responseFromCores = MessageBuffer()
        dir_cntrl.responseFromCores.in_port = network.out_port

        dir_cntrl.unblockFromCores = MessageBuffer()
        dir_cntrl.unblockFromCores.in_port = network.out_port

        dir_cntrl.probeToCore = MessageBuffer()
        dir_cntrl.probeToCore.out_port = network.in_port

        dir_cntrl.responseToCore = MessageBuffer()
        dir_cntrl.responseToCore.out_port = network.in_port

        dir_cntrl.triggerQueue = MessageBuffer(ordered=True)
        dir_cntrl.L3triggerQueue = MessageBuffer(ordered=True)
        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()

        dir_cntrl.requestFromDMA = MessageBuffer(ordered=True)
        dir_cntrl.requestFromDMA.in_port = network.out_port

        dir_cntrl.responseToDMA = MessageBuffer()
        dir_cntrl.responseToDMA.out_port = network.in_port

        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()

        # Create memory controllers too
        mem_type = ObjectList.mem_list.get(options.dgpu_mem_type)
        dram_intf = MemConfig.create_mem_intf(
            mem_type,
            gpu_mem_range,
            i,
            int(math.log(options.dgpu_num_dirs, 2)),
            options.cacheline_size,
            xor_low_bit,
        )
        if issubclass(mem_type, DRAMInterface):
            mem_ctrl = m5.objects.MemCtrl(dram=dram_intf)
        else:
            mem_ctrl = dram_intf

        mem_ctrl.port = dir_cntrl.memory_out_port
        mem_ctrl.dram.enable_dram_powerdown = False
        dir_cntrl.addr_ranges = dram_intf.range

        # Append
        exec("system.ruby.gpu_dir_cntrl%d = dir_cntrl" % i)
        dir_cntrl_nodes.append(dir_cntrl)
        mem_ctrls.append(mem_ctrl)

    system.gpu_mem_ctrls = mem_ctrls

    return dir_cntrl_nodes, mem_ctrls


def construct_corepairs(options, system, ruby_system, network):
    cpu_sequencers = []
    cp_cntrl_nodes = []

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
        cp_cntrl.requestFromCore.out_port = network.in_port

        cp_cntrl.responseFromCore = MessageBuffer()
        cp_cntrl.responseFromCore.out_port = network.in_port

        cp_cntrl.unblockFromCore = MessageBuffer()
        cp_cntrl.unblockFromCore.out_port = network.in_port

        cp_cntrl.probeToCore = MessageBuffer()
        cp_cntrl.probeToCore.in_port = network.out_port

        cp_cntrl.responseToCore = MessageBuffer()
        cp_cntrl.responseToCore.in_port = network.out_port

        cp_cntrl.mandatoryQueue = MessageBuffer()
        cp_cntrl.triggerQueue = MessageBuffer(ordered=True)

        cp_cntrl_nodes.append(cp_cntrl)

    return (cpu_sequencers, cp_cntrl_nodes)


def construct_tcps(options, system, ruby_system, network):
    tcp_sequencers = []
    tcp_cntrl_nodes = []

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    for i in range(options.num_compute_units):
        tcp_cntrl = TCPCntrl(
            TCC_select_num_bits=TCC_bits, issue_latency=1, number_of_TBEs=2560
        )
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
        tcp_sequencers.append(tcp_cntrl.coalescer)
        tcp_cntrl_nodes.append(tcp_cntrl)

        # Connect the TCP controller to the ruby network
        tcp_cntrl.requestFromTCP = MessageBuffer(ordered=True)
        tcp_cntrl.requestFromTCP.out_port = network.in_port

        tcp_cntrl.responseFromTCP = MessageBuffer(ordered=True)
        tcp_cntrl.responseFromTCP.out_port = network.in_port

        tcp_cntrl.unblockFromCore = MessageBuffer()
        tcp_cntrl.unblockFromCore.out_port = network.in_port

        tcp_cntrl.probeToTCP = MessageBuffer(ordered=True)
        tcp_cntrl.probeToTCP.in_port = network.out_port

        tcp_cntrl.responseToTCP = MessageBuffer(ordered=True)
        tcp_cntrl.responseToTCP.in_port = network.out_port

        tcp_cntrl.mandatoryQueue = MessageBuffer()

    return (tcp_sequencers, tcp_cntrl_nodes)


def construct_sqcs(options, system, ruby_system, network):
    sqc_sequencers = []
    sqc_cntrl_nodes = []

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    for i in range(options.num_sqc):
        sqc_cntrl = SQCCntrl(TCC_select_num_bits=TCC_bits)
        sqc_cntrl.create(options, ruby_system, system)

        exec("ruby_system.sqc_cntrl%d = sqc_cntrl" % i)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        sqc_sequencers.append(sqc_cntrl.sequencer)
        sqc_cntrl_nodes.append(sqc_cntrl)

        # Connect the SQC controller to the ruby network
        sqc_cntrl.requestFromSQC = MessageBuffer(ordered=True)
        sqc_cntrl.requestFromSQC.out_port = network.in_port

        sqc_cntrl.probeToSQC = MessageBuffer(ordered=True)
        sqc_cntrl.probeToSQC.in_port = network.out_port

        sqc_cntrl.responseToSQC = MessageBuffer(ordered=True)
        sqc_cntrl.responseToSQC.in_port = network.out_port

        sqc_cntrl.mandatoryQueue = MessageBuffer()

    return (sqc_sequencers, sqc_cntrl_nodes)


def construct_scalars(options, system, ruby_system, network):
    scalar_sequencers = []
    scalar_cntrl_nodes = []

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    for i in range(options.num_scalar_cache):
        scalar_cntrl = SQCCntrl(TCC_select_num_bits=TCC_bits)
        scalar_cntrl.create(options, ruby_system, system)

        exec("ruby_system.scalar_cntrl%d = scalar_cntrl" % i)

        scalar_sequencers.append(scalar_cntrl.sequencer)
        scalar_cntrl_nodes.append(scalar_cntrl)

        scalar_cntrl.requestFromSQC = MessageBuffer(ordered=True)
        scalar_cntrl.requestFromSQC.out_port = network.in_port

        scalar_cntrl.probeToSQC = MessageBuffer(ordered=True)
        scalar_cntrl.probeToSQC.in_port = network.out_port

        scalar_cntrl.responseToSQC = MessageBuffer(ordered=True)
        scalar_cntrl.responseToSQC.in_port = network.out_port

        scalar_cntrl.mandatoryQueue = MessageBuffer(
            buffer_size=options.scalar_buffer_size
        )

    return (scalar_sequencers, scalar_cntrl_nodes)


def construct_cmdprocs(options, system, ruby_system, network):
    cmdproc_sequencers = []
    cmdproc_cntrl_nodes = []

    # For an odd number of CPUs, still create the right number of controllers
    TCC_bits = int(math.log(options.num_tccs, 2))

    for i in range(options.num_cp):
        tcp_ID = options.num_compute_units + i
        sqc_ID = options.num_sqc + i

        tcp_cntrl = TCPCntrl(
            TCC_select_num_bits=TCC_bits, issue_latency=1, number_of_TBEs=2560
        )
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
        cmdproc_sequencers.append(tcp_cntrl.sequencer)
        cmdproc_cntrl_nodes.append(tcp_cntrl)

        # Connect the CP (TCP) controllers to the ruby network
        tcp_cntrl.requestFromTCP = MessageBuffer(ordered=True)
        tcp_cntrl.requestFromTCP.out_port = network.in_port

        tcp_cntrl.responseFromTCP = MessageBuffer(ordered=True)
        tcp_cntrl.responseFromTCP.out_port = network.in_port

        tcp_cntrl.unblockFromCore = MessageBuffer(ordered=True)
        tcp_cntrl.unblockFromCore.out_port = network.in_port

        tcp_cntrl.probeToTCP = MessageBuffer(ordered=True)
        tcp_cntrl.probeToTCP.in_port = network.out_port

        tcp_cntrl.responseToTCP = MessageBuffer(ordered=True)
        tcp_cntrl.responseToTCP.in_port = network.out_port

        tcp_cntrl.mandatoryQueue = MessageBuffer()

        sqc_cntrl = SQCCntrl(TCC_select_num_bits=TCC_bits)
        sqc_cntrl.create(options, ruby_system, system)

        exec("ruby_system.sqc_cntrl%d = sqc_cntrl" % sqc_ID)
        #
        # Add controllers and sequencers to the appropriate lists
        #
        cmdproc_sequencers.append(sqc_cntrl.sequencer)
        cmdproc_cntrl_nodes.append(sqc_cntrl)

    return (cmdproc_sequencers, cmdproc_cntrl_nodes)


def construct_tccs(options, system, ruby_system, network):
    tcc_cntrl_nodes = []

    for i in range(options.num_tccs):
        tcc_cntrl = TCCCntrl(l2_response_latency=options.TCC_latency)
        tcc_cntrl.create(options, ruby_system, system)
        tcc_cntrl.l2_request_latency = options.gpu_to_dir_latency
        tcc_cntrl.l2_response_latency = options.TCC_latency
        tcc_cntrl.glc_atomic_latency = options.glc_atomic_latency
        tcc_cntrl_nodes.append(tcc_cntrl)
        tcc_cntrl.WB = options.WB_L2
        tcc_cntrl.number_of_TBEs = 2560 * options.num_compute_units
        # the number_of_TBEs is inclusive of TBEs below

        # Connect the TCC controllers to the ruby network
        tcc_cntrl.requestFromTCP = MessageBuffer(ordered=True)
        tcc_cntrl.requestFromTCP.in_port = network.out_port

        tcc_cntrl.responseToCore = MessageBuffer(ordered=True)
        tcc_cntrl.responseToCore.out_port = network.in_port

        tcc_cntrl.probeFromNB = MessageBuffer()
        tcc_cntrl.probeFromNB.in_port = network.out_port

        tcc_cntrl.responseFromNB = MessageBuffer()
        tcc_cntrl.responseFromNB.in_port = network.out_port

        tcc_cntrl.requestToNB = MessageBuffer(ordered=True)
        tcc_cntrl.requestToNB.out_port = network.in_port

        tcc_cntrl.responseToNB = MessageBuffer()
        tcc_cntrl.responseToNB.out_port = network.in_port

        tcc_cntrl.unblockToNB = MessageBuffer()
        tcc_cntrl.unblockToNB.out_port = network.in_port

        tcc_cntrl.triggerQueue = MessageBuffer(ordered=True)

        exec("ruby_system.tcc_cntrl%d = tcc_cntrl" % i)

    return tcc_cntrl_nodes


def create_system(
    options, full_system, system, dma_devices, bootmem, ruby_system, cpus
):
    if buildEnv["PROTOCOL"] != "GPU_VIPER":
        panic("This script requires the GPU_VIPER protocol to be built.")

    cpu_sequencers = []

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #

    # This is the base crossbar that connects the L3s, Dirs, and cpu/gpu
    # Clusters
    crossbar_bw = None
    mainCluster = None
    cpuCluster = None
    gpuCluster = None

    if hasattr(options, "bw_scalor") and options.bw_scalor > 0:
        # Assuming a 2GHz clock
        crossbar_bw = 16 * options.num_compute_units * options.bw_scalor
        mainCluster = Cluster(intBW=crossbar_bw)
        cpuCluster = Cluster(extBW=crossbar_bw, intBW=crossbar_bw)
        gpuCluster = Cluster(extBW=crossbar_bw, intBW=crossbar_bw)
    else:
        mainCluster = Cluster(intBW=8)  # 16 GB/s
        cpuCluster = Cluster(extBW=8, intBW=8)  # 16 GB/s
        gpuCluster = Cluster(extBW=8, intBW=8)  # 16 GB/s

    # Create CPU directory controllers
    dir_cntrl_nodes = construct_dirs(
        options, system, ruby_system, ruby_system.network
    )
    for dir_cntrl in dir_cntrl_nodes:
        mainCluster.add(dir_cntrl)

    # Create CPU core pairs
    (cp_sequencers, cp_cntrl_nodes) = construct_corepairs(
        options, system, ruby_system, ruby_system.network
    )
    cpu_sequencers.extend(cp_sequencers)
    for cp_cntrl in cp_cntrl_nodes:
        cpuCluster.add(cp_cntrl)

    # Register CPUs and caches for each CorePair and directory (SE mode only)
    if not full_system:
        for i in range((options.num_cpus + 1) // 2):
            FileSystemConfig.register_cpu(
                physical_package_id=0,
                core_siblings=range(options.num_cpus),
                core_id=i * 2,
                thread_siblings=[],
            )

            FileSystemConfig.register_cpu(
                physical_package_id=0,
                core_siblings=range(options.num_cpus),
                core_id=i * 2 + 1,
                thread_siblings=[],
            )

            FileSystemConfig.register_cache(
                level=0,
                idu_type="Instruction",
                size=options.l1i_size,
                line_size=options.cacheline_size,
                assoc=options.l1i_assoc,
                cpus=[i * 2, i * 2 + 1],
            )

            FileSystemConfig.register_cache(
                level=0,
                idu_type="Data",
                size=options.l1d_size,
                line_size=options.cacheline_size,
                assoc=options.l1d_assoc,
                cpus=[i * 2],
            )

            FileSystemConfig.register_cache(
                level=0,
                idu_type="Data",
                size=options.l1d_size,
                line_size=options.cacheline_size,
                assoc=options.l1d_assoc,
                cpus=[i * 2 + 1],
            )

            FileSystemConfig.register_cache(
                level=1,
                idu_type="Unified",
                size=options.l2_size,
                line_size=options.cacheline_size,
                assoc=options.l2_assoc,
                cpus=[i * 2, i * 2 + 1],
            )

        for i in range(options.num_dirs):
            FileSystemConfig.register_cache(
                level=2,
                idu_type="Unified",
                size=options.l3_size,
                line_size=options.cacheline_size,
                assoc=options.l3_assoc,
                cpus=[n for n in range(options.num_cpus)],
            )

    # Create TCPs
    (tcp_sequencers, tcp_cntrl_nodes) = construct_tcps(
        options, system, ruby_system, ruby_system.network
    )
    cpu_sequencers.extend(tcp_sequencers)
    for tcp_cntrl in tcp_cntrl_nodes:
        gpuCluster.add(tcp_cntrl)

    # Create SQCs
    (sqc_sequencers, sqc_cntrl_nodes) = construct_sqcs(
        options, system, ruby_system, ruby_system.network
    )
    cpu_sequencers.extend(sqc_sequencers)
    for sqc_cntrl in sqc_cntrl_nodes:
        gpuCluster.add(sqc_cntrl)

    # Create Scalars
    (scalar_sequencers, scalar_cntrl_nodes) = construct_scalars(
        options, system, ruby_system, ruby_system.network
    )
    cpu_sequencers.extend(scalar_sequencers)
    for scalar_cntrl in scalar_cntrl_nodes:
        gpuCluster.add(scalar_cntrl)

    # Create command processors
    (cmdproc_sequencers, cmdproc_cntrl_nodes) = construct_cmdprocs(
        options, system, ruby_system, ruby_system.network
    )
    cpu_sequencers.extend(cmdproc_sequencers)
    for cmdproc_cntrl in cmdproc_cntrl_nodes:
        gpuCluster.add(cmdproc_cntrl)

    # Create TCCs
    tcc_cntrl_nodes = construct_tccs(
        options, system, ruby_system, ruby_system.network
    )
    for tcc_cntrl in tcc_cntrl_nodes:
        gpuCluster.add(tcc_cntrl)

    for i, dma_device in enumerate(dma_devices):
        dma_seq = DMASequencer(version=i, ruby_system=ruby_system)
        dma_cntrl = DMA_Controller(
            version=i, dma_sequencer=dma_seq, ruby_system=ruby_system
        )
        exec("system.dma_cntrl%d = dma_cntrl" % i)

        # IDE doesn't have a .type but seems like everything else does.
        if not hasattr(dma_device, "type"):
            exec("system.dma_cntrl%d.dma_sequencer.in_ports = dma_device" % i)
        elif dma_device.type == "MemTest":
            exec(
                "system.dma_cntrl%d.dma_sequencer.in_ports = dma_devices.test"
                % i
            )
        else:
            exec(
                "system.dma_cntrl%d.dma_sequencer.in_ports = dma_device.dma"
                % i
            )

        dma_cntrl.requestToDir = MessageBuffer(buffer_size=0)
        dma_cntrl.requestToDir.out_port = ruby_system.network.in_port
        dma_cntrl.responseFromDir = MessageBuffer(buffer_size=0)
        dma_cntrl.responseFromDir.in_port = ruby_system.network.out_port
        dma_cntrl.mandatoryQueue = MessageBuffer(buffer_size=0)
        gpuCluster.add(dma_cntrl)

    # Add cpu/gpu clusters to main cluster
    mainCluster.add(cpuCluster)
    mainCluster.add(gpuCluster)

    ruby_system.network.number_of_virtual_networks = 11

    return (cpu_sequencers, dir_cntrl_nodes, mainCluster)
