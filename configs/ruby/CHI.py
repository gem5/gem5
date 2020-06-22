# Copyright (c) 2021 ARM Limited
# All rights reserved.
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

import math
import yaml
import m5
from m5.objects import *
from m5.defines import buildEnv
from .Ruby import create_topology, setup_memory_controllers

def define_options(parser):
    parser.add_option("--noc-config", action="store", type="string",
                      default=None,
                      help="YAML NoC config. parameters and bindings. "
                           "required for CustomMesh topology")

class Versions:
    '''
    Helper class to obtain unique ids for a given controller class.
    These are passed as the 'version' parameter when creating the controller.
    '''
    _seqs = 0
    @classmethod
    def getSeqId(cls):
        val = cls._seqs
        cls._seqs += 1
        return val

    _version = {}
    @classmethod
    def getVersion(cls, tp):
        if tp not in cls._version:
            cls._version[tp] = 0
        val = cls._version[tp]
        cls._version[tp] = val + 1
        return val


class CHI_Node(SubSystem):
    '''
    Base class with common functions for setting up Cache or Memory
    controllers that are part of a CHI RNF, RNFI, HNF, or SNF nodes.
    Notice getNetworkSideControllers and getAllControllers must be implemented
    in the derived classes.
    '''

    def __init__(self, ruby_system):
        super(CHI_Node, self).__init__()
        self._ruby_system = ruby_system
        self._network = ruby_system.network

    def getNetworkSideControllers(self):
        '''
        Returns all ruby controllers that need to be connected to the
        network
        '''
        raise NotImplementedError()

    def getAllControllers(self):
        '''
        Returns all ruby controllers associated with this node
        '''
        raise NotImplementedError()

    def setDownstream(self, cntrls):
        '''
        Sets cntrls as the downstream list of all controllers in this node
        '''
        for c in self.getNetworkSideControllers():
            c.downstream_destinations = cntrls

    def connectController(self, cntrl):
        '''
        Creates and configures the messages buffers for the CHI input/output
        ports that connect to the network
        '''
        cntrl.reqOut = MessageBuffer()
        cntrl.rspOut = MessageBuffer()
        cntrl.snpOut = MessageBuffer()
        cntrl.datOut = MessageBuffer()
        cntrl.reqIn = MessageBuffer()
        cntrl.rspIn = MessageBuffer()
        cntrl.snpIn = MessageBuffer()
        cntrl.datIn = MessageBuffer()

        # All CHI ports are always connected to the network.
        # Controllers that are not part of the getNetworkSideControllers list
        # still communicate using internal routers, thus we need to wire-up the
        # ports
        cntrl.reqOut.out_port = self._network.in_port
        cntrl.rspOut.out_port = self._network.in_port
        cntrl.snpOut.out_port = self._network.in_port
        cntrl.datOut.out_port = self._network.in_port
        cntrl.reqIn.in_port = self._network.out_port
        cntrl.rspIn.in_port = self._network.out_port
        cntrl.snpIn.in_port = self._network.out_port
        cntrl.datIn.in_port = self._network.out_port

class TriggerMessageBuffer(MessageBuffer):
    '''
    MessageBuffer for triggering internal controller events.
    These buffers should not be affected by the Ruby tester randomization
    and allow poping messages enqueued in the same cycle.
    '''
    randomization = 'disabled'
    allow_zero_latency = True

class OrderedTriggerMessageBuffer(TriggerMessageBuffer):
    ordered = True

class CHI_Cache_Controller(Cache_Controller):
    '''
    Default parameters for a Cache controller
    The Cache_Controller can also be used as a DMA requester or as
    a pure directory if all cache allocation policies are disabled.
    '''

    def __init__(self, ruby_system):
        super(CHI_Cache_Controller, self).__init__(
            version = Versions.getVersion(Cache_Controller),
            ruby_system = ruby_system,
            mandatoryQueue = MessageBuffer(),
            prefetchQueue = MessageBuffer(),
            triggerQueue = TriggerMessageBuffer(),
            retryTriggerQueue = OrderedTriggerMessageBuffer(),
            replTriggerQueue = OrderedTriggerMessageBuffer(),
            reqRdy = TriggerMessageBuffer(),
            snpRdy = TriggerMessageBuffer())
        # Set somewhat large number since we really a lot on internal
        # triggers. To limit the controller performance, tweak other
        # params such as: input port buffer size, cache banks, and output
        # port latency
        self.transitions_per_cycle = 128
        # This should be set to true in the data cache controller to enable
        # timeouts on unique lines when a store conditional fails
        self.sc_lock_enabled = False

class CHI_L1Controller(CHI_Cache_Controller):
    '''
    Default parameters for a L1 Cache controller
    '''

    def __init__(self, ruby_system, sequencer, cache, prefetcher):
        super(CHI_L1Controller, self).__init__(ruby_system)
        self.sequencer = sequencer
        self.cache = cache
        self.use_prefetcher = False
        self.send_evictions = True
        self.is_HN = False
        self.enable_DMT = False
        self.enable_DCT = False
        # Strict inclusive MOESI
        self.allow_SD = True
        self.alloc_on_seq_acc = True
        self.alloc_on_seq_line_write = False
        self.alloc_on_readshared = True
        self.alloc_on_readunique = True
        self.alloc_on_readonce = True
        self.alloc_on_writeback = True
        self.dealloc_on_unique = False
        self.dealloc_on_shared = False
        self.dealloc_backinv_unique = True
        self.dealloc_backinv_shared = True
        # Some reasonable default TBE params
        self.number_of_TBEs = 16
        self.number_of_repl_TBEs = 16
        self.number_of_snoop_TBEs = 4
        self.unify_repl_TBEs = False

class CHI_L2Controller(CHI_Cache_Controller):
    '''
    Default parameters for a L2 Cache controller
    '''

    def __init__(self, ruby_system, cache, prefetcher):
        super(CHI_L2Controller, self).__init__(ruby_system)
        self.sequencer = NULL
        self.cache = cache
        self.use_prefetcher = False
        self.allow_SD = True
        self.is_HN = False
        self.enable_DMT = False
        self.enable_DCT = False
        self.send_evictions = False
        # Strict inclusive MOESI
        self.alloc_on_seq_acc = False
        self.alloc_on_seq_line_write = False
        self.alloc_on_readshared = True
        self.alloc_on_readunique = True
        self.alloc_on_readonce = True
        self.alloc_on_writeback = True
        self.dealloc_on_unique = False
        self.dealloc_on_shared = False
        self.dealloc_backinv_unique = True
        self.dealloc_backinv_shared = True
        # Some reasonable default TBE params
        self.number_of_TBEs = 32
        self.number_of_repl_TBEs = 32
        self.number_of_snoop_TBEs = 16
        self.unify_repl_TBEs = False

class CHI_HNFController(CHI_Cache_Controller):
    '''
    Default parameters for a coherent home node (HNF) cache controller
    '''

    def __init__(self, ruby_system, cache, prefetcher, addr_ranges):
        super(CHI_HNFController, self).__init__(ruby_system)
        self.sequencer = NULL
        self.cache = cache
        self.use_prefetcher = False
        self.addr_ranges = addr_ranges
        self.allow_SD = True
        self.is_HN = True
        self.enable_DMT = True
        self.enable_DCT = True
        self.send_evictions = False
        # MOESI / Mostly inclusive for shared / Exclusive for unique
        self.alloc_on_seq_acc = False
        self.alloc_on_seq_line_write = False
        self.alloc_on_readshared = True
        self.alloc_on_readunique = False
        self.alloc_on_readonce = True
        self.alloc_on_writeback = True
        self.dealloc_on_unique = True
        self.dealloc_on_shared = False
        self.dealloc_backinv_unique = False
        self.dealloc_backinv_shared = False
        # Some reasonable default TBE params
        self.number_of_TBEs = 32
        self.number_of_repl_TBEs = 32
        self.number_of_snoop_TBEs = 1 # should not receive any snoop
        self.unify_repl_TBEs = False

class CHI_DMAController(CHI_Cache_Controller):
    '''
    Default parameters for a DMA controller
    '''

    def __init__(self, ruby_system, sequencer):
        super(CHI_DMAController, self).__init__(ruby_system)
        self.sequencer = sequencer
        class DummyCache(RubyCache):
            dataAccessLatency = 0
            tagAccessLatency = 1
            size = "128"
            assoc = 1
        self.use_prefetcher = False
        self.cache = DummyCache()
        self.sequencer.dcache = NULL
        # All allocations are false
        # Deallocations are true (don't really matter)
        self.allow_SD = False
        self.is_HN = False
        self.enable_DMT = False
        self.enable_DCT = False
        self.alloc_on_seq_acc = False
        self.alloc_on_seq_line_write = False
        self.alloc_on_readshared = False
        self.alloc_on_readunique = False
        self.alloc_on_readonce = False
        self.alloc_on_writeback = False
        self.dealloc_on_unique = False
        self.dealloc_on_shared = False
        self.dealloc_backinv_unique = False
        self.dealloc_backinv_shared = False
        self.send_evictions = False
        self.number_of_TBEs = 16
        self.number_of_repl_TBEs = 1
        self.number_of_snoop_TBEs = 1 # should not receive any snoop
        self.unify_repl_TBEs = False

class CPUSequencerWrapper:
    '''
    Other generic configuration scripts assume a matching number of sequencers
    and cpus. This wraps the instruction and data sequencer so they are
    compatible with the other scripts. This assumes all scripts are using
    connectCpuPorts/connectIOPorts to bind ports
    '''

    def __init__(self, iseq, dseq):
        # use this style due to __setattr__ override below
        self.__dict__['inst_seq'] = iseq
        self.__dict__['data_seq'] = dseq
        self.__dict__['support_data_reqs'] = True
        self.__dict__['support_inst_reqs'] = True
        # Compatibility with certain scripts that wire up ports
        # without connectCpuPorts
        self.__dict__['slave'] = dseq.in_ports
        self.__dict__['in_ports'] = dseq.in_ports

    def connectCpuPorts(self, cpu):
        assert(isinstance(cpu, BaseCPU))
        cpu.icache_port = self.inst_seq.in_ports
        for p in cpu._cached_ports:
            if str(p) != 'icache_port':
                exec('cpu.%s = self.data_seq.in_ports' % p)
        cpu.connectUncachedPorts(self.data_seq)

    def connectIOPorts(self, piobus):
        self.data_seq.connectIOPorts(piobus)

    def __setattr__(self, name, value):
        setattr(self.inst_seq, name, value)
        setattr(self.data_seq, name, value)

class CHI_RNF(CHI_Node):
    '''
    Defines a CHI request node.
    Notice all contollers and sequencers are set as children of the cpus, so
    this object acts more like a proxy for seting things up and has no topology
    significance unless the cpus are set as its children at the top level
    '''
    def __init__(self, cpus, ruby_system,
                 l1Icache_type, l1Dcache_type,
                 cache_line_size,
                 l1Iprefetcher_type=None, l1Dprefetcher_type=None):
        super(CHI_RNF, self).__init__(ruby_system)

        self._block_size_bits = int(math.log(cache_line_size, 2))

        # All sequencers and controllers
        self._seqs = []
        self._cntrls = []

        # Last level controllers in this node, i.e., the ones that will send
        # requests to the home nodes
        self._ll_cntrls = []

        self._cpus = cpus

        # First creates L1 caches and sequencers
        for cpu in self._cpus:
            cpu.inst_sequencer = RubySequencer(version = Versions.getSeqId(),
                                         ruby_system = ruby_system)
            cpu.data_sequencer = RubySequencer(version = Versions.getSeqId(),
                                         ruby_system = ruby_system)

            self._seqs.append(CPUSequencerWrapper(cpu.inst_sequencer,
                                                  cpu.data_sequencer))

            # caches
            l1i_cache = l1Icache_type(start_index_bit = self._block_size_bits,
                                      is_icache = True)

            l1d_cache = l1Dcache_type(start_index_bit = self._block_size_bits,
                                      is_icache = False)

            # Placeholders for future prefetcher support
            if l1Iprefetcher_type != None or l1Dprefetcher_type != None:
                m5.fatal('Prefetching not supported yet')
            l1i_pf = NULL
            l1d_pf = NULL

            # cache controllers
            cpu.l1i = CHI_L1Controller(ruby_system, cpu.inst_sequencer,
                                       l1i_cache, l1i_pf)

            cpu.l1d = CHI_L1Controller(ruby_system, cpu.data_sequencer,
                                       l1d_cache, l1d_pf)

            cpu.inst_sequencer.dcache = NULL
            cpu.data_sequencer.dcache = cpu.l1d.cache

            cpu.l1d.sc_lock_enabled = True

            cpu._ll_cntrls = [cpu.l1i, cpu.l1d]
            for c in cpu._ll_cntrls:
                self._cntrls.append(c)
                self.connectController(c)
                self._ll_cntrls.append(c)

    def getSequencers(self):
        return self._seqs

    def getAllControllers(self):
        return self._cntrls

    def getNetworkSideControllers(self):
        return self._cntrls

    def setDownstream(self, cntrls):
        for c in self._ll_cntrls:
            c.downstream_destinations = cntrls

    def getCpus(self):
        return self._cpus

    # Adds a private L2 for each cpu
    def addPrivL2Cache(self, cache_type, pf_type=None):
        self._ll_cntrls = []
        for cpu in self._cpus:
            l2_cache = cache_type(start_index_bit = self._block_size_bits,
                                  is_icache = False)
            if pf_type != None:
                m5.fatal('Prefetching not supported yet')
            l2_pf = NULL

            cpu.l2 = CHI_L2Controller(self._ruby_system, l2_cache, l2_pf)

            self._cntrls.append(cpu.l2)
            self.connectController(cpu.l2)

            self._ll_cntrls.append(cpu.l2)

            for c in cpu._ll_cntrls:
                c.downstream_destinations = [cpu.l2]
            cpu._ll_cntrls = [cpu.l2]


class CHI_HNF(CHI_Node):
    '''
    Encapsulates an HNF cache/directory controller.
    Before the first controller is created, the class method
    CHI_HNF.createAddrRanges must be called before creating any CHI_HNF object
    to set-up the interleaved address ranges used by the HNFs
    '''

    _addr_ranges = []
    @classmethod
    def createAddrRanges(cls, sys_mem_ranges, cache_line_size, num_hnfs):
        # Create the HNFs interleaved addr ranges
        block_size_bits = int(math.log(cache_line_size, 2))
        cls._addr_ranges = []
        llc_bits = int(math.log(num_hnfs, 2))
        numa_bit = block_size_bits + llc_bits - 1
        for i in range(num_hnfs):
            ranges = []
            for r in sys_mem_ranges:
                addr_range = AddrRange(r.start, size = r.size(),
                                        intlvHighBit = numa_bit,
                                        intlvBits = llc_bits,
                                        intlvMatch = i)
                ranges.append(addr_range)
            cls._addr_ranges.append((ranges, numa_bit, i))

    @classmethod
    def getAddrRanges(cls, hnf_idx):
        assert(len(cls._addr_ranges) != 0)
        return cls._addr_ranges[hnf_idx]

    # The CHI controller can be a child of this object or another if
    # 'parent' if specified
    def __init__(self, hnf_idx, ruby_system, llcache_type, parent):
        super(CHI_HNF, self).__init__(ruby_system)

        addr_ranges,intlvHighBit,intlvMatch = CHI_HNF.getAddrRanges(hnf_idx)
        # All ranges should have the same interleaving
        assert(len(addr_ranges) >= 1)
        assert(intlvMatch == hnf_idx)

        ll_cache = llcache_type(start_index_bit = intlvHighBit + 1)
        self._cntrl = CHI_HNFController(ruby_system, ll_cache, NULL,
                                        addr_ranges)

        if parent == None:
            self.cntrl = self._cntrl
        else:
            parent.cntrl = self._cntrl

        self.connectController(self._cntrl)

    def getAllControllers(self):
        return [self._cntrl]

    def getNetworkSideControllers(self):
        return [self._cntrl]


class CHI_SNF_Base(CHI_Node):
    '''
    Creates CHI node controllers for the memory controllers
    '''

    # The CHI controller can be a child of this object or another if
    # 'parent' if specified
    def __init__(self, ruby_system, parent):
        super(CHI_SNF_Base, self).__init__(ruby_system)

        self._cntrl = Memory_Controller(
                          version = Versions.getVersion(Memory_Controller),
                          ruby_system = ruby_system,
                          triggerQueue = TriggerMessageBuffer(),
                          responseFromMemory = MessageBuffer(),
                          requestToMemory = MessageBuffer(ordered = True),
                          reqRdy = TriggerMessageBuffer())

        self.connectController(self._cntrl)

        if parent:
            parent.cntrl = self._cntrl
        else:
            self.cntrl = self._cntrl

    def getAllControllers(self):
        return [self._cntrl]

    def getNetworkSideControllers(self):
        return [self._cntrl]

    def getMemRange(self, mem_ctrl):
        # TODO need some kind of transparent API for
        # MemCtrl+DRAM vs SimpleMemory
        if hasattr(mem_ctrl, 'range'):
            return mem_ctrl.range
        else:
            return mem_ctrl.dram.range

class CHI_SNF_BootMem(CHI_SNF_Base):
    '''
    Create the SNF for the boot memory
    '''
    def __init__(self, ruby_system, parent, bootmem):
        super(CHI_SNF_BootMem, self).__init__(ruby_system, parent)
        self._cntrl.memory_out_port = bootmem.port
        self._cntrl.addr_ranges = self.getMemRange(bootmem)

class CHI_SNF_MainMem(CHI_SNF_Base):
    '''
    Create the SNF for a list main memory controllers
    '''
    def __init__(self, ruby_system, parent, mem_ctrl = None):
        super(CHI_SNF_MainMem, self).__init__(ruby_system, parent)
        if mem_ctrl:
            self._cntrl.memory_out_port = mem_ctrl.port
            self._cntrl.addr_ranges = self.getMemRange(mem_ctrl)
        # else bind ports and range later

class CHI_RNI_Base(CHI_Node):
    '''
    Request node without cache / DMA
    '''

    # The CHI controller can be a child of this object or another if
    # 'parent' if specified
    def __init__(self, ruby_system, parent):
        super(CHI_RNI_Base, self).__init__(ruby_system)

        self._sequencer = RubySequencer(version = Versions.getSeqId(),
                                         ruby_system = ruby_system,
                                         clk_domain = ruby_system.clk_domain)
        self._cntrl = CHI_DMAController(ruby_system, self._sequencer)

        if parent:
            parent.cntrl = self._cntrl
        else:
            self.cntrl = self._cntrl

        self.connectController(self._cntrl)

    def getAllControllers(self):
        return [self._cntrl]

    def getNetworkSideControllers(self):
        return [self._cntrl]

class CHI_RNI_DMA(CHI_RNI_Base):
    '''
    DMA controller wiredup to a given dma port
    '''
    def __init__(self, ruby_system, dma_port, parent):
        super(CHI_RNI_DMA, self).__init__(ruby_system, parent)
        assert(dma_port != None)
        self._sequencer.in_ports = dma_port

class CHI_RNI_IO(CHI_RNI_Base):
    '''
    DMA controller wiredup to ruby_system IO port
    '''
    def __init__(self, ruby_system, parent):
        super(CHI_RNI_IO, self).__init__(ruby_system, parent)
        ruby_system._io_port = self._sequencer

def noc_params_from_config(config, noc_params):
    # mesh options
    noc_params.num_rows = config['mesh']['num_rows']
    noc_params.num_cols = config['mesh']['num_cols']
    if 'router_latency' in config['mesh']:
        noc_params.router_latency = config['mesh']['router_latency']
    if 'link_latency' in config['mesh']:
        noc_params.router_link_latency = config['mesh']['link_latency']
        noc_params.node_link_latency = config['mesh']['link_latency']
    if 'router_link_latency' in config['mesh']:
        noc_params.router_link_latency = config['mesh']['router_link_latency']
    if 'node_link_latency' in config['mesh']:
        noc_params.node_link_latency = config['mesh']['node_link_latency']
    if 'cross_links' in config['mesh']:
        noc_params.cross_link_latency = \
                                config['mesh']['cross_link_latency']
        noc_params.cross_links = []
        for x, y in config['mesh']['cross_links']:
            noc_params.cross_links.append((x, y))
            noc_params.cross_links.append((y, x))
    else:
        noc_params.cross_links = []
        noc_params.cross_link_latency = 0

    # CHI_RNF options
    noc_params.CHI_RNF = config['CHI_RNF']

    # CHI_RNI_IO
    noc_params.CHI_RNI_IO = config['CHI_RNI_IO']

    # CHI_HNF options
    noc_params.CHI_HNF = config['CHI_HNF']
    if 'pairing' in config['CHI_HNF']:
        noc_params.pairing = config['CHI_HNF']['pairing']

    # CHI_SNF_MainMem
    noc_params.CHI_SNF_MainMem = config['CHI_SNF_MainMem']

    # CHI_SNF_IO (applies to CHI_SNF_Bootmem)
    noc_params.CHI_SNF_IO = config['CHI_SNF_IO']


def create_system(options, full_system, system, dma_ports, bootmem,
                  ruby_system):

    if buildEnv['PROTOCOL'] != 'CHI':
        m5.panic("This script requires the CHI build")

    if options.num_dirs < 1:
        m5.fatal('--num-dirs must be at least 1')

    if options.num_l3caches < 1:
        m5.fatal('--num-l3caches must be at least 1')

    # Default parameters for the network
    class NoC_Params(object):
        def __init__(self):
            self.topology = options.topology
            self.network = options.network
            self.router_link_latency = 1
            self.node_link_latency = 1
            self.router_latency = 1
            self.router_buffer_size = 4
            self.cntrl_msg_size = 8
            self.data_width = 32
    params = NoC_Params()

    # read additional configurations from yaml file if provided
    if options.noc_config:
        with open(options.noc_config, 'r') as file:
            noc_params_from_config(yaml.load(file), params)
    elif params.topology == 'CustomMesh':
        m5.fatal('--noc-config must be provided if topology is CustomMesh')

    # Declare caches and controller types used by the protocol
    # Notice tag and data accesses are not concurrent, so the a cache hit
    # latency = tag + data + response latencies.
    # Default response latencies are 1 cy for all controllers.
    # For L1 controllers the mandatoryQueue enqueue latency is always 1 cy and
    # this is deducted from the initial tag read latency for sequencer requests
    # dataAccessLatency may be set to 0 if one wants to consider parallel
    # data and tag lookups
    class L1ICache(RubyCache):
        dataAccessLatency = 1
        tagAccessLatency = 1
        size = options.l1i_size
        assoc = options.l1i_assoc

    class L1DCache(RubyCache):
        dataAccessLatency = 2
        tagAccessLatency = 1
        size = options.l1d_size
        assoc = options.l1d_assoc

    class L2Cache(RubyCache):
        dataAccessLatency = 6
        tagAccessLatency = 2
        size = options.l2_size
        assoc = options.l2_assoc

    class HNFCache(RubyCache):
        dataAccessLatency = 10
        tagAccessLatency = 2
        size = options.l3_size
        assoc = options.l3_assoc

    # other functions use system.cache_line_size assuming it has been set
    assert(system.cache_line_size.value == options.cacheline_size)

    cpu_sequencers = []
    mem_cntrls = []
    mem_dests = []
    network_nodes = []
    network_cntrls = []
    hnf_dests = []
    all_cntrls = []

    # Creates on RNF per cpu with priv l2 caches
    assert(len(system.cpu) == options.num_cpus)
    ruby_system.rnf = [ CHI_RNF([cpu], ruby_system, L1ICache, L1DCache,
                                system.cache_line_size.value)
                        for cpu in system.cpu ]
    for rnf in ruby_system.rnf:
        rnf.addPrivL2Cache(L2Cache)
        cpu_sequencers.extend(rnf.getSequencers())
        all_cntrls.extend(rnf.getAllControllers())
        network_nodes.append(rnf)
        network_cntrls.extend(rnf.getNetworkSideControllers())

    # Look for other memories
    other_memories = []
    if bootmem:
        other_memories.append(bootmem)
    if getattr(system, 'sram', None):
        other_memories.append(getattr(system, 'sram', None))
    on_chip_mem_ports = getattr(system, '_on_chip_mem_ports', None)
    if on_chip_mem_ports:
        other_memories.extend([p.simobj for p in on_chip_mem_ports])

    # Create the LLCs cntrls
    sysranges = [] + system.mem_ranges

    for m in other_memories:
        sysranges.append(m.range)

    CHI_HNF.createAddrRanges(sysranges, system.cache_line_size.value,
                             options.num_l3caches)
    ruby_system.hnf = [ CHI_HNF(i, ruby_system, HNFCache, None)
                        for i in range(options.num_l3caches) ]

    for hnf in ruby_system.hnf:
        network_nodes.append(hnf)
        network_cntrls.extend(hnf.getNetworkSideControllers())
        assert(hnf.getAllControllers() == hnf.getNetworkSideControllers())
        all_cntrls.extend(hnf.getAllControllers())
        hnf_dests.extend(hnf.getAllControllers())

    # Create the memory controllers
    # Notice we don't define a Directory_Controller type so we don't use
    # create_directories shared by other protocols.

    ruby_system.snf = [ CHI_SNF_MainMem(ruby_system, None, None)
                        for i in range(options.num_dirs) ]
    for snf in ruby_system.snf:
        network_nodes.append(snf)
        network_cntrls.extend(snf.getNetworkSideControllers())
        assert(snf.getAllControllers() == snf.getNetworkSideControllers())
        mem_cntrls.extend(snf.getAllControllers())
        all_cntrls.extend(snf.getAllControllers())
        mem_dests.extend(snf.getAllControllers())

    if len(other_memories) > 0:
        ruby_system.rom_snf = [ CHI_SNF_BootMem(ruby_system, None, m)
                                 for m in other_memories ]
        for snf in ruby_system.rom_snf:
            network_nodes.append(snf)
            network_cntrls.extend(snf.getNetworkSideControllers())
            all_cntrls.extend(snf.getAllControllers())
            mem_dests.extend(snf.getAllControllers())


    # Creates the controller for dma ports and io

    if len(dma_ports) > 0:
        ruby_system.dma_rni = [ CHI_RNI_DMA(ruby_system, dma_port, None)
                                for dma_port in dma_ports ]
        for rni in ruby_system.dma_rni:
            network_nodes.append(rni)
            network_cntrls.extend(rni.getNetworkSideControllers())
            all_cntrls.extend(rni.getAllControllers())

    if full_system:
        ruby_system.io_rni = CHI_RNI_IO(ruby_system, None)
        network_nodes.append(ruby_system.io_rni)
        network_cntrls.extend(ruby_system.io_rni.getNetworkSideControllers())
        all_cntrls.extend(ruby_system.io_rni.getAllControllers())


    # Assign downstream destinations
    for rnf in ruby_system.rnf:
        rnf.setDownstream(hnf_dests)
    if len(dma_ports) > 0:
        for rni in ruby_system.dma_rni:
            rni.setDownstream(hnf_dests)
    if full_system:
        ruby_system.io_rni.setDownstream(hnf_dests)
    for hnf in ruby_system.hnf:
        hnf.setDownstream(mem_dests)

    # Setup data message size for all controllers
    for cntrl in all_cntrls:
        cntrl.data_channel_size = params.data_width

    # Network configurations
    # virtual networks: 0=request, 1=snoop, 2=response, 3=data
    ruby_system.network.number_of_virtual_networks = 4

    ruby_system.network.control_msg_size = params.cntrl_msg_size
    ruby_system.network.data_msg_size = params.data_width
    ruby_system.network.buffer_size = params.router_buffer_size

    if params.topology == 'CustomMesh':
        topology = create_topology(network_nodes, params)
    elif params.topology in ['Crossbar', 'Pt2Pt']:
        topology = create_topology(network_cntrls, params)
    else:
        m5.fatal("%s not supported!" % params.topology)

    # Incorporate the params into options so it's propagated to
    # makeTopology by the parent script
    for k in dir(params):
        if not k.startswith('__'):
            setattr(options, k, getattr(params, k))

    return (cpu_sequencers, mem_cntrls, topology)
