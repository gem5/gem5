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

import m5
from m5.objects import *
from m5.defines import buildEnv
from .Ruby import create_topology


def define_options(parser):
    parser.add_argument(
        "--chi-config",
        action="store",
        type=str,
        default=None,
        help="NoC config. parameters and bindings. "
        "Required for CustomMesh topology",
    )
    parser.add_argument("--enable-dvm", default=False, action="store_true")


def read_config_file(file):
    """Read file as a module and return it"""
    import types
    import importlib.machinery

    loader = importlib.machinery.SourceFileLoader("chi_configs", file)
    chi_configs = types.ModuleType(loader.name)
    loader.exec_module(chi_configs)
    return chi_configs


def create_system(
    options, full_system, system, dma_ports, bootmem, ruby_system, cpus
):

    if buildEnv["PROTOCOL"] != "CHI":
        m5.panic("This script requires the CHI build")

    if options.num_dirs < 1:
        m5.fatal("--num-dirs must be at least 1")

    if options.num_l3caches < 1:
        m5.fatal("--num-l3caches must be at least 1")

    if full_system and options.enable_dvm:
        if len(cpus) <= 1:
            m5.fatal("--enable-dvm can't be used with a single CPU")
        for cpu in cpus:
            for decoder in cpu.decoder:
                decoder.dvm_enabled = True

    # read specialized classes from config file if provided
    if options.chi_config:
        chi_defs = read_config_file(options.chi_config)
    elif options.topology == "CustomMesh":
        m5.fatal("--noc-config must be provided if topology is CustomMesh")
    else:
        # Use the defaults from CHI_config
        from . import CHI_config as chi_defs

    # NoC params
    params = chi_defs.NoC_Params
    # Node types
    CHI_RNF = chi_defs.CHI_RNF
    CHI_HNF = chi_defs.CHI_HNF
    CHI_MN = chi_defs.CHI_MN
    CHI_SNF_MainMem = chi_defs.CHI_SNF_MainMem
    CHI_SNF_BootMem = chi_defs.CHI_SNF_BootMem
    CHI_RNI_DMA = chi_defs.CHI_RNI_DMA
    CHI_RNI_IO = chi_defs.CHI_RNI_IO

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
    assert system.cache_line_size.value == options.cacheline_size

    cpu_sequencers = []
    mem_cntrls = []
    mem_dests = []
    network_nodes = []
    network_cntrls = []
    hnf_dests = []
    all_cntrls = []

    # Creates on RNF per cpu with priv l2 caches
    assert len(cpus) == options.num_cpus
    ruby_system.rnf = [
        CHI_RNF(
            [cpu],
            ruby_system,
            L1ICache,
            L1DCache,
            system.cache_line_size.value,
        )
        for cpu in cpus
    ]
    for rnf in ruby_system.rnf:
        rnf.addPrivL2Cache(L2Cache)
        cpu_sequencers.extend(rnf.getSequencers())
        all_cntrls.extend(rnf.getAllControllers())
        network_nodes.append(rnf)
        network_cntrls.extend(rnf.getNetworkSideControllers())

    # Creates one Misc Node
    ruby_system.mn = [CHI_MN(ruby_system, [cpu.l1d for cpu in cpus])]
    for mn in ruby_system.mn:
        all_cntrls.extend(mn.getAllControllers())
        network_nodes.append(mn)
        network_cntrls.extend(mn.getNetworkSideControllers())
        assert mn.getAllControllers() == mn.getNetworkSideControllers()

    # Look for other memories
    other_memories = []
    if bootmem:
        other_memories.append(bootmem)
    if getattr(system, "sram", None):
        other_memories.append(getattr(system, "sram", None))
    on_chip_mem_ports = getattr(system, "_on_chip_mem_ports", None)
    if on_chip_mem_ports:
        other_memories.extend([p.simobj for p in on_chip_mem_ports])

    # Create the LLCs cntrls
    sysranges = [] + system.mem_ranges

    for m in other_memories:
        sysranges.append(m.range)

    hnf_list = [i for i in range(options.num_l3caches)]
    CHI_HNF.createAddrRanges(sysranges, system.cache_line_size.value, hnf_list)
    ruby_system.hnf = [
        CHI_HNF(i, ruby_system, HNFCache, None)
        for i in range(options.num_l3caches)
    ]

    for hnf in ruby_system.hnf:
        network_nodes.append(hnf)
        network_cntrls.extend(hnf.getNetworkSideControllers())
        assert hnf.getAllControllers() == hnf.getNetworkSideControllers()
        all_cntrls.extend(hnf.getAllControllers())
        hnf_dests.extend(hnf.getAllControllers())

    # Create the memory controllers
    # Notice we don't define a Directory_Controller type so we don't use
    # create_directories shared by other protocols.

    ruby_system.snf = [
        CHI_SNF_MainMem(ruby_system, None, None)
        for i in range(options.num_dirs)
    ]
    for snf in ruby_system.snf:
        network_nodes.append(snf)
        network_cntrls.extend(snf.getNetworkSideControllers())
        assert snf.getAllControllers() == snf.getNetworkSideControllers()
        mem_cntrls.extend(snf.getAllControllers())
        all_cntrls.extend(snf.getAllControllers())
        mem_dests.extend(snf.getAllControllers())

    if len(other_memories) > 0:
        ruby_system.rom_snf = [
            CHI_SNF_BootMem(ruby_system, None, m) for m in other_memories
        ]
        for snf in ruby_system.rom_snf:
            network_nodes.append(snf)
            network_cntrls.extend(snf.getNetworkSideControllers())
            all_cntrls.extend(snf.getAllControllers())
            mem_dests.extend(snf.getAllControllers())

    # Creates the controller for dma ports and io

    if len(dma_ports) > 0:
        ruby_system.dma_rni = [
            CHI_RNI_DMA(ruby_system, dma_port, None) for dma_port in dma_ports
        ]
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
    if options.network == "simple":
        ruby_system.network.buffer_size = params.router_buffer_size

    # Incorporate the params into options so it's propagated to
    # makeTopology and create_topology the parent scripts
    for k in dir(params):
        if not k.startswith("__"):
            setattr(options, k, getattr(params, k))

    if options.topology == "CustomMesh":
        topology = create_topology(network_nodes, options)
    elif options.topology in ["Crossbar", "Pt2Pt"]:
        topology = create_topology(network_cntrls, options)
    else:
        m5.fatal("%s not supported!" % options.topology)

    return (cpu_sequencers, mem_cntrls, topology)
