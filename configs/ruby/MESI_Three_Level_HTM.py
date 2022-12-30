# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2009,2015 Advanced Micro Devices, Inc.
# Copyright (c) 2013 Mark D. Hill and David A. Wood
# Copyright (c) 2020 ARM Limited
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

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from .Ruby import create_topology, create_directories
from .Ruby import send_evicts
from common import FileSystemConfig

#
# Declare caches used by the protocol
#
class L0Cache(RubyCache):
    pass


class L1Cache(RubyCache):
    pass


class L2Cache(RubyCache):
    pass


def define_options(parser):
    parser.add_argument(
        "--num-clusters",
        type=int,
        default=1,
        help="number of clusters in a design in which there are shared\
        caches private to clusters",
    )
    parser.add_argument("--l0i_size", type=str, default="4096B")
    parser.add_argument("--l0d_size", type=str, default="4096B")
    parser.add_argument("--l0i_assoc", type=int, default=1)
    parser.add_argument("--l0d_assoc", type=int, default=1)
    parser.add_argument("--l0_transitions_per_cycle", type=int, default=32)
    parser.add_argument("--l1_transitions_per_cycle", type=int, default=32)
    parser.add_argument("--l2_transitions_per_cycle", type=int, default=4)
    parser.add_argument(
        "--enable-prefetch",
        action="store_true",
        default=False,
        help="Enable Ruby hardware prefetcher",
    )
    return


def create_system(
    options, full_system, system, dma_ports, bootmem, ruby_system, cpus
):

    if buildEnv["PROTOCOL"] != "MESI_Three_Level_HTM":
        fatal(
            "This script requires the MESI_Three_Level protocol to be\
               built."
        )

    cpu_sequencers = []

    #
    # The ruby network creation expects the list of nodes in the system to be
    # consistent with the NetDest list.  Therefore the l1 controller nodes
    # must be listed before the directory nodes and directory nodes before
    # dma nodes, etc.
    #
    l0_cntrl_nodes = []
    l1_cntrl_nodes = []
    l2_cntrl_nodes = []
    dma_cntrl_nodes = []

    assert options.num_cpus % options.num_clusters == 0
    num_cpus_per_cluster = options.num_cpus // options.num_clusters

    assert options.num_l2caches % options.num_clusters == 0
    num_l2caches_per_cluster = options.num_l2caches // options.num_clusters

    l2_bits = int(math.log(num_l2caches_per_cluster, 2))
    block_size_bits = int(math.log(options.cacheline_size, 2))
    l2_index_start = block_size_bits + l2_bits

    #
    # Must create the individual controllers before the network to ensure the
    # controller constructors are called before the network constructor
    #
    for i in range(options.num_clusters):
        for j in range(num_cpus_per_cluster):
            #
            # First create the Ruby objects associated with this cpu
            #
            l0i_cache = L0Cache(
                size=options.l0i_size,
                assoc=options.l0i_assoc,
                is_icache=True,
                start_index_bit=block_size_bits,
                replacement_policy=LRURP(),
            )

            l0d_cache = L0Cache(
                size=options.l0d_size,
                assoc=options.l0d_assoc,
                is_icache=False,
                start_index_bit=block_size_bits,
                replacement_policy=LRURP(),
            )

            clk_domain = cpus[i].clk_domain

            # Ruby prefetcher
            prefetcher = RubyPrefetcher(
                num_streams=16,
                unit_filter=256,
                nonunit_filter=256,
                train_misses=5,
                num_startup_pfs=4,
                cross_page=True,
            )

            l0_cntrl = L0Cache_Controller(
                version=i * num_cpus_per_cluster + j,
                Icache=l0i_cache,
                Dcache=l0d_cache,
                transitions_per_cycle=options.l0_transitions_per_cycle,
                prefetcher=prefetcher,
                enable_prefetch=options.enable_prefetch,
                send_evictions=send_evicts(options),
                clk_domain=clk_domain,
                ruby_system=ruby_system,
            )

            cpu_seq = RubyHTMSequencer(
                version=i * num_cpus_per_cluster + j,
                clk_domain=clk_domain,
                dcache=l0d_cache,
                ruby_system=ruby_system,
            )

            l0_cntrl.sequencer = cpu_seq

            l1_cache = L1Cache(
                size=options.l1d_size,
                assoc=options.l1d_assoc,
                start_index_bit=block_size_bits,
                is_icache=False,
            )

            l1_cntrl = L1Cache_Controller(
                version=i * num_cpus_per_cluster + j,
                cache=l1_cache,
                l2_select_num_bits=l2_bits,
                cluster_id=i,
                transitions_per_cycle=options.l1_transitions_per_cycle,
                ruby_system=ruby_system,
            )

            exec(
                "ruby_system.l0_cntrl%d = l0_cntrl"
                % (i * num_cpus_per_cluster + j)
            )
            exec(
                "ruby_system.l1_cntrl%d = l1_cntrl"
                % (i * num_cpus_per_cluster + j)
            )

            #
            # Add controllers and sequencers to the appropriate lists
            #
            cpu_sequencers.append(cpu_seq)
            l0_cntrl_nodes.append(l0_cntrl)
            l1_cntrl_nodes.append(l1_cntrl)

            # Connect the L0 and L1 controllers
            l0_cntrl.prefetchQueue = MessageBuffer()
            l0_cntrl.mandatoryQueue = MessageBuffer()
            l0_cntrl.bufferToL1 = MessageBuffer(ordered=True)
            l1_cntrl.bufferFromL0 = l0_cntrl.bufferToL1
            l0_cntrl.bufferFromL1 = MessageBuffer(ordered=True)
            l1_cntrl.bufferToL0 = l0_cntrl.bufferFromL1

            # Connect the L1 controllers and the network
            l1_cntrl.requestToL2 = MessageBuffer()
            l1_cntrl.requestToL2.out_port = ruby_system.network.in_port
            l1_cntrl.responseToL2 = MessageBuffer()
            l1_cntrl.responseToL2.out_port = ruby_system.network.in_port
            l1_cntrl.unblockToL2 = MessageBuffer()
            l1_cntrl.unblockToL2.out_port = ruby_system.network.in_port

            l1_cntrl.requestFromL2 = MessageBuffer()
            l1_cntrl.requestFromL2.in_port = ruby_system.network.out_port
            l1_cntrl.responseFromL2 = MessageBuffer()
            l1_cntrl.responseFromL2.in_port = ruby_system.network.out_port

        for j in range(num_l2caches_per_cluster):
            l2_cache = L2Cache(
                size=options.l2_size,
                assoc=options.l2_assoc,
                start_index_bit=l2_index_start,
            )

            l2_cntrl = L2Cache_Controller(
                version=i * num_l2caches_per_cluster + j,
                L2cache=l2_cache,
                cluster_id=i,
                transitions_per_cycle=options.l2_transitions_per_cycle,
                ruby_system=ruby_system,
            )

            exec(
                "ruby_system.l2_cntrl%d = l2_cntrl"
                % (i * num_l2caches_per_cluster + j)
            )
            l2_cntrl_nodes.append(l2_cntrl)

            # Connect the L2 controllers and the network
            l2_cntrl.DirRequestFromL2Cache = MessageBuffer()
            l2_cntrl.DirRequestFromL2Cache.out_port = (
                ruby_system.network.in_port
            )
            l2_cntrl.L1RequestFromL2Cache = MessageBuffer()
            l2_cntrl.L1RequestFromL2Cache.out_port = (
                ruby_system.network.in_port
            )
            l2_cntrl.responseFromL2Cache = MessageBuffer()
            l2_cntrl.responseFromL2Cache.out_port = ruby_system.network.in_port

            l2_cntrl.unblockToL2Cache = MessageBuffer()
            l2_cntrl.unblockToL2Cache.in_port = ruby_system.network.out_port
            l2_cntrl.L1RequestToL2Cache = MessageBuffer()
            l2_cntrl.L1RequestToL2Cache.in_port = ruby_system.network.out_port
            l2_cntrl.responseToL2Cache = MessageBuffer()
            l2_cntrl.responseToL2Cache.in_port = ruby_system.network.out_port

    # Run each of the ruby memory controllers at a ratio of the frequency of
    # the ruby system
    # clk_divider value is a fix to pass regression.
    ruby_system.memctrl_clk_domain = DerivedClockDomain(
        clk_domain=ruby_system.clk_domain, clk_divider=3
    )

    mem_dir_cntrl_nodes, rom_dir_cntrl_node = create_directories(
        options, bootmem, ruby_system, system
    )
    dir_cntrl_nodes = mem_dir_cntrl_nodes[:]
    if rom_dir_cntrl_node is not None:
        dir_cntrl_nodes.append(rom_dir_cntrl_node)
    for dir_cntrl in dir_cntrl_nodes:
        # Connect the directory controllers and the network
        dir_cntrl.requestToDir = MessageBuffer()
        dir_cntrl.requestToDir.in_port = ruby_system.network.out_port
        dir_cntrl.responseToDir = MessageBuffer()
        dir_cntrl.responseToDir.in_port = ruby_system.network.out_port
        dir_cntrl.responseFromDir = MessageBuffer()
        dir_cntrl.responseFromDir.out_port = ruby_system.network.in_port
        dir_cntrl.requestToMemory = MessageBuffer()
        dir_cntrl.responseFromMemory = MessageBuffer()

    for i, dma_port in enumerate(dma_ports):
        #
        # Create the Ruby objects associated with the dma controller
        #
        dma_seq = DMASequencer(version=i, ruby_system=ruby_system)

        dma_cntrl = DMA_Controller(
            version=i,
            dma_sequencer=dma_seq,
            transitions_per_cycle=options.ports,
            ruby_system=ruby_system,
        )

        exec("ruby_system.dma_cntrl%d = dma_cntrl" % i)
        exec("ruby_system.dma_cntrl%d.dma_sequencer.in_ports = dma_port" % i)
        dma_cntrl_nodes.append(dma_cntrl)

        # Connect the dma controller to the network
        dma_cntrl.mandatoryQueue = MessageBuffer()
        dma_cntrl.responseFromDir = MessageBuffer(ordered=True)
        dma_cntrl.responseFromDir.in_port = ruby_system.network.out_port
        dma_cntrl.requestToDir = MessageBuffer()
        dma_cntrl.requestToDir.out_port = ruby_system.network.in_port

    all_cntrls = (
        l0_cntrl_nodes
        + l1_cntrl_nodes
        + l2_cntrl_nodes
        + dir_cntrl_nodes
        + dma_cntrl_nodes
    )

    # Create the io controller and the sequencer
    if full_system:
        io_seq = DMASequencer(version=len(dma_ports), ruby_system=ruby_system)
        ruby_system._io_port = io_seq
        io_controller = DMA_Controller(
            version=len(dma_ports),
            dma_sequencer=io_seq,
            ruby_system=ruby_system,
        )
        ruby_system.io_controller = io_controller

        # Connect the dma controller to the network
        io_controller.mandatoryQueue = MessageBuffer()
        io_controller.responseFromDir = MessageBuffer(ordered=True)
        io_controller.responseFromDir.in_port = ruby_system.network.out_port
        io_controller.requestToDir = MessageBuffer()
        io_controller.requestToDir.out_port = ruby_system.network.in_port

        all_cntrls = all_cntrls + [io_controller]
    # Register configuration with filesystem
    else:
        for i in range(options.num_clusters):
            for j in range(num_cpus_per_cluster):
                FileSystemConfig.register_cpu(
                    physical_package_id=0,
                    core_siblings=range(options.num_cpus),
                    core_id=i * num_cpus_per_cluster + j,
                    thread_siblings=[],
                )

                FileSystemConfig.register_cache(
                    level=0,
                    idu_type="Instruction",
                    size=options.l0i_size,
                    line_size=options.cacheline_size,
                    assoc=1,
                    cpus=[i * num_cpus_per_cluster + j],
                )
                FileSystemConfig.register_cache(
                    level=0,
                    idu_type="Data",
                    size=options.l0d_size,
                    line_size=options.cacheline_size,
                    assoc=1,
                    cpus=[i * num_cpus_per_cluster + j],
                )

                FileSystemConfig.register_cache(
                    level=1,
                    idu_type="Unified",
                    size=options.l1d_size,
                    line_size=options.cacheline_size,
                    assoc=options.l1d_assoc,
                    cpus=[i * num_cpus_per_cluster + j],
                )

            FileSystemConfig.register_cache(
                level=2,
                idu_type="Unified",
                size=str(
                    MemorySize(options.l2_size) * num_l2caches_per_cluster
                )
                + "B",
                line_size=options.cacheline_size,
                assoc=options.l2_assoc,
                cpus=[
                    n
                    for n in range(
                        i * num_cpus_per_cluster,
                        (i + 1) * num_cpus_per_cluster,
                    )
                ],
            )

    ruby_system.network.number_of_virtual_networks = 3
    topology = create_topology(all_cntrls, options)
    return (cpu_sequencers, mem_dir_cntrl_nodes, topology)
