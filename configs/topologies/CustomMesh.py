# Copyright (c) 2021,2022 ARM Limited
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
#
import math

from m5.defines import buildEnv
from m5.objects import *
from m5.params import *
from m5.util import fatal

if buildEnv["PROTOCOL"] == "CHI":
    import ruby.CHI_config as CHI

from topologies.BaseTopology import SimpleTopology


class CustomMesh(SimpleTopology):
    description = "CustomMesh"

    def __init__(self, controllers):
        self.nodes = controllers

    # --------------------------------------------------------------------------
    # _makeMesh
    # --------------------------------------------------------------------------

    def _makeMesh(
        self,
        IntLink,
        link_latency,
        num_rows,
        num_columns,
        cross_links,
        cross_link_latency,
    ):
        # East->West, West->East, North->South, South->North
        # XY routing weights
        link_weights = [1, 1, 2, 2]

        # East output to West input links
        for row in range(num_rows):
            for col in range(num_columns):
                if col + 1 < num_columns:
                    east_out = col + (row * num_columns)
                    west_in = (col + 1) + (row * num_columns)
                    llat = (
                        cross_link_latency
                        if (east_out, west_in) in cross_links
                        else link_latency
                    )
                    self._int_links.append(
                        IntLink(
                            link_id=self._link_count,
                            src_node=self._routers[east_out],
                            dst_node=self._routers[west_in],
                            dst_inport="West",
                            latency=llat,
                            weight=link_weights[0],
                        )
                    )
                    self._link_count += 1

        # West output to East input links
        for row in range(num_rows):
            for col in range(num_columns):
                if col + 1 < num_columns:
                    east_in = col + (row * num_columns)
                    west_out = (col + 1) + (row * num_columns)
                    llat = (
                        cross_link_latency
                        if (west_out, east_in) in cross_links
                        else link_latency
                    )
                    self._int_links.append(
                        IntLink(
                            link_id=self._link_count,
                            src_node=self._routers[west_out],
                            dst_node=self._routers[east_in],
                            dst_inport="East",
                            latency=llat,
                            weight=link_weights[1],
                        )
                    )
                    self._link_count += 1

        # North output to South input links
        for col in range(num_columns):
            for row in range(num_rows):
                if row + 1 < num_rows:
                    north_out = col + (row * num_columns)
                    south_in = col + ((row + 1) * num_columns)
                    llat = (
                        cross_link_latency
                        if (north_out, south_in) in cross_links
                        else link_latency
                    )
                    self._int_links.append(
                        IntLink(
                            link_id=self._link_count,
                            src_node=self._routers[north_out],
                            dst_node=self._routers[south_in],
                            dst_inport="South",
                            latency=llat,
                            weight=link_weights[2],
                        )
                    )
                    self._link_count += 1

        # South output to North input links
        for col in range(num_columns):
            for row in range(num_rows):
                if row + 1 < num_rows:
                    north_in = col + (row * num_columns)
                    south_out = col + ((row + 1) * num_columns)
                    llat = (
                        cross_link_latency
                        if (south_out, north_in) in cross_links
                        else link_latency
                    )
                    self._int_links.append(
                        IntLink(
                            link_id=self._link_count,
                            src_node=self._routers[south_out],
                            dst_node=self._routers[north_in],
                            dst_inport="North",
                            latency=llat,
                            weight=link_weights[3],
                        )
                    )
                    self._link_count += 1

    # --------------------------------------------------------------------------
    # distributeNodes
    # --------------------------------------------------------------------------

    def _createRNFRouter(self, mesh_router):
        # Create a zero-latency router bridging node controllers
        # and the mesh router
        node_router = self._Router(
            router_id=len(self._routers), latency=self.node_router_latency
        )
        self._routers.append(node_router)

        # connect node_router <-> mesh router
        self._int_links.append(
            self._IntLink(
                link_id=self._link_count,
                src_node=node_router,
                dst_node=mesh_router,
                latency=self._router_link_latency,
            )
        )
        self._link_count += 1

        self._int_links.append(
            self._IntLink(
                link_id=self._link_count,
                src_node=mesh_router,
                dst_node=node_router,
                latency=self._router_link_latency,
            )
        )
        self._link_count += 1

        return node_router

    def distributeNodes(self, node_placement_config, node_list):
        if len(node_list) == 0:
            return

        num_nodes_per_router = node_placement_config.num_nodes_per_router
        router_idx_list = node_placement_config.router_list

        if num_nodes_per_router:
            # evenly distribute nodes to all listed routers
            assert len(router_idx_list) * num_nodes_per_router == len(
                node_list
            )

            for idx, node in enumerate(node_list):
                mesh_router_idx = router_idx_list[idx // num_nodes_per_router]
                router = self._routers[mesh_router_idx]

                # Create another router bridging RNF node controllers
                # and the mesh router
                # for non-RNF nodes, node router is mesh router
                if isinstance(node, CHI.CHI_RNF):
                    router = self._createRNFRouter(router)

                # connect all ctrls in the node to node_router
                ctrls = node.getNetworkSideControllers()
                for c in ctrls:
                    self._ext_links.append(
                        self._ExtLink(
                            link_id=self._link_count,
                            ext_node=c,
                            int_node=router,
                            latency=self._node_link_latency,
                        )
                    )
                    self._link_count += 1
        else:
            # try to circulate all nodes to all routers, some routers may be
            # connected to zero or more than one node.
            idx = 0
            for node in node_list:
                ridx = router_idx_list[idx]
                router = self._routers[ridx]

                if isinstance(node, CHI.CHI_RNF):
                    router = self._createRNFRouter(router)
                ctrls = node.getNetworkSideControllers()
                for c in ctrls:
                    self._ext_links.append(
                        self._ExtLink(
                            link_id=self._link_count,
                            ext_node=c,
                            int_node=router,
                            latency=self._node_link_latency,
                        )
                    )
                    self._link_count += 1
                idx = (idx + 1) % len(router_idx_list)

    # --------------------------------------------------------------------------
    # makeTopology
    # --------------------------------------------------------------------------

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        assert buildEnv["PROTOCOL"] == "CHI"

        num_rows = options.num_rows
        num_cols = options.num_cols
        num_mesh_routers = num_rows * num_cols

        self._IntLink = IntLink
        self._ExtLink = ExtLink
        self._Router = Router

        self.node_router_latency = 1 if options.network == "garnet" else 0
        if hasattr(options, "router_link_latency"):
            self._router_link_latency = options.router_link_latency
            self._node_link_latency = options.node_link_latency
        else:
            print("WARNING: router/node link latencies not provided")
            self._router_link_latency = options.link_latency
            self._node_link_latency = options.link_latency

        # classify nodes into different types
        rnf_nodes = []
        hnf_nodes = []
        mn_nodes = []
        mem_nodes = []
        io_mem_nodes = []
        rni_dma_nodes = []
        rni_io_nodes = []

        # Notice below that all the type must be the same for all nodes with
        # the same base type.
        rnf_params = None
        hnf_params = None
        mn_params = None
        mem_params = None
        io_mem_params = None
        rni_dma_params = None
        rni_io_params = None

        def check_same(val, curr):
            assert curr == None or curr == val
            return val

        for n in self.nodes:
            if isinstance(n, CHI.CHI_RNF):
                rnf_nodes.append(n)
                rnf_params = check_same(type(n).NoC_Params, rnf_params)
            elif isinstance(n, CHI.CHI_HNF):
                hnf_nodes.append(n)
                hnf_params = check_same(type(n).NoC_Params, hnf_params)
            elif isinstance(n, CHI.CHI_MN):
                mn_nodes.append(n)
                mn_params = check_same(type(n).NoC_Params, mn_params)
            elif isinstance(n, CHI.CHI_SNF_MainMem):
                mem_nodes.append(n)
                mem_params = check_same(type(n).NoC_Params, mem_params)
            elif isinstance(n, CHI.CHI_SNF_BootMem):
                io_mem_nodes.append(n)
                io_mem_params = check_same(type(n).NoC_Params, io_mem_params)
            elif isinstance(n, CHI.CHI_RNI_DMA):
                rni_dma_nodes.append(n)
                rni_dma_params = check_same(type(n).NoC_Params, rni_dma_params)
            elif isinstance(n, CHI.CHI_RNI_IO):
                rni_io_nodes.append(n)
                rni_io_params = check_same(type(n).NoC_Params, rni_io_params)
            else:
                fatal(
                    f"topologies.CustomMesh: {n.__class__.__name__} not supported"
                )

        # Create all mesh routers
        self._routers = [
            Router(router_id=i, latency=options.router_latency)
            for i in range(num_mesh_routers)
        ]

        self._link_count = 0
        self._int_links = []
        self._ext_links = []

        # Create all the mesh internal links.
        self._makeMesh(
            IntLink,
            self._router_link_latency,
            num_rows,
            num_cols,
            options.cross_links,
            options.cross_link_latency,
        )

        # Place CHI_RNF on the mesh
        self.distributeNodes(rnf_params, rnf_nodes)

        # Place CHI_HNF on the mesh
        self.distributeNodes(hnf_params, hnf_nodes)

        # Place CHI_MN on the mesh
        self.distributeNodes(mn_params, mn_nodes)

        # Place CHI_SNF_MainMem on the mesh
        self.distributeNodes(mem_params, mem_nodes)

        # Place all IO mem nodes on the mesh
        self.distributeNodes(io_mem_params, io_mem_nodes)

        # Place all IO request nodes on the mesh
        self.distributeNodes(rni_dma_params, rni_dma_nodes)
        self.distributeNodes(rni_io_params, rni_io_nodes)

        # Set up
        network.int_links = self._int_links
        network.ext_links = self._ext_links
        # fix Routers being set as link child
        for r in self._routers:
            if r.has_parent():
                r.get_parent().clear_child(r.get_name())
        network.routers = self._routers

        pairing = getattr(options, "pairing", None)
        if pairing != None:
            self._autoPairHNFandSNF(hnf_list, mem_ctrls, pairing)

    # --------------------------------------------------------------------------
    # _autoPair
    # --------------------------------------------------------------------------
    def _autoPairHNFandSNF(self, cache_ctrls, mem_ctrls, pairing):
        # Use the pairing defined by the configuration to reassign the
        # memory ranges
        pair_debug = False

        print("Pairing HNFs to SNFs")
        print(pairing)

        all_cache = []
        for c in cache_ctrls:
            all_cache.extend(c.getNetworkSideControllers())
        all_mem = []
        for c in mem_ctrls:
            all_mem.extend(c.getNetworkSideControllers())

        # checks and maps index from pairing map to component
        assert len(pairing) == len(all_cache)

        def _tolist(val):
            return val if isinstance(val, list) else [val]

        for m in all_mem:
            m._pairing = []

        pairing_check = max(1, len(all_mem) / len(all_cache))
        for cidx, c in enumerate(all_cache):
            c._pairing = []
            for midx in _tolist(pairing[cidx]):
                c._pairing.append(all_mem[midx])
                if c not in all_mem[midx]._pairing:
                    all_mem[midx]._pairing.append(c)
            assert len(c._pairing) == pairing_check
            if pair_debug:
                print(c.path())
                for r in c.addr_ranges:
                    print(f"{r}")
                for p in c._pairing:
                    print("\t" + p.path())
                    for r in p.addr_ranges:
                        print(f"\t{r}")

        # all must be paired
        for c in all_cache:
            assert len(c._pairing) > 0
        for m in all_mem:
            assert len(m._pairing) > 0

        # only support a single range for the main memory controllers
        tgt_range_start = all_mem[0].addr_ranges[0].start.value
        for mem in all_mem:
            for r in mem.addr_ranges:
                if r.start.value != tgt_range_start:
                    fatal(
                        "topologies.CustomMesh: not supporting pairing of "
                        "main memory with multiple ranges"
                    )

        # reassign ranges for a 1 -> N paring
        def _rerange(src_cntrls, tgt_cntrls, fix_tgt_peer):
            assert len(tgt_cntrls) >= len(src_cntrls)

            def _rangeToBit(addr_ranges):
                bit = None
                for r in addr_ranges:
                    if bit == None:
                        bit = r.intlvMatch
                    else:
                        assert bit == r.intlvMatch
                return bit

            def _getPeer(cntrl):
                return cntrl.memory_out_port.peer.simobj

            sorted_src = list(src_cntrls)
            sorted_src.sort(key=lambda x: _rangeToBit(x.addr_ranges))

            # paired controllers need to have seq. interleaving match values
            intlvMatch = 0
            for src in sorted_src:
                for tgt in src._pairing:
                    for r in tgt.addr_ranges:
                        r.intlvMatch = intlvMatch
                    if fix_tgt_peer:
                        _getPeer(tgt).range.intlvMatch = intlvMatch
                    intlvMatch = intlvMatch + 1

            # recreate masks
            for src in sorted_src:
                for src_range in src.addr_ranges:
                    if src_range.start.value != tgt_range_start:
                        continue
                    new_src_mask = []
                    for m in src_range.masks:
                        # TODO should mask all the way to the max range size
                        new_src_mask.append(
                            m | (m * 2) | (m * 4) | (m * 8) | (m * 16)
                        )
                    for tgt in src._pairing:
                        paired = False
                        for tgt_range in tgt.addr_ranges:
                            if tgt_range.start.value == src_range.start.value:
                                src_range.masks = new_src_mask
                                new_tgt_mask = []
                                lsbs = len(tgt_range.masks) - len(new_src_mask)
                                for i in range(lsbs):
                                    new_tgt_mask.append(tgt_range.masks[i])
                                for m in new_src_mask:
                                    new_tgt_mask.append(m)
                                tgt_range.masks = new_tgt_mask
                                if fix_tgt_peer:
                                    _getPeer(tgt).range.masks = new_tgt_mask
                                paired = True
                        if not paired:
                            fatal(
                                "topologies.CustomMesh: could not "
                                "reassign ranges {} {}".format(
                                    src.path(), tgt.path()
                                )
                            )

        if len(all_mem) >= len(all_cache):
            _rerange(all_cache, all_mem, True)
        else:
            _rerange(all_mem, all_cache, False)

        if pair_debug:
            print("")
            for cidx, c in enumerate(all_cache):
                assert len(c._pairing) == pairing_check
                print(c.path())
                for r in c.addr_ranges:
                    print(f"{r}")
                for p in c._pairing:
                    print("\t" + p.path())
                    for r in p.addr_ranges:
                        print(f"\t{r}")
