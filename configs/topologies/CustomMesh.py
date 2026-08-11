# Copyright (c) 2021,2022,2026 ARM Limited
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

from m5.util.custom_mesh_dot_writer import generate_dot


class CustomMesh(SimpleTopology):
    description = "CustomMesh"

    def __init__(self, controllers):
        self.nodes = controllers

    # --------------------------------------------------------------------------
    # _makeXYMesh, _makeCustomMesh
    # --------------------------------------------------------------------------

    def _connectRouters(self, latency, weight, src_node, dst_node, dst_port):
        self._int_links.append(
            self._IntLink(
                link_id=self._link_count,
                src_node=self._routers[src_node],
                dst_node=self._routers[dst_node],
                dst_inport=dst_port,
                latency=latency,
                weight=weight,
            )
        )
        self._link_count += 1

    def _connectXYRouters(self, weight, src_node, dst_node, dst_port):
        latency = self._router_link_latency
        if (src_node, dst_node) in self._custom_links:
            custom_weight, latency = self._custom_links[(src_node, dst_node)]
            if (custom_weight != None) and (custom_weight != weight):
                fatal(
                    "XY custom link weight (src=%d, dst=%d, weight=%d) "
                    "must be either 'None' or same as XY weight (=%d)",
                    src_node,
                    dst_node,
                    custom_weight,
                    weight,
                )
            assert (src_node, dst_node) in self._custom_non_XY_links
            del self._custom_non_XY_links[(src_node, dst_node)]
        self._connectRouters(latency, weight, src_node, dst_node, dst_port)

    def _makeXYMesh(self, num_rows, num_columns):

        # East->West, West->East, North->South, South->North
        # XY routing weights
        link_weights = [1, 1, 2, 2]

        # Non-XY custom links will be created later
        self._custom_non_XY_links = self._custom_links.copy()

        # East output to West input links
        # West output to East input links
        for row in range(num_rows):
            for col in range(num_columns):
                if col + 1 < num_columns:
                    east = col + (row * num_columns)
                    west = (col + 1) + (row * num_columns)
                    # East output to West input
                    self._connectXYRouters(link_weights[0], east, west, "West")
                    # West output to East input
                    self._connectXYRouters(link_weights[1], west, east, "East")

        # North output to South input links
        # South output to North input links
        for col in range(num_columns):
            for row in range(num_rows):
                if row + 1 < num_rows:
                    north = col + (row * num_columns)
                    south = col + ((row + 1) * num_columns)
                    # North output to South input
                    self._connectXYRouters(
                        link_weights[2], north, south, "South"
                    )
                    # South output to North input
                    self._connectXYRouters(
                        link_weights[3], south, north, "North"
                    )

    def _makeCustomMesh(self):
        for src, dst in self._custom_non_XY_links:
            weight, latency = self._custom_non_XY_links[(src, dst)]
            if weight == None:
                fatal(
                    "Non XY custom link weight (src=%d, dst=%d) "
                    "not specified",
                    src,
                    dst,
                )
            self._connectRouters(latency, weight, src, dst, None)

    # --------------------------------------------------------------------------
    # distributeNodes
    # --------------------------------------------------------------------------

    def _createRNFRouter(self, noc_params, mesh_router):
        # Create a zero-latency router bridging node controllers
        # and the mesh router
        node_router = self._Router(router_id=len(self._routers))
        node_router._row = mesh_router._row
        node_router._col = mesh_router._col
        node_router._main = False

        if hasattr(node_router, "int_routing_latency"):
            node_router.int_routing_latency = self.node_router_latency
            node_router.ext_routing_latency = self.node_router_latency
        else:
            node_router.latency = self.node_router_latency

        self._routers.append(node_router)

        # connect node_router <-> mesh router
        self._int_links.append(
            self._IntLink(
                link_id=self._link_count,
                src_node=node_router,
                dst_node=mesh_router,
                latency=noc_params.node_link_latency,
            )
        )
        self._link_count += 1

        self._int_links.append(
            self._IntLink(
                link_id=self._link_count,
                src_node=mesh_router,
                dst_node=node_router,
                latency=noc_params.node_link_latency,
            )
        )
        self._link_count += 1

        return node_router

    def distributeNodes(self, noc_params, node_params, node_list):
        if len(node_list) == 0:
            return

        num_nodes_per_router = node_params.num_nodes_per_router
        router_idx_list = node_params.router_list

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
                    router = self._createRNFRouter(noc_params, router)

                # connect all ctrls in the node to node_router
                ctrls = node.getNetworkSideControllers()
                for c in ctrls:
                    self._ext_links.append(
                        self._ExtLink(
                            link_id=self._link_count,
                            ext_node=c,
                            int_node=router,
                            latency=node_params.inbound_link_latency,
                        )
                    )
                    # See CHI_config.py for outbound_link_latency
                    self._link_count += 1
                    c._row = router._row
                    c._col = router._col
        else:
            # try to circulate all nodes to all routers, some routers may be
            # connected to zero or more than one node.
            idx = 0
            for node in node_list:
                ridx = router_idx_list[idx]
                router = self._routers[ridx]

                if isinstance(node, CHI.CHI_RNF):
                    router = self._createRNFRouter(noc_params, router)
                ctrls = node.getNetworkSideControllers()
                for c in ctrls:
                    self._ext_links.append(
                        self._ExtLink(
                            link_id=self._link_count,
                            ext_node=c,
                            int_node=router,
                            latency=node_params.inbound_link_latency,
                        )
                    )
                    self._link_count += 1
                    c._row = router._row
                    c._col = router._col
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
        self._custom_links = options.custom_links

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
        self._routers = [Router(router_id=i) for i in range(num_mesh_routers)]
        # Set up latency
        if hasattr(self._routers[0], "int_routing_latency"):
            for router in self._routers:
                router.int_routing_latency = options.router_int_latency
                router.ext_routing_latency = options.router_ext_latency
        else:
            print("WARNING: router does not support int/ext routing latencies")
            for router in self._routers:
                router.latency = max(
                    options.router_int_latency, options.router_ext_latency
                )

        # Assign helpers later needed by generate_dot
        for row in range(num_rows):
            for col in range(num_cols):
                router_id = col + (row * num_cols)
                assert self._routers[router_id].router_id.value == router_id
                self._routers[router_id]._row = row
                self._routers[router_id]._col = col
                self._routers[router_id]._main = True

        self._link_count = 0
        self._int_links = []
        self._ext_links = []

        # Expands custom links
        self._custom_links = {}
        for k in options.custom_links:
            weight, latency = options.custom_links[k]
            self._custom_links[k] = weight, latency
            if (k[1], k[0]) not in options.custom_links:
                self._custom_links[(k[1], k[0])] = weight, latency

        # Create all the mesh internal links.
        self._makeXYMesh(num_rows, num_cols)
        self._makeCustomMesh()

        # Place CHI_RNF on the mesh
        self.distributeNodes(options, rnf_params, rnf_nodes)

        # Place CHI_HNF on the mesh
        self.distributeNodes(options, hnf_params, hnf_nodes)

        # Place CHI_MN on the mesh
        self.distributeNodes(options, mn_params, mn_nodes)

        # Place CHI_SNF_MainMem on the mesh
        self.distributeNodes(options, mem_params, mem_nodes)

        # Place all IO mem nodes on the mesh
        self.distributeNodes(options, io_mem_params, io_mem_nodes)

        # Place all IO request nodes on the mesh
        self.distributeNodes(options, rni_dma_params, rni_dma_nodes)
        self.distributeNodes(options, rni_io_params, rni_io_nodes)

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

        generate_dot(network, num_rows, num_cols)

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
