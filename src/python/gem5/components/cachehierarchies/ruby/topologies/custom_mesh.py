# Copyright (c) 2026 Arm Limited
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

import importlib
import importlib.machinery
import importlib.util
from dataclasses import dataclass
from pathlib import Path
from typing import (
    Any,
    Optional,
    Sequence,
    Tuple,
)

from m5.objects import (
    SimpleExtLink,
    SimpleIntLink,
    SimpleNetwork,
    Switch,
)
from m5.util import (
    fatal,
    warn,
)

from ...chi.noc import NoC_Params as CHI_NoC_Params
from ...chi.nodes.abstract_node import (
    CHI_NodeType,
    Node_Params,
)


def load_topology_config(topology_path: str):
    """Load a topology config Python file as a module."""

    path = Path(topology_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(
            f"Topology config does not exist: {topology_path}"
        )

    module_name = f"topology_config_{path.stem}"
    loader = importlib.machinery.SourceFileLoader(module_name, str(path))
    module = importlib.util.module_from_spec(
        importlib.util.spec_from_loader(module_name, loader)
    )
    loader.exec_module(module)
    return module


@dataclass(kw_only=True)
class NoC_Params(CHI_NoC_Params):
    """NoC defaults specialized for the stdlib CustomMesh topology."""

    num_rows: int = 0
    num_cols: int = 0
    pairing: Optional[Sequence[Any]] = None


class CustomMesh(SimpleNetwork):
    """Custom mesh topology for CHI stdlib cache hierarchies."""

    def __init__(self, ruby_system, noc_params: NoC_Params):
        super().__init__()
        self.netifs = []

        # TODO: These should be in a base class.
        # https://gem5.atlassian.net/browse/GEM5-1039
        self.ruby_system = ruby_system

        self._num_rows = noc_params.num_rows
        self._num_cols = noc_params.num_cols
        if self._num_rows is None or self._num_cols is None:
            raise ValueError("noc_params must define num_rows and num_cols.")
        if self._num_rows <= 0 or self._num_cols <= 0:
            raise ValueError("num_rows and num_cols must both be > 0.")

        self._num_mesh_routers = self._num_rows * self._num_cols
        self._router_int_latency = noc_params.router_int_latency
        self._router_ext_latency = noc_params.router_ext_latency
        self._router_link_latency = noc_params.router_link_latency
        self._node_link_latency = noc_params.node_link_latency
        self._custom_links = {}
        for src_dst, link_params in noc_params.custom_links.items():
            src, dst = src_dst
            weight, latency = link_params
            self._custom_links[(int(src), int(dst))] = (weight, latency)
            if (dst, src) not in noc_params.custom_links:
                self._custom_links[(int(dst), int(src))] = (weight, latency)
        self._pairing = noc_params.pairing

        # Keep aligned with CustomMesh in configs/topologies for non-garnet use.
        self._node_router_latency = 0

        self._link_count = 0
        self._int_links = []
        self._ext_links = []

    def _check_router_id(self, router_id: int, node_type: CHI_NodeType):
        if router_id < 0 or router_id >= self._num_mesh_routers:
            raise ValueError(
                f"Invalid router index {router_id} for {node_type.name}. "
                f"Valid range is [0, {self._num_mesh_routers - 1}]."
            )

    def _connect_routers(
        self,
        latency: int,
        weight: int,
        src_node: int,
        dst_node: int,
        dst_port: Optional[str],
    ):
        self._int_links.append(
            SimpleIntLink(
                link_id=self._link_count,
                src_node=self._routers[src_node],
                dst_node=self._routers[dst_node],
                dst_inport=dst_port,
                latency=latency,
                weight=weight,
            )
        )
        self._link_count += 1

    def _connect_xy_routers(
        self, weight: int, src_node: int, dst_node: int, dst_port: str
    ):
        latency = self._router_link_latency
        if (src_node, dst_node) in self._custom_links:
            custom_weight, latency = self._custom_links[(src_node, dst_node)]
            if custom_weight is not None and custom_weight != weight:
                fatal(
                    "XY custom link weight (src=%d, dst=%d, weight=%d) "
                    "must be either 'None' or same as XY weight (=%d)",
                    src_node,
                    dst_node,
                    custom_weight,
                    weight,
                )
            assert (src_node, dst_node) in self._custom_non_xy_links
            del self._custom_non_xy_links[(src_node, dst_node)]

        self._connect_routers(latency, weight, src_node, dst_node, dst_port)

    def _make_xy_mesh(self):
        # East->West, West->East, North->South, South->North
        # XY routing weights
        link_weights = [1, 1, 2, 2]

        # Non-XY custom links will be created later.
        self._custom_non_xy_links = self._custom_links.copy()

        # East output to West input links
        # West output to East input links
        for row in range(self._num_rows):
            for col in range(self._num_cols):
                if col + 1 < self._num_cols:
                    east = col + (row * self._num_cols)
                    west = (col + 1) + (row * self._num_cols)
                    self._connect_xy_routers(
                        link_weights[0], east, west, "West"
                    )
                    self._connect_xy_routers(
                        link_weights[1], west, east, "East"
                    )

        # North output to South input links
        # South output to North input links
        for col in range(self._num_cols):
            for row in range(self._num_rows):
                if row + 1 < self._num_rows:
                    north = col + (row * self._num_cols)
                    south = col + ((row + 1) * self._num_cols)
                    self._connect_xy_routers(
                        link_weights[2], north, south, "South"
                    )
                    self._connect_xy_routers(
                        link_weights[3], south, north, "North"
                    )

    def _make_custom_mesh(self):
        for (src, dst), link_params in self._custom_non_xy_links.items():
            weight, latency = link_params
            if weight is None:
                fatal(
                    "Non XY custom link weight (src=%d, dst=%d) "
                    "not specified",
                    src,
                    dst,
                )
            self._connect_routers(latency, weight, src, dst, None)

    def _create_rnf_router(self, mesh_router):
        # Create a zero-latency router bridging RNF controllers and mesh router.
        node_router = Switch(
            router_id=len(self._routers),
            int_routing_latency=self._node_router_latency,
            ext_routing_latency=self._node_router_latency,
        )
        self._routers.append(node_router)

        self._int_links.append(
            SimpleIntLink(
                link_id=self._link_count,
                src_node=node_router,
                dst_node=mesh_router,
                latency=self._router_link_latency,
            )
        )
        self._link_count += 1

        self._int_links.append(
            SimpleIntLink(
                link_id=self._link_count,
                src_node=mesh_router,
                dst_node=node_router,
                latency=self._router_link_latency,
            )
        )
        self._link_count += 1

        return node_router

    def _iter_node_controllers(self, node):
        ctrls = node.getNetworkSideControllers()
        return ctrls if isinstance(ctrls, (list, tuple)) else [ctrls]

    def distribute_nodes(
        self, node_params: Node_Params, node_list: Sequence[Any]
    ):
        if len(node_list) == 0:
            return
        if node_params is None:
            raise ValueError(
                "Node_Params must be provided for non-empty nodes."
            )
        if len(node_params.router_list) == 0:
            raise ValueError(
                f"router_list must not be empty for {node_params.node_type.name}."
            )

        num_nodes_per_router = node_params.num_nodes_per_router
        router_idx_list = node_params.router_list
        # We set the external link latency to the global node_link_larency, unless
        # the latency is specialized by defining a CHI node specific
        # inbound_link_latency
        ext_link_latency = (
            node_params.inbound_link_latency
            if node_params.inbound_link_latency
            else self._node_link_latency
        )

        for router_id in router_idx_list:
            self._check_router_id(int(router_id), node_params.node_type)

        if num_nodes_per_router < 0:
            raise ValueError("num_nodes_per_router must be >= 0.")

        if num_nodes_per_router:
            # Evenly distribute nodes to all listed routers.
            expected_nodes = len(router_idx_list) * num_nodes_per_router

            if expected_nodes < len(node_list):
                fatal(
                    f"{node_params.node_type.name}: Too many nodes for mesh. Expected "
                    f"{expected_nodes}, got {len(node_list)}"
                )
            elif expected_nodes > len(node_list):
                warn(
                    f"{node_params.node_type.name}: Too few nodes for mesh. Expected "
                    f"{expected_nodes}, got {len(node_list)}"
                )

            for idx, node in enumerate(node_list):
                mesh_router_idx = router_idx_list[idx // num_nodes_per_router]
                router = self._routers[mesh_router_idx]

                if node_params.dedicated_router:
                    if node_params.node_type != CHI_NodeType.CHI_RNF:
                        raise ValueError(
                            "Node_Params.dedicated_router is only valid for RNFs."
                        )
                    router = self._create_rnf_router(router)

                for controller in self._iter_node_controllers(node):
                    self._ext_links.append(
                        SimpleExtLink(
                            link_id=self._link_count,
                            ext_node=controller,
                            int_node=router,
                            latency=ext_link_latency,
                        )
                    )
                    self._link_count += 1
        else:
            # Circulate nodes across router_list when num_nodes_per_router == 0.
            idx = 0
            for node in node_list:
                mesh_router_idx = router_idx_list[idx]
                router = self._routers[mesh_router_idx]

                if node_params.dedicated_router:
                    if node_params.node_type != CHI_NodeType.CHI_RNF:
                        raise ValueError(
                            "Node_Params.dedicated_router is only valid for RNFs."
                        )
                    router = self._create_rnf_router(router)

                for controller in self._iter_node_controllers(node):
                    self._ext_links.append(
                        SimpleExtLink(
                            link_id=self._link_count,
                            ext_node=controller,
                            int_node=router,
                            latency=ext_link_latency,
                        )
                    )
                    self._link_count += 1

                idx = (idx + 1) % len(router_idx_list)

    def _auto_pair_hnf_and_snf(self, cache_nodes, mem_nodes, pairing):
        # Use the pairing defined by the configuration to reassign
        # memory ranges.
        pair_debug = False

        print("Pairing HNFs to SNFs")
        print(pairing)

        all_cache = []
        for cache_node in cache_nodes:
            all_cache.extend(cache_node.getNetworkSideControllers())
        all_mem = []
        for mem_node in mem_nodes:
            all_mem.extend(mem_node.getNetworkSideControllers())

        # checks and maps index from pairing map to component
        assert len(pairing) == len(all_cache)

        def _to_list(val):
            return val if isinstance(val, list) else [val]

        for mem in all_mem:
            mem._pairing = []

        pairing_check = max(1, len(all_mem) / len(all_cache))
        for cache_idx, cache in enumerate(all_cache):
            cache._pairing = []
            for mem_idx in _to_list(pairing[cache_idx]):
                cache._pairing.append(all_mem[mem_idx])
                if cache not in all_mem[mem_idx]._pairing:
                    all_mem[mem_idx]._pairing.append(cache)
            assert len(cache._pairing) == pairing_check
            if pair_debug:
                print(cache.path())
                for addr_range in cache.addr_ranges:
                    print(f"{addr_range}")
                for paired in cache._pairing:
                    print("\t" + paired.path())
                    for addr_range in paired.addr_ranges:
                        print(f"\t{addr_range}")

        # all must be paired
        for cache in all_cache:
            assert len(cache._pairing) > 0
        for mem in all_mem:
            assert len(mem._pairing) > 0

        # only support a single range for the main memory controllers
        tgt_range_start = all_mem[0].addr_ranges[0].start.value
        for mem in all_mem:
            for addr_range in mem.addr_ranges:
                if addr_range.start.value != tgt_range_start:
                    fatal(
                        "topologies.CustomMesh: not supporting pairing of "
                        "main memory with multiple ranges"
                    )

        # reassign ranges for a 1 -> N pairing
        def _rerange(src_cntrls, tgt_cntrls, fix_tgt_peer):
            assert len(tgt_cntrls) >= len(src_cntrls)

            def _range_to_bit(addr_ranges):
                bit = None
                for addr_range in addr_ranges:
                    if bit is None:
                        bit = addr_range.intlvMatch
                    else:
                        assert bit == addr_range.intlvMatch
                return bit

            def _get_peer(cntrl):
                return cntrl.memory_out_port.peer.simobj

            sorted_src = list(src_cntrls)
            sorted_src.sort(key=lambda x: _range_to_bit(x.addr_ranges))

            # Paired controllers need sequential interleaving match values.
            intlv_match = 0
            for src in sorted_src:
                for tgt in src._pairing:
                    for addr_range in tgt.addr_ranges:
                        addr_range.intlvMatch = intlv_match
                    if fix_tgt_peer:
                        _get_peer(tgt).range.intlvMatch = intlv_match
                    intlv_match = intlv_match + 1

            # Recreate masks.
            for src in sorted_src:
                for src_range in src.addr_ranges:
                    if src_range.start.value != tgt_range_start:
                        continue
                    new_src_mask = []
                    for mask in src_range.masks:
                        # TODO should mask all the way to the max range size
                        new_src_mask.append(
                            mask
                            | (mask * 2)
                            | (mask * 4)
                            | (mask * 8)
                            | (mask * 16)
                        )
                    for tgt in src._pairing:
                        paired = False
                        for tgt_range in tgt.addr_ranges:
                            if tgt_range.start.value == src_range.start.value:
                                src_range.masks = new_src_mask
                                new_tgt_mask = []
                                lsbs = len(tgt_range.masks) - len(new_src_mask)
                                for idx in range(lsbs):
                                    new_tgt_mask.append(tgt_range.masks[idx])
                                for mask in new_src_mask:
                                    new_tgt_mask.append(mask)
                                tgt_range.masks = new_tgt_mask
                                if fix_tgt_peer:
                                    _get_peer(tgt).range.masks = new_tgt_mask
                                paired = True
                        if not paired:
                            fatal(
                                "topologies.CustomMesh: could not reassign "
                                f"ranges {src.path()} {tgt.path()}"
                            )

        if len(all_mem) >= len(all_cache):
            _rerange(all_cache, all_mem, True)
        else:
            _rerange(all_mem, all_cache, False)

        if pair_debug:
            print("")
            for cache in all_cache:
                assert len(cache._pairing) == pairing_check
                print(cache.path())
                for addr_range in cache.addr_ranges:
                    print(f"{addr_range}")
                for paired in cache._pairing:
                    print("\t" + paired.path())
                    for addr_range in paired.addr_ranges:
                        print(f"\t{addr_range}")

    def connectControllers(
        self,
        rnf: Optional[Tuple[Sequence[Any], Node_Params]] = None,
        hnf: Optional[Tuple[Sequence[Any], Node_Params]] = None,
        snf: Optional[Tuple[Sequence[Any], Node_Params]] = None,
        snf_boot: Optional[Tuple[Sequence[Any], Node_Params]] = None,
        rni: Optional[Tuple[Sequence[Any], Node_Params]] = None,
    ):
        # Create all mesh routers.
        self._routers = [
            Switch(
                router_id=i,
                int_routing_latency=self._router_int_latency,
                ext_routing_latency=self._router_ext_latency,
            )
            for i in range(self._num_mesh_routers)
        ]

        self._link_count = 0
        self._int_links = []
        self._ext_links = []

        # Create all mesh internal links.
        self._make_xy_mesh()
        self._make_custom_mesh()

        # Place nodes using only provided (nodes, node_params) tuples.
        for node_group in (rnf, hnf, snf, snf_boot, rni):
            if node_group is None:
                continue
            nodes, nodes_params = node_group
            self.distribute_nodes(nodes_params, nodes)

        if self._pairing is not None:
            hnf_nodes = [] if hnf is None else hnf[0]
            snf_nodes = [] if snf is None else snf[0]
            self._auto_pair_hnf_and_snf(hnf_nodes, snf_nodes, self._pairing)

        self.routers = self._routers
        self.int_links = self._int_links
        self.ext_links = self._ext_links
