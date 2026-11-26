# Copyright (c) 2021, 2026 Arm Limited
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

from gem5.components.cachehierarchies.chi.nodes.abstract_node import (
    CHI_NodeType,
    Node_Params,
)
from gem5.components.cachehierarchies.chi.private_l1_private_l2_shared_l3_mesh_cache_hierarchy import (
    PrivateL1PrivateL2SharedL3MeshCacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.topologies.custom_mesh import (
    NoC_Params as CustomMeshNoC_Params,
)

# CustomMesh parameters for a 2x4 mesh. Routers will have the following layout:
#
# 0 --- 1 --- 2 --- 3
# |     |     |     |
# 4 --- 5 --- 6 --- 7
NoC_Params = CustomMeshNoC_Params(num_rows=2, num_cols=4)


# 4 RNFs with dedicated router for the RNF
rnf_in_mesh = Node_Params(
    node_type=CHI_NodeType.CHI_RNF,
    router_list=[1, 2, 5, 6],
    dedicated_router=True,
)
hnf_in_mesh = Node_Params(
    node_type=CHI_NodeType.CHI_HNF, router_list=[1, 2, 5, 6]
)
mn_in_mesh = Node_Params(node_type=CHI_NodeType.CHI_MN, router_list=[4])
snf_mainmem_in_mesh = Node_Params(
    node_type=CHI_NodeType.CHI_SNF_MainMem, router_list=[0, 4]
)
snf_bootmem_in_mesh = Node_Params(
    node_type=CHI_NodeType.CHI_SNF_BootMem, router_list=[3]
)
rni_dma_in_mesh = Node_Params(
    node_type=CHI_NodeType.CHI_RNI_DMA, router_list=[7], num_nodes_per_router=2
)
rni_io_in_mesh = Node_Params(
    node_type=CHI_NodeType.CHI_RNI_IO, router_list=[7]
)


class Example2x4Mesh(PrivateL1PrivateL2SharedL3MeshCacheHierarchy):
    def __init__(self):
        super().__init__(
            l1d_size="32KiB",
            l1d_assoc=4,
            l1i_size="32KiB",
            l1i_assoc=4,
            l2_size="512KiB",
            l2_assoc=8,
            l3_size="16MiB",
            l3_assoc=16,
            noc_params=NoC_Params,
        )

    def add_default_nodes(self):
        """
        This adds all defined nodes to the mesh.
        The fact that this method is not called by default
        during __init__ is because we want to be able to
        add_nodes manually in case some customization
        is required in the client config
        """
        self.add_nodes(rnf_in_mesh)
        self.add_nodes(hnf_in_mesh)
        self.add_nodes(snf_mainmem_in_mesh)
        self.add_nodes(snf_bootmem_in_mesh)
        self.add_nodes(rni_dma_in_mesh)


cache_hierarchy_type = Example2x4Mesh

_NODE_PARAMS_BY_TYPE = {
    rnf_in_mesh.node_type: rnf_in_mesh,
    hnf_in_mesh.node_type: hnf_in_mesh,
    mn_in_mesh.node_type: mn_in_mesh,
    snf_mainmem_in_mesh.node_type: snf_mainmem_in_mesh,
    snf_bootmem_in_mesh.node_type: snf_bootmem_in_mesh,
    rni_dma_in_mesh.node_type: rni_dma_in_mesh,
    rni_io_in_mesh.node_type: rni_io_in_mesh,
}


def get_node_params(node_type: CHI_NodeType) -> Node_Params:
    if node_type not in _NODE_PARAMS_BY_TYPE:
        raise ValueError(
            f"No Node_Params configured for node type: {node_type}"
        )
    return _NODE_PARAMS_BY_TYPE[node_type]
