# Copyright (c) 2011 Advanced Micro Devices, Inc.
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

from m5.objects.ClockedObject import ClockedObject
from m5.params import *


class RubyRouterType(ScopedEnum):
    vals = ["mesh", "node"]


class BasicRouter(ClockedObject):
    type = "BasicRouter"
    cxx_header = "mem/ruby/network/BasicRouter.hh"
    cxx_class = "gem5::ruby::BasicRouter"

    router_id = Param.Int("ID in relation to other routers")

    # only used by garnet
    latency = Param.Cycles(1, "number of cycles inside router")

    # Router type, used only by Garnet's mesh-only stats (no timing effect):
    #   mesh: a router of the network topology itself, e.g. one grid point
    #         of a mesh, connected to other mesh routers.
    #   node: a helper router that only joins the controllers of one node
    #         (e.g. a core's L1I, L1D and L2) to a single mesh router, like
    #         the node routers CustomMesh creates. Traffic between those
    #         controllers turns around in the node router and never enters
    #         the mesh: the mesh_* stats count mesh routers only and report
    #         such traffic as core_internal_flits.
    router_type = Param.RubyRouterType(
        "mesh", "mesh router or node router (see above)"
    )
