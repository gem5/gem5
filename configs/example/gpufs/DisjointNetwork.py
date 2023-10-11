# Copyright (c) 2021 Advanced Micro Devices, Inc.
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

from m5.objects import *
from m5.util import fatal

from importlib import *

from network import Network


class DisjointSimple(SimpleNetwork):
    def __init__(self, ruby_system):
        super().__init__()

        self.netifs = []
        self.routers = []
        self.int_links = []
        self.ext_links = []
        self.ruby_system = ruby_system

    def connectCPU(self, opts, controllers):
        # Setup parameters for makeTopology call for CPU network
        topo_module = import_module(f"topologies.{opts.cpu_topology}")
        topo_class = getattr(topo_module, opts.cpu_topology)
        _topo = topo_class(controllers)
        _topo.makeTopology(opts, self, SimpleIntLink, SimpleExtLink, Switch)

        self.initSimple(opts, self.int_links, self.ext_links)

    def connectGPU(self, opts, controllers):
        # Setup parameters for makeTopology call for GPU network
        topo_module = import_module(f"topologies.{opts.gpu_topology}")
        topo_class = getattr(topo_module, opts.gpu_topology)
        _topo = topo_class(controllers)
        _topo.makeTopology(opts, self, SimpleIntLink, SimpleExtLink, Switch)

        self.initSimple(opts, self.int_links, self.ext_links)

    def initSimple(self, opts, int_links, ext_links):
        # Attach links to network
        self.int_links = int_links
        self.ext_links = ext_links

        self.setup_buffers()


class DisjointGarnet(GarnetNetwork):
    def __init__(self, ruby_system):
        super().__init__()

        self.netifs = []
        self.ruby_system = ruby_system

    def connectCPU(self, opts, controllers):
        # Setup parameters for makeTopology call for CPU network
        topo_module = import_module(f"topologies.{opts.cpu_topology}")
        topo_class = getattr(topo_module, opts.cpu_topology)
        _topo = topo_class(controllers)
        _topo.makeTopology(
            opts, self, GarnetIntLink, GarnetExtLink, GarnetRouter
        )

        Network.init_network(opts, self, GarnetNetworkInterface)

    def connectGPU(self, opts, controllers):
        # Setup parameters for makeTopology call
        topo_module = import_module(f"topologies.{opts.gpu_topology}")
        topo_class = getattr(topo_module, opts.gpu_topology)
        _topo = topo_class(controllers)
        _topo.makeTopology(
            opts, self, GarnetIntLink, GarnetExtLink, GarnetRouter
        )

        Network.init_network(opts, self, GarnetNetworkInterface)
