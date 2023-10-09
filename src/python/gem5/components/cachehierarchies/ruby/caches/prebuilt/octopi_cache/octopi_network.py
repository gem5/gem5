# Copyright (c) 2022-2023 The Regents of the University of California
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
from m5.objects import SimpleNetwork

from .ruby_network_components import RubyIntLink
from .ruby_network_components import RubyNetworkComponent
from .ruby_network_components import RubyRouter

# . The Network owns all routers, all int links and all ext links that are not in CCD's.
# . The CCD subsystems are not of type RubyNetwork, so we need to copy the references of
# routers and links to OctopiNetwork._routers, ._int_links, and ._ext_links; which will
# be, in turns, copied to RubyNetwork.routers, .int_links, and .ext_links respectively.
class OctopiNetwork(SimpleNetwork, RubyNetworkComponent):
    def __init__(self, ruby_system):
        SimpleNetwork.__init__(self=self)
        RubyNetworkComponent.__init__(self=self)
        self.netifs = []
        self.ruby_system = ruby_system
        self.number_of_virtual_networks = (
            ruby_system.number_of_virtual_networks
        )

        self.cross_ccd_router = RubyRouter(self)
        self._add_router(self.cross_ccd_router)

    def connect_ccd_routers_to_cross_ccd_router(self, ccds):
        for ccd in ccds:
            int_link_1, int_link_2 = RubyIntLink.create_bidirectional_links(
                self.cross_ccd_router,
                ccd.get_main_router(),
                bandwidth_factor=64,
            )
            ccd.to_cross_ccd_router_link = int_link_1
            ccd.from_cross_ccd_router_link = int_link_2
            self._add_int_link(int_link_1)
            self._add_int_link(int_link_2)

    def incorporate_ccds(self, ccds):
        for ccd in ccds:
            self.incorporate_ruby_subsystem(ccd)
        self.connect_ccd_routers_to_cross_ccd_router(ccds)
