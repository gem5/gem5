# Copyright (c) 2012 Advanced Micro Devices, Inc.
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
#
# Authors: Jason Power

import m5

class BaseTopology(object):
    description = "BaseTopology"

    def __init__(self):
        """ When overriding place any objects created in
            configs/ruby/<protocol>.py that are needed in
            makeTopology (below) here. The minimum is usually
            all of the controllers created in the above file.
        """

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        """ Called from configs/ruby/Ruby.py
            The return value is ( list(Router), list(IntLink), list(ExtLink))
            The API of this function cannot change when subclassing!!
            Any additional information needed to create this topology should
            be passed into the constructor when it's instantiated in
            configs/ruby/<protocol>.py
        """
        m5.util.fatal("BaseTopology should have been overridden!!")

class SimpleTopology(BaseTopology):
    """ Provides methods needed for the topologies included in Ruby before
        topology changes.
        These topologies are "simple" in the sense that they only use a flat
        list of controllers to construct the topology.
    """
    description = "SimpleTopology"

    def __init__(self, controllers):
        self.nodes = controllers

    def addController(self, controller):
        self.nodes.append(controller)

    def __len__(self):
        return len(self.nodes)
