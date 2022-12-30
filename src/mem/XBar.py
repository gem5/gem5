# Copyright (c) 2012, 2015, 2017, 2019-2020 ARM Limited
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
# Copyright (c) 2005-2008 The Regents of The University of Michigan
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

from m5.objects.System import System
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

from m5.objects.ClockedObject import ClockedObject


class BaseXBar(ClockedObject):
    type = "BaseXBar"
    abstract = True
    cxx_header = "mem/xbar.hh"
    cxx_class = "gem5::BaseXBar"

    cpu_side_ports = VectorResponsePort(
        "Vector port for connecting " "mem side ports"
    )
    slave = DeprecatedParam(
        cpu_side_ports, "`slave` is now called `cpu_side_ports`"
    )
    mem_side_ports = VectorRequestPort(
        "Vector port for connecting " "cpu side ports"
    )
    master = DeprecatedParam(
        mem_side_ports, "`master` is now called `mem_side_ports`"
    )

    # Latencies governing the time taken for the variuos paths a
    # packet has through the crossbar. Note that the crossbar itself
    # does not add the latency due to assumptions in the coherency
    # mechanism. Instead the latency is annotated on the packet and
    # left to the neighbouring modules.
    #
    # A request incurs the frontend latency, possibly snoop filter
    # lookup latency, and forward latency. A response incurs the
    # response latency. Frontend latency encompasses arbitration and
    # deciding what to do when a request arrives. the forward latency
    # is the latency involved once a decision is made to forward the
    # request. The response latency, is similar to the forward
    # latency, but for responses rather than requests.
    frontend_latency = Param.Cycles("Frontend latency")
    forward_latency = Param.Cycles("Forward latency")
    response_latency = Param.Cycles("Response latency")

    # The XBar uses one Layer per requestor. Each Layer forwards a packet
    # to its destination and is occupied for header_latency + size /
    # width cycles
    header_latency = Param.Cycles(1, "Header latency")

    # Width governing the throughput of the crossbar
    width = Param.Unsigned("Datapath width per port (bytes)")

    # The default port can be left unconnected, or be used to connect
    # a default response port
    default = RequestPort("Port for connecting an optional default responder")

    # The default port can be used unconditionally, or based on
    # address range, in which case it may overlap with other
    # ports. The default range is always checked first, thus creating
    # a two-level hierarchical lookup. This is useful e.g. for the PCI
    # xbar configuration.
    use_default_range = Param.Bool(
        False, "Perform address mapping for " "the default port"
    )


class NoncoherentXBar(BaseXBar):
    type = "NoncoherentXBar"
    cxx_header = "mem/noncoherent_xbar.hh"
    cxx_class = "gem5::NoncoherentXBar"


class CoherentXBar(BaseXBar):
    type = "CoherentXBar"
    cxx_header = "mem/coherent_xbar.hh"
    cxx_class = "gem5::CoherentXBar"

    # The coherent crossbar additionally has snoop responses that are
    # forwarded after a specific latency.
    snoop_response_latency = Param.Cycles("Snoop response latency")

    # An optional snoop filter
    snoop_filter = Param.SnoopFilter(NULL, "Selected snoop filter")

    # Maximum number of outstanding snoop requests for sanity checks
    max_outstanding_snoops = Param.Int(512, "Max. outstanding snoops allowed")

    # Maximum routing table size for sanity checks
    max_routing_table_size = Param.Int(512, "Max. routing table size")

    # Determine how this crossbar handles packets where caches have
    # already committed to responding, by establishing if the crossbar
    # is the point of coherency or not.
    point_of_coherency = Param.Bool(
        False, "Consider this crossbar the " "point of coherency"
    )

    # Specify whether this crossbar is the point of unification.
    point_of_unification = Param.Bool(
        False, "Consider this crossbar the " "point of unification"
    )

    system = Param.System(Parent.any, "System that the crossbar belongs to.")


class SnoopFilter(SimObject):
    type = "SnoopFilter"
    cxx_header = "mem/snoop_filter.hh"
    cxx_class = "gem5::SnoopFilter"

    # Lookup latency of the snoop filter, added to requests that pass
    # through a coherent crossbar.
    lookup_latency = Param.Cycles(1, "Lookup latency")

    system = Param.System(Parent.any, "System that the crossbar belongs to.")

    # Sanity check on max capacity to track, adjust if needed.
    max_capacity = Param.MemorySize("8MiB", "Maximum capacity of snoop filter")


# We use a coherent crossbar to connect multiple requestors to the L2
# caches. Normally this crossbar would be part of the cache itself.
class L2XBar(CoherentXBar):
    # 256-bit crossbar by default
    width = 32

    # Assume that most of this is covered by the cache latencies, with
    # no more than a single pipeline stage for any packet.
    frontend_latency = 1
    forward_latency = 0
    response_latency = 1
    snoop_response_latency = 1

    # Use a snoop-filter by default, and set the latency to zero as
    # the lookup is assumed to overlap with the frontend latency of
    # the crossbar
    snoop_filter = SnoopFilter(lookup_latency=0)

    # This specialisation of the coherent crossbar is to be considered
    # the point of unification, it connects the dcache and the icache
    # to the first level of unified cache.
    point_of_unification = True


# One of the key coherent crossbar instances is the system
# interconnect, tying together the CPU clusters, GPUs, and any I/O
# coherent requestors, and DRAM controllers.
class SystemXBar(CoherentXBar):
    # 128-bit crossbar by default
    width = 16

    # A handful pipeline stages for each portion of the latency
    # contributions.
    frontend_latency = 3
    forward_latency = 4
    response_latency = 2
    snoop_response_latency = 4

    # Use a snoop-filter by default
    snoop_filter = SnoopFilter(lookup_latency=1)

    # This specialisation of the coherent crossbar is to be considered
    # the point of coherency, as there are no (coherent) downstream
    # caches.
    point_of_coherency = True

    # This specialisation of the coherent crossbar is to be considered
    # the point of unification, it connects the dcache and the icache
    # to the first level of unified cache. This is needed for systems
    # without caches where the SystemXBar is also the point of
    # unification.
    point_of_unification = True


# In addition to the system interconnect, we typically also have one
# or more on-chip I/O crossbars. Note that at some point we might want
# to also define an off-chip I/O crossbar such as PCIe.
class IOXBar(NoncoherentXBar):
    # 128-bit crossbar by default
    width = 16

    # Assume a simpler datapath than a coherent crossbar, incuring
    # less pipeline stages for decision making and forwarding of
    # requests.
    frontend_latency = 2
    forward_latency = 1
    response_latency = 2
