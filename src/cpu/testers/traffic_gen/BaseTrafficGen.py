# Copyright (c) 2012, 2016, 2018, 2019 ARM Limited
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
from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *

# Types of Stream Generators.
# Those are orthogonal to the other generators in the TrafficGen
# and are meant to initialize the stream and substream IDs for
# every memory request, regardless of how the packet has been
# generated (Random, Linear, Trace etc)
class StreamGenType(ScopedEnum):
    vals = ["none", "fixed", "random"]


# The traffic generator is a requestor module that generates stimuli for
# the memory system, based on a collection of simple behaviours that
# are either probabilistic or based on traces. It can be used stand
# alone for creating test cases for interconnect and memory
# controllers, or function as a black-box replacement for system
# components that are not yet modelled in detail, e.g. a video engine
# or baseband subsystem in an SoC.
class BaseTrafficGen(ClockedObject):
    type = "BaseTrafficGen"
    abstract = True
    cxx_header = "cpu/testers/traffic_gen/traffic_gen.hh"
    cxx_class = "gem5::BaseTrafficGen"

    # Port used for sending requests and receiving responses
    port = RequestPort("This port sends requests and receives responses")

    # System used to determine the mode of the memory system
    system = Param.System(Parent.any, "System this generator is part of")

    # Should requests respond to back-pressure or not, if true, the
    # rate of the traffic generator will be slowed down if requests
    # are not immediately accepted
    elastic_req = Param.Bool(
        False, "Slow down requests in case of backpressure"
    )

    # Maximum number of requests waiting for response. Set to 0 for an
    # unlimited number of outstanding requests.
    max_outstanding_reqs = Param.Int(
        0, "Maximum number of outstanding requests"
    )

    # Let the user know if we have waited for a retry and not made any
    # progress for a long period of time. The default value is
    # somewhat arbitrary and may well have to be tuned.
    progress_check = Param.Latency(
        "1ms", "Time before exiting due to lack of progress"
    )

    # Generator type used for applying Stream and/or Substream IDs to requests
    stream_gen = Param.StreamGenType(
        "none", "Generator for adding Stream and/or Substream ID's to requests"
    )

    # Sources for Stream/Substream IDs to apply to requests
    sids = VectorParam.Unsigned([], "StreamIDs to use")
    ssids = VectorParam.Unsigned([], "SubstreamIDs to use")

    # These additional parameters allow TrafficGen to be used with scripts
    # that expect a BaseCPU
    cpu_id = Param.Int(-1, "CPU identifier")
    socket_id = Param.Unsigned(0, "Physical Socket identifier")
    numThreads = Param.Unsigned(1, "number of HW thread contexts")

    @classmethod
    def memory_mode(cls):
        return "timing"

    @classmethod
    def require_caches(cls):
        return False

    def createThreads(self):
        pass

    def createInterruptController(self):
        pass

    def connectCachedPorts(self, in_ports):
        if hasattr(self, "_cached_ports") and (len(self._cached_ports) > 0):
            for p in self._cached_ports:
                exec(f"self.{p} = in_ports")
        else:
            self.port = in_ports

    def connectAllPorts(self, cached_in, uncached_in, uncached_out):
        self.connectCachedPorts(cached_in)

    def connectBus(self, bus):
        self.connectAllPorts(
            bus.cpu_side_ports, bus.cpu_side_ports, bus.mem_side_ports
        )

    def addPrivateSplitL1Caches(self, ic, dc, iwc=None, dwc=None):
        self.dcache = dc
        self.port = dc.cpu_side
        self._cached_ports = ["dcache.mem_side"]
        self._uncached_ports = []
