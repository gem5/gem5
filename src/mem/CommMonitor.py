# Copyright (c) 2012 ARM Limited
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

from m5.params import *
from m5.proxy import *
from m5.objects.System import System
from m5.SimObject import SimObject

# The communication monitor will most typically be used in combination
# with periodic dumping and resetting of stats using schedStatEvent
class CommMonitor(SimObject):
    type = "CommMonitor"
    cxx_header = "mem/comm_monitor.hh"
    cxx_class = "gem5::CommMonitor"

    system = Param.System(Parent.any, "System that the monitor belongs to.")

    # one port in each direction
    mem_side_port = RequestPort(
        "This port sends requests and receives responses"
    )
    master = DeprecatedParam(
        mem_side_port, "`master` is now called `mem_side_port`"
    )
    cpu_side_port = ResponsePort(
        "This port receives requests and sends responses"
    )
    slave = DeprecatedParam(
        cpu_side_port, "`slave` is now called `cpu_side_port`"
    )

    # control the sample period window length of this monitor
    sample_period = Param.Clock("1ms", "Sample period for histograms")

    # for each histogram, set the number of bins and enable the user
    # to disable the measurement, reads and writes use the same
    # parameters

    # histogram of burst length of packets (not using sample period)
    burst_length_bins = Param.Unsigned(
        "20", "# bins in burst length histograms"
    )
    disable_burst_length_hists = Param.Bool(
        False, "Disable burst length histograms"
    )

    # bandwidth per sample period
    bandwidth_bins = Param.Unsigned("20", "# bins in bandwidth histograms")
    disable_bandwidth_hists = Param.Bool(False, "Disable bandwidth histograms")

    # latency from request to response (not using sample period)
    latency_bins = Param.Unsigned("20", "# bins in latency histograms")
    disable_latency_hists = Param.Bool(False, "Disable latency histograms")

    # inter transaction time (ITT) distributions in uniformly sized
    # bins up to the maximum, independently for read-to-read,
    # write-to-write and the combined request-to-request that does not
    # separate read and write requests
    itt_bins = Param.Unsigned("20", "# bins in ITT distributions")
    itt_max_bin = Param.Latency("100ns", "Max bin of ITT distributions")
    disable_itt_dists = Param.Bool(False, "Disable ITT distributions")

    # outstanding requests (that did not yet get a response) per
    # sample period
    outstanding_bins = Param.Unsigned(
        "20", "# bins in outstanding requests histograms"
    )
    disable_outstanding_hists = Param.Bool(
        False, "Disable outstanding requests histograms"
    )

    # transactions (requests) observed per sample period
    transaction_bins = Param.Unsigned(
        "20", "# bins in transaction count histograms"
    )
    disable_transaction_hists = Param.Bool(
        False, "Disable transaction count histograms"
    )

    # address distributions (heatmaps) with associated address masks
    # to selectively only look at certain bits of the address
    read_addr_mask = Param.Addr(MaxAddr, "Address mask for read address")
    write_addr_mask = Param.Addr(MaxAddr, "Address mask for write address")
    disable_addr_dists = Param.Bool(True, "Disable address distributions")
