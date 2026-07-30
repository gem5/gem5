# Copyright 2026 Google, LLC.
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


from m5.objects.Probe import ProbeListenerObject
from m5.params import *
from m5.SimObject import SimObject
from m5.util.pybind import *


class HotPcTracker(ProbeListenerObject):
    """This probe listener tracks the number of times a particular pc has been
    executed so that the user can obtain a list of the N hottest PCs.
    When desired, the user should call getHottestPcs() on this object,
    passing in the number of hottest PCs desired.  This tracking
    optionally supports filtering by range, and grouping
    PCs by coarser granularity.
    """

    type = "HotPcTracker"
    cxx_header = "cpu/probes/hot_pc_tracker.hh"
    cxx_class = "gem5::HotPcTracker"

    cxx_exports = [
        PyBindMethod("getHottestPcs"),
    ]

    granularity = Param.Unsigned(
        0,
        "Number of low bits to mask out when grouping PCs "
        "(0 = exact PC, 1 = 2B alignment, "
        "6 = 64B cache line alignment)",
    )
    filter_ranges = VectorParam.AddrRange(
        [], "Only track PCs within these ranges (empty = track all)"
    )
