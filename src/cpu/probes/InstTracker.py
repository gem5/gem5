# Copyright (c) 2024 The Regents of the University of California
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

from m5.objects import SimObject
from m5.objects.Probe import ProbeListenerObject
from m5.params import *
from m5.util.pybind import *


class GlobalInstTracker(SimObject):
    """
    Global instruction tracker that can be used to trigger an exit event when a
    certain number of instructions have been executed across all cores.
    """

    type = "GlobalInstTracker"
    cxx_header = "cpu/probes/inst_tracker.hh"
    cxx_class = "gem5::GlobalInstTracker"

    cxx_exports = [
        PyBindMethod("addThreshold"),
        PyBindMethod("getCounter"),
        PyBindMethod("resetCounter"),
        PyBindMethod("getThresholds"),
        PyBindMethod("resetThresholds"),
    ]

    inst_thresholds = VectorParam.Counter(
        "A list of instruction thresholds to trigger an exit event"
    )


class LocalInstTracker(ProbeListenerObject):
    """
    Local instruction tracker that can be used to listen to one core and
    update the global instruction tracker.
    """

    type = "LocalInstTracker"
    cxx_header = "cpu/probes/inst_tracker.hh"
    cxx_class = "gem5::LocalInstTracker"

    cxx_exports = [
        PyBindMethod("stopListening"),
        PyBindMethod("startListening"),
    ]

    global_inst_tracker = Param.GlobalInstTracker("Global instruction tracker")
    start_listening = Param.Bool(True, "Start listening for instructions")
