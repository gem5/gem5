# Copyright (c) 2013 - 2016, 2023 Arm Limited
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


class TraceCPU(ClockedObject):
    """Trace CPU model which replays traces generated in a prior simulation
    using DerivO3CPU or its derived classes. It interfaces with L1 caches.
    """

    type = "TraceCPU"
    cxx_header = "cpu/trace/trace_cpu.hh"
    cxx_class = "gem5::TraceCPU"

    @classmethod
    def memory_mode(cls):
        return "timing"

    @classmethod
    def require_caches(cls):
        return True

    system = Param.System(Parent.any, "system object")

    icache_port = RequestPort("Instruction Port")
    dcache_port = RequestPort("Data Port")
    instTraceFile = Param.String("", "Instruction trace file")
    dataTraceFile = Param.String("", "Data dependency trace file")
    sizeStoreBuffer = Param.Unsigned(
        16, "Number of entries in the store buffer"
    )
    sizeLoadBuffer = Param.Unsigned(16, "Number of entries in the load buffer")
    sizeROB = Param.Unsigned(40, "Number of entries in the re-order buffer")

    # Frequency multiplier used to effectively scale the Trace CPU frequency
    # either up or down. Note that the Trace CPU's clock domain must also be
    # changed when frequency is scaled. A default value of 1.0 means the same
    # frequency as was used for generating the traces.
    freqMultiplier = Param.Float(
        1.0, "Multiplier scale the Trace CPU frequency up or down"
    )

    # Enable exiting when any one Trace CPU completes execution which is set to
    # false by default
    enableEarlyExit = Param.Bool(
        False, "Exit when any one Trace CPU completes execution"
    )

    # If progress msg interval is set to a non-zero value, it is treated as
    # the interval of committed instructions at which an info message is
    # printed.
    progressMsgInterval = Param.Unsigned(
        0,
        "Interval of committed "
        "instructions at which to print a"
        " progress msg",
    )
