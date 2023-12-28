# Copyright (c) 2012, 2015-2017 ARM Limited
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
#

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


# Enumerate set of allowed power states that can be used by a clocked object.
# The list is kept generic to express a base minimal set.
# State definition :-
#   Undefined: Invalid state, no power state derived information is available.
#   On: The logic block is actively running and consuming dynamic and leakage
#       energy depending on the amount of processing required.
#   Clk_gated: The clock circuity within the block is gated to save dynamic
#              energy, the power supply to the block is still on and leakage
#              energy is being consumed by the block.
#   Sram_retention: The SRAMs within the logic blocks are pulled into retention
#                   state to reduce leakage energy further.
#   Off: The logic block is power gated and is not consuming any energy.
class PwrState(Enum):
    vals = ["UNDEFINED", "ON", "CLK_GATED", "SRAM_RETENTION", "OFF"]


class PowerState(SimObject):
    type = "PowerState"
    cxx_header = "sim/power_state.hh"
    cxx_class = "gem5::PowerState"

    # Provide initial power state, should ideally get redefined in startup
    # routine
    default_state = Param.PwrState("UNDEFINED", "Default Power State")

    # Possible power states this object can be in sorted from the most
    # to the least performant
    possible_states = VectorParam.PwrState(
        [], "Power states this object can be in"
    )

    clk_gate_min = Param.Latency("1ns", "Min value of the distribution")
    clk_gate_max = Param.Latency("1s", "Max value of the distribution")
    clk_gate_bins = Param.Unsigned("20", "# bins in clk gated distribution")

    # The objects which drive the power state of this object. If the list is
    # empty, the object determines its power state independently.
    leaders = VectorParam.PowerState(
        [], "Objects which drive the power state of this object"
    )
