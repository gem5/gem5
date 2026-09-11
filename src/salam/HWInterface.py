# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

from m5.objects.CycleCounts import CycleCounts
from m5.objects.FunctionalUnits import FunctionalUnits
from m5.objects.HWStatistics import HWStatistics
from m5.objects.InstConfig import InstConfig
from m5.objects.InstOpCodes import InstOpCodes
from m5.objects.SALAMPowerModel import SALAMPowerModel
from m5.objects.SimulatorConfig import SimulatorConfig
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class HWInterface(SimObject):
    type = "HWInterface"
    cxx_header = "salam/HWModeling/hw_interface.hh"

    cycle_counts = Param.CycleCounts(Parent.any, "Cycle Counts")
    functional_units = Param.FunctionalUnits(Parent.any, "Functional Units")
    hw_statistics = Param.HWStatistics(Parent.any, "Hardware Statistics")
    inst_config = Param.InstConfig(Parent.any, "Instruction Configuration")
    salam_power_model = Param.SALAMPowerModel(Parent.any, "SALAM Power Model")
    simulator_config = Param.SimulatorConfig(
        Parent.any, "Simulation Configuration"
    )
    opcodes = Param.InstOpCodes(
        Parent.any, "Instruction LLVM OpCode Enumeration to SALAM Type"
    )
