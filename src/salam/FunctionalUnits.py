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


import copy

from m5 import _salam_profile
from m5._salam_config import install_profile as _install_profile
from m5.params import (
    Param,
    VectorParam,
)
from m5.SimObject import SimObject


class FunctionalUnit(SimObject):
    type = "FunctionalUnit"
    cxx_class = "gem5::FunctionalUnit"
    cxx_header = "salam/HWModeling/functional_units.hh"

    alias = Param.String("Functional unit name")
    enum_value = Param.UInt32("Functional unit identifier")
    limit = Param.UInt32("Instance limit; zero uses the static graph count")
    initiation_interval = Param.UInt32(
        "Cycles between admissions; zero selects an unpipelined unit"
    )

    # Profile metadata; execution latency is selected through CycleCounts.
    stages = Param.UInt32("Descriptive stage count")
    cycles = Param.UInt32("Descriptive cycle count")
    int_size = Param.String("Descriptive integer size")
    int_sign = Param.String("Descriptive integer signedness")
    int_apmode = Param.Bool("Descriptive integer arbitrary precision mode")
    fp_size = Param.String("Descriptive floating-point size")
    fp_sign = Param.String("Descriptive floating-point signedness")
    fp_apmode = Param.Bool(
        "Descriptive floating-point arbitrary precision mode"
    )
    ptr_size = Param.String("Descriptive pointer size")
    ptr_sign = Param.String("Descriptive pointer signedness")
    ptr_apmode = Param.Bool("Descriptive pointer arbitrary precision mode")

    # Legacy metadata is separate from SALAMPowerModel characterization.
    power_units = Param.String("Legacy power unit")
    energy_units = Param.String("Legacy energy unit")
    time_units = Param.String("Legacy time unit")
    area_units = Param.String("Legacy area unit")
    fu_latency = Param.UInt32("Legacy functional unit latency")
    internal_power = Param.Float("Legacy internal power")
    switch_power = Param.Float("Legacy switching power")
    dynamic_power = Param.Float("Legacy dynamic power")
    dynamic_energy = Param.Float("Legacy dynamic energy")
    leakage_power = Param.Float("Legacy leakage power")
    area = Param.Float("Legacy area")
    path_delay = Param.Float("Legacy path delay")


class FunctionalUnits(SimObject):
    type = "FunctionalUnits"
    cxx_class = "gem5::FunctionalUnits"
    cxx_header = "salam/HWModeling/functional_units.hh"

    functional_unit_list = VectorParam.FunctionalUnit(
        [], "Functional units in profile order"
    )

    @classmethod
    def from_profile(cls):
        return cls()

    @classmethod
    def characterization(cls):
        return copy.deepcopy(_salam_profile.CHARACTERIZATION)


_install_profile(
    FunctionalUnits,
    FunctionalUnit,
    "functional_unit_list",
    _salam_profile.FUNCTIONAL_UNITS,
    _salam_profile.PROFILE_PATH,
)
