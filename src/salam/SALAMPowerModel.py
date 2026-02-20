# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class SALAMPowerModel(SimObject):
    # SimObject type
    type = "SALAMPowerModel"
    # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/salam_power_model.hh"
    ### Templates
    ### YML Type: functional_unit.power_model
    ## 'power_units' = Param.String(units.power, "Hardware Model Power Units")
    ## 'energy_units' = Param.String(units.energy, "Hardware Model Energy
    ##                  Units")
    ## 'time_units' = Param.String(units.time, "Hardware Model Time Units")
    ## 'area_units' = Param.String(units.area, "Hardware Model Area Units")
    ## 'latency' = Params.UInt32(latency, "Hardware Model Functional Unit
    ##             Latency")
    ## 'internal_power' = Params.Double(internal_power, "Measured Power
    ##                    Metric")
    ## 'switch_power' = Params.Double(switch_power, "Measured Power Metric")
    ## 'dynamic_power' = Params.Double(dynamic_power, "Measured Power Metric")
    ## 'dynamic_energy' = Params.Double(dynamic_energy, "Measured Energy
    ##                    Metric")
    ## 'leakage_power' = Params.Double(leakage_power, "Measured Power Metric")
    ## 'area' = Params.Double(area, "Measure Area Metric")
    ## 'path_delay' = Params.Double(path_delay, "Measured Path Delay Metric")
