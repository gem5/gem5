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


class HWStatistics(SimObject):
    # SimObject type
    type = "HWStatistics"
    # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/hw_statistics.hh"
    ### Templates
    ### YML Type: statistics
    ## 'quick_stats' = Param.Bool(quick_stats,
    ##     "Optimized for Runtime Performance")
    ## 'detailed_stats' = Param.Bool(detailed_stats,
    ##     "Generate full Runtime Statistics, Impacts Performance")
    ### YML Type: statistics.output_format
    ## 'terminal' = Param.Bool(terminal, "Print Results to Terminal")
    ## 'to_file' = Param.Bool(file, "Print Results to File")
    ## 'to_csv' = Param.Bool(csv, "Print Results in CSV Format")
    ### YML Type: statistics.results
    ## 'runtime' = Param.Bool(runtime,
    ##     "Simulation Real and CPU Runtime Results")
    ## 'performance' = Param.Bool(performance,
    ##     "Simulation Cycle Performance Results")
    ## 'power' = Param.Bool(power, "Simulation Power Results")
    ## 'area' = Param.Bool(area, "Simulation Area Results")
    ## 'fu_occupancy' = Param.Bool(occupancy.function_units,
    ##     "Functional Unit Occupancy Results")
    ## 'runtime_queues' = Param.Bool(occupancy.runtime_queues,
    ##     "Runtime Queue Occupancy Results")
    ## 'full_trace' = Param.Bool(occupancy.full_trace,
    ##     "Detailed Occupancy Tracking, Cycle Accurate")
    ## 'params' = Param.Bool(params, "Print All Defined Configurations")
    ## 'inst_usage' = Param.Bool(inst_usage, "Usage count of each Instruction")
    ## 'memory' = Param.Bool(memory, "Memory Usage Results")
