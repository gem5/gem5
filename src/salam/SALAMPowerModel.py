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

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class SALAMPowerModel(SimObject):
    # SimObject type
    type = "SALAMPowerModel"
    # gem5-SALAM attached header
    cxx_header = "salam/HWModeling/salam_power_model.hh"

    enabled = Param.Bool(True, "Observe PPA without affecting execution")
    strict = Param.Bool(False, "Reject incomplete modeled PPA coverage")
    keys = VectorParam.String([], "Unique normalized characterization keys")
    ids = VectorParam.String([], "Physical implementation identifiers")
    sources = VectorParam.String([], "Characterization provenance as JSON")
    signatures = VectorParam.String([], "Allowed LLVM type signatures")
    timing_available = VectorParam.Bool(
        [],
        "Whether physical FU latency/issue/pipeline constraints are supplied",
    )
    minimum_latency_s = VectorParam.Float(
        [], "Minimum physical FU operation latency in seconds"
    )
    minimum_issue_interval_s = VectorParam.Float(
        [], "Minimum physical FU admission interval in seconds"
    )
    pipelined = VectorParam.Bool(
        [], "Declared physical FU pipeline organization"
    )
    area_um2 = VectorParam.Float([], "Area in square micrometres")
    leakage_w = VectorParam.Float([], "Leakage power in watts")
    event_j = VectorParam.Float([], "Dynamic energy per operation in joules")
    read_j = VectorParam.Float([], "Dynamic energy per read in joules")
    write_j = VectorParam.Float([], "Dynamic energy per write in joules")
    area_status = VectorParam.String([], "Area coverage and approximation")
    leakage_status = VectorParam.String(
        [], "Leakage coverage and approximation"
    )
    event_status = VectorParam.String([], "Operation-energy coverage")
    read_status = VectorParam.String([], "Read-energy coverage")
    write_status = VectorParam.String([], "Write-energy coverage")
    capacity_bytes = Param.Unsigned(
        0, "Characterized memory capacity in bytes"
    )
    word_bytes = Param.Unsigned(0, "Characterized access width in bytes")
    banks = Param.Unsigned(0, "Characterized physical memory banks")
    read_ports = Param.Unsigned(0, "Characterized dedicated read ports")
    write_ports = Param.Unsigned(0, "Characterized dedicated write ports")
    rw_ports = Param.Unsigned(0, "Characterized read/write ports")
    mask_model = Param.String("unsupported", "Physical masked-write model")
    memory_timing_available = Param.Bool(
        False, "Whether physical memory timing constraints are supplied"
    )
    minimum_access_interval_s = Param.Float(
        0, "Minimum characterized service interval in seconds"
    )
    minimum_access_latency_s = Param.Float(
        0, "Minimum characterized access latency in seconds"
    )
    storage_physical = Param.String("", "Storage implementation assumption")
    storage_scope = Param.String(
        "", "whole_device or storage_cells; cell-only leaves logic unknown"
    )
