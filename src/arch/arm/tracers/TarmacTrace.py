# Copyright (c) 2018 ARM Limited
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
# Authors: Giacomo Gabrielli
#          Giacomo Travaglini

from m5.SimObject import SimObject
from m5.params import *
from m5.objects.InstTracer import InstTracer

class TarmacParser(InstTracer):
    type = 'TarmacParser'
    cxx_class = 'Trace::TarmacParser'
    cxx_header = "arch/arm/tracers/tarmac_parser.hh"

    path_to_trace = Param.String("tarmac.log", "path to TARMAC trace")

    start_pc = Param.Int(
        0x0, "tracing starts when the PC gets this value; ignored if 0x0")

    exit_on_diff = Param.Bool(False,
        "stop simulation after first mismatch is detected")

    exit_on_insn_diff = Param.Bool(False,
        "stop simulation after first mismatch on PC or opcode is detected")

    mem_wr_check = Param.Bool(False,
        "enable check of memory write accesses")

    cpu_id = Param.Bool(False,
        "true if trace format includes the CPU id")

    ignore_mem_addr = Param.AddrRange(AddrRange(0, size=0),
        "Range of unverifiable memory addresses")

class TarmacTracer(InstTracer):
    type = 'TarmacTracer'
    cxx_class = 'Trace::TarmacTracer'
    cxx_header = "arch/arm/tracers/tarmac_tracer.hh"

    start_tick = Param.Tick(0,
        "tracing starts when the tick time gets this value")

    end_tick = Param.Tick(MaxTick,
        "tracing ends when the tick time gets this value")
