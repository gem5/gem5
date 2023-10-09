# Copyright (c) 2013 - 2015 ARM Limited
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
from m5.objects.Probe import *


class ElasticTrace(ProbeListenerObject):
    type = "ElasticTrace"
    cxx_class = "gem5::o3::ElasticTrace"
    cxx_header = "cpu/o3/probe/elastic_trace.hh"

    # Trace files for the following params are created in the output directory.
    # User is forced to provide these when an instance of this class is created.
    instFetchTraceFile = Param.String(
        desc="Protobuf trace file name for instruction fetch tracing"
    )
    dataDepTraceFile = Param.String(
        desc="Protobuf trace file name for data dependency tracing"
    )
    # The dependency window size param must be equal to or greater than the
    # number of entries in the O3CPU ROB, a typical value is 3 times ROB size
    depWindowSize = Param.Unsigned(
        desc="Instruction window size used for "
        "recording and processing data "
        "dependencies"
    )
    # The committed instruction count from which to start tracing
    startTraceInst = Param.UInt64(
        0,
        "The number of committed instructions "
        "after which to start tracing. Default "
        "zero means start tracing from first "
        "committed instruction.",
    )
    # Whether to trace virtual addresses for memory accesses
    traceVirtAddr = Param.Bool(
        False, "Set to true if virtual addresses are to be traced."
    )
