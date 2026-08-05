# Copyright (c) 2026 Rajesh Gangam
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

from m5.objects.Probe import *
from m5.params import *


class ETrace(ProbeListenerObject):
    type = "ETrace"
    cxx_class = "gem5::o3::ETrace"
    cxx_header = "cpu/o3/probe/etrace.hh"

    traceFile = Param.String(
        "etrace.pb.gz", "Output file for E-Trace packets (protobuf)"
    )
    startTraceInst = Param.UInt64(
        0, "Instruction count after which to start tracing"
    )
    resyncPeriod = Param.UInt64(
        10000, "Instructions between periodic resync packets"
    )

    # Implicit exception mode
    implicitException = Param.Bool(
        False, "Omit trap handler address when inferrable from mtvec/stvec"
    )

    # Implicit return mode
    implicitReturn = Param.Bool(
        False, "Enable implicit return tracking to reduce trace size"
    )
    callCounterSizeP = Param.UInt32(
        0, "Log2 of call counter max value (0 disables)"
    )

    # Branch prediction mode
    branchPrediction = Param.Bool(
        False, "Enable branch prediction compression in trace"
    )
    bpredSizeP = Param.UInt32(
        0, "Log2 of branch predictor table entries (0 disables)"
    )

    # Jump target cache mode
    jumpTargetCache = Param.Bool(
        False, "Enable jump target cache compression"
    )
    cacheSizeP = Param.UInt32(
        0, "Log2 of jump target cache entries (0 disables)"
    )

    # Sequentially inferable jump mode
    sijump = Param.Bool(
        False, "Enable sequentially inferable jump detection (lui/auipc+jalr)"
    )

    # Data trace
    dataTrace = Param.Bool(False, "Enable data trace")
    dataTraceMode = Param.UInt32(
        0, "Data trace mode: 0=addr+data, 1=addr-only, 2=data-only"
    )
    dataTraceFile = Param.String(
        "etrace_data.pb.gz", "Output file for data trace packets"
    )

    # Context tracking
    contextWidth = Param.UInt32(
        0, "Width of context field (0 disables, 16 for ASID)"
    )

    # Filtering
    filterPriv = Param.UInt32(
        0xF, "Bitmask of privilege levels to trace (bit0=U, bit1=S, bit3=M)"
    )
    filterAddrStart = Param.Addr(0, "Start of address range filter (0=off)")
    filterAddrEnd = Param.Addr(0, "End of address range filter (0=off)")
