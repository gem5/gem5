# Copyright (c) 2007 MIPS Technologies, Inc.
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
#
# Authors: Korey Sewell

from m5.params import *
from m5.proxy import *
from BaseCPU import BaseCPU

class ThreadModel(Enum):
    vals = ['Single', 'SMT', 'SwitchOnCacheMiss']

class InOrderCPU(BaseCPU):
    type = 'InOrderCPU'
    activity = Param.Unsigned(0, "Initial count")

    threadModel = Param.ThreadModel('SMT', "Multithreading model (SE-MODE only)")
    
    cachePorts = Param.Unsigned(2, "Cache Ports")
    stageWidth = Param.Unsigned(4, "Stage width")

    fetchBuffSize = Param.Unsigned(4, "Fetch Buffer Size (Number of Cache Blocks Stored)")
    memBlockSize = Param.Unsigned(64, "Memory Block Size")

    predType = Param.String("tournament", "Branch predictor type ('local', 'tournament')")
    localPredictorSize = Param.Unsigned(2048, "Size of local predictor")
    localCtrBits = Param.Unsigned(2, "Bits per counter")
    localHistoryTableSize = Param.Unsigned(2048, "Size of local history table")
    localHistoryBits = Param.Unsigned(11, "Bits for the local history")
    globalPredictorSize = Param.Unsigned(8192, "Size of global predictor")
    globalCtrBits = Param.Unsigned(2, "Bits per counter")
    globalHistoryBits = Param.Unsigned(13, "Bits of history")
    choicePredictorSize = Param.Unsigned(8192, "Size of choice predictor")
    choiceCtrBits = Param.Unsigned(2, "Bits of choice counters")

    BTBEntries = Param.Unsigned(4096, "Number of BTB entries")
    BTBTagSize = Param.Unsigned(16, "Size of the BTB tags, in bits")

    RASSize = Param.Unsigned(16, "RAS size")

    instShiftAmt = Param.Unsigned(2, "Number of bits to shift instructions by")
    functionTrace = Param.Bool(False, "Enable function trace")
    functionTraceStart = Param.Tick(0, "Cycle to start function trace")
    stageTracing = Param.Bool(False, "Enable tracing of each stage in CPU")

    multLatency = Param.Unsigned(1, "Latency for Multiply Operations")
    multRepeatRate = Param.Unsigned(1, "Repeat Rate for Multiply Operations")
    div8Latency = Param.Unsigned(1, "Latency for 8-bit Divide Operations")
    div8RepeatRate = Param.Unsigned(1, "Repeat Rate for 8-bit Divide Operations")
    div16Latency = Param.Unsigned(1, "Latency for 16-bit Divide Operations")
    div16RepeatRate = Param.Unsigned(1, "Repeat Rate for 16-bit Divide Operations")
    div24Latency = Param.Unsigned(1, "Latency for 24-bit Divide Operations")
    div24RepeatRate = Param.Unsigned(1, "Repeat Rate for 24-bit Divide Operations")
    div32Latency = Param.Unsigned(1, "Latency for 32-bit Divide Operations")
    div32RepeatRate = Param.Unsigned(1, "Repeat Rate for 32-bit Divide Operations")
