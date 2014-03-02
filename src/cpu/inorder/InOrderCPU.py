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
from BranchPredictor import BranchPredictor

class ThreadModel(Enum):
    vals = ['Single', 'SMT', 'SwitchOnCacheMiss']

class InOrderCPU(BaseCPU):
    type = 'InOrderCPU'
    cxx_header = "cpu/inorder/cpu.hh"
    activity = Param.Unsigned(0, "Initial count")

    @classmethod
    def memory_mode(cls):
        return 'timing'

    @classmethod
    def require_caches(cls):
        return True

    @classmethod
    def support_take_over(cls):
        return True

    threadModel = Param.ThreadModel('SMT', "Multithreading model (SE-MODE only)")
    
    cachePorts = Param.Unsigned(2, "Cache Ports")
    stageWidth = Param.Unsigned(4, "Stage width")

    fetchBuffSize = Param.Unsigned(4, "Fetch Buffer Size (Number of Cache Blocks Stored)")
    memBlockSize = Param.Unsigned(64, "Memory Block Size")

    stageTracing = Param.Bool(False, "Enable tracing of each stage in CPU")

    multLatency = Param.Cycles(1, "Latency for Multiply Operations")
    multRepeatRate = Param.Cycles(1, "Repeat Rate for Multiply Operations")
    div8Latency = Param.Cycles(1, "Latency for 8-bit Divide Operations")
    div8RepeatRate = Param.Cycles(1, "Repeat Rate for 8-bit Divide Operations")
    div16Latency = Param.Cycles(1, "Latency for 16-bit Divide Operations")
    div16RepeatRate = Param.Cycles(1, "Repeat Rate for 16-bit Divide Operations")
    div24Latency = Param.Cycles(1, "Latency for 24-bit Divide Operations")
    div24RepeatRate = Param.Cycles(1, "Repeat Rate for 24-bit Divide Operations")
    div32Latency = Param.Cycles(1, "Latency for 32-bit Divide Operations")
    div32RepeatRate = Param.Cycles(1, "Repeat Rate for 32-bit Divide Operations")

    branchPred = Param.BranchPredictor(BranchPredictor(numThreads =
                                                       Parent.numThreads),
                                       "Branch Predictor")
