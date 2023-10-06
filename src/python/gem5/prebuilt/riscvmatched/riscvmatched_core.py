# Copyright (c) 2022 The Regents of the University of California
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
from typing import Optional

from gem5.components.processors.base_cpu_core import BaseCPUCore
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.utils.override import overrides
from gem5.utils.requires import requires
from m5.objects import BaseCPU
from m5.objects import BaseMMU
from m5.objects import Port
from m5.objects import Process
from m5.objects.BaseMinorCPU import *
from m5.objects.RiscvCPU import RiscvMinorCPU


class U74IntFU(MinorDefaultIntFU):
    opLat = 1


class U74IntMulFU(MinorDefaultIntMulFU):
    opLat = 3


class U74IntDivFU(MinorDefaultIntDivFU):
    opLat = 6


class U74FloatSimdFU(MinorDefaultFloatSimdFU):
    pass


class U74PredFU(MinorDefaultPredFU):
    pass


class U74MemReadFU(MinorDefaultMemFU):
    opClasses = minorMakeOpClassSet(["MemRead", "FloatMemRead"])
    opLat = 2


class U74MemWriteFU(MinorDefaultMemFU):
    opClasses = minorMakeOpClassSet(["MemWrite", "FloatMemWrite"])
    opLat = 2


class U74MiscFU(MinorDefaultMiscFU):
    pass


class U74VecFU(MinorDefaultVecFU):
    pass


class U74FUPool(MinorFUPool):
    funcUnits = [
        U74IntFU(),
        U74IntFU(),
        U74IntMulFU(),
        U74IntDivFU(),
        U74FloatSimdFU(),
        U74PredFU(),
        U74MemReadFU(),
        U74MemWriteFU(),
        U74MiscFU(),
        U74VecFU(),
    ]


class U74BP(TournamentBP):
    BTBEntries = 32
    RASSize = 12
    localHistoryTableSize = 4096  # is 3.6 KiB but gem5 requires power of 2
    localPredictorSize = 16384
    globalPredictorSize = 16384
    choicePredictorSize = 16384
    localCtrBits = 4
    globalCtrBits = 4
    choiceCtrBits = 4
    indirectBranchPred = SimpleIndirectPredictor()
    indirectBranchPred.indirectSets = 16


class U74CPU(RiscvMinorCPU):
    """
    The fetch, decode, and execute stage parameters from the ARM HPI CPU
    This information about the CPU can be found on page 15 of
    gem5_rsk_gem5-21.2.pdf at https://github.com/arm-university/arm-gem5-rsk

    The parameters that are changed are:
    - threadPolicy:
        This is initialized to "SingleThreaded".
    - decodeToExecuteForwardDelay:
        This is changed from 1 to 2 to avoid a PMC address fault.
    - fetch1ToFetch2BackwardDelay:
        This is changed from 1 to 0 to better match hardware performance.
    - fetch2InputBufferSize:
        This is changed from 2 to 1 to better match hardware performance.
    - decodeInputBufferSize:
        This is changed from 3 to 2 to better match hardware performance.
    - decodeToExecuteForwardDelay:
        This is changed from 2 to 1 to better match hardware performance.
    - executeInputBufferSize:
        This is changed from 7 to 4 to better match hardware performance.
    - executeMaxAccessesInMemory:
        This is changed from 2 to 1 to better match hardware performance.
    - executeLSQStoreBufferSize:
        This is changed from 5 to 3 to better match hardware performance.
    - executeBranchDelay:
        This is changed from 1 to 2 to better match hardware performance.
    - enableIdling:
        This is changed to False to better match hardware performance.

    """

    threadPolicy = "SingleThreaded"

    # Fetch1 stage
    fetch1LineSnapWidth = 0
    fetch1LineWidth = 0
    fetch1FetchLimit = 1
    fetch1ToFetch2ForwardDelay = 1
    fetch1ToFetch2BackwardDelay = 0

    # Fetch2 stage
    fetch2InputBufferSize = 1
    fetch2ToDecodeForwardDelay = 1
    fetch2CycleInput = True

    # Decode stage
    decodeInputBufferSize = 2
    decodeToExecuteForwardDelay = 1
    decodeInputWidth = 2
    decodeCycleInput = True

    # Execute stage
    executeInputWidth = 2
    executeCycleInput = True
    executeIssueLimit = 2
    executeMemoryIssueLimit = 1
    executeCommitLimit = 2
    executeMemoryCommitLimit = 1
    executeInputBufferSize = 4
    executeMaxAccessesInMemory = 1
    executeLSQMaxStoreBufferStoresPerCycle = 2
    executeLSQRequestsQueueSize = 1
    executeLSQTransfersQueueSize = 2
    executeLSQStoreBufferSize = 3
    executeBranchDelay = 2
    executeSetTraceTimeOnCommit = True
    executeSetTraceTimeOnIssue = False
    executeAllowEarlyMemoryIssue = True
    enableIdling = False

    # Functional Units and Branch Prediction
    executeFuncUnits = U74FUPool()
    branchPred = U74BP()


class U74Core(BaseCPUCore):
    """
    U74Core models the core of the HiFive Unmatched board.
    The core has a single thread.
    The latencies of the functional units are set to values found in Table 8 on page 40.
      - IntFU: 1 cycle
      - IntMulFU: 3 cycles
      - IntDivFU: 6 cycles (NOTE: latency is variable, but is set to 6 cycles)
      - MemReadFU: 2 cycles
      - MemWriteFU: 2 cycles
    The branch predictor is a TournamentBP, based on Section 4.2.5 on page 38.
      - BTBEntries: 32 entries
      - RASSize: 12 entries
      - IndirectSets: 16 sets
      - localPredictorSize: 16384
      - globalPredictorSize: 16384
      - choicePredictorSize: 16384
      - localCtrBits: 4
      - globalCtrBits: 4
      - choiceCtrBits: 4
      - localHistoryTableSize: 4096 B
    NOTE: The TournamentBP deviates from the actual BP.
    This configuration performs the best in relation to the hardware.
    """

    def __init__(
        self,
        core_id,
    ):
        super().__init__(core=U74CPU(cpu_id=core_id), isa=ISA.RISCV)
