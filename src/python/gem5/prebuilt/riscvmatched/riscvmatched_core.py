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
from gem5.utils.requires import requires
from gem5.components.processors.base_cpu_core import BaseCPUCore
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.utils.override import overrides
from m5.objects.RiscvCPU import RiscvMinorCPU
from m5.objects import (
    BaseMMU,
    Port,
    BaseCPU,
    Process,
)
from m5.objects.BaseMinorCPU import *
from gem5.isas import ISA


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


class U74MemFU(MinorDefaultMemFU):
    opLat = 3


class U74MiscFU(MinorDefaultMiscFU):
    pass


class U74FUPool(MinorFUPool):
    funcUnits = [
        U74IntFU(),
        U74IntFU(),
        U74IntMulFU(),
        U74IntDivFU(),
        U74FloatSimdFU(),
        U74PredFU(),
        U74MemFU(),
        U74MiscFU(),
    ]


class U74BP(TournamentBP):
    BTBEntries = 16
    RASSize = 6
    localHistoryTableSize = 4096  # is 3.6 KiB but gem5 requires power of 2

    indirectBranchPred = SimpleIndirectPredictor()
    indirectBranchPred.indirectSets = 8


class U74CPU(RiscvMinorCPU):
    """
    The fetch, decode, and execute stage parameters from the ARM HPI CPU
    This information about the CPU can be found on page 15 of
    gem5_rsk_gem5-21.2.pdf at https://github.com/arm-university/arm-gem5-rsk

    The only parameter that is changed is the decodeToExecuteForwardDelay.
    This is changed from 1 to 2 to avoid a PMC address fault.

    """

    # Fetch1 stage
    fetch1LineSnapWidth = 0
    fetch1LineWidth = 0
    fetch1FetchLimit = 1
    fetch1ToFetch2ForwardDelay = 1
    fetch1ToFetch2BackwardDelay = 1

    # Fetch2 stage
    fetch2InputBufferSize = 2
    fetch2ToDecodeForwardDelay = 1
    fetch2CycleInput = True

    # Decode stage
    decodeInputBufferSize = 3
    decodeToExecuteForwardDelay = 2
    decodeInputWidth = 2
    decodeCycleInput = True

    # Execute stage
    executeInputWidth = 2
    executeCycleInput = True
    executeIssueLimit = 2
    executeMemoryIssueLimit = 1
    executeCommitLimit = 2
    executeMemoryCommitLimit = 1
    executeInputBufferSize = 7
    executeMaxAccessesInMemory = 2
    executeLSQMaxStoreBufferStoresPerCycle = 2
    executeLSQRequestsQueueSize = 1
    executeLSQTransfersQueueSize = 2
    executeLSQStoreBufferSize = 5
    executeBranchDelay = 1
    executeSetTraceTimeOnCommit = True
    executeSetTraceTimeOnIssue = False
    executeAllowEarlyMemoryIssue = True
    enableIdling = True

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
      - MemFU: 3 cycles
    The branch predictor is a TournamentBP, based on Section 4.2.5 on page 38.
      - BTBEntries: 16 entries
      - RASSize: 6 entries
      - IndirectSets: 8 sets
      - localHistoryTableSize: 4096 B
    NOTE: The BHT of the HiFive Board is 3.6KiB but gem5 requires a power of 2, so the BHT is 4096B.
    """

    def __init__(
        self,
        core_id,
    ):
        super().__init__(core=U74CPU(cpu_id=core_id), isa=ISA.RISCV)
