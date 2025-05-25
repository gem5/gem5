# Copyright (c) 2024 REDS-HEIG-VD and ESL-EPFL
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
# * Author: Karan Pathak

from m5.objects.BaseMinorCPU import *
from m5.objects.RiscvCPU import RiscvMinorCPU

from gem5.components.processors.base_cpu_core import BaseCPUCore
from gem5.isas import ISA


class RocketIntFU(MinorDefaultIntFU):
    opLat = 1


class RocketIntMulFU(MinorDefaultIntMulFU):
    opLat = 10


class RocketIntDivFU(MinorDefaultIntDivFU):
    opLat = 16


class RocketFloatSimdFU(MinorDefaultFloatSimdFU):
    pass


class RocketPredFU(MinorDefaultPredFU):
    pass


class RocketMemReadFU(MinorDefaultMemFU):
    opClasses = minorMakeOpClassSet(["MemRead", "FloatMemRead"])
    opLat = 1


class RocketMemWriteFU(MinorDefaultMemFU):
    opClasses = minorMakeOpClassSet(["MemWrite", "FloatMemWrite"])
    opLat = 1


class RocketMiscFU(MinorDefaultMiscFU):
    pass


class RocketFUPool(MinorFUPool):
    funcUnits = [
        RocketIntFU(),
        RocketIntMulFU(),
        RocketIntDivFU(),
        RocketFloatSimdFU(),
        RocketPredFU(),
        RocketMemReadFU(),
        RocketMemWriteFU(),
        RocketMiscFU(),
    ]


class RocketBP(GshareBP):
    btb = SimpleBTB(numEntries=32)
    ras = ReturnAddrStack(numEntries=12)
    globalPredictorSize = 16384
    globalCtrBits = 4
    indirectBranchPred = SimpleIndirectPredictor()
    indirectBranchPred.indirectSets = 16


class RocketCPU(RiscvMinorCPU):
    """
    The Rocket core is a 64-bit RISCV core capable of
    booting linux. The core has no prefetchers and comes with Gshare
    branch predictor. One can configure some other branch predictor.
    Rest assured the parameters match the post-validation, application
    of "component-level" calibration methodology.
    """

    threadPolicy = "RoundRobin"

    # Fetch1 stage
    fetch1LineSnapWidth = 8
    fetch1LineWidth = 8
    fetch1FetchLimit = 1
    fetch1ToFetch2ForwardDelay = 1
    fetch1ToFetch2BackwardDelay = 0

    # Fetch2 stage
    fetch2InputBufferSize = 1
    fetch2ToDecodeForwardDelay = 1
    fetch2CycleInput = True

    # Decode stage
    decodeInputBufferSize = 1
    decodeToExecuteForwardDelay = 1
    decodeInputWidth = 1
    decodeCycleInput = True

    # Execute stage
    executeInputWidth = 1
    executeCycleInput = True
    executeIssueLimit = 2
    executeMemoryIssueLimit = 1
    executeCommitLimit = 2
    executeMemoryCommitLimit = 1
    executeInputBufferSize = 7
    executeMaxAccessesInMemory = 3
    executeLSQMaxStoreBufferStoresPerCycle = 2
    executeLSQRequestsQueueSize = 1
    executeLSQTransfersQueueSize = 2
    executeLSQStoreBufferSize = 5
    executeBranchDelay = 3
    executeSetTraceTimeOnCommit = True
    executeSetTraceTimeOnIssue = False
    executeAllowEarlyMemoryIssue = True
    enableIdling = True
    executeMemoryWidth = 8
    # Functional Units and Branch Prediction
    executeFuncUnits = RocketFUPool()
    branchPred = RocketBP()


class RocketCore(BaseCPUCore):
    """

    .. note::

        The GshareBP deviates from the actual GshareBP.
        This configuration performs the best in relation to the hardware.
    """

    def __init__(
        self,
        core_id,
    ):
        super().__init__(core=RocketCPU(cpu_id=core_id), isa=ISA.RISCV)
        self.core.isa[0].enable_rvv = False
