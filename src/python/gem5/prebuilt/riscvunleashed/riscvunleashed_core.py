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


class U54IntFU(MinorDefaultIntFU):
    opLat = 2


class U54IntMulFU(MinorDefaultIntMulFU):
    opLat = 3


class U54IntDivFU(MinorDefaultIntDivFU):
    opLat = 19


class U54FloatSimdFU(MinorDefaultFloatSimdFU):
    pass


class U54PredFU(MinorDefaultPredFU):
    pass


class U54MemReadFU(MinorDefaultMemFU):
    opClasses = minorMakeOpClassSet(["MemRead", "FloatMemRead"])
    opLat = 3


class U54MemWriteFU(MinorDefaultMemFU):
    opClasses = minorMakeOpClassSet(["MemWrite", "FloatMemWrite"])
    opLat = 1


class U54MiscFU(MinorDefaultMiscFU):
    pass


class U54FUPool(MinorFUPool):
    funcUnits = [
        U54IntFU(),
        U54IntMulFU(),
        U54IntDivFU(),
        U54FloatSimdFU(),
        U54PredFU(),
        U54MemReadFU(),
        U54MemWriteFU(),
        U54MiscFU(),
    ]


class U54BP(MultiperspectivePerceptron64KB):
    budgetbits = 65536 * 8 + 2048
    num_local_histories = 510
    num_filter_entries = 18025
    imli_mask1 = 0xC1000
    imli_mask4 = 0x80008000
    recencypos_mask = 0x100000090
    indirectBranchPred = SimpleIndirectPredictor()
    indirectBranchPred.indirectSets = 256
    btb = SimpleBTB(numEntries=4096)
    ras = ReturnAddrStack(numEntries=16)


class U54CPU(RiscvMinorCPU):
    """
    The U54 core is a 64-bit RISCV core capable of
    booting linux. The core has no prefetchers and comes
    with Multiperspective branch predictor model that
    has shown good results with respect to the hardware.
    Rest assured the parameters match the post-validation, application
    of "component-level" calibration methodology.


    """

    threadPolicy = "RoundRobin"

    # Fetch1 stage
    fetch1LineSnapWidth = 16
    fetch1LineWidth = 16
    fetch1FetchLimit = 1
    fetch1ToFetch2ForwardDelay = 1
    fetch1ToFetch2BackwardDelay = 0

    # Fetch2 stage
    fetch2InputBufferSize = 2
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
    executeMaxAccessesInMemory = 2
    executeLSQMaxStoreBufferStoresPerCycle = 2
    executeLSQRequestsQueueSize = 1
    executeLSQTransfersQueueSize = 2
    executeLSQStoreBufferSize = 5
    executeBranchDelay = 3
    executeSetTraceTimeOnCommit = True
    executeSetTraceTimeOnIssue = False
    executeAllowEarlyMemoryIssue = True
    enableIdling = True
    executeMemoryWidth = 16

    # Functional Units and Branch Prediction
    executeFuncUnits = U54FUPool()
    branchPred = U54BP()


class U54Core(BaseCPUCore):
    """


    .. note::

        The Multiperspective branch predictor is chosen.
        This configuration performs the best in relation to the hardware.
    """

    def __init__(
        self,
        core_id,
    ):
        super().__init__(core=U54CPU(cpu_id=core_id), isa=ISA.RISCV)
        self.core.isa[0].enable_rvv = False
