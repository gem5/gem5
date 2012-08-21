# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Kevin Lim

from m5.defines import buildEnv
from m5.params import *
from BaseCPU import BaseCPU
from OzoneChecker import OzoneChecker

class DerivOzoneCPU(BaseCPU):
    type = 'DerivOzoneCPU'

    numThreads = Param.Unsigned("number of HW thread contexts")

    width = Param.Unsigned("Width")
    frontEndWidth = Param.Unsigned("Front end width")
    frontEndLatency = Param.Unsigned("Front end latency")
    backEndWidth = Param.Unsigned("Back end width")
    backEndSquashLatency = Param.Unsigned("Back end squash latency")
    backEndLatency = Param.Unsigned("Back end latency")
    maxInstBufferSize = Param.Unsigned("Maximum instruction buffer size")
    maxOutstandingMemOps = Param.Unsigned("Maximum number of outstanding memory operations")
    decodeToFetchDelay = Param.Unsigned("Decode to fetch delay")
    renameToFetchDelay = Param.Unsigned("Rename to fetch delay")
    iewToFetchDelay = Param.Unsigned("Issue/Execute/Writeback to fetch "
               "delay")
    commitToFetchDelay = Param.Unsigned("Commit to fetch delay")
    fetchWidth = Param.Unsigned("Fetch width")

    renameToDecodeDelay = Param.Unsigned("Rename to decode delay")
    iewToDecodeDelay = Param.Unsigned("Issue/Execute/Writeback to decode "
               "delay")
    commitToDecodeDelay = Param.Unsigned("Commit to decode delay")
    fetchToDecodeDelay = Param.Unsigned("Fetch to decode delay")
    decodeWidth = Param.Unsigned("Decode width")

    iewToRenameDelay = Param.Unsigned("Issue/Execute/Writeback to rename "
               "delay")
    commitToRenameDelay = Param.Unsigned("Commit to rename delay")
    decodeToRenameDelay = Param.Unsigned("Decode to rename delay")
    renameWidth = Param.Unsigned("Rename width")

    commitToIEWDelay = Param.Unsigned("Commit to "
               "Issue/Execute/Writeback delay")
    renameToIEWDelay = Param.Unsigned("Rename to "
               "Issue/Execute/Writeback delay")
    issueToExecuteDelay = Param.Unsigned("Issue to execute delay (internal "
              "to the IEW stage)")
    issueWidth = Param.Unsigned("Issue width")
    executeWidth = Param.Unsigned("Execute width")
    executeIntWidth = Param.Unsigned("Integer execute width")
    executeFloatWidth = Param.Unsigned("Floating point execute width")
    executeBranchWidth = Param.Unsigned("Branch execute width")
    executeMemoryWidth = Param.Unsigned("Memory execute width")

    iewToCommitDelay = Param.Unsigned("Issue/Execute/Writeback to commit "
               "delay")
    renameToROBDelay = Param.Unsigned("Rename to reorder buffer delay")
    commitWidth = Param.Unsigned("Commit width")
    squashWidth = Param.Unsigned("Squash width")

    predType = Param.String("Type of branch predictor ('local', 'tournament')")
    localPredictorSize = Param.Unsigned("Size of local predictor")
    localCtrBits = Param.Unsigned("Bits per counter")
    localHistoryTableSize = Param.Unsigned("Size of local history table")
    localHistoryBits = Param.Unsigned("Bits for the local history")
    globalPredictorSize = Param.Unsigned("Size of global predictor")
    globalCtrBits = Param.Unsigned("Bits per counter")
    globalHistoryBits = Param.Unsigned("Bits of history")
    choicePredictorSize = Param.Unsigned("Size of choice predictor")
    choiceCtrBits = Param.Unsigned("Bits of choice counters")

    BTBEntries = Param.Unsigned("Number of BTB entries")
    BTBTagSize = Param.Unsigned("Size of the BTB tags, in bits")

    RASSize = Param.Unsigned("RAS size")

    LQEntries = Param.Unsigned("Number of load queue entries")
    SQEntries = Param.Unsigned("Number of store queue entries")
    lsqLimits = Param.Bool(True, "LSQ size limits dispatch")
    LFSTSize = Param.Unsigned("Last fetched store table size")
    SSITSize = Param.Unsigned("Store set ID table size")

    numPhysIntRegs = Param.Unsigned("Number of physical integer registers")
    numPhysFloatRegs = Param.Unsigned("Number of physical floating point "
               "registers")
    numIQEntries = Param.Unsigned("Number of instruction queue entries")
    numROBEntries = Param.Unsigned("Number of reorder buffer entries")

    instShiftAmt = Param.Unsigned("Number of bits to shift instructions by")

    # If the CheckerCPU is brought back to useability in the OzoneCPU, create a
    # function here called addCheckerCpu() to create a non-NULL Checker and
    # connect its TLBs (if needed)
