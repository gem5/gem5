# Copyright (c) 2012-2014 ARM Limited
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
# Copyright (c) 2007 The Regents of The University of Michigan
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
# Authors: Gabe Black
#          Nathan Binkert
#          Andrew Bardsley

from m5.defines import buildEnv
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject
from BaseCPU import BaseCPU
from DummyChecker import DummyChecker
from BranchPredictor import *
from TimingExpr import TimingExpr

from FuncUnit import OpClass

class MinorOpClass(SimObject):
    """Boxing of OpClass to get around build problems and provide a hook for
    future additions to OpClass checks"""

    type = 'MinorOpClass'
    cxx_header = "cpu/minor/func_unit.hh"

    opClass = Param.OpClass("op class to match")

class MinorOpClassSet(SimObject):
    """A set of matchable op classes"""

    type = 'MinorOpClassSet'
    cxx_header = "cpu/minor/func_unit.hh"

    opClasses = VectorParam.MinorOpClass([], "op classes to be matched."
        "  An empty list means any class")

class MinorFUTiming(SimObject):
    type = 'MinorFUTiming'
    cxx_header = "cpu/minor/func_unit.hh"

    mask = Param.UInt64(0, "mask for testing ExtMachInst")
    match = Param.UInt64(0, "match value for testing ExtMachInst:"
        " (ext_mach_inst & mask) == match")
    suppress = Param.Bool(False, "if true, this inst. is not executed by"
        " this FU")
    extraCommitLat = Param.Cycles(0, "extra cycles to stall commit for"
        " this inst.")
    extraCommitLatExpr = Param.TimingExpr(NULL, "extra cycles as a"
        " run-time evaluated expression")
    extraAssumedLat = Param.Cycles(0, "extra cycles to add to scoreboard"
        " retire time for this insts dest registers once it leaves the"
        " functional unit.  For mem refs, if this is 0, the result's time"
        " is marked as unpredictable and no forwarding can take place.")
    srcRegsRelativeLats = VectorParam.Cycles("the maximum number of cycles"
        " after inst. issue that each src reg can be available for this"
        " inst. to issue")
    opClasses = Param.MinorOpClassSet(MinorOpClassSet(),
        "op classes to be considered for this decode.  An empty set means any"
        " class")
    description = Param.String('', "description string of the decoding/inst."
        " class")

def minorMakeOpClassSet(op_classes):
    """Make a MinorOpClassSet from a list of OpClass enum value strings"""
    def boxOpClass(op_class):
        return MinorOpClass(opClass=op_class)

    return MinorOpClassSet(opClasses=map(boxOpClass, op_classes))

class MinorFU(SimObject):
    type = 'MinorFU'
    cxx_header = "cpu/minor/func_unit.hh"

    opClasses = Param.MinorOpClassSet(MinorOpClassSet(), "type of operations"
        " allowed on this functional unit")
    opLat = Param.Cycles(1, "latency in cycles")
    issueLat = Param.Cycles(1, "cycles until another instruction can be"
        " issued")
    timings = VectorParam.MinorFUTiming([], "extra decoding rules")

    cantForwardFromFUIndices = VectorParam.Unsigned([],
        "list of FU indices from which this FU can't receive and early"
        " (forwarded) result")

class MinorFUPool(SimObject):
    type = 'MinorFUPool'
    cxx_header = "cpu/minor/func_unit.hh"

    funcUnits = VectorParam.MinorFU("functional units")

class MinorDefaultIntFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntAlu'])
    timings = [MinorFUTiming(description="Int",
        srcRegsRelativeLats=[2])]
    opLat = 3

class MinorDefaultIntMulFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntMult'])
    timings = [MinorFUTiming(description='Mul',
        srcRegsRelativeLats=[0])]
    opLat = 3

class MinorDefaultIntDivFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IntDiv'])
    issueLat = 9
    opLat = 9

class MinorDefaultFloatSimdFU(MinorFU):
    opClasses = minorMakeOpClassSet([
        'FloatAdd', 'FloatCmp', 'FloatCvt', 'FloatMult', 'FloatDiv',
        'FloatSqrt',
        'SimdAdd', 'SimdAddAcc', 'SimdAlu', 'SimdCmp', 'SimdCvt',
        'SimdMisc', 'SimdMult', 'SimdMultAcc', 'SimdShift', 'SimdShiftAcc',
        'SimdSqrt', 'SimdFloatAdd', 'SimdFloatAlu', 'SimdFloatCmp',
        'SimdFloatCvt', 'SimdFloatDiv', 'SimdFloatMisc', 'SimdFloatMult',
        'SimdFloatMultAcc', 'SimdFloatSqrt'])
    timings = [MinorFUTiming(description='FloatSimd',
        srcRegsRelativeLats=[2])]
    opLat = 6

class MinorDefaultMemFU(MinorFU):
    opClasses = minorMakeOpClassSet(['MemRead', 'MemWrite'])
    timings = [MinorFUTiming(description='Mem',
        srcRegsRelativeLats=[1], extraAssumedLat=2)]
    opLat = 1

class MinorDefaultMiscFU(MinorFU):
    opClasses = minorMakeOpClassSet(['IprAccess', 'InstPrefetch'])
    opLat = 1

class MinorDefaultFUPool(MinorFUPool):
    funcUnits = [MinorDefaultIntFU(), MinorDefaultIntFU(),
        MinorDefaultIntMulFU(), MinorDefaultIntDivFU(),
        MinorDefaultFloatSimdFU(), MinorDefaultMemFU(),
        MinorDefaultMiscFU()]

class MinorCPU(BaseCPU):
    type = 'MinorCPU'
    cxx_header = "cpu/minor/cpu.hh"

    @classmethod
    def memory_mode(cls):
        return 'timing'

    @classmethod
    def require_caches(cls):
        return True

    @classmethod
    def support_take_over(cls):
        return True

    fetch1FetchLimit = Param.Unsigned(1,
        "Number of line fetches allowable in flight at once")
    fetch1LineSnapWidth = Param.Unsigned(0,
        "Fetch1 'line' fetch snap size in bytes"
        " (0 means use system cache line size)")
    fetch1LineWidth = Param.Unsigned(0,
        "Fetch1 maximum fetch size in bytes (0 means use system cache"
        " line size)")
    fetch1ToFetch2ForwardDelay = Param.Cycles(1,
        "Forward cycle delay from Fetch1 to Fetch2 (1 means next cycle)")
    fetch1ToFetch2BackwardDelay = Param.Cycles(1,
        "Backward cycle delay from Fetch2 to Fetch1 for branch prediction"
        " signalling (0 means in the same cycle, 1 mean the next cycle)")

    fetch2InputBufferSize = Param.Unsigned(2,
        "Size of input buffer to Fetch2 in cycles-worth of insts.")
    fetch2ToDecodeForwardDelay = Param.Cycles(1,
        "Forward cycle delay from Fetch2 to Decode (1 means next cycle)")
    fetch2CycleInput = Param.Bool(True,
        "Allow Fetch2 to cross input lines to generate full output each"
        " cycle")

    decodeInputBufferSize = Param.Unsigned(3,
        "Size of input buffer to Decode in cycles-worth of insts.")
    decodeToExecuteForwardDelay = Param.Cycles(1,
        "Forward cycle delay from Decode to Execute (1 means next cycle)")
    decodeInputWidth = Param.Unsigned(2,
        "Width (in instructions) of input to Decode (and implicitly"
        " Decode's own width)")
    decodeCycleInput = Param.Bool(True,
        "Allow Decode to pack instructions from more than one input cycle"
        " to fill its output each cycle")

    executeInputWidth = Param.Unsigned(2,
        "Width (in instructions) of input to Execute")
    executeCycleInput = Param.Bool(True,
        "Allow Execute to use instructions from more than one input cycle"
        " each cycle")
    executeIssueLimit = Param.Unsigned(2,
        "Number of issuable instructions in Execute each cycle")
    executeMemoryIssueLimit = Param.Unsigned(1,
        "Number of issuable memory instructions in Execute each cycle")
    executeCommitLimit = Param.Unsigned(2,
        "Number of committable instructions in Execute each cycle")
    executeMemoryCommitLimit = Param.Unsigned(1,
        "Number of committable memory references in Execute each cycle")
    executeInputBufferSize = Param.Unsigned(7,
        "Size of input buffer to Execute in cycles-worth of insts.")
    executeMemoryWidth = Param.Unsigned(0,
        "Width (and snap) in bytes of the data memory interface. (0 mean use"
        " the system cacheLineSize)")
    executeMaxAccessesInMemory = Param.Unsigned(2,
        "Maximum number of concurrent accesses allowed to the memory system"
        " from the dcache port")
    executeLSQMaxStoreBufferStoresPerCycle = Param.Unsigned(2,
        "Maximum number of stores that the store buffer can issue per cycle")
    executeLSQRequestsQueueSize = Param.Unsigned(1,
        "Size of LSQ requests queue (address translation queue)")
    executeLSQTransfersQueueSize = Param.Unsigned(2,
        "Size of LSQ transfers queue (memory transaction queue)")
    executeLSQStoreBufferSize = Param.Unsigned(5,
        "Size of LSQ store buffer")
    executeBranchDelay = Param.Cycles(1,
        "Delay from Execute deciding to branch and Fetch1 reacting"
        " (1 means next cycle)")

    executeFuncUnits = Param.MinorFUPool(MinorDefaultFUPool(),
        "FUlines for this processor")

    executeSetTraceTimeOnCommit = Param.Bool(True,
        "Set inst. trace times to be commit times")
    executeSetTraceTimeOnIssue = Param.Bool(False,
        "Set inst. trace times to be issue times")

    executeAllowEarlyMemoryIssue = Param.Bool(True,
        "Allow mem refs to be issued to the LSQ before reaching the head of"
        " the in flight insts queue")

    enableIdling = Param.Bool(True,
        "Enable cycle skipping when the processor is idle\n");

    branchPred = Param.BranchPredictor(TournamentBP(
        numThreads = Parent.numThreads), "Branch Predictor")

    def addCheckerCpu(self):
        print "Checker not yet supported by MinorCPU"
        exit(1)
