# Copyright (c) 2025 Technical University of Munich
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

from m5.objects import *

# -----------------------------------------------------------------------
#                Neoverse V2 core configuration
# -----------------------------------------------------------------------
# This configuration is based on publicly available information about
# the Neoverse V2 architecture.
# https://hc2023.hotchips.org/assets/program/conference/day1/CPU1/HC2023.Arm.MagnusBruce.v04.FINAL.pdf
# https://chipsandcheese.com/p/arms-neoverse-v2-in-awss-graviton-4


# Simple ALU Instructions have a latency of 1
class NeoverseV2_Simple_Int(FUDesc):
    opList = [OpDesc(opClass="IntAlu", opLat=1)]


class Neoverse_V2_SI_FUP(FUPool):
    FUList = [NeoverseV2_Simple_Int(count=2)]


# Complex ALU instructions have a variable latencies
class NeoverseV2_Complex_Int(FUDesc):
    opList = [
        OpDesc(opClass="IntMult", opLat=4, pipelined=True),
        OpDesc(opClass="IntDiv", opLat=11, pipelined=False),
        # Treat system register (IPR) accesses as regular integer ops.
        OpDesc(opClass="IntAlu", opLat=3, pipelined=True),
        OpDesc(opClass="System", opLat=3, pipelined=True),
    ]


class Neoverse_V2_CI_FUP(FUPool):
    FUList = [NeoverseV2_Complex_Int(count=1)]


# Floating point and SIMD instructions
class NeoverseV2_FP(FUDesc):
    opList = [
        OpDesc(opClass="SimdAdd", opLat=3),
        OpDesc(opClass="SimdAddAcc", opLat=4),
        OpDesc(opClass="SimdAlu", opLat=4),
        OpDesc(opClass="SimdCmp", opLat=4),
        OpDesc(opClass="SimdCvt", opLat=3),
        OpDesc(opClass="SimdMisc", opLat=3),
        OpDesc(opClass="SimdMult", opLat=6),
        OpDesc(opClass="SimdMultAcc", opLat=5),
        OpDesc(opClass="SimdMatMultAcc", opLat=5),
        OpDesc(opClass="SimdShift", opLat=3),
        OpDesc(opClass="SimdShiftAcc", opLat=3),
        OpDesc(opClass="SimdSqrt", opLat=9),
        OpDesc(opClass="SimdFloatAdd", opLat=6),
        OpDesc(opClass="SimdFloatAlu", opLat=5),
        OpDesc(opClass="SimdFloatCmp", opLat=3),
        OpDesc(opClass="SimdFloatCvt", opLat=3),
        OpDesc(opClass="SimdFloatDiv", opLat=21),
        OpDesc(opClass="SimdFloatMisc", opLat=3),
        OpDesc(opClass="SimdFloatMult", opLat=6),
        OpDesc(opClass="SimdFloatMultAcc", opLat=1),
        OpDesc(opClass="SimdFloatMatMultAcc", opLat=1),
        OpDesc(opClass="SimdFloatSqrt", opLat=9),
        OpDesc(opClass="FloatAdd", opLat=2),
        OpDesc(opClass="FloatCmp", opLat=2),
        OpDesc(opClass="FloatCvt", opLat=2),
        OpDesc(opClass="FloatDiv", opLat=12, pipelined=False),
        OpDesc(opClass="FloatSqrt", opLat=33, pipelined=False),
        OpDesc(opClass="FloatMult", opLat=3),
        OpDesc(opClass="FloatMisc", opLat=4),
    ]


class Neoverse_V2_FP_FUP(FUPool):
    FUList = [NeoverseV2_FP(count=2)]


# Load/Store Unit
class NeoverseV2_Load(FUDesc):
    opList = [OpDesc(opClass="MemRead", opLat=2)]


class NeoverseV2_Store(FUDesc):
    opList = [OpDesc(opClass="MemWrite", opLat=2)]


# Load only pool
class Neoverse_V2_Load_FUP(FUPool):
    FUList = [NeoverseV2_Load(count=1)]


# Loads/store pool
class Neoverse_V2_LoadStore_FUP(FUPool):
    FUList = [NeoverseV2_Load(count=1), NeoverseV2_Store(count=1)]


class Neoverse_V2_IQ0(IQUnit):
    """
    Scheduler 0:
    Simple integer (ALU + branch)
    """

    numEntries = 22
    fuPool = Neoverse_V2_SI_FUP()


class Neoverse_V2_IQ1(IQUnit):
    """
    Scheduler 1:
    Simple integer (ALU + branch)
    """

    numEntries = 22
    fuPool = Neoverse_V2_SI_FUP()


class Neoverse_V2_IQ2(IQUnit):
    """
    Scheduler 2:
    Complex integer (ALU + MUL + DIV + MADD)
    """

    numEntries = 22
    fuPool = Neoverse_V2_CI_FUP()


class Neoverse_V2_IQ3(IQUnit):
    """
    Scheduler 3:
    Complex integer (ALU + MUL + DIV + MADD)
    """

    numEntries = 22
    fuPool = Neoverse_V2_CI_FUP()


class Neoverse_V2_IQ4(IQUnit):
    """
    Scheduler 4:
    Floating point and vector units (128b FP + 128b ALU)
    """

    numEntries = 28
    fuPool = Neoverse_V2_FP_FUP()


class Neoverse_V2_IQ5(IQUnit):
    """
    Scheduler 5:
    Floating point and vector units (128b FP + 128b ALU)
    """

    numEntries = 28
    fuPool = Neoverse_V2_FP_FUP()


class Neoverse_V2_IQ6(IQUnit):
    """
    Scheduler 6:
    Load Unit (Load AGU)
    """

    numEntries = 16
    fuPool = Neoverse_V2_Load_FUP()


class Neoverse_V2_IQ7(IQUnit):
    """
    Scheduler 7:
    Load + Store Units
    """

    numEntries = 16
    fuPool = Neoverse_V2_LoadStore_FUP()


class Neoverse_V2_IQ8(IQUnit):
    """
    Scheduler 8:
    Load + Store Units
    """

    numEntries = 16
    fuPool = Neoverse_V2_LoadStore_FUP()


class NeoverseV2_BTB(SimpleBTB):
    numEntries = 12 * 1024
    tagBits = 18
    associativity = 6
    instShiftAmt = 2
    btbReplPolicy = LRURP()
    btbIndexingPolicy = BTBSetAssociative(
        num_entries=Parent.numEntries,
        set_shift=Parent.instShiftAmt,
        assoc=Parent.associativity,
        tag_bits=Parent.tagBits,
    )


# TAGE Branch Predictor
class NeoverseV2_BP(BranchPredictor):
    conditionalBranchPred = TAGE_SC_L_64KB()
    btb = NeoverseV2_BTB()
    ras = ReturnAddrStack(numEntries=31)
    instShiftAmt = 2
    requiresBTBHit = True
    takenOnlyHistory = True


class NeoverseMMU(ArmMMU):
    itb = ArmTLB(
        entry_type="instruction",
        size=48,
        assoc=12,
    )
    dtb = ArmTLB(
        entry_type="data",
        size=48,
        assoc=48,
    )
    l2_shared = ArmTLB(
        entry_type="unified", size=2048, assoc=8, partial_levels=["L2"]
    )


class NeoverseV2(ArmO3CPU):

    # Backward latencies
    decodeToFetchDelay = 1
    renameToFetchDelay = 1
    iewToFetchDelay = 1
    commitToFetchDelay = 1
    renameToDecodeDelay = 1
    iewToDecodeDelay = 1
    commitToDecodeDelay = 1
    iewToRenameDelay = 1
    commitToRenameDelay = 1
    commitToIEWDelay = 1

    # Pipeline widths and delays
    fetchWidth = 6
    fetchBufferSize = 64
    fetchToDecodeDelay = 3
    decodeWidth = 6
    decodeToRenameDelay = 2
    renameWidth = 8
    renameToIEWDelay = 1
    renameToROBDelay = 1
    dispatchWidth = 8
    issueWidth = 8
    issueToExecuteDelay = 1
    wbWidth = 8
    iewToCommitDelay = 1
    commitWidth = 8
    # Uncomment sqaush with for instant squash (restoring checkpoint)
    # squashWidth = 8
    trapLatency = 13

    # The Neoverse Scheduler
    # Configured according to https://chipsandcheese.com/p/arms-neoverse-v2-in-awss-graviton-4
    instQueues = [
        Neoverse_V2_IQ0(),
        Neoverse_V2_IQ1(),
        Neoverse_V2_IQ2(),
        Neoverse_V2_IQ3(),
        Neoverse_V2_IQ4(),
        Neoverse_V2_IQ5(),
        Neoverse_V2_IQ6(),
        Neoverse_V2_IQ7(),
        Neoverse_V2_IQ8(),
    ]

    backComSize = 5
    forwardComSize = 5

    numPhysIntRegs = 213
    numPhysFloatRegs = 188
    numROBEntries = 320
    LQEntries = 175
    SQEntries = 80

    LSQDepCheckShift = 0
    LFSTSize = 1024
    SSITSize = "1024"

    branchPred = NeoverseV2_BP()
    mmu = NeoverseMMU()

    # Decoupled front-end parameters
    decoupledFrontEnd = True
    fetchTargetWidth = 64
    minInstSize = 4
    numFTQEntries = 32


class L1Cache(Cache):
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    tgts_per_mshr = 8
    # Consider the L2 a victim cache also for clean lines
    writeback_clean = True
    replacement_policy = RRIPRP()


# Instruction Cache
class L1I(L1Cache):
    mshrs = 16
    size = "64KiB"
    assoc = 8
    mshrs = 12
    is_read_only = True


# Data Cache
class L1D(L1Cache):
    mshrs = 16
    size = "64KiB"
    assoc = 8
    mshrs = 12
    tag_latency = 4
    data_latency = 4
    response_latency = 4
    write_buffers = 16
    prefetcher = MultiPrefetcher(
        prefetchers=[
            StridePrefetcher(degree=8, latency=1, prefetch_on_access=True),
            SmsPrefetcher(),
            BOPPrefetcher(),
        ]
    )


# L2 Cache
class L2(Cache):
    tag_latency = 11
    data_latency = 11
    response_latency = 11
    mshrs = 96  # 96-entry Transaction Queue
    tgts_per_mshr = 8
    size = "2MiB"
    assoc = 8
    write_buffers = 8
    clusivity = "mostly_incl"
    # Simple stride prefetcher
    tags = BaseSetAssoc()
    replacement_policy = RRIPRP()
