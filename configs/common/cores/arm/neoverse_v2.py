# Copyright (c) 2026 Arm Limited
# All rights reserved
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


# -----------------------------------------------------------------------
#                Architecture version
# -----------------------------------------------------------------------
# Neoverse V2 implements Armv9.0-A: SVE2 with 128-bit vectors, no SME,
# no TME. Without stating that here the model inherits gem5's generic
# defaults, which advertise FEAT_SME and TME and omit FEAT_SVE2 -- so the
# HWCAP bits seen by the simulated program describe a different machine,
# and libraries that dispatch on them (OpenBLAS, Arm PL, oneDNN) select
# kernels the real core would not run.


class NeoverseV2_Release(Armv90):
    """Full-system release: Armv9.0-A, what Neoverse V2 implements.

    Use as ``system.release = NeoverseV2_Release()``; pair it with
    ``system.sve_vl = 1`` (V2 is 4x128-bit, so one 128-bit granule).
    """


class NeoverseV2_SE_Release(ArmRelease):
    """Syscall-emulation release: the same ISA surface without the
    EL2/EL3 features, which SE mode has no use for. Built flat for the
    same reason ArmDefaultSERelease is.
    """

    extensions = [
        "FEAT_AES",
        "FEAT_PMULL",
        "FEAT_SHA1",
        "FEAT_SHA256",
        "FEAT_CRC32",
        # Armv8.1
        "FEAT_LSE",
        "FEAT_RDM",
        "FEAT_FHM",
        # Armv8.2
        "FEAT_SVE",
        "FEAT_FP16",
        "FEAT_DOTPROD",
        "FEAT_F32MM",
        "FEAT_F64MM",
        "FEAT_I8MM",
        "FEAT_BF16",
        # Armv8.3
        "FEAT_FCMA",
        "FEAT_JSCVT",
        "FEAT_PAuth",
        "FEAT_LRCPC",
        # Armv8.4
        "FEAT_FLAGM",
        "FEAT_FRINTTS",
        "FEAT_LRCPC2",
        # Armv8.5
        "FEAT_FLAGM2",
        "FEAT_RNG",
        # Armv9.0
        "FEAT_SVE2",
        # The SVE crypto extensions are optional in Armv9.0. Graviton4
        # (a Neoverse V2 implementation) reports SVEAES, SVEPMULL,
        # SVEBITPERM and SVESHA3 in its HWCAP2 bits, so V2 has these four.
        # See aws/aws-graviton-getting-started, runtime-feature-detection.md
        "FEAT_SVE_AES",
        "FEAT_SVE_PMULL128",
        "FEAT_SVE_BitPerm",
        "FEAT_SVE_SHA3",
        # FEAT_SVE_SM4 is deliberately absent: Graviton4 reports SM3/SM4
        # (the NEON forms) but not SVESM4.
        #
        # Also deliberately absent, because Graviton4 reports neither:
        #   FEAT_SME, TME
    ]


class NeoverseV2_ISA(ArmISA):
    """The ISA every Neoverse V2 thread gets.

    BaseCPU.createThreads() instantiates ``self.ArchISA`` once per
    thread, so overriding ArchISA on the core below gives every user the
    right feature set with no action on their part, for any numThreads.
    """

    release_se = NeoverseV2_SE_Release()
    # SVE vector length in 128-bit granules. V2 is 4x128-bit, so 1.
    sve_vl_se = 1


# Simple ALU Instructions have a latency of 1
class NeoverseV2_Simple_Int(FUDesc):
    opList = [OpDesc(opClass="IntAlu", opLat=1)]


class Neoverse_V2_SI_FUP(FUPool):
    FUList = [NeoverseV2_Simple_Int(count=2)]


# Complex ALU instructions have a variable latencies
class NeoverseV2_Complex_Int(FUDesc):
    opList = [
        OpDesc(opClass="IntMult", opLat=2, pipelined=True),
        OpDesc(opClass="IntDiv", opLat=11, pipelined=False),
        # Treat system register (IPR) accesses as regular integer ops.
        OpDesc(opClass="IntAlu", opLat=1, pipelined=True),
        OpDesc(opClass="InstPrefetch", opLat=4, pipelined=True),
        OpDesc(opClass="System", opLat=3, pipelined=True),
    ]


class Neoverse_V2_CI_FUP(FUPool):
    FUList = [NeoverseV2_Complex_Int(count=1)]


# Floating point and SIMD instructions
class NeoverseV2_FP(FUDesc):
    opList = [
        OpDesc(opClass="SimdAdd", opLat=2),
        OpDesc(opClass="SimdAddAcc", opLat=4),
        OpDesc(opClass="SimdAlu", opLat=2),
        OpDesc(opClass="SimdCmp", opLat=2),
        OpDesc(opClass="SimdCvt", opLat=3),
        OpDesc(opClass="SimdMisc", opLat=3),
        OpDesc(opClass="SimdMult", opLat=4),
        OpDesc(opClass="SimdMultAcc", opLat=4),
        OpDesc(opClass="SimdMatMultAcc", opLat=3),
        OpDesc(opClass="SimdShift", opLat=3),
        OpDesc(opClass="SimdShiftAcc", opLat=4),
        OpDesc(opClass="SimdDiv", opLat=11),
        OpDesc(opClass="SimdSqrt", opLat=5),
        OpDesc(opClass="SimdFloatAdd", opLat=2),
        OpDesc(opClass="SimdFloatAlu", opLat=3),
        OpDesc(opClass="SimdFloatCmp", opLat=2),
        OpDesc(opClass="SimdFloatCvt", opLat=4),
        OpDesc(opClass="SimdFloatDiv", opLat=11),
        OpDesc(opClass="SimdFloatMisc", opLat=3),
        OpDesc(opClass="SimdFloatMult", opLat=3),
        OpDesc(opClass="SimdFloatMultAcc", opLat=4),
        OpDesc(opClass="SimdFloatMatMultAcc", opLat=6),
        OpDesc(opClass="SimdFloatSqrt", opLat=9),
        OpDesc(opClass="SimdReduceAdd", opLat=4),
        OpDesc(opClass="SimdReduceAlu", opLat=4),
        OpDesc(opClass="SimdReduceCmp", opLat=4),
        OpDesc(opClass="SimdFloatReduceAdd", opLat=6),
        OpDesc(opClass="SimdFloatReduceCmp", opLat=6),
        OpDesc(opClass="SimdExt", opLat=2),
        OpDesc(opClass="SimdFloatExt", opLat=2),
        OpDesc(opClass="SimdConfig", opLat=2),
        OpDesc(opClass="SimdDotProd", opLat=3),
        OpDesc(opClass="SimdAes", opLat=2),
        OpDesc(opClass="SimdAesMix", opLat=2),
        OpDesc(opClass="SimdSha1Hash", opLat=2),
        OpDesc(opClass="SimdSha1Hash2", opLat=4),
        OpDesc(opClass="SimdSha256Hash", opLat=4),
        OpDesc(opClass="SimdSha256Hash2", opLat=4),
        OpDesc(opClass="SimdShaSigma2", opLat=2),
        OpDesc(opClass="SimdShaSigma3", opLat=2),
        OpDesc(opClass="SimdSha512Hash", opLat=2),
        OpDesc(opClass="SimdSha3", opLat=2),
        OpDesc(opClass="SimdSm3", opLat=2),
        OpDesc(opClass="SimdSm4e", opLat=4),
        OpDesc(opClass="SimdCrc", opLat=2),
        OpDesc(opClass="SimdBf16Add", opLat=2),
        OpDesc(opClass="SimdBf16Cmp", opLat=2),
        OpDesc(opClass="SimdBf16Cvt", opLat=4),
        OpDesc(opClass="SimdBf16DotProd", opLat=5),
        OpDesc(opClass="SimdBf16MatMultAcc", opLat=6),
        OpDesc(opClass="SimdBf16Mult", opLat=3),
        OpDesc(opClass="SimdBf16MultAcc", opLat=5),
        OpDesc(opClass="FloatAdd", opLat=2),
        OpDesc(opClass="FloatCmp", opLat=2),
        OpDesc(opClass="FloatCvt", opLat=3),
        OpDesc(opClass="FloatDiv", opLat=11, pipelined=False),
        OpDesc(opClass="FloatSqrt", opLat=12, pipelined=False),
        OpDesc(opClass="FloatMult", opLat=3),
        OpDesc(opClass="FloatMisc", opLat=4),
        OpDesc(opClass="FloatMultAcc", opLat=4),
    ]


class Neoverse_V2_FP_FUP(FUPool):
    FUList = [NeoverseV2_FP(count=2)]


# Load/Store Unit
class NeoverseV2_Load(FUDesc):
    opList = [
        OpDesc(opClass="MemRead", opLat=4),
        OpDesc(opClass="FloatMemRead", opLat=6),
    ]


class NeoverseV2_Store(FUDesc):
    opList = [
        OpDesc(opClass="MemWrite", opLat=1),
        OpDesc(opClass="FloatMemWrite", opLat=2),
    ]


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
    indirectBranchPred = ITTAGE()
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

    # Every thread gets the Armv9.0-A feature set defined above instead
    # of gem5's generic default.
    ArchISA = NeoverseV2_ISA

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
    iqInsertionPolicy = "LeastLoaded"
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
    size = "64KiB"
    assoc = 8
    mshrs = 12
    is_read_only = True


# Data Cache
class L1D(L1Cache):
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
