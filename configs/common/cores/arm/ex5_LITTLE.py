# Copyright (c) 2012 The Regents of The University of Michigan
# Copyright (c) 2016 Centre National de la Recherche Scientifique
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
#                ex5 LITTLE core (based on the ARM Cortex-A7)
# -----------------------------------------------------------------------


# Simple ALU Instructions have a latency of 3
class ex5_LITTLE_Simple_Int(MinorDefaultIntFU):
    opList = [OpDesc(opClass="IntAlu", opLat=4)]


# Complex ALU instructions have a variable latencies
class ex5_LITTLE_Complex_IntMul(MinorDefaultIntMulFU):
    opList = [OpDesc(opClass="IntMult", opLat=7)]


class ex5_LITTLE_Complex_IntDiv(MinorDefaultIntDivFU):
    opList = [OpDesc(opClass="IntDiv", opLat=9)]


# Floating point and SIMD instructions
class ex5_LITTLE_FP(MinorDefaultFloatSimdFU):
    opList = [
        OpDesc(opClass="SimdAdd", opLat=6),
        OpDesc(opClass="SimdAddAcc", opLat=4),
        OpDesc(opClass="SimdAlu", opLat=4),
        OpDesc(opClass="SimdCmp", opLat=1),
        OpDesc(opClass="SimdCvt", opLat=3),
        OpDesc(opClass="SimdMisc", opLat=3),
        OpDesc(opClass="SimdMult", opLat=4),
        OpDesc(opClass="SimdMultAcc", opLat=5),
        OpDesc(opClass="SimdMatMultAcc", opLat=5),
        OpDesc(opClass="SimdShift", opLat=3),
        OpDesc(opClass="SimdShiftAcc", opLat=3),
        OpDesc(opClass="SimdSqrt", opLat=9),
        OpDesc(opClass="SimdFloatAdd", opLat=8),
        OpDesc(opClass="SimdFloatAlu", opLat=6),
        OpDesc(opClass="SimdFloatCmp", opLat=6),
        OpDesc(opClass="SimdFloatCvt", opLat=6),
        OpDesc(opClass="SimdFloatDiv", opLat=20, pipelined=False),
        OpDesc(opClass="SimdFloatMisc", opLat=6),
        OpDesc(opClass="SimdFloatMult", opLat=15),
        OpDesc(opClass="SimdFloatMultAcc", opLat=6),
        OpDesc(opClass="SimdFloatMatMultAcc", opLat=6),
        OpDesc(opClass="SimdFloatSqrt", opLat=17),
        OpDesc(opClass="FloatAdd", opLat=8),
        OpDesc(opClass="FloatCmp", opLat=6),
        OpDesc(opClass="FloatCvt", opLat=6),
        OpDesc(opClass="FloatDiv", opLat=15, pipelined=False),
        OpDesc(opClass="FloatSqrt", opLat=33),
        OpDesc(opClass="FloatMult", opLat=6),
    ]


# Load/Store Units
class ex5_LITTLE_MemFU(MinorDefaultMemFU):
    opList = [
        OpDesc(opClass="MemRead", opLat=1),
        OpDesc(opClass="MemWrite", opLat=1),
    ]


# Misc Unit
class ex5_LITTLE_MiscFU(MinorDefaultMiscFU):
    opList = [
        OpDesc(opClass="IprAccess", opLat=1),
        OpDesc(opClass="InstPrefetch", opLat=1),
    ]


# Functional Units for this CPU
class ex5_LITTLE_FUP(MinorFUPool):
    funcUnits = [
        ex5_LITTLE_Simple_Int(),
        ex5_LITTLE_Simple_Int(),
        ex5_LITTLE_Complex_IntMul(),
        ex5_LITTLE_Complex_IntDiv(),
        ex5_LITTLE_FP(),
        ex5_LITTLE_MemFU(),
        ex5_LITTLE_MiscFU(),
    ]


class ex5_LITTLE(ArmMinorCPU):
    executeFuncUnits = ex5_LITTLE_FUP()


class L1Cache(Cache):
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    tgts_per_mshr = 8
    # Consider the L2 a victim cache also for clean lines
    writeback_clean = True


class L1I(L1Cache):
    mshrs = 2
    size = "32kB"
    assoc = 2
    is_read_only = True
    tgts_per_mshr = 20


class L1D(L1Cache):
    mshrs = 4
    size = "32kB"
    assoc = 4
    write_buffers = 4


# L2 Cache
class L2(Cache):
    tag_latency = 9
    data_latency = 9
    response_latency = 9
    mshrs = 8
    tgts_per_mshr = 12
    size = "512kB"
    assoc = 8
    write_buffers = 16
    clusivity = "mostly_excl"
    # Simple stride prefetcher
    prefetcher = StridePrefetcher(degree=1, latency=1, prefetch_on_access=True)
    tags = BaseSetAssoc()
    replacement_policy = RandomRP()
