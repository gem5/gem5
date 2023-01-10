# Copyright (c) 2010, 2017-2018, 2022 ARM Limited
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

from m5.SimObject import SimObject
from m5.params import *


class OpClass(Enum):
    vals = [
        "No_OpClass",
        "IntAlu",
        "IntMult",
        "IntDiv",
        "FloatAdd",
        "FloatCmp",
        "FloatCvt",
        "FloatMult",
        "FloatMultAcc",
        "FloatDiv",
        "FloatMisc",
        "FloatSqrt",
        "SimdAdd",
        "SimdAddAcc",
        "SimdAlu",
        "SimdCmp",
        "SimdCvt",
        "SimdMisc",
        "SimdMult",
        "SimdMultAcc",
        "SimdShift",
        "SimdShiftAcc",
        "SimdDiv",
        "SimdSqrt",
        "SimdFloatAdd",
        "SimdFloatAlu",
        "SimdFloatCmp",
        "SimdFloatCvt",
        "SimdFloatDiv",
        "SimdFloatMisc",
        "SimdFloatMult",
        "SimdFloatMultAcc",
        "SimdFloatSqrt",
        "SimdReduceAdd",
        "SimdReduceAlu",
        "SimdReduceCmp",
        "SimdFloatReduceAdd",
        "SimdFloatReduceCmp",
        "SimdAes",
        "SimdAesMix",
        "SimdSha1Hash",
        "SimdSha1Hash2",
        "SimdSha256Hash",
        "SimdSha256Hash2",
        "SimdShaSigma2",
        "SimdShaSigma3",
        "SimdPredAlu",
        "Matrix",
        "MatrixMov",
        "MatrixOP",
        "MemRead",
        "MemWrite",
        "FloatMemRead",
        "FloatMemWrite",
        "IprAccess",
        "InstPrefetch",
        "VectorUnitStrideLoad",
        "VectorUnitStrideStore",
        "VectorUnitStrideMaskLoad",
        "VectorUnitStrideMaskStore",
        "VectorStridedLoad",
        "VectorStridedStore",
        "VectorIndexedLoad",
        "VectorIndexedStore",
        "VectorUnitStrideFaultOnlyFirstLoad",
        "VectorWholeRegisterLoad",
        "VectorWholeRegisterStore",
        "VectorIntegerArith",
        "VectorFloatArith",
        "VectorFloatConvert",
        "VectorIntegerReduce",
        "VectorFloatReduce",
        "VectorMisc",
        "VectorIntegerExtension",
        "VectorConfig",
    ]


class OpDesc(SimObject):
    type = "OpDesc"
    cxx_header = "cpu/func_unit.hh"
    cxx_class = "gem5::OpDesc"

    opClass = Param.OpClass("type of operation")
    opLat = Param.Cycles(1, "cycles until result is available")
    pipelined = Param.Bool(
        True,
        "set to true when the functional unit for"
        "this op is fully pipelined. False means not pipelined at all.",
    )


class FUDesc(SimObject):
    type = "FUDesc"
    cxx_header = "cpu/func_unit.hh"
    cxx_class = "gem5::FUDesc"

    count = Param.Int("number of these FU's available")
    opList = VectorParam.OpDesc("operation classes for this FU type")
