/*
 * Copyright (c) 2010, 2017-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU__OP_CLASS_HH__
#define __CPU__OP_CLASS_HH__

#include "enums/OpClass.hh"

namespace gem5
{

/*
 * Do a bunch of wonky stuff to maintain backward compatability so I
 * don't have to change code in a zillion places.
 */
using enums::OpClass;
using enums::No_OpClass;

static const OpClass IntAluOp = enums::IntAlu;
static const OpClass IntMultOp = enums::IntMult;
static const OpClass IntDivOp = enums::IntDiv;
static const OpClass FloatAddOp = enums::FloatAdd;
static const OpClass FloatCmpOp = enums::FloatCmp;
static const OpClass FloatCvtOp = enums::FloatCvt;
static const OpClass FloatMultOp = enums::FloatMult;
static const OpClass FloatMultAccOp = enums::FloatMultAcc;
static const OpClass FloatDivOp = enums::FloatDiv;
static const OpClass FloatMiscOp = enums::FloatMisc;
static const OpClass FloatSqrtOp = enums::FloatSqrt;
static const OpClass SimdAddOp = enums::SimdAdd;
static const OpClass SimdAddAccOp = enums::SimdAddAcc;
static const OpClass SimdAluOp = enums::SimdAlu;
static const OpClass SimdCmpOp = enums::SimdCmp;
static const OpClass SimdCvtOp = enums::SimdCvt;
static const OpClass SimdMiscOp = enums::SimdMisc;
static const OpClass SimdMultOp = enums::SimdMult;
static const OpClass SimdMultAccOp = enums::SimdMultAcc;
static const OpClass SimdShiftOp = enums::SimdShift;
static const OpClass SimdShiftAccOp = enums::SimdShiftAcc;
static const OpClass SimdDivOp = enums::SimdDiv;
static const OpClass SimdSqrtOp = enums::SimdSqrt;
static const OpClass SimdReduceAddOp = enums::SimdReduceAdd;
static const OpClass SimdReduceAluOp = enums::SimdReduceAlu;
static const OpClass SimdReduceCmpOp = enums::SimdReduceCmp;
static const OpClass SimdFloatAddOp = enums::SimdFloatAdd;
static const OpClass SimdFloatAluOp = enums::SimdFloatAlu;
static const OpClass SimdFloatCmpOp = enums::SimdFloatCmp;
static const OpClass SimdFloatCvtOp = enums::SimdFloatCvt;
static const OpClass SimdFloatDivOp = enums::SimdFloatDiv;
static const OpClass SimdFloatMiscOp = enums::SimdFloatMisc;
static const OpClass SimdFloatMultOp = enums::SimdFloatMult;
static const OpClass SimdFloatMultAccOp = enums::SimdFloatMultAcc;
static const OpClass SimdFloatSqrtOp = enums::SimdFloatSqrt;
static const OpClass SimdFloatReduceCmpOp = enums::SimdFloatReduceCmp;
static const OpClass SimdFloatReduceAddOp = enums::SimdFloatReduceAdd;
static const OpClass SimdAesOp = enums::SimdAes;
static const OpClass SimdAesMixOp = enums::SimdAesMix;
static const OpClass SimdSha1HashOp = enums::SimdSha1Hash;
static const OpClass SimdSha1Hash2Op = enums::SimdSha1Hash2;
static const OpClass SimdSha256HashOp = enums::SimdSha256Hash;
static const OpClass SimdSha256Hash2Op = enums::SimdSha256Hash2;
static const OpClass SimdShaSigma2Op = enums::SimdShaSigma2;
static const OpClass SimdShaSigma3Op = enums::SimdShaSigma3;
static const OpClass SimdPredAluOp = enums::SimdPredAlu;
static const OpClass MemReadOp = enums::MemRead;
static const OpClass MemWriteOp = enums::MemWrite;
static const OpClass FloatMemReadOp = enums::FloatMemRead;
static const OpClass FloatMemWriteOp = enums::FloatMemWrite;
static const OpClass IprAccessOp = enums::IprAccess;
static const OpClass InstPrefetchOp = enums::InstPrefetch;
static const OpClass Num_OpClasses = enums::Num_OpClass;

} // namespace gem5

#endif // __CPU__OP_CLASS_HH__
