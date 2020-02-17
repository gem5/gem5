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

/*
 * Do a bunch of wonky stuff to maintain backward compatability so I
 * don't have to change code in a zillion places.
 */
using Enums::OpClass;
using Enums::No_OpClass;

static const OpClass IntAluOp = Enums::IntAlu;
static const OpClass IntMultOp = Enums::IntMult;
static const OpClass IntDivOp = Enums::IntDiv;
static const OpClass FloatAddOp = Enums::FloatAdd;
static const OpClass FloatCmpOp = Enums::FloatCmp;
static const OpClass FloatCvtOp = Enums::FloatCvt;
static const OpClass FloatMultOp = Enums::FloatMult;
static const OpClass FloatMultAccOp = Enums::FloatMultAcc;
static const OpClass FloatDivOp = Enums::FloatDiv;
static const OpClass FloatMiscOp = Enums::FloatMisc;
static const OpClass FloatSqrtOp = Enums::FloatSqrt;
static const OpClass SimdAddOp = Enums::SimdAdd;
static const OpClass SimdAddAccOp = Enums::SimdAddAcc;
static const OpClass SimdAluOp = Enums::SimdAlu;
static const OpClass SimdCmpOp = Enums::SimdCmp;
static const OpClass SimdCvtOp = Enums::SimdCvt;
static const OpClass SimdMiscOp = Enums::SimdMisc;
static const OpClass SimdMultOp = Enums::SimdMult;
static const OpClass SimdMultAccOp = Enums::SimdMultAcc;
static const OpClass SimdShiftOp = Enums::SimdShift;
static const OpClass SimdShiftAccOp = Enums::SimdShiftAcc;
static const OpClass SimdDivOp = Enums::SimdDiv;
static const OpClass SimdSqrtOp = Enums::SimdSqrt;
static const OpClass SimdReduceAddOp = Enums::SimdReduceAdd;
static const OpClass SimdReduceAluOp = Enums::SimdReduceAlu;
static const OpClass SimdReduceCmpOp = Enums::SimdReduceCmp;
static const OpClass SimdFloatAddOp = Enums::SimdFloatAdd;
static const OpClass SimdFloatAluOp = Enums::SimdFloatAlu;
static const OpClass SimdFloatCmpOp = Enums::SimdFloatCmp;
static const OpClass SimdFloatCvtOp = Enums::SimdFloatCvt;
static const OpClass SimdFloatDivOp = Enums::SimdFloatDiv;
static const OpClass SimdFloatMiscOp = Enums::SimdFloatMisc;
static const OpClass SimdFloatMultOp = Enums::SimdFloatMult;
static const OpClass SimdFloatMultAccOp = Enums::SimdFloatMultAcc;
static const OpClass SimdFloatSqrtOp = Enums::SimdFloatSqrt;
static const OpClass SimdFloatReduceCmpOp = Enums::SimdFloatReduceCmp;
static const OpClass SimdFloatReduceAddOp = Enums::SimdFloatReduceAdd;
static const OpClass SimdAesOp = Enums::SimdAes;
static const OpClass SimdAesMixOp = Enums::SimdAesMix;
static const OpClass SimdSha1HashOp = Enums::SimdSha1Hash;
static const OpClass SimdSha1Hash2Op = Enums::SimdSha1Hash2;
static const OpClass SimdSha256HashOp = Enums::SimdSha256Hash;
static const OpClass SimdSha256Hash2Op = Enums::SimdSha256Hash2;
static const OpClass SimdShaSigma2Op = Enums::SimdShaSigma2;
static const OpClass SimdShaSigma3Op = Enums::SimdShaSigma3;
static const OpClass SimdPredAluOp = Enums::SimdPredAlu;
static const OpClass MemReadOp = Enums::MemRead;
static const OpClass MemWriteOp = Enums::MemWrite;
static const OpClass FloatMemReadOp = Enums::FloatMemRead;
static const OpClass FloatMemWriteOp = Enums::FloatMemWrite;
static const OpClass IprAccessOp = Enums::IprAccess;
static const OpClass InstPrefetchOp = Enums::InstPrefetch;
static const OpClass Num_OpClasses = Enums::Num_OpClass;

#endif // __CPU__OP_CLASS_HH__
