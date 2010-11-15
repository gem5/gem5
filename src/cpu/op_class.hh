/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Nathan Binkert
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
using Enums::Num_OpClass;

const OpClass IntAluOp = Enums::IntAlu;
const OpClass IntMultOp = Enums::IntMult;
const OpClass IntDivOp = Enums::IntDiv;
const OpClass FloatAddOp = Enums::FloatAdd;
const OpClass FloatCmpOp = Enums::FloatCmp;
const OpClass FloatCvtOp = Enums::FloatCvt;
const OpClass FloatMultOp = Enums::FloatMult;
const OpClass FloatDivOp = Enums::FloatDiv;
const OpClass FloatSqrtOp = Enums::FloatSqrt;
const OpClass SimdAddOp = Enums::SimdAdd;
const OpClass SimdAddAccOp = Enums::SimdAddAcc;
const OpClass SimdAluOp = Enums::SimdAlu;
const OpClass SimdCmpOp = Enums::SimdCmp;
const OpClass SimdCvtOp = Enums::SimdCvt;
const OpClass SimdMiscOp = Enums::SimdMisc;
const OpClass SimdMultOp = Enums::SimdMult;
const OpClass SimdMultAccOp = Enums::SimdMultAcc;
const OpClass SimdShiftOp = Enums::SimdShift;
const OpClass SimdShiftAccOp = Enums::SimdShiftAcc;
const OpClass SimdSqrtOp = Enums::SimdSqrt;
const OpClass SimdFloatAddOp = Enums::SimdFloatAdd;
const OpClass SimdFloatAluOp = Enums::SimdFloatAlu;
const OpClass SimdFloatCmpOp = Enums::SimdFloatCmp;
const OpClass SimdFloatCvtOp = Enums::SimdFloatCvt;
const OpClass SimdFloatDivOp = Enums::SimdFloatDiv;
const OpClass SimdFloatMiscOp = Enums::SimdFloatMisc;
const OpClass SimdFloatMultOp = Enums::SimdFloatMult;
const OpClass SimdFloatMultAccOp = Enums::SimdFloatMultAcc;
const OpClass SimdFloatSqrtOp = Enums::SimdFloatSqrt;
const OpClass MemReadOp = Enums::MemRead;
const OpClass MemWriteOp = Enums::MemWrite;
const OpClass IprAccessOp = Enums::IprAccess;
const OpClass InstPrefetchOp = Enums::InstPrefetch;
const OpClass Num_OpClasses = Num_OpClass;

#endif // __CPU__OP_CLASS_HH__
