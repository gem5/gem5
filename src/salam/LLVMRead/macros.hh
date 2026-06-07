/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#ifndef MACROS_HH
#define MACROS_HH

// Function Macros
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// LLVM Instructions Definitions (Re-Define From llvm/IR/Instructions.def)
#define LLVM_IR_Move 0
#define LLVM_IR_Ret 1
#define LLVM_IR_Br 2
#define LLVM_IR_Switch 3
#define LLVM_IR_IndirectBr 4
#define LLVM_IR_Invoke 5
#define LLVM_IR_Resume 6
#define LLVM_IR_Unreachable 7
#define LLVM_IR_Add 13
#define LLVM_IR_FAdd 14
#define LLVM_IR_Sub 15
#define LLVM_IR_FSub 16
#define LLVM_IR_Mul 17
#define LLVM_IR_FMul 18
#define LLVM_IR_UDiv 19
#define LLVM_IR_SDiv 20
#define LLVM_IR_FDiv 21
#define LLVM_IR_URem 22
#define LLVM_IR_SRem 23
#define LLVM_IR_FRem 24
#define LLVM_IR_Shl 25
#define LLVM_IR_LShr 26
#define LLVM_IR_AShr 27
#define LLVM_IR_And 28
#define LLVM_IR_Or 29
#define LLVM_IR_Xor 30
#define LLVM_IR_Alloca 31
#define LLVM_IR_Load 32
#define LLVM_IR_Store 33
#define LLVM_IR_GetElementPtr 34
#define LLVM_IR_Fence 35
#define LLVM_IR_AtomicCmpXchg 36
#define LLVM_IR_AtomicRMW 37
#define LLVM_IR_Trunc 38
#define LLVM_IR_ZExt 39
#define LLVM_IR_SExt 40
#define LLVM_IR_FPToUI 41
#define LLVM_IR_FPToSI 42
#define LLVM_IR_UIToFP 43
#define LLVM_IR_SIToFP 44
#define LLVM_IR_FPTrunc 45
#define LLVM_IR_FPExt 46
#define LLVM_IR_PtrToInt 47
#define LLVM_IR_IntToPtr 48
#define LLVM_IR_BitCast 49
#define LLVM_IR_AddrSpaceCast 50
#define LLVM_IR_ICmp 53
#define LLVM_IR_FCmp 54
#define LLVM_IR_PHI 55
#define LLVM_IR_Call 56
#define LLVM_IR_Select 57
#define LLVM_IR_VAArg 60
#define LLVM_IR_ExtractElement 61
#define LLVM_IR_InsertElement 62
#define LLVM_IR_ShuffleVector 63
#define LLVM_IR_ExtractValue 64
#define LLVM_IR_InsertValue 65
#define LLVM_IR_LandingPad 66
#define LLVM_IR_DMAFence 97
#define LLVM_IR_DMAStore 98
#define LLVM_IR_DMALoad 99
#define LLVM_IR_IndexAdd 100
#define LLVM_IR_SilentStore 101
#define LLVM_IR_Sine 102
#define LLVM_IR_Cosine 103

/**
 * @namespace SALAM
 */

namespace SALAM
{
// LLVM comparison codes
enum Predicate : unsigned
{
    FCMP_FALSE = 0,
    FCMP_OEQ = 1,
    FCMP_OGT = 2,
    FCMP_OGE = 3,
    FCMP_OLT = 4,
    FCMP_OLE = 5,
    FCMP_ONE = 6,
    FCMP_ORD = 7,
    FCMP_UNO = 8,
    FCMP_UEQ = 9,
    FCMP_UGT = 10,
    FCMP_UGE = 11,
    FCMP_ULT = 12,
    FCMP_ULE = 13,
    FCMP_UNE = 14,
    FCMP_TRUE = 15,
    FIRST_FCMP_PREDICATE = FCMP_FALSE,
    LAST_FCMP_PREDICATE = FCMP_TRUE,
    BAD_FCMP_PREDICATE = FCMP_TRUE + 1,
    ICMP_EQ = 32,
    ICMP_NE = 33,
    ICMP_UGT = 34,
    ICMP_UGE = 35,
    ICMP_ULT = 36,
    ICMP_ULE = 37,
    ICMP_SGT = 38,
    ICMP_SGE = 39,
    ICMP_SLT = 40,
    ICMP_SLE = 41,
    FIRST_ICMP_PREDICATE = ICMP_EQ,
    LAST_ICMP_PREDICATE = ICMP_SLE,
    BAD_ICMP_PREDICATE = ICMP_SLE + 1
};
} // namespace SALAM

#endif //__MACROS_HH__
