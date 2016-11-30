/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
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
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Andreas Hansson
 *          Sven Karlsson
 *          Alec Roelke
 */

#ifndef __ARCH_RISCV_REGISTERS_HH__
#define __ARCH_RISCV_REGISTERS_HH__

#include <map>
#include <string>

#include "arch/riscv/generated/max_inst_regs.hh"
#include "base/types.hh"
#include "sim/system.hh"

namespace RiscvISA {

using RiscvISAInst::MaxInstSrcRegs;
using RiscvISAInst::MaxInstDestRegs;
const int MaxMiscDestRegs = 1;

typedef uint_fast16_t RegIndex;
typedef uint64_t IntReg;
typedef uint64_t FloatRegBits;
typedef double FloatReg;
typedef uint8_t CCReg; // Not applicable to Riscv
typedef uint64_t MiscReg;

const int NumIntArchRegs = 32;
const int NumIntRegs = NumIntArchRegs;
const int NumFloatRegs = 32;
const int NumCCRegs = 0;
const int NumMiscRegs = 4096;

// These help enumerate all the registers for dependence tracking.
const int FP_Reg_Base = NumIntRegs;
const int CC_Reg_Base = FP_Reg_Base + NumFloatRegs;
const int Misc_Reg_Base = CC_Reg_Base + NumCCRegs;
const int Max_Reg_Index = Misc_Reg_Base + NumMiscRegs;


// Semantically meaningful register indices
const int ZeroReg = 0;
const int ReturnAddrReg = 1;
const int StackPointerReg = 2;
const int GlobalPointerReg = 3;
const int ThreadPointerReg = 4;
const int FramePointerReg = 8;
const int ReturnValueRegs[] = {10, 11};
const int ReturnValueReg = ReturnValueRegs[0];
const int ArgumentRegs[] = {10, 11, 12, 13, 14, 15, 16, 17};

const char* const RegisterNames[] = {"zero", "ra", "sp", "gp",
    "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1",
    "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3",
    "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11",
    "t3", "t4", "t5", "t6"};

const int SyscallNumReg = ArgumentRegs[7];
const int SyscallArgumentRegs[] = {ArgumentRegs[0], ArgumentRegs[1],
    ArgumentRegs[2], ArgumentRegs[3]};
const int SyscallPseudoReturnReg = ReturnValueRegs[0];

enum MiscRegIndex {
    MISCREG_FFLAGS = 0x001,
    MISCREG_FRM = 0x002,
    MISCREG_FCSR = 0x003,
    MISCREG_CYCLE = 0xC00,
    MISCREG_TIME = 0xC01,
    MISCREG_INSTRET = 0xC02,
    MISCREG_CYCLEH = 0xC80,
    MISCREG_TIMEH = 0xC81,
    MISCREG_INSTRETH = 0xC82,

    MISCREG_SSTATUS = 0x100,
    MISCREG_STVEC = 0x101,
    MISCREG_SIE = 0x104,
    MISCREG_STIMECMP = 0x121,
    MISCREG_STIME = 0xD01,
    MISCREG_STIMEH = 0xD81,
    MISCREG_SSCRATCH = 0x140,
    MISCREG_SEPC = 0x141,
    MISCREG_SCAUSE = 0xD42,
    MISCREG_SBADADDR = 0xD43,
    MISCREG_SIP = 0x144,
    MISCREG_SPTBR = 0x180,
    MISCREG_SASID = 0x181,
    MISCREG_CYCLEW = 0x900,
    MISCREG_TIMEW = 0x901,
    MISCREG_INSTRETW = 0x902,
    MISCREG_CYCLEHW = 0x980,
    MISCREG_TIMEHW = 0x981,
    MISCREG_INSTRETHW = 0x982,

    MISCREG_HSTATUS = 0x200,
    MISCREG_HTVEC = 0x201,
    MISCREG_HTDELEG = 0x202,
    MISCREG_HTIMECMP = 0x221,
    MISCREG_HTIME = 0xE01,
    MISCREG_HTIMEH = 0xE81,
    MISCREG_HSCRATCH = 0x240,
    MISCREG_HEPC = 0x241,
    MISCREG_HCAUSE = 0x242,
    MISCREG_HBADADDR = 0x243,
    MISCREG_STIMEW = 0xA01,
    MISCREG_STIMEHW = 0xA81,

    MISCREG_MCPUID = 0xF00,
    MISCREG_MIMPID = 0xF01,
    MISCREG_MHARTID = 0xF10,
    MISCREG_MSTATUS = 0x300,
    MISCREG_MTVEC = 0x301,
    MISCREG_MTDELEG = 0x302,
    MISCREG_MIE = 0x304,
    MISCREG_MTIMECMP = 0x321,
    MISCREG_MTIME = 0x701,
    MISCREG_MTIMEH = 0x741,
    MISCREG_MSCRATCH = 0x340,
    MISCREG_MEPC = 0x341,
    MISCREG_MCAUSE = 0x342,
    MISCREG_MBADADDR = 0x343,
    MISCREG_MIP = 0x344,
    MISCREG_MBASE = 0x380,
    MISCREG_MBOUND = 0x381,
    MISCREG_MIBASE = 0x382,
    MISCREG_MIBOUND = 0x383,
    MISCREG_MDBASE = 0x384,
    MISCREG_MDBOUND = 0x385,
    MISCREG_HTIMEW = 0xB01,
    MISCREG_HTIMEHW = 0xB81,
    MISCREG_MTOHOST = 0x780,
    MISCREG_MFROMHOST = 0x781
};

}

#endif // __ARCH_RISCV_REGISTERS_HH__
