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
#include <vector>

#include "arch/generic/types.hh"
#include "arch/generic/vec_reg.hh"
#include "arch/isa_traits.hh"
#include "arch/riscv/generated/max_inst_regs.hh"
#include "base/types.hh"

namespace RiscvISA {

using RiscvISAInst::MaxInstSrcRegs;
using RiscvISAInst::MaxInstDestRegs;
const int MaxMiscDestRegs = 1;

typedef uint64_t IntReg;
typedef uint64_t FloatRegBits;
typedef double FloatReg;
typedef uint8_t CCReg; // Not applicable to Riscv
typedef uint64_t MiscReg;

// dummy typedefs since we don't have vector regs
const unsigned NumVecElemPerVecReg = 2;
using VecElem = uint32_t;
using VecReg = ::VecRegT<VecElem, NumVecElemPerVecReg, false>;
using ConstVecReg = ::VecRegT<VecElem, NumVecElemPerVecReg, true>;
using VecRegContainer = VecReg::Container;

const int NumIntArchRegs = 32;
const int NumMicroIntRegs = 1;
const int NumIntRegs = NumIntArchRegs + NumMicroIntRegs;
const int NumFloatRegs = 32;
// This has to be one to prevent warnings that are treated as errors
const unsigned NumVecRegs = 1;
const int NumCCRegs = 0;
const int NumMiscRegs = 4096;

// Semantically meaningful register indices
const int ZeroReg = 0;
const int ReturnAddrReg = 1;
const int StackPointerReg = 2;
const int GlobalPointerReg = 3;
const int ThreadPointerReg = 4;
const int FramePointerReg = 8;
const int ReturnValueReg = 10;
const std::vector<int> ReturnValueRegs = {10, 11};
const std::vector<int> ArgumentRegs = {10, 11, 12, 13, 14, 15, 16, 17};
const int AMOTempReg = 32;

const std::vector<std::string> IntRegNames = {
    "zero", "ra", "sp", "gp",
    "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1",
    "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3",
    "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11",
    "t3", "t4", "t5", "t6"
};
const std::vector<std::string> FloatRegNames = {
    "ft0", "ft1", "ft2", "ft3",
    "ft4", "ft5", "ft6", "ft7",
    "fs0", "fs1", "fa0", "fa1",
    "fa2", "fa3", "fa4", "fa5",
    "fa6", "fa7", "fs2", "fs3",
    "fs4", "fs5", "fs6", "fs7",
    "fs8", "fs9", "fs10", "fs11",
    "ft8", "ft9", "ft10", "ft11"
};

const int SyscallNumReg = 17;
const std::vector<int> SyscallArgumentRegs = {10, 11, 12, 13};
const int SyscallPseudoReturnReg = 10;

enum MiscRegIndex {
    MISCREG_USTATUS = 0x000,
    MISCREG_UIE = 0x004,
    MISCREG_UTVEC = 0x005,
    MISCREG_USCRATCH = 0x040,
    MISCREG_UEPC = 0x041,
    MISCREG_UCAUSE = 0x042,
    MISCREG_UBADADDR = 0x043,
    MISCREG_UIP = 0x044,
    MISCREG_FFLAGS = 0x001,
    MISCREG_FRM = 0x002,
    MISCREG_FCSR = 0x003,
    MISCREG_CYCLE = 0xC00,
    MISCREG_TIME = 0xC01,
    MISCREG_INSTRET = 0xC02,
    MISCREG_HPMCOUNTER_BASE = 0xC03,
    MISCREG_CYCLEH = 0xC80,
    MISCREG_TIMEH = 0xC81,
    MISCREG_INSTRETH = 0xC82,
    MISCREG_HPMCOUNTERH_BASE = 0xC83,

    MISCREG_SSTATUS = 0x100,
    MISCREG_SEDELEG = 0x102,
    MISCREG_SIDELEG = 0x103,
    MISCREG_SIE = 0x104,
    MISCREG_STVEC = 0x105,
    MISCREG_SSCRATCH = 0x140,
    MISCREG_SEPC = 0x141,
    MISCREG_SCAUSE = 0x142,
    MISCREG_SBADADDR = 0x143,
    MISCREG_SIP = 0x144,
    MISCREG_SPTBR = 0x180,

    MISCREG_HSTATUS = 0x200,
    MISCREG_HEDELEG = 0x202,
    MISCREG_HIDELEG = 0x203,
    MISCREG_HIE = 0x204,
    MISCREG_HTVEC = 0x205,
    MISCREG_HSCRATCH = 0x240,
    MISCREG_HEPC = 0x241,
    MISCREG_HCAUSE = 0x242,
    MISCREG_HBADADDR = 0x243,
    MISCREG_HIP = 0x244,

    MISCREG_MVENDORID = 0xF11,
    MISCREG_MARCHID = 0xF12,
    MISCREG_MIMPID = 0xF13,
    MISCREG_MHARTID = 0xF14,
    MISCREG_MSTATUS = 0x300,
    MISCREG_MISA = 0x301,
    MISCREG_MEDELEG = 0x302,
    MISCREG_MIDELEG = 0x303,
    MISCREG_MIE = 0x304,
    MISCREG_MTVEC = 0x305,
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
    MISCREG_MCYCLE = 0xB00,
    MISCREG_MINSTRET = 0xB02,
    MISCREG_MHPMCOUNTER_BASE = 0xB03,
    MISCREG_MUCOUNTEREN = 0x320,
    MISCREG_MSCOUNTEREN = 0x321,
    MISCREG_MHCOUNTEREN = 0x322,
    MISCREG_MHPMEVENT_BASE = 0x323,

    MISCREG_TSELECT = 0x7A0,
    MISCREG_TDATA1 = 0x7A1,
    MISCREG_TDATA2 = 0x7A2,
    MISCREG_TDATA3 = 0x7A3,
    MISCREG_DCSR = 0x7B0,
    MISCREG_DPC = 0x7B1,
    MISCREG_DSCRATCH = 0x7B2
};

const std::map<int, std::string> MiscRegNames = {
    {MISCREG_USTATUS, "ustatus"},
    {MISCREG_UIE, "uie"},
    {MISCREG_UTVEC, "utvec"},
    {MISCREG_USCRATCH, "uscratch"},
    {MISCREG_UEPC, "uepc"},
    {MISCREG_UCAUSE, "ucause"},
    {MISCREG_UBADADDR, "ubadaddr"},
    {MISCREG_UIP, "uip"},
    {MISCREG_FFLAGS, "fflags"},
    {MISCREG_FRM, "frm"},
    {MISCREG_FCSR, "fcsr"},
    {MISCREG_CYCLE, "cycle"},
    {MISCREG_TIME, "time"},
    {MISCREG_INSTRET, "instret"},
    {MISCREG_HPMCOUNTER_BASE + 0, "hpmcounter03"},
    {MISCREG_HPMCOUNTER_BASE + 1, "hpmcounter04"},
    {MISCREG_HPMCOUNTER_BASE + 2, "hpmcounter05"},
    {MISCREG_HPMCOUNTER_BASE + 3, "hpmcounter06"},
    {MISCREG_HPMCOUNTER_BASE + 4, "hpmcounter07"},
    {MISCREG_HPMCOUNTER_BASE + 5, "hpmcounter08"},
    {MISCREG_HPMCOUNTER_BASE + 6, "hpmcounter09"},
    {MISCREG_HPMCOUNTER_BASE + 7, "hpmcounter10"},
    {MISCREG_HPMCOUNTER_BASE + 8, "hpmcounter11"},
    {MISCREG_HPMCOUNTER_BASE + 9, "hpmcounter12"},
    {MISCREG_HPMCOUNTER_BASE + 10, "hpmcounter13"},
    {MISCREG_HPMCOUNTER_BASE + 11, "hpmcounter14"},
    {MISCREG_HPMCOUNTER_BASE + 12, "hpmcounter15"},
    {MISCREG_HPMCOUNTER_BASE + 13, "hpmcounter16"},
    {MISCREG_HPMCOUNTER_BASE + 14, "hpmcounter17"},
    {MISCREG_HPMCOUNTER_BASE + 15, "hpmcounter18"},
    {MISCREG_HPMCOUNTER_BASE + 16, "hpmcounter19"},
    {MISCREG_HPMCOUNTER_BASE + 17, "hpmcounter20"},
    {MISCREG_HPMCOUNTER_BASE + 18, "hpmcounter21"},
    {MISCREG_HPMCOUNTER_BASE + 19, "hpmcounter22"},
    {MISCREG_HPMCOUNTER_BASE + 20, "hpmcounter23"},
    {MISCREG_HPMCOUNTER_BASE + 21, "hpmcounter24"},
    {MISCREG_HPMCOUNTER_BASE + 22, "hpmcounter25"},
    {MISCREG_HPMCOUNTER_BASE + 23, "hpmcounter26"},
    {MISCREG_HPMCOUNTER_BASE + 24, "hpmcounter27"},
    {MISCREG_HPMCOUNTER_BASE + 25, "hpmcounter28"},
    {MISCREG_HPMCOUNTER_BASE + 26, "hpmcounter29"},
    {MISCREG_HPMCOUNTER_BASE + 27, "hpmcounter30"},
    {MISCREG_HPMCOUNTER_BASE + 28, "hpmcounter31"},
    {MISCREG_CYCLEH, "cycleh"},
    {MISCREG_TIMEH, "timeh"},
    {MISCREG_INSTRETH, "instreth"},
    {MISCREG_HPMCOUNTERH_BASE + 0, "hpmcounterh03"},
    {MISCREG_HPMCOUNTERH_BASE + 1, "hpmcounterh04"},
    {MISCREG_HPMCOUNTERH_BASE + 2, "hpmcounterh05"},
    {MISCREG_HPMCOUNTERH_BASE + 3, "hpmcounterh06"},
    {MISCREG_HPMCOUNTERH_BASE + 4, "hpmcounterh07"},
    {MISCREG_HPMCOUNTERH_BASE + 5, "hpmcounterh08"},
    {MISCREG_HPMCOUNTERH_BASE + 6, "hpmcounterh09"},
    {MISCREG_HPMCOUNTERH_BASE + 7, "hpmcounterh10"},
    {MISCREG_HPMCOUNTERH_BASE + 8, "hpmcounterh11"},
    {MISCREG_HPMCOUNTERH_BASE + 9, "hpmcounterh12"},
    {MISCREG_HPMCOUNTERH_BASE + 10, "hpmcounterh13"},
    {MISCREG_HPMCOUNTERH_BASE + 11, "hpmcounterh14"},
    {MISCREG_HPMCOUNTERH_BASE + 12, "hpmcounterh15"},
    {MISCREG_HPMCOUNTERH_BASE + 13, "hpmcounterh16"},
    {MISCREG_HPMCOUNTERH_BASE + 14, "hpmcounterh17"},
    {MISCREG_HPMCOUNTERH_BASE + 15, "hpmcounterh18"},
    {MISCREG_HPMCOUNTERH_BASE + 16, "hpmcounterh19"},
    {MISCREG_HPMCOUNTERH_BASE + 17, "hpmcounterh20"},
    {MISCREG_HPMCOUNTERH_BASE + 18, "hpmcounterh21"},
    {MISCREG_HPMCOUNTERH_BASE + 19, "hpmcounterh22"},
    {MISCREG_HPMCOUNTERH_BASE + 20, "hpmcounterh23"},
    {MISCREG_HPMCOUNTERH_BASE + 21, "hpmcounterh24"},
    {MISCREG_HPMCOUNTERH_BASE + 22, "hpmcounterh25"},
    {MISCREG_HPMCOUNTERH_BASE + 23, "hpmcounterh26"},
    {MISCREG_HPMCOUNTERH_BASE + 24, "hpmcounterh27"},
    {MISCREG_HPMCOUNTERH_BASE + 25, "hpmcounterh28"},
    {MISCREG_HPMCOUNTERH_BASE + 26, "hpmcounterh29"},
    {MISCREG_HPMCOUNTERH_BASE + 27, "hpmcounterh30"},
    {MISCREG_HPMCOUNTERH_BASE + 28, "hpmcounterh31"},

    {MISCREG_SSTATUS, "sstatus"},
    {MISCREG_SEDELEG, "sedeleg"},
    {MISCREG_SIDELEG, "sideleg"},
    {MISCREG_SIE, "sie"},
    {MISCREG_STVEC, "stvec"},
    {MISCREG_SSCRATCH, "sscratch"},
    {MISCREG_SEPC, "sepc"},
    {MISCREG_SCAUSE, "scause"},
    {MISCREG_SBADADDR, "sbadaddr"},
    {MISCREG_SIP, "sip"},
    {MISCREG_SPTBR, "sptbr"},

    {MISCREG_HSTATUS, "hstatus"},
    {MISCREG_HEDELEG, "hedeleg"},
    {MISCREG_HIDELEG, "hideleg"},
    {MISCREG_HIE, "hie"},
    {MISCREG_HTVEC, "htvec"},
    {MISCREG_HSCRATCH, "hscratch"},
    {MISCREG_HEPC, "hepc"},
    {MISCREG_HCAUSE, "hcause"},
    {MISCREG_HBADADDR, "hbadaddr"},
    {MISCREG_HIP, "hip"},

    {MISCREG_MVENDORID, "mvendorid"},
    {MISCREG_MARCHID, "marchid"},
    {MISCREG_MIMPID, "mimpid"},
    {MISCREG_MHARTID, "mhartid"},
    {MISCREG_MSTATUS, "mstatus"},
    {MISCREG_MISA, "misa"},
    {MISCREG_MEDELEG, "medeleg"},
    {MISCREG_MIDELEG, "mideleg"},
    {MISCREG_MIE, "mie"},
    {MISCREG_MTVEC, "mtvec"},
    {MISCREG_MSCRATCH, "mscratch"},
    {MISCREG_MEPC, "mepc"},
    {MISCREG_MCAUSE, "mcause"},
    {MISCREG_MBADADDR, "mbadaddr"},
    {MISCREG_MIP, "mip"},
    {MISCREG_MBASE, "mbase"},
    {MISCREG_MBOUND, "mbound"},
    {MISCREG_MIBASE, "mibase"},
    {MISCREG_MIBOUND, "mibound"},
    {MISCREG_MDBASE, "mdbase"},
    {MISCREG_MDBOUND, "mdbound"},
    {MISCREG_MCYCLE, "mcycle"},
    {MISCREG_MINSTRET, "minstret"},
    {MISCREG_MHPMCOUNTER_BASE + 0, "mhpmcounter03"},
    {MISCREG_MHPMCOUNTER_BASE + 1, "mhpmcounter04"},
    {MISCREG_MHPMCOUNTER_BASE + 2, "mhpmcounter05"},
    {MISCREG_MHPMCOUNTER_BASE + 3, "mhpmcounter06"},
    {MISCREG_MHPMCOUNTER_BASE + 4, "mhpmcounter07"},
    {MISCREG_MHPMCOUNTER_BASE + 5, "mhpmcounter08"},
    {MISCREG_MHPMCOUNTER_BASE + 6, "mhpmcounter09"},
    {MISCREG_MHPMCOUNTER_BASE + 7, "mhpmcounter10"},
    {MISCREG_MHPMCOUNTER_BASE + 8, "mhpmcounter11"},
    {MISCREG_MHPMCOUNTER_BASE + 9, "mhpmcounter12"},
    {MISCREG_MHPMCOUNTER_BASE + 10, "mhpmcounter13"},
    {MISCREG_MHPMCOUNTER_BASE + 11, "mhpmcounter14"},
    {MISCREG_MHPMCOUNTER_BASE + 12, "mhpmcounter15"},
    {MISCREG_MHPMCOUNTER_BASE + 13, "mhpmcounter16"},
    {MISCREG_MHPMCOUNTER_BASE + 14, "mhpmcounter17"},
    {MISCREG_MHPMCOUNTER_BASE + 15, "mhpmcounter18"},
    {MISCREG_MHPMCOUNTER_BASE + 16, "mhpmcounter19"},
    {MISCREG_MHPMCOUNTER_BASE + 17, "mhpmcounter20"},
    {MISCREG_MHPMCOUNTER_BASE + 18, "mhpmcounter21"},
    {MISCREG_MHPMCOUNTER_BASE + 19, "mhpmcounter22"},
    {MISCREG_MHPMCOUNTER_BASE + 20, "mhpmcounter23"},
    {MISCREG_MHPMCOUNTER_BASE + 21, "mhpmcounter24"},
    {MISCREG_MHPMCOUNTER_BASE + 22, "mhpmcounter25"},
    {MISCREG_MHPMCOUNTER_BASE + 23, "mhpmcounter26"},
    {MISCREG_MHPMCOUNTER_BASE + 24, "mhpmcounter27"},
    {MISCREG_MHPMCOUNTER_BASE + 25, "mhpmcounter28"},
    {MISCREG_MHPMCOUNTER_BASE + 26, "mhpmcounter29"},
    {MISCREG_MHPMCOUNTER_BASE + 27, "mhpmcounter30"},
    {MISCREG_MHPMCOUNTER_BASE + 28, "mhpmcounter31"},
    {MISCREG_MUCOUNTEREN, "mucounteren"},
    {MISCREG_MSCOUNTEREN, "mscounteren"},
    {MISCREG_MHCOUNTEREN, "mhcounteren"},
    {MISCREG_MHPMEVENT_BASE + 0, "mhpmevent03"},
    {MISCREG_MHPMEVENT_BASE + 1, "mhpmevent04"},
    {MISCREG_MHPMEVENT_BASE + 2, "mhpmevent05"},
    {MISCREG_MHPMEVENT_BASE + 3, "mhpmevent06"},
    {MISCREG_MHPMEVENT_BASE + 4, "mhpmevent07"},
    {MISCREG_MHPMEVENT_BASE + 5, "mhpmevent08"},
    {MISCREG_MHPMEVENT_BASE + 6, "mhpmevent09"},
    {MISCREG_MHPMEVENT_BASE + 7, "mhpmevent10"},
    {MISCREG_MHPMEVENT_BASE + 8, "mhpmevent11"},
    {MISCREG_MHPMEVENT_BASE + 9, "mhpmevent12"},
    {MISCREG_MHPMEVENT_BASE + 10, "mhpmevent13"},
    {MISCREG_MHPMEVENT_BASE + 11, "mhpmevent14"},
    {MISCREG_MHPMEVENT_BASE + 12, "mhpmevent15"},
    {MISCREG_MHPMEVENT_BASE + 13, "mhpmevent16"},
    {MISCREG_MHPMEVENT_BASE + 14, "mhpmevent17"},
    {MISCREG_MHPMEVENT_BASE + 15, "mhpmevent18"},
    {MISCREG_MHPMEVENT_BASE + 16, "mhpmevent19"},
    {MISCREG_MHPMEVENT_BASE + 17, "mhpmevent20"},
    {MISCREG_MHPMEVENT_BASE + 18, "mhpmevent21"},
    {MISCREG_MHPMEVENT_BASE + 19, "mhpmevent22"},
    {MISCREG_MHPMEVENT_BASE + 20, "mhpmevent23"},
    {MISCREG_MHPMEVENT_BASE + 21, "mhpmevent24"},
    {MISCREG_MHPMEVENT_BASE + 22, "mhpmevent25"},
    {MISCREG_MHPMEVENT_BASE + 23, "mhpmevent26"},
    {MISCREG_MHPMEVENT_BASE + 24, "mhpmevent27"},
    {MISCREG_MHPMEVENT_BASE + 25, "mhpmevent28"},
    {MISCREG_MHPMEVENT_BASE + 26, "mhpmevent29"},
    {MISCREG_MHPMEVENT_BASE + 27, "mhpmevent30"},
    {MISCREG_MHPMEVENT_BASE + 28, "mhpmevent31"},

    {MISCREG_TSELECT, "tselect"},
    {MISCREG_TDATA1, "tdata1"},
    {MISCREG_TDATA2, "tdata2"},
    {MISCREG_TDATA3, "tdata3"},
    {MISCREG_DCSR, "dcsr"},
    {MISCREG_DPC, "dpc"},
    {MISCREG_DSCRATCH, "dscratch"}
};

}

#endif // __ARCH_RISCV_REGISTERS_HH__
