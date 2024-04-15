/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
 * Copyright (c) 2019 Yifei Liu
 * Copyright (c) 2020 Barkhausen Institut
 * Copyright (c) 2021 StreamComputing Corp
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
 * Copyright (c) 2024 University of Rostock
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

#ifndef __ARCH_RISCV_REGS_MISC_HH__
#define __ARCH_RISCV_REGS_MISC_HH__

#include <string>
#include <unordered_map>

#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"
#include "arch/riscv/types.hh"
#include "base/bitunion.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "debug/MiscRegs.hh"
#include "enums/RiscvType.hh"

namespace gem5
{

namespace RiscvISA
{

enum MiscRegIndex
{
    MISCREG_PRV = 0,
    MISCREG_ISA,
    MISCREG_VENDORID,
    MISCREG_ARCHID,
    MISCREG_IMPID,
    MISCREG_HARTID,
    MISCREG_STATUS,
    MISCREG_IP,
    MISCREG_IE,
    MISCREG_CYCLE,
    MISCREG_TIME,
    MISCREG_INSTRET,
    MISCREG_HPMCOUNTER03,
    MISCREG_HPMCOUNTER04,
    MISCREG_HPMCOUNTER05,
    MISCREG_HPMCOUNTER06,
    MISCREG_HPMCOUNTER07,
    MISCREG_HPMCOUNTER08,
    MISCREG_HPMCOUNTER09,
    MISCREG_HPMCOUNTER10,
    MISCREG_HPMCOUNTER11,
    MISCREG_HPMCOUNTER12,
    MISCREG_HPMCOUNTER13,
    MISCREG_HPMCOUNTER14,
    MISCREG_HPMCOUNTER15,
    MISCREG_HPMCOUNTER16,
    MISCREG_HPMCOUNTER17,
    MISCREG_HPMCOUNTER18,
    MISCREG_HPMCOUNTER19,
    MISCREG_HPMCOUNTER20,
    MISCREG_HPMCOUNTER21,
    MISCREG_HPMCOUNTER22,
    MISCREG_HPMCOUNTER23,
    MISCREG_HPMCOUNTER24,
    MISCREG_HPMCOUNTER25,
    MISCREG_HPMCOUNTER26,
    MISCREG_HPMCOUNTER27,
    MISCREG_HPMCOUNTER28,
    MISCREG_HPMCOUNTER29,
    MISCREG_HPMCOUNTER30,
    MISCREG_HPMCOUNTER31,
    MISCREG_HPMEVENT03,
    MISCREG_HPMEVENT04,
    MISCREG_HPMEVENT05,
    MISCREG_HPMEVENT06,
    MISCREG_HPMEVENT07,
    MISCREG_HPMEVENT08,
    MISCREG_HPMEVENT09,
    MISCREG_HPMEVENT10,
    MISCREG_HPMEVENT11,
    MISCREG_HPMEVENT12,
    MISCREG_HPMEVENT13,
    MISCREG_HPMEVENT14,
    MISCREG_HPMEVENT15,
    MISCREG_HPMEVENT16,
    MISCREG_HPMEVENT17,
    MISCREG_HPMEVENT18,
    MISCREG_HPMEVENT19,
    MISCREG_HPMEVENT20,
    MISCREG_HPMEVENT21,
    MISCREG_HPMEVENT22,
    MISCREG_HPMEVENT23,
    MISCREG_HPMEVENT24,
    MISCREG_HPMEVENT25,
    MISCREG_HPMEVENT26,
    MISCREG_HPMEVENT27,
    MISCREG_HPMEVENT28,
    MISCREG_HPMEVENT29,
    MISCREG_HPMEVENT30,
    MISCREG_HPMEVENT31,
    MISCREG_TSELECT,
    MISCREG_TDATA1,
    MISCREG_TDATA2,
    MISCREG_TDATA3,
    MISCREG_DCSR,
    MISCREG_DPC,
    MISCREG_DSCRATCH,

    MISCREG_MEDELEG,
    MISCREG_MIDELEG,
    MISCREG_MTVEC,
    MISCREG_MCOUNTEREN,
    MISCREG_MSCRATCH,
    MISCREG_MEPC,
    MISCREG_MCAUSE,
    MISCREG_MTVAL,
    MISCREG_PMPCFG0,
    MISCREG_PMPCFG1, // pmpcfg1 is rv32 only
    MISCREG_PMPCFG2,
    MISCREG_PMPCFG3, // pmpcfg3 is rv32 only
    MISCREG_PMPADDR00,
    MISCREG_PMPADDR01,
    MISCREG_PMPADDR02,
    MISCREG_PMPADDR03,
    MISCREG_PMPADDR04,
    MISCREG_PMPADDR05,
    MISCREG_PMPADDR06,
    MISCREG_PMPADDR07,
    MISCREG_PMPADDR08,
    MISCREG_PMPADDR09,
    MISCREG_PMPADDR10,
    MISCREG_PMPADDR11,
    MISCREG_PMPADDR12,
    MISCREG_PMPADDR13,
    MISCREG_PMPADDR14,
    MISCREG_PMPADDR15,

    MISCREG_SEDELEG,
    MISCREG_SIDELEG,
    MISCREG_STVEC,
    MISCREG_SCOUNTEREN,
    MISCREG_SSCRATCH,
    MISCREG_SEPC,
    MISCREG_SCAUSE,
    MISCREG_STVAL,
    MISCREG_SATP,

    MISCREG_UTVEC,
    MISCREG_USCRATCH,
    MISCREG_UEPC,
    MISCREG_UCAUSE,
    MISCREG_UTVAL,
    MISCREG_FFLAGS,
    MISCREG_FRM,

    MISCREG_VSTART,
    MISCREG_VXSAT,
    MISCREG_VXRM,
    MISCREG_VCSR,
    MISCREG_VL,
    MISCREG_VTYPE,
    MISCREG_VLENB,

    // These registers are not in the standard, hence does not exist in the
    // CSRData map. These are mainly used to provide a minimal implementation
    // for non-maskable-interrupt in our simple cpu.
    // non-maskable-interrupt-vector-base-address: NMI version of xTVEC
    MISCREG_NMIVEC,
    // non-maskable-interrupt-enable: NMI version of xIE
    MISCREG_NMIE,
    // non-maskable-interrupt-pending: NMI version of xIP
    MISCREG_NMIP,

    // the following MicsRegIndex are RV32 only
    MISCREG_MSTATUSH,

    MISCREG_CYCLEH,
    MISCREG_TIMEH,
    MISCREG_INSTRETH,
    MISCREG_HPMCOUNTER03H,
    MISCREG_HPMCOUNTER04H,
    MISCREG_HPMCOUNTER05H,
    MISCREG_HPMCOUNTER06H,
    MISCREG_HPMCOUNTER07H,
    MISCREG_HPMCOUNTER08H,
    MISCREG_HPMCOUNTER09H,
    MISCREG_HPMCOUNTER10H,
    MISCREG_HPMCOUNTER11H,
    MISCREG_HPMCOUNTER12H,
    MISCREG_HPMCOUNTER13H,
    MISCREG_HPMCOUNTER14H,
    MISCREG_HPMCOUNTER15H,
    MISCREG_HPMCOUNTER16H,
    MISCREG_HPMCOUNTER17H,
    MISCREG_HPMCOUNTER18H,
    MISCREG_HPMCOUNTER19H,
    MISCREG_HPMCOUNTER20H,
    MISCREG_HPMCOUNTER21H,
    MISCREG_HPMCOUNTER22H,
    MISCREG_HPMCOUNTER23H,
    MISCREG_HPMCOUNTER24H,
    MISCREG_HPMCOUNTER25H,
    MISCREG_HPMCOUNTER26H,
    MISCREG_HPMCOUNTER27H,
    MISCREG_HPMCOUNTER28H,
    MISCREG_HPMCOUNTER29H,
    MISCREG_HPMCOUNTER30H,
    MISCREG_HPMCOUNTER31H,

    NUM_PHYS_MISCREGS,

    // This CSR shared the same space with MISCREG_FFLAGS
    MISCREG_FFLAGS_EXE = NUM_PHYS_MISCREGS,

    NUM_MISCREGS
};

inline constexpr RegClass miscRegClass(MiscRegClass, MiscRegClassName,
                                       NUM_MISCREGS, debug::MiscRegs);

enum CSRIndex
{
    CSR_USTATUS = 0x000,
    CSR_UIE = 0x004,
    CSR_UTVEC = 0x005,
    CSR_USCRATCH = 0x040,
    CSR_UEPC = 0x041,
    CSR_UCAUSE = 0x042,
    CSR_UTVAL = 0x043,
    CSR_UIP = 0x044,
    CSR_FFLAGS = 0x001,
    CSR_FRM = 0x002,
    CSR_FCSR = 0x003,
    CSR_CYCLE = 0xC00,
    CSR_TIME = 0xC01,
    CSR_INSTRET = 0xC02,
    CSR_HPMCOUNTER03 = 0xC03,
    CSR_HPMCOUNTER04 = 0xC04,
    CSR_HPMCOUNTER05 = 0xC05,
    CSR_HPMCOUNTER06 = 0xC06,
    CSR_HPMCOUNTER07 = 0xC07,
    CSR_HPMCOUNTER08 = 0xC08,
    CSR_HPMCOUNTER09 = 0xC09,
    CSR_HPMCOUNTER10 = 0xC0A,
    CSR_HPMCOUNTER11 = 0xC0B,
    CSR_HPMCOUNTER12 = 0xC0C,
    CSR_HPMCOUNTER13 = 0xC0D,
    CSR_HPMCOUNTER14 = 0xC0E,
    CSR_HPMCOUNTER15 = 0xC0F,
    CSR_HPMCOUNTER16 = 0xC10,
    CSR_HPMCOUNTER17 = 0xC11,
    CSR_HPMCOUNTER18 = 0xC12,
    CSR_HPMCOUNTER19 = 0xC13,
    CSR_HPMCOUNTER20 = 0xC14,
    CSR_HPMCOUNTER21 = 0xC15,
    CSR_HPMCOUNTER22 = 0xC16,
    CSR_HPMCOUNTER23 = 0xC17,
    CSR_HPMCOUNTER24 = 0xC18,
    CSR_HPMCOUNTER25 = 0xC19,
    CSR_HPMCOUNTER26 = 0xC1A,
    CSR_HPMCOUNTER27 = 0xC1B,
    CSR_HPMCOUNTER28 = 0xC1C,
    CSR_HPMCOUNTER29 = 0xC1D,
    CSR_HPMCOUNTER30 = 0xC1E,
    CSR_HPMCOUNTER31 = 0xC1F,

    // rv32 only csr register begin
    CSR_CYCLEH = 0xC80,
    CSR_TIMEH = 0xC81,
    CSR_INSTRETH = 0xC82,
    CSR_HPMCOUNTER03H = 0xC83,
    CSR_HPMCOUNTER04H = 0xC84,
    CSR_HPMCOUNTER05H = 0xC85,
    CSR_HPMCOUNTER06H = 0xC86,
    CSR_HPMCOUNTER07H = 0xC87,
    CSR_HPMCOUNTER08H = 0xC88,
    CSR_HPMCOUNTER09H = 0xC89,
    CSR_HPMCOUNTER10H = 0xC8A,
    CSR_HPMCOUNTER11H = 0xC8B,
    CSR_HPMCOUNTER12H = 0xC8C,
    CSR_HPMCOUNTER13H = 0xC8D,
    CSR_HPMCOUNTER14H = 0xC8E,
    CSR_HPMCOUNTER15H = 0xC8F,
    CSR_HPMCOUNTER16H = 0xC90,
    CSR_HPMCOUNTER17H = 0xC91,
    CSR_HPMCOUNTER18H = 0xC92,
    CSR_HPMCOUNTER19H = 0xC93,
    CSR_HPMCOUNTER20H = 0xC94,
    CSR_HPMCOUNTER21H = 0xC95,
    CSR_HPMCOUNTER22H = 0xC96,
    CSR_HPMCOUNTER23H = 0xC97,
    CSR_HPMCOUNTER24H = 0xC98,
    CSR_HPMCOUNTER25H = 0xC99,
    CSR_HPMCOUNTER26H = 0xC9A,
    CSR_HPMCOUNTER27H = 0xC9B,
    CSR_HPMCOUNTER28H = 0xC9C,
    CSR_HPMCOUNTER29H = 0xC9D,
    CSR_HPMCOUNTER30H = 0xC9E,
    CSR_HPMCOUNTER31H = 0xC9F,
    // rv32 only csr register end

    CSR_SSTATUS = 0x100,
    CSR_SEDELEG = 0x102,
    CSR_SIDELEG = 0x103,
    CSR_SIE = 0x104,
    CSR_STVEC = 0x105,
    CSR_SCOUNTEREN = 0x106,
    CSR_SSCRATCH = 0x140,
    CSR_SEPC = 0x141,
    CSR_SCAUSE = 0x142,
    CSR_STVAL = 0x143,
    CSR_SIP = 0x144,
    CSR_SATP = 0x180,

    CSR_MVENDORID = 0xF11,
    CSR_MARCHID = 0xF12,
    CSR_MIMPID = 0xF13,
    CSR_MHARTID = 0xF14,
    CSR_MSTATUS = 0x300,
    CSR_MISA = 0x301,
    CSR_MEDELEG = 0x302,
    CSR_MIDELEG = 0x303,
    CSR_MIE = 0x304,
    CSR_MTVEC = 0x305,
    CSR_MCOUNTEREN = 0x306,
    CSR_MSTATUSH = 0x310, // rv32 only
    CSR_MSCRATCH = 0x340,
    CSR_MEPC = 0x341,
    CSR_MCAUSE = 0x342,
    CSR_MTVAL = 0x343,
    CSR_MIP = 0x344,
    CSR_PMPCFG0 = 0x3A0,
    CSR_PMPCFG1 = 0x3A1, // pmpcfg1 rv32 only
    CSR_PMPCFG2 = 0x3A2,
    CSR_PMPCFG3 = 0x3A3, // pmpcfg3 rv32 only
    CSR_PMPADDR00 = 0x3B0,
    CSR_PMPADDR01 = 0x3B1,
    CSR_PMPADDR02 = 0x3B2,
    CSR_PMPADDR03 = 0x3B3,
    CSR_PMPADDR04 = 0x3B4,
    CSR_PMPADDR05 = 0x3B5,
    CSR_PMPADDR06 = 0x3B6,
    CSR_PMPADDR07 = 0x3B7,
    CSR_PMPADDR08 = 0x3B8,
    CSR_PMPADDR09 = 0x3B9,
    CSR_PMPADDR10 = 0x3BA,
    CSR_PMPADDR11 = 0x3BB,
    CSR_PMPADDR12 = 0x3BC,
    CSR_PMPADDR13 = 0x3BD,
    CSR_PMPADDR14 = 0x3BE,
    CSR_PMPADDR15 = 0x3BF,
    CSR_MCYCLE = 0xB00,
    CSR_MINSTRET = 0xB02,
    CSR_MHPMCOUNTER03 = 0xB03,
    CSR_MHPMCOUNTER04 = 0xB04,
    CSR_MHPMCOUNTER05 = 0xB05,
    CSR_MHPMCOUNTER06 = 0xB06,
    CSR_MHPMCOUNTER07 = 0xB07,
    CSR_MHPMCOUNTER08 = 0xB08,
    CSR_MHPMCOUNTER09 = 0xB09,
    CSR_MHPMCOUNTER10 = 0xB0A,
    CSR_MHPMCOUNTER11 = 0xB0B,
    CSR_MHPMCOUNTER12 = 0xB0C,
    CSR_MHPMCOUNTER13 = 0xB0D,
    CSR_MHPMCOUNTER14 = 0xB0E,
    CSR_MHPMCOUNTER15 = 0xB0F,
    CSR_MHPMCOUNTER16 = 0xB10,
    CSR_MHPMCOUNTER17 = 0xB11,
    CSR_MHPMCOUNTER18 = 0xB12,
    CSR_MHPMCOUNTER19 = 0xB13,
    CSR_MHPMCOUNTER20 = 0xB14,
    CSR_MHPMCOUNTER21 = 0xB15,
    CSR_MHPMCOUNTER22 = 0xB16,
    CSR_MHPMCOUNTER23 = 0xB17,
    CSR_MHPMCOUNTER24 = 0xB18,
    CSR_MHPMCOUNTER25 = 0xB19,
    CSR_MHPMCOUNTER26 = 0xB1A,
    CSR_MHPMCOUNTER27 = 0xB1B,
    CSR_MHPMCOUNTER28 = 0xB1C,
    CSR_MHPMCOUNTER29 = 0xB1D,
    CSR_MHPMCOUNTER30 = 0xB1E,
    CSR_MHPMCOUNTER31 = 0xB1F,

    // rv32 only csr register begin
    CSR_MCYCLEH = 0xB80,
    CSR_MINSTRETH = 0xB82,
    CSR_MHPMCOUNTER03H = 0xB83,
    CSR_MHPMCOUNTER04H = 0xB84,
    CSR_MHPMCOUNTER05H = 0xB85,
    CSR_MHPMCOUNTER06H = 0xB86,
    CSR_MHPMCOUNTER07H = 0xB87,
    CSR_MHPMCOUNTER08H = 0xB88,
    CSR_MHPMCOUNTER09H = 0xB89,
    CSR_MHPMCOUNTER10H = 0xB8A,
    CSR_MHPMCOUNTER11H = 0xB8B,
    CSR_MHPMCOUNTER12H = 0xB8C,
    CSR_MHPMCOUNTER13H = 0xB8D,
    CSR_MHPMCOUNTER14H = 0xB8E,
    CSR_MHPMCOUNTER15H = 0xB8F,
    CSR_MHPMCOUNTER16H = 0xB90,
    CSR_MHPMCOUNTER17H = 0xB91,
    CSR_MHPMCOUNTER18H = 0xB92,
    CSR_MHPMCOUNTER19H = 0xB93,
    CSR_MHPMCOUNTER20H = 0xB94,
    CSR_MHPMCOUNTER21H = 0xB95,
    CSR_MHPMCOUNTER22H = 0xB96,
    CSR_MHPMCOUNTER23H = 0xB97,
    CSR_MHPMCOUNTER24H = 0xB98,
    CSR_MHPMCOUNTER25H = 0xB99,
    CSR_MHPMCOUNTER26H = 0xB9A,
    CSR_MHPMCOUNTER27H = 0xB9B,
    CSR_MHPMCOUNTER28H = 0xB9C,
    CSR_MHPMCOUNTER29H = 0xB9D,
    CSR_MHPMCOUNTER30H = 0xB9E,
    CSR_MHPMCOUNTER31H = 0xB9F,
    // rv32 only csr register end

    CSR_MHPMEVENT03 = 0x323,
    CSR_MHPMEVENT04 = 0x324,
    CSR_MHPMEVENT05 = 0x325,
    CSR_MHPMEVENT06 = 0x326,
    CSR_MHPMEVENT07 = 0x327,
    CSR_MHPMEVENT08 = 0x328,
    CSR_MHPMEVENT09 = 0x329,
    CSR_MHPMEVENT10 = 0x32A,
    CSR_MHPMEVENT11 = 0x32B,
    CSR_MHPMEVENT12 = 0x32C,
    CSR_MHPMEVENT13 = 0x32D,
    CSR_MHPMEVENT14 = 0x32E,
    CSR_MHPMEVENT15 = 0x32F,
    CSR_MHPMEVENT16 = 0x330,
    CSR_MHPMEVENT17 = 0x331,
    CSR_MHPMEVENT18 = 0x332,
    CSR_MHPMEVENT19 = 0x333,
    CSR_MHPMEVENT20 = 0x334,
    CSR_MHPMEVENT21 = 0x335,
    CSR_MHPMEVENT22 = 0x336,
    CSR_MHPMEVENT23 = 0x337,
    CSR_MHPMEVENT24 = 0x338,
    CSR_MHPMEVENT25 = 0x339,
    CSR_MHPMEVENT26 = 0x33A,
    CSR_MHPMEVENT27 = 0x33B,
    CSR_MHPMEVENT28 = 0x33C,
    CSR_MHPMEVENT29 = 0x33D,
    CSR_MHPMEVENT30 = 0x33E,
    CSR_MHPMEVENT31 = 0x33F,

    CSR_TSELECT = 0x7A0,
    CSR_TDATA1 = 0x7A1,
    CSR_TDATA2 = 0x7A2,
    CSR_TDATA3 = 0x7A3,
    CSR_DCSR = 0x7B0,
    CSR_DPC = 0x7B1,
    CSR_DSCRATCH = 0x7B2,

    CSR_VSTART = 0x008,
    CSR_VXSAT = 0x009,
    CSR_VXRM = 0x00A,
    CSR_VCSR = 0x00F,
    CSR_VL = 0xC20,
    CSR_VTYPE = 0xC21,
    CSR_VLENB = 0xC22
};

struct CSRMetadata
{
    const std::string name;
    const int physIndex;
    const uint64_t rvTypes;
    const uint64_t isaExts;
};

template <typename... T>
constexpr uint64_t
rvTypeFlags(T... args)
{
    return ((1 << args) | ...);
}

template <typename... T>
constexpr uint64_t
isaExtsFlags(T... isa_exts)
{
    return ((1ULL << (isa_exts - 'a')) | ...);
}

constexpr uint64_t
isaExtsFlags()
{
    return 0ULL;
}

const std::unordered_map<int, CSRMetadata> CSRData = {
    { CSR_USTATUS,
      { "ustatus", MISCREG_STATUS, rvTypeFlags(RV64, RV32),
        isaExtsFlags('n') } },
    { CSR_UIE,
      { "uie", MISCREG_IE, rvTypeFlags(RV64, RV32), isaExtsFlags('n') } },
    { CSR_UTVEC,
      { "utvec", MISCREG_UTVEC, rvTypeFlags(RV64, RV32), isaExtsFlags('n') } },
    { CSR_USCRATCH,
      { "uscratch", MISCREG_USCRATCH, rvTypeFlags(RV64, RV32),
        isaExtsFlags('n') } },
    { CSR_UEPC,
      { "uepc", MISCREG_UEPC, rvTypeFlags(RV64, RV32), isaExtsFlags('n') } },
    { CSR_UCAUSE,
      { "ucause", MISCREG_UCAUSE, rvTypeFlags(RV64, RV32),
        isaExtsFlags('n') } },
    { CSR_UTVAL,
      { "utval", MISCREG_UTVAL, rvTypeFlags(RV64, RV32), isaExtsFlags('n') } },
    { CSR_UIP,
      { "uip", MISCREG_IP, rvTypeFlags(RV64, RV32), isaExtsFlags('n') } },
    { CSR_FFLAGS,
      { "fflags", MISCREG_FFLAGS, rvTypeFlags(RV64, RV32),
        isaExtsFlags('f') } },
    { CSR_FRM,
      { "frm", MISCREG_FRM, rvTypeFlags(RV64, RV32), isaExtsFlags('f') } },
    // Actually FRM << 5 | FFLAGS
    { CSR_FCSR,
      { "fcsr", MISCREG_FFLAGS, rvTypeFlags(RV64, RV32), isaExtsFlags('f') } },
    { CSR_CYCLE,
      { "cycle", MISCREG_CYCLE, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_TIME,
      { "time", MISCREG_TIME, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_INSTRET,
      { "instret", MISCREG_INSTRET, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER03,
      { "hpmcounter03", MISCREG_HPMCOUNTER03, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER04,
      { "hpmcounter04", MISCREG_HPMCOUNTER04, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER05,
      { "hpmcounter05", MISCREG_HPMCOUNTER05, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER06,
      { "hpmcounter06", MISCREG_HPMCOUNTER06, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER07,
      { "hpmcounter07", MISCREG_HPMCOUNTER07, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER08,
      { "hpmcounter08", MISCREG_HPMCOUNTER08, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER09,
      { "hpmcounter09", MISCREG_HPMCOUNTER09, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER10,
      { "hpmcounter10", MISCREG_HPMCOUNTER10, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER11,
      { "hpmcounter11", MISCREG_HPMCOUNTER11, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER12,
      { "hpmcounter12", MISCREG_HPMCOUNTER12, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER13,
      { "hpmcounter13", MISCREG_HPMCOUNTER13, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER14,
      { "hpmcounter14", MISCREG_HPMCOUNTER14, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER15,
      { "hpmcounter15", MISCREG_HPMCOUNTER15, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER16,
      { "hpmcounter16", MISCREG_HPMCOUNTER16, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER17,
      { "hpmcounter17", MISCREG_HPMCOUNTER17, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER18,
      { "hpmcounter18", MISCREG_HPMCOUNTER18, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER19,
      { "hpmcounter19", MISCREG_HPMCOUNTER19, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER20,
      { "hpmcounter20", MISCREG_HPMCOUNTER20, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER21,
      { "hpmcounter21", MISCREG_HPMCOUNTER21, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER22,
      { "hpmcounter22", MISCREG_HPMCOUNTER22, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER23,
      { "hpmcounter23", MISCREG_HPMCOUNTER23, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER24,
      { "hpmcounter24", MISCREG_HPMCOUNTER24, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER25,
      { "hpmcounter25", MISCREG_HPMCOUNTER25, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER26,
      { "hpmcounter26", MISCREG_HPMCOUNTER26, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER27,
      { "hpmcounter27", MISCREG_HPMCOUNTER27, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER28,
      { "hpmcounter28", MISCREG_HPMCOUNTER28, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER29,
      { "hpmcounter29", MISCREG_HPMCOUNTER29, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER30,
      { "hpmcounter30", MISCREG_HPMCOUNTER30, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER31,
      { "hpmcounter31", MISCREG_HPMCOUNTER31, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_CYCLEH,
      { "cycleh", MISCREG_CYCLEH, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_TIMEH,
      { "timeh", MISCREG_TIMEH, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_INSTRETH,
      { "instreth", MISCREG_INSTRETH, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_HPMCOUNTER03H,
      { "hpmcounter03h", MISCREG_HPMCOUNTER03H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER04H,
      { "hpmcounter04h", MISCREG_HPMCOUNTER04H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER05H,
      { "hpmcounter05h", MISCREG_HPMCOUNTER05H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER06H,
      { "hpmcounter06h", MISCREG_HPMCOUNTER06H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER07H,
      { "hpmcounter07h", MISCREG_HPMCOUNTER07H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER08H,
      { "hpmcounter08h", MISCREG_HPMCOUNTER08H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER09H,
      { "hpmcounter09h", MISCREG_HPMCOUNTER09H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER10H,
      { "hpmcounter10h", MISCREG_HPMCOUNTER10H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER11H,
      { "hpmcounter11h", MISCREG_HPMCOUNTER11H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER12H,
      { "hpmcounter12h", MISCREG_HPMCOUNTER12H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER13H,
      { "hpmcounter13h", MISCREG_HPMCOUNTER13H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER14H,
      { "hpmcounter14h", MISCREG_HPMCOUNTER14H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER15H,
      { "hpmcounter15h", MISCREG_HPMCOUNTER15H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER16H,
      { "hpmcounter16h", MISCREG_HPMCOUNTER16H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER17H,
      { "hpmcounter17h", MISCREG_HPMCOUNTER17H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER18H,
      { "hpmcounter18h", MISCREG_HPMCOUNTER18H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER19H,
      { "hpmcounter19h", MISCREG_HPMCOUNTER19H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER20H,
      { "hpmcounter20h", MISCREG_HPMCOUNTER20H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER21H,
      { "hpmcounter21h", MISCREG_HPMCOUNTER21H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER22H,
      { "hpmcounter22h", MISCREG_HPMCOUNTER22H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER23H,
      { "hpmcounter23h", MISCREG_HPMCOUNTER23H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER24H,
      { "hpmcounter24h", MISCREG_HPMCOUNTER24H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER25H,
      { "hpmcounter25h", MISCREG_HPMCOUNTER25H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER26H,
      { "hpmcounter26h", MISCREG_HPMCOUNTER26H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER27H,
      { "hpmcounter27h", MISCREG_HPMCOUNTER27H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER28H,
      { "hpmcounter28h", MISCREG_HPMCOUNTER28H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER29H,
      { "hpmcounter29h", MISCREG_HPMCOUNTER29H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER30H,
      { "hpmcounter30h", MISCREG_HPMCOUNTER30H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_HPMCOUNTER31H,
      { "hpmcounter31h", MISCREG_HPMCOUNTER31H, rvTypeFlags(RV32),
        isaExtsFlags() } },

    { CSR_SSTATUS,
      { "sstatus", MISCREG_STATUS, rvTypeFlags(RV64, RV32),
        isaExtsFlags('s') } },
    { CSR_SEDELEG,
      { "sedeleg", MISCREG_SEDELEG, rvTypeFlags(RV64, RV32),
        isaExtsFlags('s') } },
    { CSR_SIDELEG,
      { "sideleg", MISCREG_SIDELEG, rvTypeFlags(RV64, RV32),
        isaExtsFlags('s') } },
    { CSR_SIE,
      { "sie", MISCREG_IE, rvTypeFlags(RV64, RV32), isaExtsFlags('s') } },
    { CSR_STVEC,
      { "stvec", MISCREG_STVEC, rvTypeFlags(RV64, RV32), isaExtsFlags('s') } },
    { CSR_SCOUNTEREN,
      { "scounteren", MISCREG_SCOUNTEREN, rvTypeFlags(RV64, RV32),
        isaExtsFlags('s') } },
    { CSR_SSCRATCH,
      { "sscratch", MISCREG_SSCRATCH, rvTypeFlags(RV64, RV32),
        isaExtsFlags('s') } },
    { CSR_SEPC,
      { "sepc", MISCREG_SEPC, rvTypeFlags(RV64, RV32), isaExtsFlags('s') } },
    { CSR_SCAUSE,
      { "scause", MISCREG_SCAUSE, rvTypeFlags(RV64, RV32),
        isaExtsFlags('s') } },
    { CSR_STVAL,
      { "stval", MISCREG_STVAL, rvTypeFlags(RV64, RV32), isaExtsFlags('s') } },
    { CSR_SIP,
      { "sip", MISCREG_IP, rvTypeFlags(RV64, RV32), isaExtsFlags('s') } },
    { CSR_SATP,
      { "satp", MISCREG_SATP, rvTypeFlags(RV64, RV32), isaExtsFlags('s') } },

    { CSR_MVENDORID,
      { "mvendorid", MISCREG_VENDORID, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MARCHID,
      { "marchid", MISCREG_ARCHID, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MIMPID,
      { "mimpid", MISCREG_IMPID, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MHARTID,
      { "mhartid", MISCREG_HARTID, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MSTATUS,
      { "mstatus", MISCREG_STATUS, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MISA,
      { "misa", MISCREG_ISA, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MEDELEG,
      { "medeleg", MISCREG_MEDELEG, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MIDELEG,
      { "mideleg", MISCREG_MIDELEG, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MIE,
      { "mie", MISCREG_IE, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MTVEC,
      { "mtvec", MISCREG_MTVEC, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MCOUNTEREN,
      { "mcounteren", MISCREG_MCOUNTEREN, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MSTATUSH,
      { "mstatush", MISCREG_MSTATUSH, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_MSCRATCH,
      { "mscratch", MISCREG_MSCRATCH, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MEPC,
      { "mepc", MISCREG_MEPC, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MCAUSE,
      { "mcause", MISCREG_MCAUSE, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MTVAL,
      { "mtval", MISCREG_MTVAL, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MIP,
      { "mip", MISCREG_IP, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_PMPCFG0,
      { "pmpcfg0", MISCREG_PMPCFG0, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    // pmpcfg1 rv32 only
    { CSR_PMPCFG1,
      { "pmpcfg1", MISCREG_PMPCFG1, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_PMPCFG2,
      { "pmpcfg2", MISCREG_PMPCFG2, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    // pmpcfg3 rv32 only
    { CSR_PMPCFG3,
      { "pmpcfg3", MISCREG_PMPCFG3, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_PMPADDR00,
      { "pmpaddr0", MISCREG_PMPADDR00, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR01,
      { "pmpaddr1", MISCREG_PMPADDR01, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR02,
      { "pmpaddr2", MISCREG_PMPADDR02, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR03,
      { "pmpaddr3", MISCREG_PMPADDR03, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR04,
      { "pmpaddr4", MISCREG_PMPADDR04, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR05,
      { "pmpaddr5", MISCREG_PMPADDR05, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR06,
      { "pmpaddr6", MISCREG_PMPADDR06, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR07,
      { "pmpaddr7", MISCREG_PMPADDR07, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR08,
      { "pmpaddr8", MISCREG_PMPADDR08, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR09,
      { "pmpaddr9", MISCREG_PMPADDR09, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR10,
      { "pmpaddr10", MISCREG_PMPADDR10, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR11,
      { "pmpaddr11", MISCREG_PMPADDR11, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR12,
      { "pmpaddr12", MISCREG_PMPADDR12, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR13,
      { "pmpaddr13", MISCREG_PMPADDR13, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR14,
      { "pmpaddr14", MISCREG_PMPADDR14, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_PMPADDR15,
      { "pmpaddr15", MISCREG_PMPADDR15, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MCYCLE,
      { "mcycle", MISCREG_CYCLE, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_MINSTRET,
      { "minstret", MISCREG_INSTRET, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER03,
      { "mhpmcounter03", MISCREG_HPMCOUNTER03, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER04,
      { "mhpmcounter04", MISCREG_HPMCOUNTER04, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER05,
      { "mhpmcounter05", MISCREG_HPMCOUNTER05, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER06,
      { "mhpmcounter06", MISCREG_HPMCOUNTER06, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER07,
      { "mhpmcounter07", MISCREG_HPMCOUNTER07, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER08,
      { "mhpmcounter08", MISCREG_HPMCOUNTER08, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER09,
      { "mhpmcounter09", MISCREG_HPMCOUNTER09, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER10,
      { "mhpmcounter10", MISCREG_HPMCOUNTER10, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER11,
      { "mhpmcounter11", MISCREG_HPMCOUNTER11, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER12,
      { "mhpmcounter12", MISCREG_HPMCOUNTER12, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER13,
      { "mhpmcounter13", MISCREG_HPMCOUNTER13, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER14,
      { "mhpmcounter14", MISCREG_HPMCOUNTER14, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER15,
      { "mhpmcounter15", MISCREG_HPMCOUNTER15, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER16,
      { "mhpmcounter16", MISCREG_HPMCOUNTER16, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER17,
      { "mhpmcounter17", MISCREG_HPMCOUNTER17, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER18,
      { "mhpmcounter18", MISCREG_HPMCOUNTER18, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER19,
      { "mhpmcounter19", MISCREG_HPMCOUNTER19, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER20,
      { "mhpmcounter20", MISCREG_HPMCOUNTER20, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER21,
      { "mhpmcounter21", MISCREG_HPMCOUNTER21, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER22,
      { "mhpmcounter22", MISCREG_HPMCOUNTER22, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER23,
      { "mhpmcounter23", MISCREG_HPMCOUNTER23, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER24,
      { "mhpmcounter24", MISCREG_HPMCOUNTER24, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER25,
      { "mhpmcounter25", MISCREG_HPMCOUNTER25, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER26,
      { "mhpmcounter26", MISCREG_HPMCOUNTER26, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER27,
      { "mhpmcounter27", MISCREG_HPMCOUNTER27, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER28,
      { "mhpmcounter28", MISCREG_HPMCOUNTER28, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER29,
      { "mhpmcounter29", MISCREG_HPMCOUNTER29, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER30,
      { "mhpmcounter30", MISCREG_HPMCOUNTER30, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER31,
      { "mhpmcounter31", MISCREG_HPMCOUNTER31, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },

    { CSR_MCYCLEH,
      { "mcycleh", MISCREG_CYCLEH, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_MINSTRETH,
      { "minstreth", MISCREG_INSTRETH, rvTypeFlags(RV32), isaExtsFlags() } },
    { CSR_MHPMCOUNTER03H,
      { "mhpmcounter03h", MISCREG_HPMCOUNTER03H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER04H,
      { "mhpmcounter04h", MISCREG_HPMCOUNTER04H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER05H,
      { "mhpmcounter05h", MISCREG_HPMCOUNTER05H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER06H,
      { "mhpmcounter06h", MISCREG_HPMCOUNTER06H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER07H,
      { "mhpmcounter07h", MISCREG_HPMCOUNTER07H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER08H,
      { "mhpmcounter08h", MISCREG_HPMCOUNTER08H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER09H,
      { "mhpmcounter09h", MISCREG_HPMCOUNTER09H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER10H,
      { "mhpmcounter10h", MISCREG_HPMCOUNTER10H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER11H,
      { "mhpmcounter11h", MISCREG_HPMCOUNTER11H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER12H,
      { "mhpmcounter12h", MISCREG_HPMCOUNTER12H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER13H,
      { "mhpmcounter13h", MISCREG_HPMCOUNTER13H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER14H,
      { "mhpmcounter14h", MISCREG_HPMCOUNTER14H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER15H,
      { "mhpmcounter15h", MISCREG_HPMCOUNTER15H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER16H,
      { "mhpmcounter16h", MISCREG_HPMCOUNTER16H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER17H,
      { "mhpmcounter17h", MISCREG_HPMCOUNTER17H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER18H,
      { "mhpmcounter18h", MISCREG_HPMCOUNTER18H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER19H,
      { "mhpmcounter19h", MISCREG_HPMCOUNTER19H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER20H,
      { "mhpmcounter20h", MISCREG_HPMCOUNTER20H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER21H,
      { "mhpmcounter21h", MISCREG_HPMCOUNTER21H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER22H,
      { "mhpmcounter22h", MISCREG_HPMCOUNTER22H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER23H,
      { "mhpmcounter23h", MISCREG_HPMCOUNTER23H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER24H,
      { "mhpmcounter24h", MISCREG_HPMCOUNTER24H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER25H,
      { "mhpmcounter25h", MISCREG_HPMCOUNTER25H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER26H,
      { "mhpmcounter26h", MISCREG_HPMCOUNTER26H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER27H,
      { "mhpmcounter27h", MISCREG_HPMCOUNTER27H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER28H,
      { "mhpmcounter28h", MISCREG_HPMCOUNTER28H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER29H,
      { "mhpmcounter29h", MISCREG_HPMCOUNTER29H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER30H,
      { "mhpmcounter30h", MISCREG_HPMCOUNTER30H, rvTypeFlags(RV32),
        isaExtsFlags() } },
    { CSR_MHPMCOUNTER31H,
      { "mhpmcounter31h", MISCREG_HPMCOUNTER31H, rvTypeFlags(RV32),
        isaExtsFlags() } },

    { CSR_MHPMEVENT03,
      { "mhpmevent03", MISCREG_HPMEVENT03, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT04,
      { "mhpmevent04", MISCREG_HPMEVENT04, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT05,
      { "mhpmevent05", MISCREG_HPMEVENT05, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT06,
      { "mhpmevent06", MISCREG_HPMEVENT06, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT07,
      { "mhpmevent07", MISCREG_HPMEVENT07, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT08,
      { "mhpmevent08", MISCREG_HPMEVENT08, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT09,
      { "mhpmevent09", MISCREG_HPMEVENT09, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT10,
      { "mhpmevent10", MISCREG_HPMEVENT10, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT11,
      { "mhpmevent11", MISCREG_HPMEVENT11, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT12,
      { "mhpmevent12", MISCREG_HPMEVENT12, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT13,
      { "mhpmevent13", MISCREG_HPMEVENT13, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT14,
      { "mhpmevent14", MISCREG_HPMEVENT14, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT15,
      { "mhpmevent15", MISCREG_HPMEVENT15, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT16,
      { "mhpmevent16", MISCREG_HPMEVENT16, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT17,
      { "mhpmevent17", MISCREG_HPMEVENT17, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT18,
      { "mhpmevent18", MISCREG_HPMEVENT18, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT19,
      { "mhpmevent19", MISCREG_HPMEVENT19, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT20,
      { "mhpmevent20", MISCREG_HPMEVENT20, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT21,
      { "mhpmevent21", MISCREG_HPMEVENT21, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT22,
      { "mhpmevent22", MISCREG_HPMEVENT22, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT23,
      { "mhpmevent23", MISCREG_HPMEVENT23, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT24,
      { "mhpmevent24", MISCREG_HPMEVENT24, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT25,
      { "mhpmevent25", MISCREG_HPMEVENT25, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT26,
      { "mhpmevent26", MISCREG_HPMEVENT26, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT27,
      { "mhpmevent27", MISCREG_HPMEVENT27, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT28,
      { "mhpmevent28", MISCREG_HPMEVENT28, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT29,
      { "mhpmevent29", MISCREG_HPMEVENT29, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT30,
      { "mhpmevent30", MISCREG_HPMEVENT30, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_MHPMEVENT31,
      { "mhpmevent31", MISCREG_HPMEVENT31, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },

    { CSR_TSELECT,
      { "tselect", MISCREG_TSELECT, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },
    { CSR_TDATA1,
      { "tdata1", MISCREG_TDATA1, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_TDATA2,
      { "tdata2", MISCREG_TDATA2, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_TDATA3,
      { "tdata3", MISCREG_TDATA3, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_DCSR,
      { "dcsr", MISCREG_DCSR, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_DPC,
      { "dpc", MISCREG_DPC, rvTypeFlags(RV64, RV32), isaExtsFlags() } },
    { CSR_DSCRATCH,
      { "dscratch", MISCREG_DSCRATCH, rvTypeFlags(RV64, RV32),
        isaExtsFlags() } },

    { CSR_VSTART,
      { "vstart", MISCREG_VSTART, rvTypeFlags(RV64, RV32),
        isaExtsFlags('v') } },
    { CSR_VXSAT,
      { "vxsat", MISCREG_VXSAT, rvTypeFlags(RV64, RV32), isaExtsFlags('v') } },
    { CSR_VXRM,
      { "vxrm", MISCREG_VXRM, rvTypeFlags(RV64, RV32), isaExtsFlags('v') } },
    { CSR_VCSR,
      { "vcsr", MISCREG_VCSR, rvTypeFlags(RV64, RV32), isaExtsFlags('v') } },
    { CSR_VL,
      { "vl", MISCREG_VL, rvTypeFlags(RV64, RV32), isaExtsFlags('v') } },
    { CSR_VTYPE,
      { "vtype", MISCREG_VTYPE, rvTypeFlags(RV64, RV32), isaExtsFlags('v') } },
    { CSR_VLENB,
      { "VLENB", MISCREG_VLENB, rvTypeFlags(RV64, RV32), isaExtsFlags('v') } }
};

/**
 * These fields are specified in the RISC-V Instruction Set Manual, Volume II,
 * v1.10, accessible at www.riscv.org. in Figure 3.7. The main register that
 * uses these fields is the MSTATUS register, which is shadowed by two others
 * accessible at lower privilege levels (SSTATUS and USTATUS) that can't see
 * the fields for higher privileges.
 */
BitUnion64(STATUS)
    Bitfield<63> rv64_sd;
    Bitfield<35, 34> sxl;
    Bitfield<33, 32> uxl;
    Bitfield<31> rv32_sd;
    Bitfield<22> tsr;
    Bitfield<21> tw;
    Bitfield<20> tvm;
    Bitfield<19> mxr;
    Bitfield<18> sum;
    Bitfield<17> mprv;
    Bitfield<16, 15> xs;
    Bitfield<14, 13> fs;
    Bitfield<12, 11> mpp;
    Bitfield<10, 9> vs;
    Bitfield<8> spp;
    Bitfield<7> mpie;
    Bitfield<5> spie;
    Bitfield<4> upie;
    Bitfield<3> mie;
    Bitfield<1> sie;
    Bitfield<0> uie;
EndBitUnion(STATUS)

/**
 * These fields are specified in the RISC-V Instruction Set Manual, Volume II,
 * v1.10, v1.11 and v1.12 in Figure 3.1, accessible at www.riscv.org. The
 * register is used to control instruction extensions.
 */
BitUnion64(MISA)
    Bitfield<63, 62> rv64_mxl;
    Bitfield<31, 30> rv32_mxl;
    Bitfield<23> rvx;
    Bitfield<21> rvv;
    Bitfield<20> rvu;
    Bitfield<19> rvt;
    Bitfield<18> rvs;
    Bitfield<16> rvq;
    Bitfield<15> rvp;
    Bitfield<13> rvn;
    Bitfield<12> rvm;
    Bitfield<11> rvl;
    Bitfield<10> rvk;
    Bitfield<9> rvj;
    Bitfield<8> rvi;
    Bitfield<7> rvh;
    Bitfield<6> rvg;
    Bitfield<5> rvf;
    Bitfield<4> rve;
    Bitfield<3> rvd;
    Bitfield<2> rvc;
    Bitfield<1> rvb;
    Bitfield<0> rva;
EndBitUnion(MISA)

/**
 * These fields are specified in the RISC-V Instruction Set Manual, Volume II,
 * v1.10 in Figures 3.11 and 3.12, accessible at www.riscv.org. Both the MIP
 * and MIE registers have the same fields, so accesses to either should use
 * this bit union.
 */
BitUnion64(INTERRUPT)
    Bitfield<63, 16> local;
    Bitfield<11> mei;
    Bitfield<9> sei;
    Bitfield<8> uei;
    Bitfield<7> mti;
    Bitfield<5> sti;
    Bitfield<4> uti;
    Bitfield<3> msi;
    Bitfield<1> ssi;
    Bitfield<0> usi;
EndBitUnion(INTERRUPT)

const off_t MXL_OFFSETS[enums::Num_RiscvType] = {
    [RV32] = (sizeof(uint32_t) * 8 - 2),
    [RV64] = (sizeof(uint64_t) * 8 - 2),
};
const off_t MBE_OFFSET[enums::Num_RiscvType] = {
    [RV32] = 5,
    [RV64] = 37,
};
const off_t SBE_OFFSET[enums::Num_RiscvType] = {
    [RV32] = 4,
    [RV64] = 36,
};
const off_t SXL_OFFSET = 34;
const off_t UXL_OFFSET = 32;
const off_t FS_OFFSET = 13;
const off_t VS_OFFSET = 9;
const off_t FRM_OFFSET = 5;

const RegVal ISA_MXL_MASKS[enums::Num_RiscvType] = {
    [RV32] = 3ULL << MXL_OFFSETS[RV32],
    [RV64] = 3ULL << MXL_OFFSETS[RV64],
};
const RegVal ISA_EXT_MASK = mask(26);
const RegVal ISA_EXT_C_MASK = 1UL << ('c' - 'a');
const RegVal MISA_MASKS[enums::Num_RiscvType] = {
    [RV32] = ISA_MXL_MASKS[RV32] | ISA_EXT_MASK,
    [RV64] = ISA_MXL_MASKS[RV64] | ISA_EXT_MASK,
};

const RegVal STATUS_SD_MASKS[enums::Num_RiscvType] = {
    [RV32] = 1ULL << ((sizeof(uint32_t) * 8) - 1),
    [RV64] = 1ULL << ((sizeof(uint64_t) * 8) - 1),
};
const RegVal STATUS_MBE_MASK[enums::Num_RiscvType] = {
    [RV32] = 1ULL << MBE_OFFSET[RV32],
    [RV64] = 1ULL << MBE_OFFSET[RV64],
};
const RegVal STATUS_SBE_MASK[enums::Num_RiscvType] = {
    [RV32] = 1ULL << SBE_OFFSET[RV32],
    [RV64] = 1ULL << SBE_OFFSET[RV64],
};
const RegVal STATUS_SXL_MASK = 3ULL << SXL_OFFSET;
const RegVal STATUS_UXL_MASK = 3ULL << UXL_OFFSET;
const RegVal STATUS_TSR_MASK = 1ULL << 22;
const RegVal STATUS_TW_MASK = 1ULL << 21;
const RegVal STATUS_TVM_MASK = 1ULL << 20;
const RegVal STATUS_MXR_MASK = 1ULL << 19;
const RegVal STATUS_SUM_MASK = 1ULL << 18;
const RegVal STATUS_MPRV_MASK = 1ULL << 17;
const RegVal STATUS_XS_MASK = 3ULL << 15;
const RegVal STATUS_FS_MASK = 3ULL << FS_OFFSET;
const RegVal STATUS_MPP_MASK = 3ULL << 11;
const RegVal STATUS_VS_MASK = 3ULL << VS_OFFSET;
const RegVal STATUS_SPP_MASK = 1ULL << 8;
const RegVal STATUS_MPIE_MASK = 1ULL << 7;
const RegVal STATUS_SPIE_MASK = 1ULL << 5;
const RegVal STATUS_UPIE_MASK = 1ULL << 4;
const RegVal STATUS_MIE_MASK = 1ULL << 3;
const RegVal STATUS_SIE_MASK = 1ULL << 1;
const RegVal STATUS_UIE_MASK = 1ULL << 0;
const RegVal
MSTATUS_MASKS[enums::Num_RiscvType][enums::Num_PrivilegeModeSet] = {
    [RV32] = {
        [enums::M] = STATUS_SD_MASKS[RV32] |
                     STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                     STATUS_MPIE_MASK | STATUS_MIE_MASK,
        [enums::MU] = STATUS_SD_MASKS[RV32] | STATUS_TW_MASK  |
                      STATUS_MPRV_MASK |
                      STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                      STATUS_MPP_MASK | STATUS_MPIE_MASK | STATUS_MIE_MASK,
        [enums::MNU] = STATUS_SD_MASKS[RV32] | STATUS_TW_MASK  |
                       STATUS_MPRV_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_MPP_MASK |
                       STATUS_MPIE_MASK | STATUS_UPIE_MASK |
                       STATUS_MIE_MASK | STATUS_UIE_MASK,
        [enums::MSU] = STATUS_SD_MASKS[RV32] | STATUS_TSR_MASK |
                       STATUS_TW_MASK | STATUS_TVM_MASK | STATUS_MXR_MASK |
                       STATUS_SUM_MASK | STATUS_MPRV_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_MPP_MASK | STATUS_SPP_MASK |
                       STATUS_MPIE_MASK | STATUS_SPIE_MASK |
                       STATUS_MIE_MASK | STATUS_SIE_MASK,
        [enums::MNSU] = STATUS_SD_MASKS[RV32] | STATUS_TSR_MASK |
                        STATUS_TW_MASK | STATUS_TVM_MASK | STATUS_MXR_MASK |
                        STATUS_SUM_MASK | STATUS_MPRV_MASK |
                        STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                        STATUS_MPP_MASK | STATUS_SPP_MASK |
                        STATUS_MPIE_MASK | STATUS_SPIE_MASK |
                        STATUS_UPIE_MASK | STATUS_MIE_MASK | STATUS_SIE_MASK |
                        STATUS_UIE_MASK,
    },
    [RV64] = {
        [enums::M] = STATUS_SD_MASKS[RV64] | STATUS_MBE_MASK[RV64] |
                     STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                     STATUS_MPIE_MASK | STATUS_MIE_MASK,
        [enums::MU] = STATUS_SD_MASKS[RV64] | STATUS_MBE_MASK[RV64] |
                      STATUS_UXL_MASK | STATUS_TW_MASK |  STATUS_MPRV_MASK |
                      STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                      STATUS_MPP_MASK | STATUS_MPIE_MASK | STATUS_MIE_MASK,
        [enums::MNU] = STATUS_SD_MASKS[RV64] | STATUS_MBE_MASK[RV64] |
                       STATUS_UXL_MASK | STATUS_TW_MASK | STATUS_MPRV_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_MPP_MASK |
                       STATUS_MPIE_MASK | STATUS_UPIE_MASK |
                       STATUS_MIE_MASK | STATUS_UIE_MASK,
        [enums::MSU] = STATUS_SD_MASKS[RV64] |
                       STATUS_MBE_MASK[RV64] | STATUS_SBE_MASK[RV64] |
                       STATUS_SXL_MASK | STATUS_UXL_MASK |
                       STATUS_TSR_MASK | STATUS_TW_MASK | STATUS_TVM_MASK |
                       STATUS_MXR_MASK | STATUS_SUM_MASK | STATUS_MPRV_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_MPP_MASK | STATUS_SPP_MASK |
                       STATUS_MPIE_MASK | STATUS_SPIE_MASK |
                       STATUS_MIE_MASK | STATUS_SIE_MASK,
        [enums::MNSU] = STATUS_SD_MASKS[RV64] |
                       STATUS_MBE_MASK[RV64] | STATUS_SBE_MASK[RV64] |
                       STATUS_SXL_MASK | STATUS_UXL_MASK |
                       STATUS_TSR_MASK | STATUS_TW_MASK | STATUS_TVM_MASK |
                       STATUS_MXR_MASK | STATUS_SUM_MASK | STATUS_MPRV_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_MPP_MASK | STATUS_SPP_MASK |
                       STATUS_MPIE_MASK | STATUS_SPIE_MASK | STATUS_UPIE_MASK |
                       STATUS_MIE_MASK | STATUS_SIE_MASK | STATUS_UIE_MASK,
    },
};
// rv32 only
const RegVal MSTATUSH_MASKS[enums::Num_PrivilegeModeSet] = {
    [enums::M] = STATUS_MBE_MASK[RV32],
    [enums::MU] = STATUS_MBE_MASK[RV32],
    [enums::MNU] = STATUS_MBE_MASK[RV32],
    [enums::MSU] = STATUS_MBE_MASK[RV32] | STATUS_SBE_MASK[RV32],
    [enums::MNSU] = STATUS_MBE_MASK[RV32] | STATUS_SBE_MASK[RV32],
};
const RegVal
SSTATUS_MASKS[enums::Num_RiscvType][enums::Num_PrivilegeModeSet] = {
    [RV32] = {
        [enums::M] = 0ULL,
        [enums::MU] = 0ULL,
        [enums::MNU] = 0ULL,
        [enums::MSU] = STATUS_SD_MASKS[RV32] | STATUS_MXR_MASK |
                       STATUS_SUM_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_SPP_MASK | STATUS_SPIE_MASK | STATUS_SIE_MASK,
        [enums::MNSU] = STATUS_SD_MASKS[RV32] | STATUS_MXR_MASK |
                        STATUS_SUM_MASK |
                        STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                        STATUS_SPP_MASK | STATUS_SPIE_MASK | STATUS_UPIE_MASK |
                        STATUS_SIE_MASK | STATUS_UIE_MASK,
    },
    [RV64] = {
        [enums::M] = 0ULL,
        [enums::MU] = 0ULL,
        [enums::MNU] = 0ULL,
        [enums::MSU] = STATUS_SD_MASKS[RV64] | STATUS_UXL_MASK |
                       STATUS_MXR_MASK | STATUS_SUM_MASK |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_SPP_MASK | STATUS_SPIE_MASK | STATUS_SIE_MASK,
        [enums::MNSU] = STATUS_SD_MASKS[RV64] | STATUS_UXL_MASK |
                        STATUS_MXR_MASK | STATUS_SUM_MASK |
                        STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                        STATUS_SPP_MASK | STATUS_SPIE_MASK |
                        STATUS_UPIE_MASK | STATUS_SIE_MASK | STATUS_UIE_MASK,
    },
};
const RegVal
USTATUS_MASKS[enums::Num_RiscvType][enums::Num_PrivilegeModeSet] = {
    [RV32] = {
        [enums::M] = 0ULL,
        [enums::MU] = 0ULL,
        [enums::MNU] = STATUS_SD_MASKS[RV32] |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_UPIE_MASK | STATUS_UIE_MASK,
        [enums::MSU] = 0ULL,
        [enums::MNSU] = STATUS_SD_MASKS[RV32] | STATUS_MXR_MASK |
                        STATUS_SUM_MASK |
                        STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                        STATUS_UPIE_MASK | STATUS_UIE_MASK,
    },
    [RV64] = {
        [enums::M] = 0ULL,
        [enums::MU] = 0ULL,
        [enums::MNU] = STATUS_SD_MASKS[RV64] |
                       STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                       STATUS_UPIE_MASK | STATUS_UIE_MASK,
        [enums::MSU] = 0ULL,
        [enums::MNSU] = STATUS_SD_MASKS[RV64] | STATUS_MXR_MASK |
                        STATUS_SUM_MASK |
                        STATUS_XS_MASK | STATUS_FS_MASK | STATUS_VS_MASK |
                        STATUS_UPIE_MASK | STATUS_UIE_MASK,
    },
};

const RegVal LOCAL_MASK = mask(63, 16);
const RegVal MEI_MASK = 1ULL << 11;
const RegVal SEI_MASK = 1ULL << 9;
const RegVal UEI_MASK = 1ULL << 8;
const RegVal MTI_MASK = 1ULL << 7;
const RegVal STI_MASK = 1ULL << 5;
const RegVal UTI_MASK = 1ULL << 4;
const RegVal MSI_MASK = 1ULL << 3;
const RegVal SSI_MASK = 1ULL << 1;
const RegVal USI_MASK = 1ULL << 0;
const RegVal MI_MASK[enums::Num_PrivilegeModeSet] = {
    [enums::M] = LOCAL_MASK | MEI_MASK | MTI_MASK | MSI_MASK,
    [enums::MU] = LOCAL_MASK | MEI_MASK | MTI_MASK | MSI_MASK,
    [enums::MNU] = LOCAL_MASK | MEI_MASK | UEI_MASK | MTI_MASK | UTI_MASK |
                   MSI_MASK | USI_MASK,
    [enums::MSU] = LOCAL_MASK | MEI_MASK | SEI_MASK | MTI_MASK | STI_MASK |
                   MSI_MASK | SSI_MASK,
    [enums::MNSU] = LOCAL_MASK | MEI_MASK | SEI_MASK | UEI_MASK | MTI_MASK |
                    STI_MASK | UTI_MASK | MSI_MASK | SSI_MASK | USI_MASK,
};
const RegVal SI_MASK[enums::Num_PrivilegeModeSet] = {
    [enums::M] = 0ULL,
    [enums::MU] = 0ULL,
    [enums::MNU] = UEI_MASK | UTI_MASK | USI_MASK,
    [enums::MSU] = SEI_MASK | STI_MASK | SSI_MASK,
    [enums::MNSU] =
        SEI_MASK | UEI_MASK | STI_MASK | UTI_MASK | SSI_MASK | USI_MASK,
};
const RegVal UI_MASK[enums::Num_PrivilegeModeSet] = {
    [enums::M] = 0ULL,
    [enums::MU] = 0ULL,
    [enums::MNU] = UEI_MASK | UTI_MASK | USI_MASK,
    [enums::MSU] = 0ULL,
    [enums::MNSU] = UEI_MASK | UTI_MASK | USI_MASK,
};
const RegVal FFLAGS_MASK = (1 << FRM_OFFSET) - 1;
const RegVal FRM_MASK = 0x7;

const RegVal CAUSE_INTERRUPT_MASKS[enums::Num_RiscvType] = {
    [RV32] = (1ULL << 31),
    [RV64] = (1ULL << 63),
};

const std::unordered_map<int, RegVal>
CSRMasks[enums::Num_RiscvType][enums::Num_PrivilegeModeSet] = {
    [RV32] = {
        [enums::M] = {
            {CSR_USTATUS, USTATUS_MASKS[RV32][enums::M]},
            {CSR_UIE, UI_MASK[enums::M]},
            {CSR_UIP, UI_MASK[enums::M]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV32][enums::M]},
            {CSR_SIE, SI_MASK[enums::M]},
            {CSR_SIP, SI_MASK[enums::M]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV32][enums::M]},
            {CSR_MISA, MISA_MASKS[RV32]},
            {CSR_MIE, MI_MASK[enums::M]},
            {CSR_MSTATUSH, MSTATUSH_MASKS[enums::M]},
            {CSR_MIP, MI_MASK[enums::M]},
        },
        [enums::MU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV32][enums::MU]},
            {CSR_UIE, UI_MASK[enums::MU]},
            {CSR_UIP, UI_MASK[enums::MU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV32][enums::MU]},
            {CSR_SIE, SI_MASK[enums::MU]},
            {CSR_SIP, SI_MASK[enums::MU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV32][enums::MU]},
            {CSR_MISA, MISA_MASKS[RV32]},
            {CSR_MIE, MI_MASK[enums::MU]},
            {CSR_MSTATUSH, MSTATUSH_MASKS[enums::MU]},
            {CSR_MIP, MI_MASK[enums::MU]},
        },
        [enums::MNU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV32][enums::MNU]},
            {CSR_UIE, UI_MASK[enums::MNU]},
            {CSR_UIP, UI_MASK[enums::MNU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV32][enums::MNU]},
            {CSR_SIE, SI_MASK[enums::MNU]},
            {CSR_SIP, SI_MASK[enums::MNU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV32][enums::MNU]},
            {CSR_MISA, MISA_MASKS[RV32]},
            {CSR_MIE, MI_MASK[enums::MNU]},
            {CSR_MSTATUSH, MSTATUSH_MASKS[enums::MNU]},
            {CSR_MIP, MI_MASK[enums::MNU]},
        },
        [enums::MSU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV32][enums::MSU]},
            {CSR_UIE, UI_MASK[enums::MSU]},
            {CSR_UIP, UI_MASK[enums::MSU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV32][enums::MSU]},
            {CSR_SIE, SI_MASK[enums::MSU]},
            {CSR_SIP, SI_MASK[enums::MSU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV32][enums::MSU]},
            {CSR_MISA, MISA_MASKS[RV32]},
            {CSR_MIE, MI_MASK[enums::MSU]},
            {CSR_MSTATUSH, MSTATUSH_MASKS[enums::MSU]},
            {CSR_MIP, MI_MASK[enums::MSU]},
        },
        [enums::MNSU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV32][enums::MNSU]},
            {CSR_UIE, UI_MASK[enums::MNSU]},
            {CSR_UIP, UI_MASK[enums::MNSU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV32][enums::MNSU]},
            {CSR_SIE, SI_MASK[enums::MNSU]},
            {CSR_SIP, SI_MASK[enums::MNSU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV32][enums::MNSU]},
            {CSR_MISA, MISA_MASKS[RV32]},
            {CSR_MIE, MI_MASK[enums::MNSU]},
            {CSR_MSTATUSH, MSTATUSH_MASKS[enums::MNSU]},
            {CSR_MIP, MI_MASK[enums::MNSU]},
        },
    },
    [RV64] = {
        [enums::M] = {
            {CSR_USTATUS, USTATUS_MASKS[RV64][enums::M]},
            {CSR_UIE, UI_MASK[enums::M]},
            {CSR_UIP, UI_MASK[enums::M]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV64][enums::M]},
            {CSR_SIE, SI_MASK[enums::M]},
            {CSR_SIP, SI_MASK[enums::M]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV64][enums::M]},
            {CSR_MISA, MISA_MASKS[RV64]},
            {CSR_MIE, MI_MASK[enums::M]},
            {CSR_MIP, MI_MASK[enums::M]},
        },
        [enums::MU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV64][enums::MU]},
            {CSR_UIE, UI_MASK[enums::MU]},
            {CSR_UIP, UI_MASK[enums::MU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV64][enums::MU]},
            {CSR_SIE, SI_MASK[enums::MU]},
            {CSR_SIP, SI_MASK[enums::MU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV64][enums::MU]},
            {CSR_MISA, MISA_MASKS[RV64]},
            {CSR_MIE, MI_MASK[enums::MU]},
            {CSR_MIP, MI_MASK[enums::MU]},
        },
        [enums::MNU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV64][enums::MNU]},
            {CSR_UIE, UI_MASK[enums::MNU]},
            {CSR_UIP, UI_MASK[enums::MNU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV64][enums::MNU]},
            {CSR_SIE, SI_MASK[enums::MNU]},
            {CSR_SIP, SI_MASK[enums::MNU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV64][enums::MNU]},
            {CSR_MISA, MISA_MASKS[RV64]},
            {CSR_MIE, MI_MASK[enums::MNU]},
            {CSR_MIP, MI_MASK[enums::MNU]},
        },
        [enums::MSU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV64][enums::MSU]},
            {CSR_UIE, UI_MASK[enums::MSU]},
            {CSR_UIP, UI_MASK[enums::MSU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV64][enums::MSU]},
            {CSR_SIE, SI_MASK[enums::MSU]},
            {CSR_SIP, SI_MASK[enums::MSU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV64][enums::MSU]},
            {CSR_MISA, MISA_MASKS[RV64]},
            {CSR_MIE, MI_MASK[enums::MSU]},
            {CSR_MIP, MI_MASK[enums::MSU]},
        },
        [enums::MNSU] = {
            {CSR_USTATUS, USTATUS_MASKS[RV64][enums::MNSU]},
            {CSR_UIE, UI_MASK[enums::MNSU]},
            {CSR_UIP, UI_MASK[enums::MNSU]},
            {CSR_FFLAGS, FFLAGS_MASK},
            {CSR_FRM, FRM_MASK},
            {CSR_FCSR, FFLAGS_MASK | (FRM_MASK << FRM_OFFSET)},
            {CSR_SSTATUS, SSTATUS_MASKS[RV64][enums::MNSU]},
            {CSR_SIE, SI_MASK[enums::MNSU]},
            {CSR_SIP, SI_MASK[enums::MNSU]},
            {CSR_MSTATUS, MSTATUS_MASKS[RV64][enums::MNSU]},
            {CSR_MISA, MISA_MASKS[RV64]},
            {CSR_MIE, MI_MASK[enums::MNSU]},
            {CSR_MIP, MI_MASK[enums::MNSU]},
        },
    },
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_MISC_HH__
