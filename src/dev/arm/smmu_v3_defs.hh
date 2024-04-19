/*
 * Copyright (c) 2013, 2018-2019, 2024 Arm Limited
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

#ifndef __DEV_ARM_SMMU_V3_DEFS_HH__
#define __DEV_ARM_SMMU_V3_DEFS_HH__

#include <stdint.h>

#include "base/bitunion.hh"

namespace gem5
{

enum
{
    SMMU_SECURE_SZ = 0x184, // Secure regs are within page0
    SMMU_PAGE_ZERO_SZ = 0x10000,
    SMMU_PAGE_ONE_SZ = 0x10000,
    SMMU_REG_SIZE = SMMU_PAGE_ONE_SZ + SMMU_PAGE_ZERO_SZ
};

enum
{
    STE_CONFIG_ABORT        = 0x0,
    STE_CONFIG_BYPASS       = 0x4,
    STE_CONFIG_STAGE1_ONLY  = 0x5,
    STE_CONFIG_STAGE2_ONLY  = 0x6,
    STE_CONFIG_STAGE1_AND_2 = 0x7,
};

enum
{
    STAGE1_CFG_1L     = 0x0,
    STAGE1_CFG_2L_4K  = 0x1,
    STAGE1_CFG_2L_64K = 0x2,
};

enum
{
    ST_CFG_SPLIT_SHIFT = 6,
    ST_CD_ADDR_SHIFT   = 6,
    CD_TTB_SHIFT       = 4,
    STE_S2TTB_SHIFT    = 4,
};

enum
{
    TRANS_GRANULE_4K      = 0x0,
    TRANS_GRANULE_64K     = 0x1,
    TRANS_GRANULE_16K     = 0x2,
    TRANS_GRANULE_INVALID = 0x3,
};

enum
{
    ST_BASE_ADDR_MASK  = 0x0000ffffffffffe0ULL,
    ST_CFG_SIZE_MASK   = 0x000000000000003fULL,
    ST_CFG_SPLIT_MASK  = 0x00000000000007c0ULL,
    ST_CFG_FMT_MASK    = 0x0000000000030000ULL,
    ST_CFG_FMT_LINEAR  = 0x0000000000000000ULL,
    ST_CFG_FMT_2LEVEL  = 0x0000000000010000ULL,
    ST_L2_SPAN_MASK    = 0x000000000000001fULL,
    ST_L2_ADDR_MASK    = 0x0000ffffffffffe0ULL,

    VMT_BASE_ADDR_MASK = 0x0000ffffffffffe0ULL,
    VMT_BASE_SIZE_MASK = 0x000000000000001fULL,

    Q_BASE_ADDR_MASK   = 0x0000ffffffffffe0ULL,
    Q_BASE_SIZE_MASK   = 0x000000000000001fULL,

    E_BASE_ADDR_MASK   = 0x0000fffffffffffcULL,
};

BitUnion32(IDR0)
    Bitfield<0> s2p;
    Bitfield<1> s1p;
    Bitfield<3, 2> ttf;
    Bitfield<4> cohacc;
    Bitfield<5> btm;
    Bitfield<7, 6> httu;
    Bitfield<8> dormhint;
    Bitfield<9> hyp;
    Bitfield<10> ats;
    Bitfield<11> ns1ats;
    Bitfield<12> asid16;
    Bitfield<13> msi;
    Bitfield<14> sev;
    Bitfield<15> atos;
    Bitfield<16> pri;
    Bitfield<17> vmw;
    Bitfield<18> vmid16;
    Bitfield<19> cd2l;
    Bitfield<20> vatos;
    Bitfield<22, 21> ttEndian;
    Bitfield<23> atsRecErr;
    Bitfield<25, 24> stallModel;
    Bitfield<26> termModel;
    Bitfield<28, 27> stLevel;
EndBitUnion(IDR0)

BitUnion32(IRQCtrl)
    Bitfield<0> gerrorIrqEn;
    Bitfield<1> priqIrqEn;
    Bitfield<2> eventqIrqEn;
EndBitUnion(IRQCtrl)

union SMMURegs
{
    uint8_t data[SMMU_REG_SIZE];

    struct
    {
        uint32_t idr0;        // 0x0000
        uint32_t idr1;        // 0x0004
        uint32_t idr2;        // 0x0008
        uint32_t idr3;        // 0x000c
        uint32_t idr4;        // 0x0010
        uint32_t idr5;        // 0x0014
        uint32_t iidr;        // 0x0018
        uint32_t aidr;        // 0x001c
        uint32_t cr0;         // 0x0020
        uint32_t cr0ack;      // 0x0024
        uint32_t cr1;         // 0x0028
        uint32_t cr2;         // 0x002c
        uint32_t _pad1;       // 0x0030
        uint32_t _pad2;       // 0x0034
        uint32_t _pad3;       // 0x0038
        uint32_t _pad4;       // 0x003c
        uint32_t statusr;     // 0x0040
        uint32_t gbpa;        // 0x0044
        uint32_t agbpa;       // 0x0048
        uint32_t _pad5;       // 0x004c
        uint32_t irq_ctrl;    // 0x0050
        uint32_t irq_ctrlack; // 0x0054
        uint32_t _pad6;       // 0x0058
        uint32_t _pad7;       // 0x005c

        uint32_t gerror;          // 0x0060
        uint32_t gerrorn;         // 0x0064
        uint64_t gerror_irq_cfg0; // 0x0068, 64 bit
        uint32_t gerror_irq_cfg1; // 0x0070
        uint32_t gerror_irq_cfg2; // 0x0074
        uint32_t _pad_1;          // 0x0078
        uint32_t _pad_2;          // 0x007c

        uint64_t strtab_base;     // 0x0080, 64 bit
        uint32_t strtab_base_cfg; // 0x0088

        uint64_t cmdq_base;       // 0x0090, 64 bit
        uint32_t cmdq_prod;       // 0x0098
        uint32_t cmdq_cons;       // 0x009c
        uint64_t eventq_base;     // 0x00a0, 64 bit
        uint32_t _pad8;           // 0x00a8
        uint32_t _pad9;           // 0x00ac
        uint64_t eventq_irq_cfg0; // 0x00b0, 64 bit
        uint32_t eventq_irq_cfg1; // 0x00b8
        uint32_t eventq_irq_cfg2; // 0x00bc
        uint64_t priq_base;       // 0x00c0, 64 bit
        uint32_t _pad10;          // 0x00c8
        uint32_t _pad11;          // 0x00cc

        uint64_t priq_irq_cfg0;   // 0x00d0
        uint32_t priq_irq_cfg1;   // 0x00d8
        uint32_t priq_irq_cfg2;   // 0x00dc

        uint32_t _pad12[8];       // 0x00e0 - 0x0100
        uint32_t gatos_ctrl;      // 0x0100
        uint32_t _pad13;          // 0x0104
        uint64_t gatos_sid;       // 0x0108
        uint64_t gatos_addr;      // 0x0110
        uint64_t gatos_par;       // 0x0118
        uint32_t _pad14[24];      // 0x0120
        uint32_t vatos_sel;       // 0x0180

        uint32_t _pad15[8095];    // 0x184 - 0x7ffc

        uint8_t  _secure_regs[SMMU_SECURE_SZ]; // 0x8000 - 0x8180

        uint32_t _pad16[8095];    // 0x8184 - 0x10000

        // Page 1
        uint32_t _pad17[42];      // 0x10000
        uint32_t eventq_prod;     // 0x100A8
        uint32_t eventq_cons;     // 0x100AC

        uint32_t _pad18[6];       // 0x100B0
        uint32_t priq_prod;       // 0x100C8
        uint32_t priq_cons;       // 0x100CC
    };
};

struct StreamTableEntry
{
    BitUnion64(DWORD0)
        Bitfield<0>       valid;
        Bitfield<3, 1>    config;
        Bitfield<5, 4>    s1fmt;
        Bitfield<51, 6>   s1ctxptr;
        Bitfield<63, 59>  s1cdmax;
    EndBitUnion(DWORD0)
    DWORD0 dw0;

    BitUnion64(DWORD1)
        Bitfield<1, 0>   s1dss;
        Bitfield<3, 2>   s1cir;
        Bitfield<5, 4>   s1cor;
        Bitfield<7, 6>   s1csh;
        Bitfield<8>      s2hwu59;
        Bitfield<9>      s2hwu60;
        Bitfield<10>     s2hwu61;
        Bitfield<11>     s2hwu62;
        Bitfield<12>     dre;
        Bitfield<16, 13> cont;
        Bitfield<17>     dcp;
        Bitfield<18>     ppar;
        Bitfield<19>     mev;
        Bitfield<27>     s1stalld;
        Bitfield<29, 28> eats;
        Bitfield<31, 30> strw;
        Bitfield<35, 32> memattr;
        Bitfield<36>     mtcfg;
        Bitfield<40, 37> alloccfg;
        Bitfield<45, 44> shcfg;
        Bitfield<47, 46> nscfg;
        Bitfield<49, 48> privcfg;
        Bitfield<51, 50> instcfg;
    EndBitUnion(DWORD1)
    DWORD1 dw1;

    BitUnion64(DWORD2)
        Bitfield<15, 0>  s2vmid;
        Bitfield<37, 32> s2t0sz;
        Bitfield<39, 38> s2sl0;
        Bitfield<41, 40> s2ir0;
        Bitfield<43, 42> s2or0;
        Bitfield<45, 44> s2sh0;
        Bitfield<47, 46> s2tg;
        Bitfield<50, 48> s2ps;
        Bitfield<51>     s2aa64;
        Bitfield<52>     s2endi;
        Bitfield<53>     s2affd;
        Bitfield<54>     s2ptw;
        Bitfield<55>     s2hd;
        Bitfield<56>     s2ha;
        Bitfield<57>     s2s;
        Bitfield<58>     s2r;
    EndBitUnion(DWORD2)
    DWORD2 dw2;

    BitUnion64(DWORD3)
        Bitfield<51, 4> s2ttb;
    EndBitUnion(DWORD3)
    DWORD3 dw3;

    uint64_t _pad[4];
};

struct ContextDescriptor
{
    BitUnion64(DWORD0)
        Bitfield<5, 0>   t0sz;
        Bitfield<7, 6>   tg0;
        Bitfield<9, 8>   ir0;
        Bitfield<11, 10> or0;
        Bitfield<13, 12> sh0;
        Bitfield<14>     epd0;
        Bitfield<15>     endi;
        Bitfield<21, 16> t1sz;
        Bitfield<23, 22> tg1;
        Bitfield<25, 24> ir1;
        Bitfield<27, 26> or1;
        Bitfield<29, 28> sh1;
        Bitfield<30>     epd1;
        Bitfield<31>     valid;
        Bitfield<34, 32> ips;
        Bitfield<35>     affd;
        Bitfield<36>     wxn;
        Bitfield<37>     uwxn;
        Bitfield<39, 38> tbi;
        Bitfield<40>     pan;
        Bitfield<41>     aa64;
        Bitfield<42>     hd;
        Bitfield<43>     ha;
        Bitfield<44>     s;
        Bitfield<45>     r;
        Bitfield<46>     a;
        Bitfield<47>     aset;
        Bitfield<63, 48> asid;
    EndBitUnion(DWORD0)
    DWORD0 dw0;

    BitUnion64(DWORD1)
        Bitfield<0>      nscfg0;
        Bitfield<1>      had0;
        Bitfield<51, 4>  ttb0;
        Bitfield<60>     hwu0g59;
        Bitfield<61>     hwu0g60;
        Bitfield<62>     hwu0g61;
        Bitfield<63>     hwu0g62;
    EndBitUnion(DWORD1)
    DWORD1 dw1;

    BitUnion64(DWORD2)
        Bitfield<0>      nscfg1;
        Bitfield<1>      had1;
        Bitfield<51, 4>  ttb1;
        Bitfield<60>     hwu1g59;
        Bitfield<61>     hwu1g60;
        Bitfield<62>     hwu1g61;
        Bitfield<63>     hwu1g62;
    EndBitUnion(DWORD2)
    DWORD2 dw2;

    uint64_t mair;
    uint64_t amair;
    uint64_t _pad[3];
};

enum
{
    CR0_SMMUEN_MASK = 0x1,
    CR0_PRIQEN_MASK = 0x2,
    CR0_EVENTQEN_MASK = 0x4,
    CR0_CMDQEN_MASK = 0x8,
    CR0_ATSCHK_MASK = 0x10,
    CR0_VMW_MASK = 0x1C0,
};

enum SMMUCommandType
{
    CMD_PRF_CONFIG     = 0x01,
    CMD_PRF_ADDR       = 0x02,
    CMD_CFGI_STE       = 0x03,
    CMD_CFGI_STE_RANGE = 0x04,
    CMD_CFGI_CD        = 0x05,
    CMD_CFGI_CD_ALL    = 0x06,
    CMD_TLBI_NH_ALL    = 0x10,
    CMD_TLBI_NH_ASID   = 0x11,
    CMD_TLBI_NH_VAA    = 0x13,
    CMD_TLBI_NH_VA     = 0x12,
    CMD_TLBI_EL3_ALL   = 0x18,
    CMD_TLBI_EL3_VA    = 0x1A,
    CMD_TLBI_EL2_ALL   = 0x20,
    CMD_TLBI_EL2_ASID  = 0x21,
    CMD_TLBI_EL2_VA    = 0x22,
    CMD_TLBI_EL2_VAA   = 0x23,
    CMD_TLBI_S2_IPA    = 0x2a,
    CMD_TLBI_S12_VMALL = 0x28,
    CMD_TLBI_NSNH_ALL  = 0x30,
    CMD_ATC_INV        = 0x40,
    CMD_PRI_RESP       = 0x41,
    CMD_RESUME         = 0x44,
    CMD_STALL_TERM     = 0x45,
    CMD_SYNC           = 0x46,
};

struct SMMUCommand
{
    BitUnion64(DWORD0)
        Bitfield<7, 0>   type;
        Bitfield<10>     ssec;
        Bitfield<11>     ssv;
        Bitfield<31, 12> ssid;
        Bitfield<47, 32> vmid;
        Bitfield<63, 48> asid;
        Bitfield<63, 32> sid;
    EndBitUnion(DWORD0)
    DWORD0 dw0;

    BitUnion64(DWORD1)
        Bitfield<0>      leaf;
        Bitfield<4, 0>   size;
        Bitfield<4, 0>   range;
        Bitfield<63, 12> address;
    EndBitUnion(DWORD1)
    DWORD1 dw1;

    uint64_t addr() const
    {
        uint64_t address = (uint64_t)(dw1.address) << 12;
        return address;
    }
};

struct SMMUEvent
{
    struct Data {
        BitUnion64(DWORD0)
            Bitfield<7, 0> eventType;
            Bitfield<11> ssv;
            Bitfield<31, 12> substreamId;
            Bitfield<63, 32> streamId;
        EndBitUnion(DWORD0)
        DWORD0 dw0;

        BitUnion64(DWORD1)
            Bitfield<16, 0> stag;
            Bitfield<33> pnu;
            Bitfield<34> ind;
            Bitfield<35> rnw;
            Bitfield<38> nsipa;
            Bitfield<39> s2;
            Bitfield<41, 40> clss;
        EndBitUnion(DWORD1)
        DWORD1 dw1;

        BitUnion64(DWORD2)
            Bitfield<63, 0> inputAddr;
        EndBitUnion(DWORD2)
        DWORD2 dw2;

        BitUnion64(DWORD3)
            Bitfield<51, 3> fetchAddr;
            Bitfield<51, 12> ipa;
        EndBitUnion(DWORD3)
        DWORD3 dw3;
    } data;

    std::string print() const;
};

enum
{
    SMMU_MAX_TRANS_ID = 64
};

} // namespace gem5

#endif /* __DEV_ARM_SMMU_V3_DEFS_HH__ */
