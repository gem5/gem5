/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
 * All rights reserved.
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

#ifndef __ARCH_X86_MISCREGS_HH__
#define __ARCH_X86_MISCREGS_HH__

#include "arch/x86/regs/segment.hh"
#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"
#include "base/logging.hh"
#include "cpu/reg_class.hh"
#include "debug/MiscRegs.hh"

//These get defined in some system headers (at least termbits.h). That confuses
//things here significantly.
#undef CR0
#undef CR2
#undef CR3

namespace gem5
{
namespace X86ISA
{

enum CondFlagBit
{
    CFBit = 1 << 0,
    PFBit = 1 << 2,
    ECFBit = 1 << 3,
    AFBit = 1 << 4,
    EZFBit = 1 << 5,
    ZFBit = 1 << 6,
    SFBit = 1 << 7,
    DFBit = 1 << 10,
    OFBit = 1 << 11
};

constexpr uint32_t CfofMask = CFBit | OFBit;
constexpr uint32_t CcFlagMask = PFBit | AFBit | ZFBit | SFBit;

enum RFLAGBit
{
    TFBit = 1 << 8,
    IFBit = 1 << 9,
    NTBit = 1 << 14,
    RFBit = 1 << 16,
    VMBit = 1 << 17,
    ACBit = 1 << 18,
    VIFBit = 1 << 19,
    VIPBit = 1 << 20,
    IDBit = 1 << 21
};

enum X87StatusBit
{
    // Exception Flags
    IEBit = 1 << 0,
    DEBit = 1 << 1,
    ZEBit = 1 << 2,
    OEBit = 1 << 3,
    UEBit = 1 << 4,
    PEBit = 1 << 5,

    // !Exception Flags
    StackFaultBit = 1 << 6,
    ErrSummaryBit = 1 << 7,
    CC0Bit = 1 << 8,
    CC1Bit = 1 << 9,
    CC2Bit = 1 << 10,
    CC3Bit = 1 << 14,
    BusyBit = 1 << 15,
};

namespace misc_reg
{

enum : RegIndex
{
    // Control registers
    // Most of these are invalid.  See isValid() below.
    CrBase,
    Cr0 = CrBase,
    Cr1,
    Cr2,
    Cr3,
    Cr4,
    Cr5,
    Cr6,
    Cr7,
    Cr8,
    Cr9,
    Cr10,
    Cr11,
    Cr12,
    Cr13,
    Cr14,
    Cr15,

    // Debug registers
    DrBase = CrBase + NumCRegs,
    Dr0 = DrBase,
    Dr1,
    Dr2,
    Dr3,
    Dr4,
    Dr5,
    Dr6,
    Dr7,

    // Flags register
    Rflags = DrBase + NumDRegs,

    //Register to keep handy values like the CPU mode in.
    M5Reg,

    /*
     * Model Specific Registers
     */
    // Time stamp counter
    Tsc,

    Mtrrcap,

    SysenterCs,
    SysenterEsp,
    SysenterEip,

    McgCap,
    McgStatus,
    McgCtl,

    DebugCtlMsr,

    LastBranchFromIp,
    LastBranchToIp,
    LastExceptionFromIp,
    LastExceptionToIp,

    MtrrPhysBaseBase,
    MtrrPhysBase0 = MtrrPhysBaseBase,
    MtrrPhysBase1,
    MtrrPhysBase2,
    MtrrPhysBase3,
    MtrrPhysBase4,
    MtrrPhysBase5,
    MtrrPhysBase6,
    MtrrPhysBase7,
    MtrrPhysBaseEnd,

    MtrrPhysMaskBase = MtrrPhysBaseEnd,
    MtrrPhysMask0 = MtrrPhysMaskBase,
    MtrrPhysMask1,
    MtrrPhysMask2,
    MtrrPhysMask3,
    MtrrPhysMask4,
    MtrrPhysMask5,
    MtrrPhysMask6,
    MtrrPhysMask7,
    MtrrPhysMaskEnd,

    MtrrFix64k00000 = MtrrPhysMaskEnd,
    MtrrFix16k80000,
    MtrrFix16kA0000,
    MtrrFix4kC0000,
    MtrrFix4kC8000,
    MtrrFix4kD0000,
    MtrrFix4kD8000,
    MtrrFix4kE0000,
    MtrrFix4kE8000,
    MtrrFix4kF0000,
    MtrrFix4kF8000,

    Pat,

    DefType,

    McCtlBase,
    Mc0Ctl = McCtlBase,
    Mc1Ctl,
    Mc2Ctl,
    Mc3Ctl,
    Mc4Ctl,
    Mc5Ctl,
    Mc6Ctl,
    Mc7Ctl,
    McCtlEnd,

    McStatusBase = McCtlEnd,
    Mc0Status = McStatusBase,
    Mc1Status,
    Mc2Status,
    Mc3Status,
    Mc4Status,
    Mc5Status,
    Mc6Status,
    Mc7Status,
    McStatusEnd,

    McAddrBase = McStatusEnd,
    Mc0Addr = McAddrBase,
    Mc1Addr,
    Mc2Addr,
    Mc3Addr,
    Mc4Addr,
    Mc5Addr,
    Mc6Addr,
    Mc7Addr,
    McAddrEnd,

    McMiscBase = McAddrEnd,
    Mc0Misc = McMiscBase,
    Mc1Misc,
    Mc2Misc,
    Mc3Misc,
    Mc4Misc,
    Mc5Misc,
    Mc6Misc,
    Mc7Misc,
    McMiscEnd,

    // Extended feature enable register
    Efer = McMiscEnd,

    Star,
    Lstar,
    Cstar,

    SfMask,

    KernelGsBase,

    TscAux,

    PerfEvtSelBase,
    PerfEvtSel0 = PerfEvtSelBase,
    PerfEvtSel1,
    PerfEvtSel2,
    PerfEvtSel3,
    PerfEvtSelEnd,

    PerfEvtCtrBase = PerfEvtSelEnd,
    PerfEvtCtr0 = PerfEvtCtrBase,
    PerfEvtCtr1,
    PerfEvtCtr2,
    PerfEvtCtr3,
    PerfEvtCtrEnd,

    Syscfg = PerfEvtCtrEnd,

    IorrBaseBase,
    IorrBase0 = IorrBaseBase,
    IorrBase1,
    IorrBaseEnd,

    IorrMaskBase = IorrBaseEnd,
    IorrMask0 = IorrMaskBase,
    IorrMask1,
    IorrMaskEnd,

    TopMem = IorrMaskEnd,
    TopMem2,

    VmCr,
    Ignne,
    SmmCtl,
    VmHsavePa,

    /*
     * Segment registers
     */
    // Segment selectors
    SegSelBase,
    Es = SegSelBase,
    Cs,
    Ss,
    Ds,
    Fs,
    Gs,
    Hs,
    Tsl,
    Tsg,
    Ls,
    Ms,
    Tr,
    Idtr,

    // Hidden segment base field
    SegBaseBase = SegSelBase + segment_idx::NumIdxs,
    EsBase = SegBaseBase,
    CsBase,
    SsBase,
    DsBase,
    FsBase,
    GsBase,
    HsBase,
    TslBase,
    TsgBase,
    LsBase,
    MsBase,
    TrBase,
    IdtrBase,

    // The effective segment base, ie what is actually added to an
    // address. In 64 bit mode this can be different from the above,
    // namely 0.
    SegEffBaseBase = SegBaseBase + segment_idx::NumIdxs,
    EsEffBase = SegEffBaseBase,
    CsEffBase,
    SsEffBase,
    DsEffBase,
    FsEffBase,
    GsEffBase,
    HsEffBase,
    TslEffBase,
    TsgEffBase,
    LsEffBase,
    MsEffBase,
    TrEffBase,
    IdtrEffBase,

    // Hidden segment limit field
    SegLimitBase = SegEffBaseBase + segment_idx::NumIdxs,
    EsLimit = SegLimitBase,
    CsLimit,
    SsLimit,
    DsLimit,
    FsLimit,
    GsLimit,
    HsLimit,
    TslLimit,
    TsgLimit,
    LsLimit,
    MsLimit,
    TrLimit,
    IdtrLimit,

    // Hidden segment limit attributes
    SegAttrBase = SegLimitBase + segment_idx::NumIdxs,
    EsAttr = SegAttrBase,
    CsAttr,
    SsAttr,
    DsAttr,
    FsAttr,
    GsAttr,
    HsAttr,
    TslAttr,
    TsgAttr,
    LsAttr,
    MsAttr,
    TrAttr,
    IdtrAttr,

    // Floating point control registers
    X87Top = SegAttrBase + segment_idx::NumIdxs,

    Mxcsr,
    Fcw,
    Fsw,
    Ftw,
    Ftag,
    Fiseg,
    Fioff,
    Foseg,
    Fooff,
    Fop,

    //XXX Add "Model-Specific Registers"

    ApicBase,

    // "Fake" MSRs for internally implemented devices
    PciConfigAddress,

    NumRegs
};

static inline bool
isValid(int index)
{
    return (index >= Cr0 && index < NumRegs &&
            index != Cr1 &&
            !(index > Cr4 && index < Cr8) &&
            !(index > Cr8 && index <= Cr15));
}

static inline RegIndex
cr(int index)
{
    assert(index >= 0 && index < NumCRegs);
    return CrBase + index;
}

static inline RegIndex
dr(int index)
{
    assert(index >= 0 && index < NumDRegs);
    return DrBase + index;
}

static inline RegIndex
mtrrPhysBase(int index)
{
    assert(index >= 0 && index < (MtrrPhysBaseEnd - MtrrPhysBaseBase));
    return MtrrPhysBaseBase + index;
}

static inline RegIndex
mtrrPhysMask(int index)
{
    assert(index >= 0 && index < (MtrrPhysMaskEnd - MtrrPhysMaskBase));
    return MtrrPhysMaskBase + index;
}

static inline RegIndex
mcCtl(int index)
{
    assert(index >= 0 && index < (McCtlEnd - McCtlBase));
    return McCtlBase + index;
}

static inline RegIndex
mcStatus(int index)
{
    assert(index >= 0 && index < (McStatusEnd - McStatusBase));
    return McStatusBase + index;
}

static inline RegIndex
mcAddr(int index)
{
    assert(index >= 0 && index < (McAddrEnd - McAddrBase));
    return McAddrBase + index;
}

static inline RegIndex
mcMisc(int index)
{
    assert(index >= 0 && index < (McMiscEnd - McMiscBase));
    return McMiscBase + index;
}

static inline RegIndex
perfEvtSel(int index)
{
    assert(index >= 0 && index < (PerfEvtSelEnd - PerfEvtSelBase));
    return PerfEvtSelBase + index;
}

static inline RegIndex
perfEvtCtr(int index)
{
    assert(index >= 0 && index < (PerfEvtCtrEnd - PerfEvtCtrBase));
    return PerfEvtCtrBase + index;
}

static inline RegIndex
iorrBase(int index)
{
    assert(index >= 0 && index < (IorrBaseEnd - IorrBaseBase));
    return IorrBaseBase + index;
}

static inline RegIndex
iorrMask(int index)
{
    assert(index >= 0 && index < (IorrMaskEnd - IorrMaskBase));
    return IorrMaskBase + index;
}

static inline RegIndex
segSel(int index)
{
    assert(index >= 0 && index < segment_idx::NumIdxs);
    return SegSelBase + index;
}

static inline RegIndex
segBase(int index)
{
    assert(index >= 0 && index < segment_idx::NumIdxs);
    return SegBaseBase + index;
}

static inline RegIndex
segEffBase(int index)
{
    assert(index >= 0 && index < segment_idx::NumIdxs);
    return SegEffBaseBase + index;
}

static inline RegIndex
segLimit(int index)
{
    assert(index >= 0 && index < segment_idx::NumIdxs);
    return SegLimitBase + index;
}

static inline RegIndex
segAttr(int index)
{
    assert(index >= 0 && index < segment_idx::NumIdxs);
    return SegAttrBase + index;
}

} // namespace misc_reg

inline constexpr RegClass miscRegClass(MiscRegClass, MiscRegClassName,
        misc_reg::NumRegs, debug::MiscRegs);

/**
 * A type to describe the condition code bits of the RFLAGS register,
 * plus two flags, EZF and ECF, which are only visible to microcode.
 */
BitUnion64(CCFlagBits)
    Bitfield<11> of;
    Bitfield<7> sf;
    Bitfield<6> zf;
    Bitfield<5> ezf;
    Bitfield<4> af;
    Bitfield<3> ecf;
    Bitfield<2> pf;
    Bitfield<0> cf;
EndBitUnion(CCFlagBits)

/**
 * RFLAGS
 */
BitUnion64(RFLAGS)
    Bitfield<21> id; // ID Flag
    Bitfield<20> vip; // Virtual Interrupt Pending
    Bitfield<19> vif; // Virtual Interrupt Flag
    Bitfield<18> ac; // Alignment Check
    Bitfield<17> vm; // Virtual-8086 Mode
    Bitfield<16> rf; // Resume Flag
    Bitfield<14> nt; // Nested Task
    Bitfield<13, 12> iopl; // I/O Privilege Level
    Bitfield<11> of; // Overflow Flag
    Bitfield<10> df; // Direction Flag
    Bitfield<9> intf; // Interrupt Flag
    Bitfield<8> tf; // Trap Flag
    Bitfield<7> sf; // Sign Flag
    Bitfield<6> zf; // Zero Flag
    Bitfield<4> af; // Auxiliary Flag
    Bitfield<2> pf; // Parity Flag
    Bitfield<0> cf; // Carry Flag
EndBitUnion(RFLAGS)

BitUnion64(HandyM5Reg)
    Bitfield<0> mode;
    Bitfield<3, 1> submode;
    Bitfield<5, 4> cpl;
    Bitfield<6> paging;
    Bitfield<7> prot;
    Bitfield<9, 8> defOp;
    Bitfield<11, 10> altOp;
    Bitfield<13, 12> defAddr;
    Bitfield<15, 14> altAddr;
    Bitfield<17, 16> stack;
EndBitUnion(HandyM5Reg)

/**
 * Control registers
 */
BitUnion64(CR0)
    Bitfield<31> pg; // Paging
    Bitfield<30> cd; // Cache Disable
    Bitfield<29> nw; // Not Writethrough
    Bitfield<18> am; // Alignment Mask
    Bitfield<16> wp; // Write Protect
    Bitfield<5> ne; // Numeric Error
    Bitfield<4> et; // Extension Type
    Bitfield<3> ts; // Task Switched
    Bitfield<2> em; // Emulation
    Bitfield<1> mp; // Monitor Coprocessor
    Bitfield<0> pe; // Protection Enabled
EndBitUnion(CR0)

// Page Fault Virtual Address
BitUnion64(CR2)
    Bitfield<31, 0> legacy;
EndBitUnion(CR2)

BitUnion64(CR3)
    Bitfield<51, 12> longPdtb; // Long Mode Page-Directory-Table
                               // Base Address
    Bitfield<31, 12> pdtb; // Non-PAE Addressing Page-Directory-Table
                           // Base Address
    Bitfield<31, 5> paePdtb; // PAE Addressing Page-Directory-Table
                             // Base Address
    Bitfield<11, 0> pcid; // Process-Context Identifier
    Bitfield<4> pcd; // Page-Level Cache Disable
    Bitfield<3> pwt; // Page-Level Writethrough
EndBitUnion(CR3)

BitUnion64(CR4)
    Bitfield<18> osxsave; // Enable XSAVE and Proc Extended States
    Bitfield<17> pcide; // PCID Enable
    Bitfield<16> fsgsbase; // Enable RDFSBASE, RDGSBASE, WRFSBASE,
                           // WRGSBASE instructions
    Bitfield<10> osxmmexcpt; // Operating System Unmasked
                             // Exception Support
    Bitfield<9> osfxsr; // Operating System FXSave/FSRSTOR Support
    Bitfield<8> pce; // Performance-Monitoring Counter Enable
    Bitfield<7> pge; // Page-Global Enable
    Bitfield<6> mce; // Machine Check Enable
    Bitfield<5> pae; // Physical-Address Extension
    Bitfield<4> pse; // Page Size Extensions
    Bitfield<3> de; // Debugging Extensions
    Bitfield<2> tsd; // Time Stamp Disable
    Bitfield<1> pvi; // Protected-Mode Virtual Interrupts
    Bitfield<0> vme; // Virtual-8086 Mode Extensions
EndBitUnion(CR4)

BitUnion64(CR8)
    Bitfield<3, 0> tpr; // Task Priority Register
EndBitUnion(CR8)

BitUnion64(DR6)
    Bitfield<0> b0;
    Bitfield<1> b1;
    Bitfield<2> b2;
    Bitfield<3> b3;
    Bitfield<13> bd;
    Bitfield<14> bs;
    Bitfield<15> bt;
EndBitUnion(DR6)

BitUnion64(DR7)
    Bitfield<0> l0;
    Bitfield<1> g0;
    Bitfield<2> l1;
    Bitfield<3> g1;
    Bitfield<4> l2;
    Bitfield<5> g2;
    Bitfield<6> l3;
    Bitfield<7> g3;
    Bitfield<8> le;
    Bitfield<9> ge;
    Bitfield<13> gd;
    Bitfield<17, 16> rw0;
    Bitfield<19, 18> len0;
    Bitfield<21, 20> rw1;
    Bitfield<23, 22> len1;
    Bitfield<25, 24> rw2;
    Bitfield<27, 26> len2;
    Bitfield<29, 28> rw3;
    Bitfield<31, 30> len3;
EndBitUnion(DR7)

// MTRR capabilities
BitUnion64(MTRRcap)
    Bitfield<7, 0> vcnt; // Variable-Range Register Count
    Bitfield<8> fix; // Fixed-Range Registers
    Bitfield<10> wc; // Write-Combining
EndBitUnion(MTRRcap)

/**
 * SYSENTER configuration registers
 */
BitUnion64(SysenterCS)
    Bitfield<15, 0> targetCS;
EndBitUnion(SysenterCS)

BitUnion64(SysenterESP)
    Bitfield<31, 0> targetESP;
EndBitUnion(SysenterESP)

BitUnion64(SysenterEIP)
    Bitfield<31, 0> targetEIP;
EndBitUnion(SysenterEIP)

/**
 * Global machine check registers
 */
BitUnion64(McgCap)
    Bitfield<7, 0> count; // Number of error reporting register banks
    Bitfield<8> MCGCP; // MCG_CTL register present.
EndBitUnion(McgCap)

BitUnion64(McgStatus)
    Bitfield<0> ripv; // Restart-IP valid
    Bitfield<1> eipv; // Error-IP valid
    Bitfield<2> mcip; // Machine check in-progress
EndBitUnion(McgStatus)

BitUnion64(DebugCtlMsr)
    Bitfield<0> lbr; // Last-branch record
    Bitfield<1> btf; // Branch single step
    Bitfield<2> pb0; // Performance monitoring pin control 0
    Bitfield<3> pb1; // Performance monitoring pin control 1
    Bitfield<4> pb2; // Performance monitoring pin control 2
    Bitfield<5> pb3; // Performance monitoring pin control 3
    /*uint64_t pb(int index)
    {
        return bits(__data, index + 2);
    }*/
EndBitUnion(DebugCtlMsr)

BitUnion64(MtrrPhysBase)
    Bitfield<7, 0> type; // Default memory type
    Bitfield<51, 12> physbase; // Range physical base address
EndBitUnion(MtrrPhysBase)

BitUnion64(MtrrPhysMask)
    Bitfield<11> valid; // MTRR pair enable
    Bitfield<51, 12> physmask; // Range physical mask
EndBitUnion(MtrrPhysMask)

BitUnion64(MtrrFixed)
    /*uint64_t type(int index)
    {
        return bits(__data, index * 8 + 7, index * 8);
    }*/
EndBitUnion(MtrrFixed)

BitUnion64(Pat)
    /*uint64_t pa(int index)
    {
        return bits(__data, index * 8 + 2, index * 8);
    }*/
EndBitUnion(Pat)

BitUnion64(MtrrDefType)
    Bitfield<7, 0> type; // Default type
    Bitfield<10> fe; // Fixed range enable
    Bitfield<11> e; // MTRR enable
EndBitUnion(MtrrDefType)

/**
 * Machine check
 */
BitUnion64(McStatus)
    Bitfield<15,0> mcaErrorCode;
    Bitfield<31,16> modelSpecificCode;
    Bitfield<56,32> otherInfo;
    Bitfield<57> pcc; // Processor-context corrupt
    Bitfield<58> addrv; // Error-address register valid
    Bitfield<59> miscv; // Miscellaneous-error register valid
    Bitfield<60> en; // Error condition enabled
    Bitfield<61> uc; // Uncorrected error
    Bitfield<62> over; // Status register overflow
    Bitfield<63> val; // Valid
EndBitUnion(McStatus)

BitUnion64(McCtl)
    /*uint64_t en(int index)
    {
        return bits(__data, index);
    }*/
EndBitUnion(McCtl)

// Extended feature enable register
BitUnion64(Efer)
    Bitfield<0> sce; // System call extensions
    Bitfield<8> lme; // Long mode enable
    Bitfield<10> lma; // Long mode active
    Bitfield<11> nxe; // No-execute enable
    Bitfield<12> svme; // Secure virtual machine enable
    Bitfield<14> ffxsr; // Fast fxsave/fxrstor
EndBitUnion(Efer)

BitUnion64(Star)
    Bitfield<31,0> targetEip;
    Bitfield<47,32> syscallCsAndSs;
    Bitfield<63,48> sysretCsAndSs;
EndBitUnion(Star)

BitUnion64(SfMask)
    Bitfield<31,0> mask;
EndBitUnion(SfMask)

BitUnion64(PerfEvtSel)
    Bitfield<7,0> eventMask;
    Bitfield<15,8> unitMask;
    Bitfield<16> usr; // User mode
    Bitfield<17> os; // Operating-system mode
    Bitfield<18> e; // Edge detect
    Bitfield<19> pc; // Pin control
    Bitfield<20> intEn; // Interrupt enable
    Bitfield<22> en; // Counter enable
    Bitfield<23> inv; // Invert mask
    Bitfield<31,24> counterMask;
EndBitUnion(PerfEvtSel)

BitUnion32(Syscfg)
    Bitfield<18> mfde; // MtrrFixDramEn
    Bitfield<19> mfdm; // MtrrFixDramModEn
    Bitfield<20> mvdm; // MtrrVarDramEn
    Bitfield<21> tom2; // MtrrTom2En
EndBitUnion(Syscfg)

BitUnion64(IorrBase)
    Bitfield<3> wr; // WrMem Enable
    Bitfield<4> rd; // RdMem Enable
    Bitfield<51,12> physbase; // Range physical base address
EndBitUnion(IorrBase)

BitUnion64(IorrMask)
    Bitfield<11> v; // I/O register pair enable (valid)
    Bitfield<51,12> physmask; // Range physical mask
EndBitUnion(IorrMask)

BitUnion64(Tom)
    Bitfield<51,23> physAddr; // Top of memory physical address
EndBitUnion(Tom)

BitUnion64(VmCrMsr)
    Bitfield<0> dpd;
    Bitfield<1> rInit;
    Bitfield<2> disA20M;
EndBitUnion(VmCrMsr)

BitUnion64(IgnneMsr)
    Bitfield<0> ignne;
EndBitUnion(IgnneMsr)

BitUnion64(SmmCtlMsr)
    Bitfield<0> dismiss;
    Bitfield<1> enter;
    Bitfield<2> smiCycle;
    Bitfield<3> exit;
    Bitfield<4> rsmCycle;
EndBitUnion(SmmCtlMsr)

/**
 * Segment Selector
 */
BitUnion64(SegSelector)
    // The following bitfield is not defined in the ISA, but it's useful
    // when checking selectors in larger data types to make sure they
    // aren't too large.
    Bitfield<63, 3> esi; // Extended selector
    Bitfield<15, 3> si; // Selector Index
    Bitfield<2> ti; // Table Indicator
    Bitfield<1, 0> rpl; // Requestor Privilege Level
EndBitUnion(SegSelector)

/**
 * Segment Descriptors
 */

class SegDescriptorBase
{
  public:
    uint32_t
    getter(const uint64_t &storage) const
    {
        return (bits(storage, 63, 56) << 24) | bits(storage, 39, 16);
    }

    void
    setter(uint64_t &storage, uint32_t base)
    {
        replaceBits(storage, 63, 56, bits(base, 31, 24));
        replaceBits(storage, 39, 16, bits(base, 23, 0));
    }
};

class SegDescriptorLimit
{
  public:
    uint32_t
    getter(const uint64_t &storage) const
    {
        uint32_t limit = (bits(storage, 51, 48) << 16) |
                         bits(storage, 15, 0);
        if (bits(storage, 55))
            limit = (limit << 12) | mask(12);
        return limit;
    }

    void
    setter(uint64_t &storage, uint32_t limit)
    {
        bool g = (bits(limit, 31, 24) != 0);
        panic_if(g && bits(limit, 11, 0) != mask(12),
                 "Inlimitid segment limit %#x", limit);
        if (g)
            limit = limit >> 12;
        replaceBits(storage, 51, 48, bits(limit, 23, 16));
        replaceBits(storage, 15, 0, bits(limit, 15, 0));
        replaceBits(storage, 55, g ? 1 : 0);
    }
};

BitUnion64(SegDescriptor)
    Bitfield<63, 56> baseHigh;
    Bitfield<39, 16> baseLow;
    BitfieldType<SegDescriptorBase> base;
    Bitfield<55> g; // Granularity
    Bitfield<54> d; // Default Operand Size
    Bitfield<54> b; // Default Operand Size
    Bitfield<53> l; // Long Attribute Bit
    Bitfield<52> avl; // Available To Software
    Bitfield<51, 48> limitHigh;
    Bitfield<15, 0> limitLow;
    BitfieldType<SegDescriptorLimit> limit;
    Bitfield<47> p; // Present
    Bitfield<46, 45> dpl; // Descriptor Privilege-Level
    Bitfield<44> s; // System
    SubBitUnion(type, 43, 40)
        // Specifies whether this descriptor is for code or data.
        Bitfield<43> codeOrData;

        // These bit fields are for code segments
        Bitfield<42> c; // Conforming
        Bitfield<41> r; // Readable

        // These bit fields are for data segments
        Bitfield<42> e; // Expand-Down
        Bitfield<41> w; // Writable

        // This is used for both code and data segments.
        Bitfield<40> a; // Accessed
    EndSubBitUnion(type)
EndBitUnion(SegDescriptor)

/**
 * TSS Descriptor (long mode - 128 bits)
 * the lower 64 bits
 */
BitUnion64(TSSlow)
    Bitfield<63, 56> baseHigh;
    Bitfield<39, 16> baseLow;
    BitfieldType<SegDescriptorBase> base;
    Bitfield<55> g; // Granularity
    Bitfield<52> avl; // Available To Software
    Bitfield<51, 48> limitHigh;
    Bitfield<15, 0> limitLow;
    BitfieldType<SegDescriptorLimit> limit;
    Bitfield<47> p; // Present
    Bitfield<46, 45> dpl; // Descriptor Privilege-Level
    SubBitUnion(type, 43, 40)
        // Specifies whether this descriptor is for code or data.
        Bitfield<43> codeOrData;

        // These bit fields are for code segments
        Bitfield<42> c; // Conforming
        Bitfield<41> r; // Readable

        // These bit fields are for data segments
        Bitfield<42> e; // Expand-Down
        Bitfield<41> w; // Writable

        // This is used for both code and data segments.
        Bitfield<40> a; // Accessed
    EndSubBitUnion(type)
EndBitUnion(TSSlow)

/**
 * TSS Descriptor (long mode - 128 bits)
 * the upper 64 bits
 */
BitUnion64(TSShigh)
    Bitfield<31, 0> base;
EndBitUnion(TSShigh)

BitUnion64(SegAttr)
    Bitfield<1, 0> dpl;
    Bitfield<2> unusable;
    Bitfield<3> defaultSize;
    Bitfield<4> longMode;
    Bitfield<5> avl;
    Bitfield<6> granularity;
    Bitfield<7> present;
    Bitfield<11, 8> type;
    Bitfield<12> writable;
    Bitfield<13> readable;
    Bitfield<14> expandDown;
    Bitfield<15> system;
EndBitUnion(SegAttr)

BitUnion64(GateDescriptor)
    Bitfield<63, 48> offsetHigh; // Target Code-Segment Offset
    Bitfield<15, 0> offsetLow; // Target Code-Segment Offset
    Bitfield<31, 16> selector; // Target Code-Segment Selector
    Bitfield<47> p; // Present
    Bitfield<46, 45> dpl; // Descriptor Privilege-Level
    Bitfield<43, 40> type;
    Bitfield<36, 32> count; // Parameter Count
EndBitUnion(GateDescriptor)

/**
 * Long Mode Gate Descriptor
 */
BitUnion64(GateDescriptorLow)
    Bitfield<63, 48> offsetHigh; // Target Code-Segment Offset
    Bitfield<47> p; // Present
    Bitfield<46, 45> dpl; // Descriptor Privilege-Level
    Bitfield<43, 40> type;
    Bitfield<35, 32> IST; // IST pointer to TSS, new stack for exceptions
    Bitfield<31, 16> selector; // Target Code-Segment Selector
    Bitfield<15, 0> offsetLow; // Target Code-Segment Offset
EndBitUnion(GateDescriptorLow)

BitUnion64(GateDescriptorHigh)
    Bitfield<31, 0> offset; // Target Code-Segment Offset
EndBitUnion(GateDescriptorHigh)

/**
 * Descriptor-Table Registers
 */
BitUnion64(GDTR)
EndBitUnion(GDTR)

BitUnion64(IDTR)
EndBitUnion(IDTR)

BitUnion64(LDTR)
EndBitUnion(LDTR)

/**
 * Task Register
 */
BitUnion64(TR)
EndBitUnion(TR)


/**
 * Local APIC Base Register
 */
BitUnion64(LocalApicBase)
    Bitfield<51, 12> base;
    Bitfield<11> enable;
    Bitfield<8> bsp;
EndBitUnion(LocalApicBase)

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_INTREGS_HH__
