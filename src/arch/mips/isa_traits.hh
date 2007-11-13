/*
 * Copyright N) 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright N) <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. ($(B!H(BMIPS$(B!I(B) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED $(B!H(BAS IS.$(B!I(B  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Gabe M. Black
 *          Korey L. Sewell
 *          Jaidev Patwardhan
 */

#ifndef __ARCH_MIPS_ISA_TRAITS_HH__
#define __ARCH_MIPS_ISA_TRAITS_HH__

#include "arch/mips/types.hh"
#include "config/full_system.hh"
#include "sim/host.hh"

namespace LittleEndianGuest {};

#define TARGET_MIPS

class StaticInstPtr;

namespace MipsISA
{
    using namespace LittleEndianGuest;

    StaticInstPtr decodeInst(ExtMachInst);

    // MIPS DOES have a delay slot
    #define ISA_HAS_DELAY_SLOT 1

    const Addr PageShift = 13;
    const Addr PageBytes = ULL(1) << PageShift;
    const Addr Page_Mask = ~(PageBytes - 1);
    const Addr PageOffset = PageBytes - 1;


    ////////////////////////////////////////////////////////////////////////
    //
    //  Translation stuff
    //

    const Addr PteShift = 3;
    const Addr NPtePageShift = PageShift - PteShift;
    const Addr NPtePage = ULL(1) << NPtePageShift;
    const Addr PteMask = NPtePage - 1;

    //// All 'Mapped' segments go through the TLB
    //// All other segments are translated by dropping the MSB, to give
    //// the corresponding physical address
    // User Segment - Mapped
    const Addr USegBase = ULL(0x0);
    const Addr USegEnd = ULL(0x7FFFFFFF);

    // Kernel Segment 0 - Unmapped
    const Addr KSeg0End = ULL(0x9FFFFFFF);
    const Addr KSeg0Base =  ULL(0x80000000);
    const Addr KSeg0Mask = ULL(0x1FFFFFFF);

    // Kernel Segment 1 - Unmapped, Uncached
    const Addr KSeg1End = ULL(0xBFFFFFFF);
    const Addr KSeg1Base = ULL(0xA0000000);
    const Addr KSeg1Mask = ULL(0x1FFFFFFF);

    // Kernel/Supervisor Segment - Mapped
    const Addr KSSegEnd = ULL(0xDFFFFFFF);
    const Addr KSSegBase = ULL(0xC0000000);

    // Kernel Segment 3 - Mapped
    const Addr KSeg3End = ULL(0xFFFFFFFF);
    const Addr KSeg3Base = ULL(0xE0000000);


    // For loading... XXX This maybe could be USegEnd?? --ali
    const Addr LoadAddrMask = ULL(0xffffffffff);

    inline Addr Phys2K0Seg(Addr addr)
    {
   //  if (addr & PAddrUncachedBit43) {
//         addr &= PAddrUncachedMask;
//         addr |= PAddrUncachedBit40;
//     }
        return addr | KSeg0Base;
    }

    ////////////////////////////////////////////////////////////////////////
    //
    //  Interrupt levels
    //
    enum InterruptLevels
    {
        INTLEVEL_SOFTWARE_MIN = 4,
        INTLEVEL_SOFTWARE_MAX = 19,

        INTLEVEL_EXTERNAL_MIN = 20,
        INTLEVEL_EXTERNAL_MAX = 34,

        INTLEVEL_IRQ0 = 20,
        INTLEVEL_IRQ1 = 21,
        INTINDEX_ETHERNET = 0,
        INTINDEX_SCSI = 1,
        INTLEVEL_IRQ2 = 22,
        INTLEVEL_IRQ3 = 23,

        INTLEVEL_SERIAL = 33,

        NumInterruptLevels = INTLEVEL_EXTERNAL_MAX
    };


    // MIPS modes
    enum mode_type
    {
        mode_kernel = 0,		// kernel
        mode_supervisor = 1,	// supervisor
        mode_user = 2,		// user mode
        mode_debug = 3,         // debug mode
        mode_number			// number of modes
    };

  inline mode_type getOperatingMode(MiscReg Stat)
  {
    if((Stat & 0x10000006) != 0 || (Stat & 0x18) ==0)
      return mode_kernel;
    else{
      if((Stat & 0x18) == 0x8)
        return mode_supervisor;
      else if((Stat & 0x18) == 0x10)
        return mode_user;
      else return mode_number;
    }
  }


    // return a no-op instruction... used for instruction fetch faults
    const ExtMachInst NoopMachInst = 0x00000000;

    // Constants Related to the number of registers
    const int NumIntArchRegs = 32;
    const int NumIntSpecialRegs = 9;
    const int NumFloatArchRegs = 32;
    const int NumFloatSpecialRegs = 5;

    const int NumShadowRegSets = 16; // Maximum number of shadow register sets
    const int NumIntRegs = NumIntArchRegs*NumShadowRegSets + NumIntSpecialRegs;        //HI & LO Regs
    const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;//

    // Static instruction parameters
    const int MaxInstSrcRegs = 10;
    const int MaxInstDestRegs = 8;

    // semantically meaningful register indices
    const int ZeroReg = 0;
    const int AssemblerReg = 1;
    const int ReturnValueReg = 2;
    const int ReturnValueReg1 = 2;
    const int ReturnValueReg2 = 3;
    const int ArgumentReg0 = 4;
    const int ArgumentReg1 = 5;
    const int ArgumentReg2 = 6;
    const int ArgumentReg3 = 7;
    const int KernelReg0 = 26;
    const int KernelReg1 = 27;
    const int GlobalPointerReg = 28;
    const int StackPointerReg = 29;
    const int FramePointerReg = 30;
    const int ReturnAddressReg = 31;

    const int ArgumentReg[] = {4, 5, 6, 7};
    const int NumArgumentRegs = sizeof(ArgumentReg) / sizeof(const int);

    const int SyscallNumReg = ReturnValueReg1;
    const int SyscallPseudoReturnReg = ReturnValueReg2;
    const int SyscallSuccessReg = ArgumentReg3;

    const int LogVMPageSize = 13;	// 8K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int MachineBytes = 4;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;

    const int ANNOTE_NONE = 0;
    const uint32_t ITOUCH_ANNOTE = 0xffffffff;

    // These help enumerate all the registers for dependence tracking.
    const int FP_Base_DepTag = NumIntRegs;
    const int Ctrl_Base_DepTag = FP_Base_DepTag + NumFloatRegs;

    // Enumerate names for 'Control' Registers in the CPU
    // Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
    // (Register Number-Register Select) Summary of Register
    //------------------------------------------------------
    // The first set of names classify the CP0 names as Register Banks
    // for easy indexing when using the 'RD + SEL' index combination
    // in CP0 instructions.
    enum MiscRegTags {
        Index = Ctrl_Base_DepTag + 0,       //Bank 0: 0 - 3
        MVPControl,
        MVPConf0,
        MVPConf1,

        CP0_Random = Ctrl_Base_DepTag + 8,      //Bank 1: 8 - 15
        VPEControl,
        VPEConf0,
        VPEConf1,
        YQMask,
        VPESchedule,
        VPEScheFBack,
        VPEOpt,

        EntryLo0 = Ctrl_Base_DepTag + 16,   //Bank 2: 16 - 23
        TCStatus,
        TCBind,
        TCRestart,
        TCHalt,
        TCContext,
        TCSchedule,
        TCScheFBack,

        EntryLo1 = Ctrl_Base_DepTag + 24,   // Bank 3: 24

        Context = Ctrl_Base_DepTag + 32,    // Bank 4: 32 - 33
        ContextConfig,

        PageMask = Ctrl_Base_DepTag + 40, //Bank 5: 40 - 41
        PageGrain = Ctrl_Base_DepTag + 41,

        Wired = Ctrl_Base_DepTag + 48,          //Bank 6:48-55
        SRSConf0,
        SRSConf1,
        SRSConf2,
        SRSConf3,
        SRSConf4,

        HWRena = Ctrl_Base_DepTag + 56,         //Bank 7: 56-63

        BadVAddr = Ctrl_Base_DepTag + 64,       //Bank 8: 64-71

        Count = Ctrl_Base_DepTag + 72,          //Bank 9: 72-79

        EntryHi = Ctrl_Base_DepTag + 80,        //Bank 10: 80-87

        Compare = Ctrl_Base_DepTag + 88,        //Bank 11: 88-95

        Status = Ctrl_Base_DepTag + 96,         //Bank 12: 96-103
        IntCtl,
        SRSCtl,
        SRSMap,

        Cause = Ctrl_Base_DepTag + 104,         //Bank 13: 104-111

        EPC = Ctrl_Base_DepTag + 112,           //Bank 14: 112-119

        PRId = Ctrl_Base_DepTag + 120,          //Bank 15: 120-127,
        EBase,

        Config = Ctrl_Base_DepTag + 128,        //Bank 16: 128-135
        Config1,
        Config2,
        Config3,
        Config4,
        Config5,
        Config6,
        Config7,


        LLAddr = Ctrl_Base_DepTag + 136,        //Bank 17: 136-143

        WatchLo0 = Ctrl_Base_DepTag + 144,      //Bank 18: 144-151
        WatchLo1,
        WatchLo2,
        WatchLo3,
        WatchLo4,
        WatchLo5,
        WatchLo6,
        WatchLo7,

        WatchHi0 = Ctrl_Base_DepTag + 152,     //Bank 19: 152-159
        WatchHi1,
        WatchHi2,
        WatchHi3,
        WatchHi4,
        WatchHi5,
        WatchHi6,
        WatchHi7,

        XCContext64 = Ctrl_Base_DepTag + 160, //Bank 20: 160-167

                           //Bank 21: 168-175

                           //Bank 22: 176-183

        Debug = Ctrl_Base_DepTag + 184,       //Bank 23: 184-191
        TraceControl1,
        TraceControl2,
        UserTraceData,
        TraceBPC,

        DEPC = Ctrl_Base_DepTag + 192,        //Bank 24: 192-199

        PerfCnt0 = Ctrl_Base_DepTag + 200,    //Bank 25: 200-207
        PerfCnt1,
        PerfCnt2,
        PerfCnt3,
        PerfCnt4,
        PerfCnt5,
        PerfCnt6,
        PerfCnt7,

        ErrCtl = Ctrl_Base_DepTag + 208,      //Bank 26: 208-215

        CacheErr0 = Ctrl_Base_DepTag + 216,   //Bank 27: 216-223
        CacheErr1,
        CacheErr2,
        CacheErr3,

        TagLo0 = Ctrl_Base_DepTag + 224,      //Bank 28: 224-231
        DataLo1,
        TagLo2,
        DataLo3,
        TagLo4,
        DataLo5,
        TagLo6,
        DataLo7,

        TagHi0 = Ctrl_Base_DepTag + 232,      //Bank 29: 232-239
        DataHi1,
        TagHi2,
        DataHi3,
        TagHi4,
        DataHi5,
        TagHi6,
        DataHi7,


        ErrorEPC = Ctrl_Base_DepTag + 240,    //Bank 30: 240-247

        DESAVE = Ctrl_Base_DepTag + 248,       //Bank 31: 248-256

        LLFlag = Ctrl_Base_DepTag + 257,

        NumControlRegs
    };

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    const int NumMiscRegs = NumControlRegs;

    const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumMiscRegs;


};

using namespace MipsISA;

#endif // __ARCH_MIPS_ISA_TRAITS_HH__
