/*
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
 * Authors: Gabe Black
 *          Ali Saidi
 */

#ifndef __ARCH_SPARC_MISCREGFILE_HH__
#define __ARCH_SPARC_MISCREGFILE_HH__

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/types.hh"
#include "cpu/cpuevent.hh"

#include <string>

class Checkpoint;

namespace SparcISA
{
    //These functions map register indices to names
    std::string getMiscRegName(RegIndex);

    enum MiscRegIndex
    {
        /** Ancillary State Registers */
//        MISCREG_Y,
//        MISCREG_CCR,
        MISCREG_ASI,
        MISCREG_TICK,
        MISCREG_FPRS,
        MISCREG_PCR,
        MISCREG_PIC,
        MISCREG_GSR,
        MISCREG_SOFTINT_SET,
        MISCREG_SOFTINT_CLR,
        MISCREG_SOFTINT, /* 10 */
        MISCREG_TICK_CMPR,
        MISCREG_STICK,
        MISCREG_STICK_CMPR,

        /** Privilged Registers */
        MISCREG_TPC,
        MISCREG_TNPC,
        MISCREG_TSTATE,
        MISCREG_TT,
        MISCREG_PRIVTICK,
        MISCREG_TBA,
        MISCREG_PSTATE, /* 20 */
        MISCREG_TL,
        MISCREG_PIL,
        MISCREG_CWP,
//        MISCREG_CANSAVE,
//        MISCREG_CANRESTORE,
//        MISCREG_CLEANWIN,
//        MISCREG_OTHERWIN,
//        MISCREG_WSTATE,
        MISCREG_GL,

        /** Hyper privileged registers */
        MISCREG_HPSTATE, /* 30 */
        MISCREG_HTSTATE,
        MISCREG_HINTP,
        MISCREG_HTBA,
        MISCREG_HVER,
        MISCREG_STRAND_STS_REG,
        MISCREG_HSTICK_CMPR,

        /** Floating Point Status Register */
        MISCREG_FSR,

        /** MMU Internal Registers */
        MISCREG_MMU_P_CONTEXT,
        MISCREG_MMU_S_CONTEXT, /* 40 */
        MISCREG_MMU_PART_ID,
        MISCREG_MMU_LSU_CTRL,

        MISCREG_MMU_ITLB_C0_TSB_PS0,
        MISCREG_MMU_ITLB_C0_TSB_PS1,
        MISCREG_MMU_ITLB_C0_CONFIG,
        MISCREG_MMU_ITLB_CX_TSB_PS0,
        MISCREG_MMU_ITLB_CX_TSB_PS1,
        MISCREG_MMU_ITLB_CX_CONFIG,
        MISCREG_MMU_ITLB_SFSR,
        MISCREG_MMU_ITLB_TAG_ACCESS, /* 50 */

        MISCREG_MMU_DTLB_C0_TSB_PS0,
        MISCREG_MMU_DTLB_C0_TSB_PS1,
        MISCREG_MMU_DTLB_C0_CONFIG,
        MISCREG_MMU_DTLB_CX_TSB_PS0,
        MISCREG_MMU_DTLB_CX_TSB_PS1,
        MISCREG_MMU_DTLB_CX_CONFIG,
        MISCREG_MMU_DTLB_SFSR,
        MISCREG_MMU_DTLB_SFAR,
        MISCREG_MMU_DTLB_TAG_ACCESS,

        /** Scratchpad regiscers **/
        MISCREG_SCRATCHPAD_R0, /* 60 */
        MISCREG_SCRATCHPAD_R1,
        MISCREG_SCRATCHPAD_R2,
        MISCREG_SCRATCHPAD_R3,
        MISCREG_SCRATCHPAD_R4,
        MISCREG_SCRATCHPAD_R5,
        MISCREG_SCRATCHPAD_R6,
        MISCREG_SCRATCHPAD_R7,

        /* CPU Queue Registers */
        MISCREG_QUEUE_CPU_MONDO_HEAD,
        MISCREG_QUEUE_CPU_MONDO_TAIL,
        MISCREG_QUEUE_DEV_MONDO_HEAD, /* 70 */
        MISCREG_QUEUE_DEV_MONDO_TAIL,
        MISCREG_QUEUE_RES_ERROR_HEAD,
        MISCREG_QUEUE_RES_ERROR_TAIL,
        MISCREG_QUEUE_NRES_ERROR_HEAD,
        MISCREG_QUEUE_NRES_ERROR_TAIL,

        /* All the data for the TLB packed up in one register. */
        MISCREG_TLB_DATA,
        MISCREG_NUMMISCREGS
    };

    struct HPSTATE {
        const static uint64_t id = 0x800;   // this impl. dependent (id) field m
        const static uint64_t ibe = 0x400;
        const static uint64_t red = 0x20;
        const static uint64_t hpriv = 0x4;
        const static uint64_t tlz = 0x1;
    };


    struct PSTATE {
        const static int cle = 0x200;
        const static int tle = 0x100;
        const static int mm = 0xC0;
        const static int pef = 0x10;
        const static int am = 0x8;
        const static int priv = 0x4;
        const static int ie = 0x2;
    };


    const int NumMiscArchRegs = MISCREG_NUMMISCREGS;
    const int NumMiscRegs = MISCREG_NUMMISCREGS;

    // The control registers, broken out into fields
    class MiscRegFile
    {
      private:

        /* ASR Registers */
        //uint64_t y;		// Y (used in obsolete multiplication)
        //uint8_t ccr;		// Condition Code Register
        uint8_t asi;		// Address Space Identifier
        uint64_t tick;		// Hardware clock-tick counter
        uint8_t	fprs;		// Floating-Point Register State
        uint64_t gsr;		// General Status Register
        uint64_t softint;
        uint64_t tick_cmpr;	// Hardware tick compare registers
        uint64_t stick;		// Hardware clock-tick counter
        uint64_t stick_cmpr;	// Hardware tick compare registers


        /* Privileged Registers */
        uint64_t tpc[MaxTL];	// Trap Program Counter (value from
                                // previous trap level)
        uint64_t tnpc[MaxTL];	// Trap Next Program Counter (value from
                                // previous trap level)
        uint64_t tstate[MaxTL];	// Trap State
        uint16_t tt[MaxTL];	// Trap Type (Type of trap which occured
                                // on the previous level)
        uint64_t tba;		// Trap Base Address

        uint16_t pstate;	// Process State Register
        uint8_t tl;		// Trap Level
        uint8_t pil;		// Process Interrupt Register
        uint8_t cwp;		// Current Window Pointer
        //uint8_t cansave;	// Savable windows
        //uint8_t canrestore;	// Restorable windows
        //uint8_t cleanwin;	// Clean windows
        //uint8_t otherwin;	// Other windows
        //uint8_t wstate;		// Window State
        uint8_t gl;             // Global level register

        /** Hyperprivileged Registers */
        uint64_t hpstate;	// Hyperprivileged State Register
        uint64_t htstate[MaxTL];// Hyperprivileged Trap State Register
        uint64_t hintp;
        uint64_t htba;		// Hyperprivileged Trap Base Address register
        uint64_t hstick_cmpr;	// Hardware tick compare registers

        uint64_t strandStatusReg;// Per strand status register

        /** Floating point misc registers. */
        uint64_t fsr;		// Floating-Point State Register

        /** MMU Internal Registers */
        uint16_t priContext;
        uint16_t secContext;
        uint16_t partId;
        uint64_t lsuCtrlReg;

        uint64_t iTlbC0TsbPs0;
        uint64_t iTlbC0TsbPs1;
        uint64_t iTlbC0Config;
        uint64_t iTlbCXTsbPs0;
        uint64_t iTlbCXTsbPs1;
        uint64_t iTlbCXConfig;
        uint64_t iTlbSfsr;
        uint64_t iTlbTagAccess;

        uint64_t dTlbC0TsbPs0;
        uint64_t dTlbC0TsbPs1;
        uint64_t dTlbC0Config;
        uint64_t dTlbCXTsbPs0;
        uint64_t dTlbCXTsbPs1;
        uint64_t dTlbCXConfig;
        uint64_t dTlbSfsr;
        uint64_t dTlbSfar;
        uint64_t dTlbTagAccess;

        uint64_t scratchPad[8];

        uint64_t cpu_mondo_head;
        uint64_t cpu_mondo_tail;
        uint64_t dev_mondo_head;
        uint64_t dev_mondo_tail;
        uint64_t res_error_head;
        uint64_t res_error_tail;
        uint64_t nres_error_head;
        uint64_t nres_error_tail;

        // These need to check the int_dis field and if 0 then
        // set appropriate bit in softint and checkinterrutps on the cpu
#if FULL_SYSTEM
        void  setFSReg(int miscReg, const MiscReg &val, ThreadContext *tc);
        MiscReg readFSReg(int miscReg, ThreadContext * tc);

        // Update interrupt state on softint or pil change
        void checkSoftInt(ThreadContext *tc);

        /** Process a tick compare event and generate an interrupt on the cpu if
         * appropriate. */
        void processTickCompare(ThreadContext *tc);
        void processSTickCompare(ThreadContext *tc);
        void processHSTickCompare(ThreadContext *tc);

        typedef CpuEventWrapper<MiscRegFile,
                &MiscRegFile::processTickCompare> TickCompareEvent;
        TickCompareEvent *tickCompare;

        typedef CpuEventWrapper<MiscRegFile,
                &MiscRegFile::processSTickCompare> STickCompareEvent;
        STickCompareEvent *sTickCompare;

        typedef CpuEventWrapper<MiscRegFile,
                &MiscRegFile::processHSTickCompare> HSTickCompareEvent;
        HSTickCompareEvent *hSTickCompare;
#endif
      public:

        void clear();

        MiscRegFile()
        {
            clear();
        }

        MiscReg readRegNoEffect(int miscReg);

        MiscReg readReg(int miscReg, ThreadContext *tc);

        void setRegNoEffect(int miscReg, const MiscReg &val);

        void setReg(int miscReg,
                const MiscReg &val, ThreadContext * tc);

        int getInstAsid()
        {
            return priContext | (uint32_t)partId << 13;
        }

        int getDataAsid()
        {
            return priContext | (uint32_t)partId << 13;
        }

        void serialize(std::ostream & os);

        void unserialize(Checkpoint * cp, const std::string & section);

        void copyMiscRegs(ThreadContext * tc);

      protected:

        bool isHyperPriv() { return (hpstate & (1 << 2)); }
        bool isPriv() { return (hpstate & (1 << 2)) || (pstate & (1 << 2)); }
        bool isNonPriv() { return !isPriv(); }
    };
}

#endif
