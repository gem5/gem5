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
 * Authors: Korey L. Sewell
 */

#ifndef __ARCH_MIPS_UTILITY_HH__
#define __ARCH_MIPS_UTILITY_HH__
#include "config/full_system.hh"
#include "arch/mips/types.hh"
#include "arch/mips/isa_traits.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
//XXX This is needed for size_t. We should use something other than size_t
//#include "kern/linux/linux.hh"
#include "sim/host.hh"

#include "cpu/thread_context.hh"

class ThreadContext;

namespace MipsISA {

    uint64_t getArgument(ThreadContext *tc, int number, bool fp);

    //Floating Point Utility Functions
    uint64_t fpConvert(ConvertType cvt_type, double fp_val);
    double roundFP(double val, int digits);
    double truncFP(double val);

    bool getCondCode(uint32_t fcsr, int cc);
    uint32_t genCCVector(uint32_t fcsr, int num, uint32_t cc_val);
    uint32_t genInvalidVector(uint32_t fcsr);

    bool isNan(void *val_ptr, int size);
    bool isQnan(void *val_ptr, int size);
    bool isSnan(void *val_ptr, int size);

    void startupCPU(ThreadContext *tc, int cpuId);

    static inline bool
    inUserMode(ThreadContext *tc)
    {
        MiscReg Stat = tc->readMiscReg(MipsISA::Status);
        MiscReg Dbg = tc->readMiscReg(MipsISA::Debug);

        if((Stat & 0x10000006) == 0  // EXL, ERL or CU0 set, CP0 accessible
           && (Dbg & 0x40000000) == 0 // DM bit set, CP0 accessible
           && (Stat & 0x00000018) != 0) {  // KSU = 0, kernel mode is base mode
            // Unable to use Status_CU0, etc directly, using bitfields & masks
            return true;
        } else {
            return false;
        }
    }

    // Instruction address compression hooks
    static inline Addr realPCToFetchPC(const Addr &addr) {
        return addr;
    }

    static inline Addr fetchPCToRealPC(const Addr &addr) {
        return addr;
    }

    // the size of "fetched" instructions (not necessarily the size
    // of real instructions for PISA)
    static inline size_t fetchInstSize() {
        return sizeof(MachInst);
    }

    static inline MachInst makeRegisterCopy(int dest, int src) {
        panic("makeRegisterCopy not implemented");
        return 0;
    }

    static inline int flattenFloatIndex(ThreadContext * tc, int reg)
    {
        return reg;
    }

    int flattenIntIndex(ThreadContext * tc, int reg);

    void copyRegs(ThreadContext *src, ThreadContext *dest);

    void copyMiscRegs(ThreadContext *src, ThreadContext *dest);


    template <class CPU>
    void zeroRegisters(CPU *cpu);

    ////////////////////////////////////////////////////////////////////////
    //
    //  Translation stuff
    //

    inline Addr PteAddr(Addr a) { return (a & PteMask) << PteShift; }

    // User Virtual
    inline bool IsUSeg(Addr a) { return USegBase <= a && a <= USegEnd; }

    inline bool IsKSeg0(Addr a) { return KSeg0Base <= a && a <= KSeg0End; }

    inline Addr KSeg02Phys(Addr addr) { return addr & KSeg0Mask; }

    inline Addr KSeg12Phys(Addr addr) { return addr & KSeg1Mask; }

    inline bool IsKSeg1(Addr a) { return KSeg1Base <= a && a <= KSeg1End; }

    inline bool IsKSSeg(Addr a) { return KSSegBase <= a && a <= KSSegEnd; }

    inline bool IsKSeg3(Addr a) { return KSeg3Base <= a && a <= KSeg3End; }

    inline Addr
    TruncPage(Addr addr)
    { return addr & ~(PageBytes - 1); }

    inline Addr
    RoundPage(Addr addr)
    { return (addr + PageBytes - 1) & ~(PageBytes - 1); }

    void initCPU(ThreadContext *tc, int cpuId);
    void initIPRs(ThreadContext *tc, int cpuId);

    /**
     * Function to check for and process any interrupts.
     * @param tc The thread context.
     */
    template <class TC>
    void processInterrupts(TC *tc);

};


#endif
