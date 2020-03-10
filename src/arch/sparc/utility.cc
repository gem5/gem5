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
 */

#include "arch/sparc/utility.hh"

#include "arch/sparc/faults.hh"
#include "mem/port_proxy.hh"

namespace SparcISA {


// The caller uses %o0-%05 for the first 6 arguments even if their floating
// point. Double precision floating point values take two registers/args.
// Quads, structs, and unions are passed as pointers. All arguments beyond
// the sixth are passed on the stack past the 16 word window save area,
// space for the struct/union return pointer, and space reserved for the
// first 6 arguments which the caller may use but doesn't have to.
uint64_t
getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp)
{
    if (!FullSystem) {
        panic("getArgument() only implemented for full system\n");
        M5_DUMMY_RETURN
    }

    const int NumArgumentRegs = 6;
    if (number < NumArgumentRegs) {
        return tc->readIntReg(8 + number);
    } else {
        Addr sp = tc->readIntReg(StackPointerReg);
        PortProxy &vp = tc->getVirtProxy();
        uint64_t arg = vp.read<uint64_t>(sp + 92 +
                            (number-NumArgumentRegs) * sizeof(uint64_t));
        return arg;
    }
}

void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{

    uint8_t tl = src->readMiscRegNoEffect(MISCREG_TL);

    // Read all the trap level dependent registers and save them off
    for (int i = 1; i <= MaxTL; i++) {
        src->setMiscRegNoEffect(MISCREG_TL, i);
        dest->setMiscRegNoEffect(MISCREG_TL, i);

        dest->setMiscRegNoEffect(MISCREG_TT,
                src->readMiscRegNoEffect(MISCREG_TT));
        dest->setMiscRegNoEffect(MISCREG_TPC,
                src->readMiscRegNoEffect(MISCREG_TPC));
        dest->setMiscRegNoEffect(MISCREG_TNPC,
                src->readMiscRegNoEffect(MISCREG_TNPC));
        dest->setMiscRegNoEffect(MISCREG_TSTATE,
                src->readMiscRegNoEffect(MISCREG_TSTATE));
    }

    // Save off the traplevel
    dest->setMiscRegNoEffect(MISCREG_TL, tl);
    src->setMiscRegNoEffect(MISCREG_TL, tl);


    // ASRs
//    dest->setMiscRegNoEffect(MISCREG_Y,
//            src->readMiscRegNoEffect(MISCREG_Y));
//    dest->setMiscRegNoEffect(MISCREG_CCR,
//            src->readMiscRegNoEffect(MISCREG_CCR));
    dest->setMiscReg(MISCREG_ASI,
            src->readMiscRegNoEffect(MISCREG_ASI));
    dest->setMiscRegNoEffect(MISCREG_TICK,
            src->readMiscRegNoEffect(MISCREG_TICK));
    dest->setMiscRegNoEffect(MISCREG_FPRS,
            src->readMiscRegNoEffect(MISCREG_FPRS));
    dest->setMiscRegNoEffect(MISCREG_SOFTINT,
            src->readMiscRegNoEffect(MISCREG_SOFTINT));
    dest->setMiscRegNoEffect(MISCREG_TICK_CMPR,
            src->readMiscRegNoEffect(MISCREG_TICK_CMPR));
    dest->setMiscRegNoEffect(MISCREG_STICK,
            src->readMiscRegNoEffect(MISCREG_STICK));
    dest->setMiscRegNoEffect(MISCREG_STICK_CMPR,
            src->readMiscRegNoEffect(MISCREG_STICK_CMPR));

    // Priv Registers
    dest->setMiscRegNoEffect(MISCREG_TICK,
            src->readMiscRegNoEffect(MISCREG_TICK));
    dest->setMiscRegNoEffect(MISCREG_TBA,
            src->readMiscRegNoEffect(MISCREG_TBA));
    dest->setMiscRegNoEffect(MISCREG_PSTATE,
            src->readMiscRegNoEffect(MISCREG_PSTATE));
    dest->setMiscRegNoEffect(MISCREG_PIL,
            src->readMiscRegNoEffect(MISCREG_PIL));
    dest->setMiscReg(MISCREG_CWP,
            src->readMiscRegNoEffect(MISCREG_CWP));
//    dest->setMiscRegNoEffect(MISCREG_CANSAVE,
//            src->readMiscRegNoEffect(MISCREG_CANSAVE));
//    dest->setMiscRegNoEffect(MISCREG_CANRESTORE,
//            src->readMiscRegNoEffect(MISCREG_CANRESTORE));
//    dest->setMiscRegNoEffect(MISCREG_OTHERWIN,
//            src->readMiscRegNoEffect(MISCREG_OTHERWIN));
//    dest->setMiscRegNoEffect(MISCREG_CLEANWIN,
//            src->readMiscRegNoEffect(MISCREG_CLEANWIN));
//    dest->setMiscRegNoEffect(MISCREG_WSTATE,
//            src->readMiscRegNoEffect(MISCREG_WSTATE));
    dest->setMiscReg(MISCREG_GL, src->readMiscRegNoEffect(MISCREG_GL));

    // Hyperprivilged registers
    dest->setMiscRegNoEffect(MISCREG_HPSTATE,
            src->readMiscRegNoEffect(MISCREG_HPSTATE));
    dest->setMiscRegNoEffect(MISCREG_HINTP,
            src->readMiscRegNoEffect(MISCREG_HINTP));
    dest->setMiscRegNoEffect(MISCREG_HTBA,
            src->readMiscRegNoEffect(MISCREG_HTBA));
    dest->setMiscRegNoEffect(MISCREG_STRAND_STS_REG,
            src->readMiscRegNoEffect(MISCREG_STRAND_STS_REG));
    dest->setMiscRegNoEffect(MISCREG_HSTICK_CMPR,
            src->readMiscRegNoEffect(MISCREG_HSTICK_CMPR));

    // FSR
    dest->setMiscRegNoEffect(MISCREG_FSR,
            src->readMiscRegNoEffect(MISCREG_FSR));

    // Strand Status Register
    dest->setMiscRegNoEffect(MISCREG_STRAND_STS_REG,
            src->readMiscRegNoEffect(MISCREG_STRAND_STS_REG));

    // MMU Registers
    dest->setMiscRegNoEffect(MISCREG_MMU_P_CONTEXT,
            src->readMiscRegNoEffect(MISCREG_MMU_P_CONTEXT));
    dest->setMiscRegNoEffect(MISCREG_MMU_S_CONTEXT,
            src->readMiscRegNoEffect(MISCREG_MMU_S_CONTEXT));
    dest->setMiscRegNoEffect(MISCREG_MMU_PART_ID,
            src->readMiscRegNoEffect(MISCREG_MMU_PART_ID));
    dest->setMiscRegNoEffect(MISCREG_MMU_LSU_CTRL,
            src->readMiscRegNoEffect(MISCREG_MMU_LSU_CTRL));

    // Scratchpad Registers
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R0,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R0));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R1,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R1));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R2,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R2));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R3,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R3));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R4,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R4));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R5,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R5));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R6,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R6));
    dest->setMiscRegNoEffect(MISCREG_SCRATCHPAD_R7,
            src->readMiscRegNoEffect(MISCREG_SCRATCHPAD_R7));

    // Queue Registers
    dest->setMiscRegNoEffect(MISCREG_QUEUE_CPU_MONDO_HEAD,
            src->readMiscRegNoEffect(MISCREG_QUEUE_CPU_MONDO_HEAD));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_CPU_MONDO_TAIL,
            src->readMiscRegNoEffect(MISCREG_QUEUE_CPU_MONDO_TAIL));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_DEV_MONDO_HEAD,
            src->readMiscRegNoEffect(MISCREG_QUEUE_DEV_MONDO_HEAD));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_DEV_MONDO_TAIL,
            src->readMiscRegNoEffect(MISCREG_QUEUE_DEV_MONDO_TAIL));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_RES_ERROR_HEAD,
            src->readMiscRegNoEffect(MISCREG_QUEUE_RES_ERROR_HEAD));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_RES_ERROR_TAIL,
            src->readMiscRegNoEffect(MISCREG_QUEUE_RES_ERROR_TAIL));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_NRES_ERROR_HEAD,
            src->readMiscRegNoEffect(MISCREG_QUEUE_NRES_ERROR_HEAD));
    dest->setMiscRegNoEffect(MISCREG_QUEUE_NRES_ERROR_TAIL,
            src->readMiscRegNoEffect(MISCREG_QUEUE_NRES_ERROR_TAIL));
}

void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    // First loop through the integer registers.
    int old_gl = src->readMiscRegNoEffect(MISCREG_GL);
    int old_cwp = src->readMiscRegNoEffect(MISCREG_CWP);
    // Globals
    for (int x = 0; x < MaxGL; ++x) {
        src->setMiscReg(MISCREG_GL, x);
        dest->setMiscReg(MISCREG_GL, x);
        // Skip %g0 which is always zero.
        for (int y = 1; y < 8; y++)
            dest->setIntReg(y, src->readIntReg(y));
    }
    // Locals and ins. Outs are all also ins.
    for (int x = 0; x < NWindows; ++x) {
         src->setMiscReg(MISCREG_CWP, x);
         dest->setMiscReg(MISCREG_CWP, x);
         for (int y = 16; y < 32; y++)
             dest->setIntReg(y, src->readIntReg(y));
    }
    // Microcode reg and pseudo int regs (misc regs in the integer regfile).
    for (int y = NumIntArchRegs; y < NumIntArchRegs + NumMicroIntRegs; ++y)
        dest->setIntReg(y, src->readIntReg(y));

    // Restore src's GL, CWP
    src->setMiscReg(MISCREG_GL, old_gl);
    src->setMiscReg(MISCREG_CWP, old_cwp);


    // Then loop through the floating point registers.
    for (int i = 0; i < SparcISA::NumFloatArchRegs; ++i) {
        dest->setFloatReg(i, src->readFloatReg(i));
    }

    // Would need to add condition-code regs if implemented
    assert(NumCCRegs == 0);

    // Copy misc. registers
    copyMiscRegs(src, dest);

    // Lastly copy PC/NPC
    dest->pcState(src->pcState());
}

} // namespace SPARC_ISA
