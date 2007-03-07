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

#include "arch/sparc/regfile.hh"
#include "cpu/thread_context.hh"

class Checkpoint;

using namespace SparcISA;
using namespace std;

//RegFile class methods
Addr RegFile::readPC()
{
    return pc;
}

void RegFile::setPC(Addr val)
{
    pc = val;
}

Addr RegFile::readNextPC()
{
    return npc;
}

void RegFile::setNextPC(Addr val)
{
    npc = val;
}

Addr RegFile::readNextNPC()
{
    return nnpc;
}

void RegFile::setNextNPC(Addr val)
{
    nnpc = val;
}

void RegFile::clear()
{
    floatRegFile.clear();
    intRegFile.clear();
    miscRegFile.clear();
}

MiscReg RegFile::readMiscRegNoEffect(int miscReg)
{
    return miscRegFile.readRegNoEffect(miscReg);
}

MiscReg RegFile::readMiscReg(int miscReg, ThreadContext *tc)
{
    return miscRegFile.readReg(miscReg, tc);
}

void RegFile::setMiscRegNoEffect(int miscReg, const MiscReg &val)
{
    miscRegFile.setRegNoEffect(miscReg, val);
}

void RegFile::setMiscReg(int miscReg, const MiscReg &val,
        ThreadContext * tc)
{
    miscRegFile.setReg(miscReg, val, tc);
}

FloatReg RegFile::readFloatReg(int floatReg, int width)
{
    return floatRegFile.readReg(floatReg, width);
}

FloatReg RegFile::readFloatReg(int floatReg)
{
    //Use the "natural" width of a single float
    return floatRegFile.readReg(floatReg, FloatRegFile::SingleWidth);
}

FloatRegBits RegFile::readFloatRegBits(int floatReg, int width)
{
    return floatRegFile.readRegBits(floatReg, width);
}

FloatRegBits RegFile::readFloatRegBits(int floatReg)
{
    //Use the "natural" width of a single float
    return floatRegFile.readRegBits(floatReg,
            FloatRegFile::SingleWidth);
}

void RegFile::setFloatReg(int floatReg, const FloatReg &val, int width)
{
    floatRegFile.setReg(floatReg, val, width);
}

void RegFile::setFloatReg(int floatReg, const FloatReg &val)
{
    //Use the "natural" width of a single float
    setFloatReg(floatReg, val, FloatRegFile::SingleWidth);
}

void RegFile::setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
{
    floatRegFile.setRegBits(floatReg, val, width);
}

void RegFile::setFloatRegBits(int floatReg, const FloatRegBits &val)
{
    //Use the "natural" width of a single float
    floatRegFile.setRegBits(floatReg, val, FloatRegFile::SingleWidth);
}

IntReg RegFile::readIntReg(int intReg)
{
    return intRegFile.readReg(intReg);
}

void RegFile::setIntReg(int intReg, const IntReg &val)
{
    intRegFile.setReg(intReg, val);
}

int SparcISA::flattenIntIndex(ThreadContext * tc, int reg)
{
    int gl = tc->readMiscRegNoEffect(MISCREG_GL);
    int cwp = tc->readMiscRegNoEffect(MISCREG_CWP);
    //DPRINTF(Sparc, "Global Level = %d, Current Window Pointer = %d\n", gl, cwp);
    int newReg;
    //The total number of global registers
    int numGlobals = (MaxGL + 1) * 8;
    if(reg < 8)
    {
        //Global register
        //Put it in the appropriate set of globals
        newReg = reg + gl * 8;
    }
    else if(reg < NumIntArchRegs)
    {
        //Regular windowed register
        //Put it in the window pointed to by cwp
        newReg = numGlobals +
            ((reg - 8 - cwp * 16 + NWindows * 16) % (NWindows * 16));
    }
    else if(reg < NumIntArchRegs + NumMicroIntRegs)
    {
        //Microcode register
        //Displace from the end of the regular registers
        newReg = reg - NumIntArchRegs + numGlobals + NWindows * 16;
    }
    else if(reg < 2 * NumIntArchRegs + NumMicroIntRegs)
    {
        reg -= (NumIntArchRegs + NumMicroIntRegs);
        if(reg < 8)
        {
            //Global register from the next window
            //Put it in the appropriate set of globals
            newReg = reg + gl * 8;
        }
        else
        {
            //Windowed register from the previous window
            //Put it in the window before the one pointed to by cwp
            newReg = numGlobals +
                ((reg - 8 - (cwp - 1) * 16 + NWindows * 16) % (NWindows * 16));
        }
    }
    else if(reg < 3 * NumIntArchRegs + NumMicroIntRegs)
    {
        reg -= (2 * NumIntArchRegs + NumMicroIntRegs);
        if(reg < 8)
        {
            //Global register from the previous window
            //Put it in the appropriate set of globals
            newReg = reg + gl * 8;
        }
        else
        {
            //Windowed register from the next window
            //Put it in the window after the one pointed to by cwp
            newReg = numGlobals +
                ((reg - 8 - (cwp + 1) * 16 + NWindows * 16) % (NWindows * 16));
        }
    }
    else
        panic("Tried to flatten invalid register index %d!\n", reg);
    DPRINTF(Sparc, "Flattened register %d to %d.\n", reg, newReg);
    return newReg;
    //return intRegFile.flattenIndex(reg);
}

void RegFile::serialize(std::ostream &os)
{
    intRegFile.serialize(os);
    floatRegFile.serialize(os);
    miscRegFile.serialize(os);
    SERIALIZE_SCALAR(pc);
    SERIALIZE_SCALAR(npc);
    SERIALIZE_SCALAR(nnpc);
}

void RegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    intRegFile.unserialize(cp, section);
    floatRegFile.unserialize(cp, section);
    miscRegFile.unserialize(cp, section);
    UNSERIALIZE_SCALAR(pc);
    UNSERIALIZE_SCALAR(npc);
    UNSERIALIZE_SCALAR(nnpc);
}

void RegFile::changeContext(RegContextParam param, RegContextVal val)
{
    switch(param)
    {
      case CONTEXT_CWP:
        intRegFile.setCWP(val);
        break;
      case CONTEXT_GLOBALS:
        intRegFile.setGlobals(val);
        break;
      default:
        panic("Tried to set illegal context parameter in the SPARC regfile.\n");
    }
}

void SparcISA::copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{

    uint8_t tl = src->readMiscRegNoEffect(MISCREG_TL);

    // Read all the trap level dependent registers and save them off
    for(int i = 1; i <= MaxTL; i++)
    {
        src->setMiscRegNoEffect(MISCREG_TL, i);
        dest->setMiscRegNoEffect(MISCREG_TL, i);

        dest->setMiscRegNoEffect(MISCREG_TT, src->readMiscRegNoEffect(MISCREG_TT));
        dest->setMiscRegNoEffect(MISCREG_TPC, src->readMiscRegNoEffect(MISCREG_TPC));
        dest->setMiscRegNoEffect(MISCREG_TNPC, src->readMiscRegNoEffect(MISCREG_TNPC));
        dest->setMiscRegNoEffect(MISCREG_TSTATE, src->readMiscRegNoEffect(MISCREG_TSTATE));
    }

    // Save off the traplevel
    dest->setMiscRegNoEffect(MISCREG_TL, tl);
    src->setMiscRegNoEffect(MISCREG_TL, tl);


    // ASRs
//    dest->setMiscRegNoEffect(MISCREG_Y, src->readMiscRegNoEffect(MISCREG_Y));
//    dest->setMiscRegNoEffect(MISCREG_CCR, src->readMiscRegNoEffect(MISCREG_CCR));
    dest->setMiscRegNoEffect(MISCREG_ASI, src->readMiscRegNoEffect(MISCREG_ASI));
    dest->setMiscRegNoEffect(MISCREG_TICK, src->readMiscRegNoEffect(MISCREG_TICK));
    dest->setMiscRegNoEffect(MISCREG_FPRS, src->readMiscRegNoEffect(MISCREG_FPRS));
    dest->setMiscRegNoEffect(MISCREG_SOFTINT, src->readMiscRegNoEffect(MISCREG_SOFTINT));
    dest->setMiscRegNoEffect(MISCREG_TICK_CMPR, src->readMiscRegNoEffect(MISCREG_TICK_CMPR));
    dest->setMiscRegNoEffect(MISCREG_STICK, src->readMiscRegNoEffect(MISCREG_STICK));
    dest->setMiscRegNoEffect(MISCREG_STICK_CMPR, src->readMiscRegNoEffect(MISCREG_STICK_CMPR));

    // Priv Registers
    dest->setMiscRegNoEffect(MISCREG_TICK, src->readMiscRegNoEffect(MISCREG_TICK));
    dest->setMiscRegNoEffect(MISCREG_TBA, src->readMiscRegNoEffect(MISCREG_TBA));
    dest->setMiscRegNoEffect(MISCREG_PSTATE, src->readMiscRegNoEffect(MISCREG_PSTATE));
    dest->setMiscRegNoEffect(MISCREG_PIL, src->readMiscRegNoEffect(MISCREG_PIL));
    dest->setMiscRegNoEffect(MISCREG_CWP, src->readMiscRegNoEffect(MISCREG_CWP));
//    dest->setMiscRegNoEffect(MISCREG_CANSAVE, src->readMiscRegNoEffect(MISCREG_CANSAVE));
//    dest->setMiscRegNoEffect(MISCREG_CANRESTORE, src->readMiscRegNoEffect(MISCREG_CANRESTORE));
//    dest->setMiscRegNoEffect(MISCREG_OTHERWIN, src->readMiscRegNoEffect(MISCREG_OTHERWIN));
//    dest->setMiscRegNoEffect(MISCREG_CLEANWIN, src->readMiscRegNoEffect(MISCREG_CLEANWIN));
//    dest->setMiscRegNoEffect(MISCREG_WSTATE, src->readMiscRegNoEffect(MISCREG_WSTATE));
    dest->setMiscRegNoEffect(MISCREG_GL, src->readMiscRegNoEffect(MISCREG_GL));

    // Hyperprivilged registers
    dest->setMiscRegNoEffect(MISCREG_HPSTATE, src->readMiscRegNoEffect(MISCREG_HPSTATE));
    dest->setMiscRegNoEffect(MISCREG_HINTP, src->readMiscRegNoEffect(MISCREG_HINTP));
    dest->setMiscRegNoEffect(MISCREG_HTBA, src->readMiscRegNoEffect(MISCREG_HTBA));
    dest->setMiscRegNoEffect(MISCREG_STRAND_STS_REG,
            src->readMiscRegNoEffect(MISCREG_STRAND_STS_REG));
    dest->setMiscRegNoEffect(MISCREG_HSTICK_CMPR,
            src->readMiscRegNoEffect(MISCREG_HSTICK_CMPR));

    // FSR
    dest->setMiscRegNoEffect(MISCREG_FSR, src->readMiscRegNoEffect(MISCREG_FSR));

    //Strand Status Register
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

    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_C0_TSB_PS0,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_C0_TSB_PS0));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_C0_TSB_PS1,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_C0_TSB_PS1));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_C0_CONFIG,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_C0_CONFIG));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_CX_TSB_PS0,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_CX_TSB_PS0));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_CX_TSB_PS1,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_CX_TSB_PS1));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_CX_CONFIG,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_CX_CONFIG));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_SFSR,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_SFSR));
    dest->setMiscRegNoEffect(MISCREG_MMU_ITLB_TAG_ACCESS,
            src->readMiscRegNoEffect(MISCREG_MMU_ITLB_TAG_ACCESS));

    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_C0_TSB_PS0,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_C0_TSB_PS0));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_C0_TSB_PS1,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_C0_TSB_PS1));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_C0_CONFIG,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_C0_CONFIG));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_CX_TSB_PS0,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_CX_TSB_PS0));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_CX_TSB_PS1,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_CX_TSB_PS1));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_CX_CONFIG,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_CX_CONFIG));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_SFSR,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_SFSR));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_SFAR,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_SFAR));
    dest->setMiscRegNoEffect(MISCREG_MMU_DTLB_TAG_ACCESS,
            src->readMiscRegNoEffect(MISCREG_MMU_DTLB_TAG_ACCESS));

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

void SparcISA::copyRegs(ThreadContext *src, ThreadContext *dest)
{
    // First loop through the integer registers.
    for (int i = 0; i < TheISA::NumIntRegs; ++i) {
        dest->setIntReg(i, src->readIntReg(i));
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < TheISA::NumFloatRegs; ++i) {
        dest->setFloatRegBits(i, src->readFloatRegBits(i));
    }

    // Copy misc. registers
    copyMiscRegs(src, dest);

    // Lastly copy PC/NPC
    dest->setPC(src->readPC());
    dest->setNextPC(src->readNextPC());
    dest->setNextNPC(src->readNextNPC());
}
