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

#include "arch/sparc/miscregfile.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"

using namespace SparcISA;
using namespace std;

class Checkpoint;

//These functions map register indices to names
string SparcISA::getMiscRegName(RegIndex index)
{
    static::string miscRegName[NumMiscRegs] =
        {"y", "ccr", "asi", "tick", "pc", "fprs", "pcr", "pic",
         "gsr", "softint_set", "softint_clr", "softint", "tick_cmpr",
         "stick", "stick_cmpr",
         "tpc", "tnpc", "tstate", "tt", "privtick", "tba", "pstate", "tl",
         "pil", "cwp", "cansave", "canrestore", "cleanwin", "otherwin",
         "wstate", "gl",
         "hpstate", "htstate", "hintp", "htba", "hver", "strand_sts_reg",
         "hstick_cmpr",
         "fsr"};
    return miscRegName[index];
}

#if FULL_SYSTEM

//XXX These need an implementation someplace
/** Fullsystem only register version of ReadRegWithEffect() */
MiscReg MiscRegFile::readFSRegWithEffect(int miscReg, Fault &fault, ThreadContext *tc);
/** Fullsystem only register version of SetRegWithEffect() */
Fault MiscRegFile::setFSRegWithEffect(int miscReg, const MiscReg &val,
        ThreadContext * tc);
#endif

void MiscRegFile::reset()
{
    pstateFields.pef = 0; //No FPU
    //pstateFields.pef = 1; //FPU
#if FULL_SYSTEM
    //For SPARC, when a system is first started, there is a power
    //on reset Trap which sets the processor into the following state.
    //Bits that aren't set aren't defined on startup.
    tl = MaxTL;
    gl = MaxGL;

    tickFields.counter = 0; //The TICK register is unreadable bya
    tickFields.npt = 1; //The TICK register is unreadable by by !priv

    softint = 0; // Clear all the soft interrupt bits
    tick_cmprFields.int_dis = 1; // disable timer compare interrupts
    tick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing
    stickFields.npt = 1; //The TICK register is unreadable by by !priv
    stick_cmprFields.int_dis = 1; // disable timer compare interrupts
    stick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing


    tt[tl] = power_on_reset;
    pstate = 0; // fields 0 but pef
    pstateFields.pef = 1;

    hpstate = 0;
    hpstateFields.red = 1;
    hpstateFields.hpriv = 1;
    hpstateFields.tlz = 0; // this is a guess
    hintp = 0; // no interrupts pending
    hstick_cmprFields.int_dis = 1; // disable timer compare interrupts
    hstick_cmprFields.tick_cmpr = 0; // Reset to 0 for pretty printing
#else
/*	    //This sets up the initial state of the processor for usermode processes
    pstateFields.priv = 0; //Process runs in user mode
    pstateFields.ie = 1; //Interrupts are enabled
    fsrFields.rd = 0; //Round to nearest
    fsrFields.tem = 0; //Floating point traps not enabled
    fsrFields.ns = 0; //Non standard mode off
    fsrFields.qne = 0; //Floating point queue is empty
    fsrFields.aexc = 0; //No accrued exceptions
    fsrFields.cexc = 0; //No current exceptions

    //Register window management registers
    otherwin = 0; //No windows contain info from other programs
    canrestore = 0; //There are no windows to pop
    cansave = MaxTL - 2; //All windows are available to save into
    cleanwin = MaxTL;*/
#endif
}

MiscReg MiscRegFile::readReg(int miscReg)
{
    switch (miscReg) {
        case MISCREG_Y:
          return y;
        case MISCREG_CCR:
          return ccr;
        case MISCREG_ASI:
          return asi;
        case MISCREG_FPRS:
          return fprs;
        case MISCREG_TICK:
           return tick;
        case MISCREG_PCR:
        case MISCREG_PIC:
          panic("ASR number %d not implemented\n", miscReg - AsrStart);
        case MISCREG_GSR:
          return gsr;
        case MISCREG_SOFTINT:
           return softint;
        case MISCREG_TICK_CMPR:
           return tick_cmpr;
        case MISCREG_STICK:
           return stick;
        case MISCREG_STICK_CMPR:
           return stick_cmpr;

        /** Privilged Registers */
        case MISCREG_TPC:
          return tpc[tl-1];
        case MISCREG_TNPC:
          return tnpc[tl-1];
        case MISCREG_TSTATE:
          return tstate[tl-1];
        case MISCREG_TT:
          return tt[tl-1];
        case MISCREG_PRIVTICK:
          panic("Priviliged access to tick registers not implemented\n");
        case MISCREG_TBA:
          return tba;
        case MISCREG_PSTATE:
          return pstate;
        case MISCREG_TL:
          return tl;
        case MISCREG_PIL:
          return pil;
        case MISCREG_CWP:
          return cwp;
        case MISCREG_CANSAVE:
          return cansave;
        case MISCREG_CANRESTORE:
          return canrestore;
        case MISCREG_CLEANWIN:
          return cleanwin;
        case MISCREG_OTHERWIN:
          return otherwin;
        case MISCREG_WSTATE:
          return wstate;
        case MISCREG_GL:
          return gl;

        /** Hyper privileged registers */
        case MISCREG_HPSTATE:
          return hpstate;
        case MISCREG_HTSTATE:
          return htstate[tl-1];
        case MISCREG_HINTP:
          panic("HINTP not implemented\n");
        case MISCREG_HTBA:
          return htba;
        case MISCREG_HVER:
          return NWindows | MaxTL << 8 | MaxGL << 16;
        case MISCREG_STRAND_STS_REG:
          return strandStatusReg;
        case MISCREG_HSTICK_CMPR:
          return hstick_cmpr;

        /** Floating Point Status Register */
        case MISCREG_FSR:
          return fsr;
        default:
          panic("Miscellaneous register %d not implemented\n", miscReg);
    }
}

MiscReg MiscRegFile::readRegWithEffect(int miscReg,
        Fault &fault, ThreadContext * tc)
{
    fault = NoFault;
    switch (miscReg) {
        case MISCREG_Y:
        case MISCREG_CCR:
        case MISCREG_ASI:
          return readReg(miscReg);

        case MISCREG_TICK:
        case MISCREG_PRIVTICK:
          // Check  for reading privilege
          if (tickFields.npt && !isNonPriv()) {
              fault = new PrivilegedAction;
              return 0;
          }
          return tc->getCpuPtr()->curCycle() - tickFields.counter |
              tickFields.npt << 63;
        case MISCREG_PC:
          return tc->readPC();
        case MISCREG_FPRS:
          fault = new UnimpFault("FPU not implemented\n");
          return 0;
        case MISCREG_PCR:
          fault = new UnimpFault("Performance Instrumentation not impl\n");
          return 0;
        case MISCREG_PIC:
          fault = new UnimpFault("Performance Instrumentation not impl\n");
          return 0;
        case MISCREG_GSR:
          return readReg(miscReg);

        /** Privilged Registers */
        case MISCREG_TPC:
        case MISCREG_TNPC:
        case MISCREG_TSTATE:
        case MISCREG_TT:
          if (tl == 0) {
              fault = new IllegalInstruction;
              return 0;
          } // NOTE THE FALL THROUGH!
        case MISCREG_PSTATE:
        case MISCREG_TL:
          return readReg(miscReg);

        case MISCREG_TBA:
          return readReg(miscReg) & ULL(~0x7FFF);

        case MISCREG_PIL:

        case MISCREG_CWP:
        case MISCREG_CANSAVE:
        case MISCREG_CANRESTORE:
        case MISCREG_CLEANWIN:
        case MISCREG_OTHERWIN:
        case MISCREG_WSTATE:
        case MISCREG_GL:
          return readReg(miscReg);

        /** Floating Point Status Register */
        case MISCREG_FSR:
          panic("Floating Point not implemented\n");
        default:
#if FULL_SYSTEM
          return readFSRegWithEffect(miscReg, fault, tc);
#else
          fault = new IllegalInstruction;
          return 0;
#endif
    }
}

Fault MiscRegFile::setReg(int miscReg, const MiscReg &val)
{
    switch (miscReg) {
        case MISCREG_Y:
          y = val;
          return NoFault;
        case MISCREG_CCR:
          ccr = val;
          return NoFault;
        case MISCREG_ASI:
          asi = val;
          return NoFault;
        case MISCREG_FPRS:
          fprs = val;
          return NoFault;
        case MISCREG_TICK:
           tick = val;
          return NoFault;
        case MISCREG_PCR:
        case MISCREG_PIC:
          panic("ASR number %d not implemented\n", miscReg - AsrStart);
        case MISCREG_GSR:
          gsr = val;
        case MISCREG_SOFTINT:
           softint = val;
          return NoFault;
        case MISCREG_TICK_CMPR:
           tick_cmpr = val;
          return NoFault;
        case MISCREG_STICK:
           stick = val;
          return NoFault;
        case MISCREG_STICK_CMPR:
           stick_cmpr = val;
          return NoFault;

        /** Privilged Registers */
        case MISCREG_TPC:
          tpc[tl-1] = val;
          return NoFault;
        case MISCREG_TNPC:
          tnpc[tl-1] = val;
          return NoFault;
        case MISCREG_TSTATE:
          tstate[tl-1] = val;
          return NoFault;
        case MISCREG_TT:
          tt[tl-1] = val;
          return NoFault;
        case MISCREG_PRIVTICK:
          panic("Priviliged access to tick regesiters not implemented\n");
        case MISCREG_TBA:
          tba = val;
          return NoFault;
        case MISCREG_PSTATE:
          pstate = val;
          return NoFault;
        case MISCREG_TL:
          tl = val;
          return NoFault;
        case MISCREG_PIL:
          pil = val;
          return NoFault;
        case MISCREG_CWP:
          cwp = val;
          return NoFault;
        case MISCREG_CANSAVE:
          cansave = val;
          return NoFault;
        case MISCREG_CANRESTORE:
          canrestore = val;
          return NoFault;
        case MISCREG_CLEANWIN:
          cleanwin = val;
          return NoFault;
        case MISCREG_OTHERWIN:
          otherwin = val;
          return NoFault;
        case MISCREG_WSTATE:
          wstate = val;
          return NoFault;
        case MISCREG_GL:
          gl = val;
          return NoFault;

        /** Hyper privileged registers */
        case MISCREG_HPSTATE:
          hpstate = val;
          return NoFault;
        case MISCREG_HTSTATE:
          htstate[tl-1] = val;
          return NoFault;
        case MISCREG_HINTP:
          panic("HINTP not implemented\n");
        case MISCREG_HTBA:
          htba = val;
          return NoFault;
        case MISCREG_STRAND_STS_REG:
          strandStatusReg = val;
          return NoFault;
        case MISCREG_HSTICK_CMPR:
          hstick_cmpr = val;
          return NoFault;

        /** Floating Point Status Register */
        case MISCREG_FSR:
          fsr = val;
          return NoFault;
        default:
          panic("Miscellaneous register %d not implemented\n", miscReg);
    }
}

Fault MiscRegFile::setRegWithEffect(int miscReg,
        const MiscReg &val, ThreadContext * tc)
{
    const uint64_t Bit64 = (1ULL << 63);
    switch (miscReg) {
        case MISCREG_Y:
        case MISCREG_CCR:
        case MISCREG_ASI:
          setReg(miscReg, val);
          return NoFault;
        case MISCREG_PRIVTICK:
        case MISCREG_TICK:
          if (isNonPriv())
              return new PrivilegedOpcode;
          if (isPriv())
              return new PrivilegedAction;
          tickFields.counter = tc->getCpuPtr()->curCycle() - val  & ~Bit64;
          tickFields.npt = val & Bit64 ? 1 : 0;
           return NoFault;
        case MISCREG_PC:
           return new IllegalInstruction;
        case MISCREG_FPRS:
           return new UnimpFault("FPU not implemented\n");
        case MISCREG_PCR:
           return new UnimpFault("Performance Instrumentation not impl\n");
        case MISCREG_PIC:
           return new UnimpFault("Performance Instrumentation not impl\n");
        case MISCREG_GSR:
           return setReg(miscReg, val);

        /** Privilged Registers */
        case MISCREG_TPC:
        case MISCREG_TNPC:
        case MISCREG_TSTATE:
        case MISCREG_TT:
          if (tl == 0)
              return new IllegalInstruction;
          setReg(miscReg, val);
          return NoFault;

        case MISCREG_TBA:
          // clear lower 7 bits on writes.
          setReg(miscReg, val & ULL(~0x7FFF));
          return NoFault;

        case MISCREG_PSTATE:
          setReg(miscReg, val);
          return NoFault;

        case MISCREG_TL:
          if (isHyperPriv() && val > MaxTL)
              setReg(miscReg, MaxTL);
          else if (isPriv() && !isHyperPriv() && val > MaxPTL)
              setReg(miscReg, MaxPTL);
          else
              setReg(miscReg, val);
          return NoFault;

        case MISCREG_CWP:
          tc->changeRegFileContext(CONTEXT_CWP, val);
        case MISCREG_CANSAVE:
        case MISCREG_CANRESTORE:
        case MISCREG_CLEANWIN:
        case MISCREG_OTHERWIN:
        case MISCREG_WSTATE:
          setReg(miscReg, val);
          return NoFault;

        case MISCREG_GL:
          int newval;
          if (isHyperPriv() && val > MaxGL)
              newval = MaxGL;
          else if (isPriv() && !isHyperPriv() && val > MaxPGL)
              newval =  MaxPGL;
          else
              newval = val;
          tc->changeRegFileContext(CONTEXT_GLOBALS, newval);
          setReg(miscReg, newval);
          return NoFault;

        /** Floating Point Status Register */
        case MISCREG_FSR:
          panic("Floating Point not implemented\n");
        default:
#if FULL_SYSTEM
              setFSRegWithEffect(miscReg, val, tc);
#else
              return new IllegalInstruction;
#endif
    }
}

void MiscRegFile::serialize(std::ostream & os)
{
    SERIALIZE_SCALAR(pstate);
    SERIALIZE_SCALAR(tba);
    SERIALIZE_SCALAR(y);
    SERIALIZE_SCALAR(pil);
    SERIALIZE_SCALAR(gl);
    SERIALIZE_SCALAR(cwp);
    SERIALIZE_ARRAY(tt, MaxTL);
    SERIALIZE_SCALAR(ccr);
    SERIALIZE_SCALAR(asi);
    SERIALIZE_SCALAR(tl);
    SERIALIZE_ARRAY(tpc, MaxTL);
    SERIALIZE_ARRAY(tnpc, MaxTL);
    SERIALIZE_ARRAY(tstate, MaxTL);
    SERIALIZE_SCALAR(tick);
    SERIALIZE_SCALAR(cansave);
    SERIALIZE_SCALAR(canrestore);
    SERIALIZE_SCALAR(otherwin);
    SERIALIZE_SCALAR(cleanwin);
    SERIALIZE_SCALAR(wstate);
    SERIALIZE_SCALAR(fsr);
    SERIALIZE_SCALAR(fprs);
    SERIALIZE_SCALAR(hpstate);
    SERIALIZE_ARRAY(htstate, MaxTL);
    SERIALIZE_SCALAR(htba);
    SERIALIZE_SCALAR(hstick_cmpr);
}

void MiscRegFile::unserialize(Checkpoint * cp, const std::string & section)
{
    UNSERIALIZE_SCALAR(pstate);
    UNSERIALIZE_SCALAR(tba);
    UNSERIALIZE_SCALAR(y);
    UNSERIALIZE_SCALAR(pil);
    UNSERIALIZE_SCALAR(gl);
    UNSERIALIZE_SCALAR(cwp);
    UNSERIALIZE_ARRAY(tt, MaxTL);
    UNSERIALIZE_SCALAR(ccr);
    UNSERIALIZE_SCALAR(asi);
    UNSERIALIZE_SCALAR(tl);
    UNSERIALIZE_ARRAY(tpc, MaxTL);
    UNSERIALIZE_ARRAY(tnpc, MaxTL);
    UNSERIALIZE_ARRAY(tstate, MaxTL);
    UNSERIALIZE_SCALAR(tick);
    UNSERIALIZE_SCALAR(cansave);
    UNSERIALIZE_SCALAR(canrestore);
    UNSERIALIZE_SCALAR(otherwin);
    UNSERIALIZE_SCALAR(cleanwin);
    UNSERIALIZE_SCALAR(wstate);
    UNSERIALIZE_SCALAR(fsr);
    UNSERIALIZE_SCALAR(fprs);
    UNSERIALIZE_SCALAR(hpstate);
    UNSERIALIZE_ARRAY(htstate, MaxTL);
    UNSERIALIZE_SCALAR(htba);
    UNSERIALIZE_SCALAR(hstick_cmpr);
}

