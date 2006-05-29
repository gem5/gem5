/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "arch/sparc/regfile.hh"

Fault
SparcISA::MiscRegFile::setFSRegWithEffect(int miscReg, const MiscReg &val,
        ExecContext *xc)
{
    int64_t time;
    SparcSystem *sys;
    switch (miscReg) {
        /** Full system only ASRs */
        case MISCREG_SOFTINT:
          if (isNonPriv())
              return new PrivilegedOpcode;
          // Check if we are going to interrupt because of something
          int oldLevel = InterruptLevel(softint);
          int newLevel = InterruptLevel(val);
          setReg(miscReg, val);
          if (newLevel > oldLevel)
              ; // MUST DO SOMETHING HERE TO TELL CPU TO LOOK FOR INTERRUPTS XXX
              //xc->getCpuPtr()->checkInterrupts = true;
          return NoFault;

        case MISCREG_SOFTINT_CLR:
          return setRegWithEffect(miscReg, ~val & softint, xc);
        case MISCREG_SOFTINT_SET:
          return setRegWithEffect(miscReg, val | softint, xc);

        case MISCREG_TICK_CMPR:
          if (isNonPriv())
              return new PrivilegedOpcode;
          setReg(miscReg, val);
          if (tick_cmprFields.int_dis && tickCompare.scheduled())
                  tickCompare.deschedule();
          time = tick_cmprFields.tick_cmpr - tickFields.counter;
          if (!tick_cmprFields.int_dis && time > 0)
              tickCompare.schedule(time * xc->getCpuPtr()->cycles(1));
          return NoFault;

        case MISCREG_STICK:
          if (isNonPriv())
              return new PrivilegedOpcode;
          if (isPriv())
              return new PrivilegedAction;
          sys = dynamic_cast<SparcSystem*>(xc->getSystemPtr());
          assert(sys != NULL);
          sys->sysTick = curTick/Clock::Int::ns - val & ~Bit64;
          stickFields.npt = val & Bit64 ? 1 : 0;
          return NoFault;

        case MISCREG_STICK_CMPR:
          if (isNonPriv())
              return new PrivilegedOpcode;
          sys = dynamic_cast<SparcSystem*>(xc->getSystemPtr());
          assert(sys != NULL);
          setReg(miscReg, val);
          if (stick_cmprFields.int_dis && sTickCompare.scheduled())
                  sTickCompare.deschedule();
          time = stick_cmprFields.tick_cmpr - sys->sysTick;
          if (!stick_cmprFields.int_dis && time > 0)
              sTickCompare.schedule(time * Clock::Int::ns);
          return NoFault;

        /** Fullsystem only Priv registers. */
        case MISCREG_PIL:
          if (FULL_SYSTEM) {
              setReg(miscReg, val);
              //xc->getCpuPtr()->checkInterrupts;
               // MUST DO SOMETHING HERE TO TELL CPU TO LOOK FOR INTERRUPTS XXX
              return NoFault;
          } else
              panic("PIL not implemented for syscall emulation\n");

        /** Hyper privileged registers */
        case MISCREG_HPSTATE:
        case MISCREG_HINTP:
          setReg(miscReg, val);
          return NoFault;
        case MISCREG_HTSTATE:
          if (tl == 0)
              return new IllegalInstruction;
          setReg(miscReg, val);
          return NoFault;

        case MISCREG_HTBA:
          // clear lower 7 bits on writes.
          setReg(miscReg, val & ULL(~0x7FFF));
          return NoFault;

        case MISCREG_STRAND_STS_REG:
          setReg(miscReg, strandStatusReg);
          return NoFault;
        case MISCREG_HSTICK_CMPR:
          if (isNonPriv())
              return new PrivilegedOpcode;
          sys = dynamic_cast<SparcSystem*>(xc->getSystemPtr());
          assert(sys != NULL);
          setReg(miscReg, val);
          if (hstick_cmprFields.int_dis && hSTickCompare.scheduled())
                  hSTickCompare.deschedule();
          int64_t time = hstick_cmprFields.tick_cmpr - sys->sysTick;
          if (!hstick_cmprFields.int_dis && time > 0)
              hSTickCompare.schedule(time * Clock::Int::ns);
          return NoFault;
        default:
          return new IllegalInstruction;
    }
}

MiscReg
MiscRegFile::readFSRegWithEffect(int miscReg, Fault &fault, ExecContext * xc)
{
    switch (miscReg) {

        /** Privileged registers. */
        case MISCREG_SOFTINT:
           if (isNonPriv()) {
               fault = new PrivilegedOpcode;
               return 0;
           }
           return readReg(miscReg);
        case MISCREG_TICK_CMPR:
           if (isNonPriv()) {
               fault =  new PrivilegedOpcode;
               return 0;
           }
           return readReg(miscReg);
        case MISCREG_STICK:
          SparcSystem *sys;
          if (stickFields.npt && !isNonPriv()) {
              fault = new PrivilegedAction;
              return 0;
          }
          sys = dynamic_cast<SparcSystem*>(xc->getSystemPtr());
          assert(sys != NULL);
          return curTick/Clock::Int::ns - sys->sysTick | stickFields.npt << 63;
        case MISCREG_STICK_CMPR:
           if (isNonPriv()) {
               fault =  new PrivilegedOpcode;
               return 0;
           }
           return readReg(miscReg);


        /** Hyper privileged registers */
        case MISCREG_HPSTATE:
        case MISCREG_HINTP:
          return readReg(miscReg);
        case MISCREG_HTSTATE:
          if (tl == 0) {
              fault = new IllegalInstruction;
              return 0;
          }
          return readReg(miscReg);

        case MISCREG_HTBA:
          return readReg(miscReg) & ULL(~0x7FFF);
        case MISCREG_HVER:
          return NWindows | MaxTL << 8 | MaxGL << 16;
        case MISCREG_STRAND_STS_REG:
          return strandStatusReg;
        case MISCREG_HSTICK_CMPR:
          return hstick_cmpr;

        default:
          fault = new IllegalInstruction;
          return 0;
    }
}


}; // namespace SparcISA
