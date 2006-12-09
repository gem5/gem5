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
 *
 * Authors: Ali Saidi
 */

#include "arch/sparc/miscregfile.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"

using namespace SparcISA;

void
MiscRegFile::setFSRegWithEffect(int miscReg, const MiscReg &val,
        ThreadContext *tc)
{
    int64_t time;
    switch (miscReg) {
        /* Full system only ASRs */
        case MISCREG_SOFTINT:
          // Check if we are going to interrupt because of something
          setReg(miscReg, val);
          tc->getCpuPtr()->checkInterrupts = true;
          warn("Writing to softint not really supported, writing: %#x\n", val);
          break;

        case MISCREG_SOFTINT_CLR:
          return setRegWithEffect(miscReg, ~val & softint, tc);
        case MISCREG_SOFTINT_SET:
          return setRegWithEffect(miscReg, val | softint, tc);

        case MISCREG_TICK_CMPR:
          if (tickCompare == NULL)
              tickCompare = new TickCompareEvent(this, tc);
          setReg(miscReg, val);
          if ((tick_cmpr & mask(63)) && tickCompare->scheduled())
                  tickCompare->deschedule();
          time = (tick_cmpr & mask(63)) - (tick & mask(63));
          if (!(tick_cmpr & ~mask(63)) && time > 0)
              tickCompare->schedule(time * tc->getCpuPtr()->cycles(1));
          warn ("writing to TICK compare register %#X\n", val);
          break;

        case MISCREG_STICK_CMPR:
          if (sTickCompare == NULL)
              sTickCompare = new STickCompareEvent(this, tc);
          setReg(miscReg, val);
          if ((stick_cmpr & mask(63)) && sTickCompare->scheduled())
                  sTickCompare->deschedule();
          time = (stick_cmpr & mask(63)) - (stick & mask(63));
          if (!(stick_cmpr & ~mask(63)) && time > 0)
              sTickCompare->schedule(time * tc->getCpuPtr()->cycles(1));
          warn ("writing to sTICK compare register value %#X\n", val);
          break;

        case MISCREG_PSTATE:
          if (val & ie && !(pstate & ie)) {
              tc->getCpuPtr()->checkInterrupts = true;
          }
          setReg(miscReg, val);

        case MISCREG_PIL:
          if (val < pil) {
              tc->getCpuPtr()->checkInterrupts = true;
          }
          setReg(miscReg, val);
          break;

        case MISCREG_HVER:
          panic("Shouldn't be writing HVER\n");

        case MISCREG_HTBA:
          // clear lower 7 bits on writes.
          setReg(miscReg, val & ULL(~0x7FFF));
          break;

        case MISCREG_QUEUE_CPU_MONDO_HEAD:
        case MISCREG_QUEUE_CPU_MONDO_TAIL:
        case MISCREG_QUEUE_DEV_MONDO_HEAD:
        case MISCREG_QUEUE_DEV_MONDO_TAIL:
        case MISCREG_QUEUE_RES_ERROR_HEAD:
        case MISCREG_QUEUE_RES_ERROR_TAIL:
        case MISCREG_QUEUE_NRES_ERROR_HEAD:
        case MISCREG_QUEUE_NRES_ERROR_TAIL:
          setReg(miscReg, val);
          tc->getCpuPtr()->checkInterrupts = true;
          break;

        case MISCREG_HSTICK_CMPR:
          if (hSTickCompare == NULL)
              hSTickCompare = new HSTickCompareEvent(this, tc);
          setReg(miscReg, val);
          if ((hstick_cmpr & mask(63)) && hSTickCompare->scheduled())
                hSTickCompare->deschedule();
          time = (hstick_cmpr & mask(63)) - (stick & mask(63));
          if (!(hstick_cmpr & ~mask(63)) && time > 0)
              hSTickCompare->schedule(time * tc->getCpuPtr()->cycles(1));
          warn ("writing to hsTICK compare register value %#X\n", val);
          break;

        case MISCREG_HPSTATE:
          // T1000 spec says impl. dependent val must always be 1
          setReg(miscReg, val | id);
          break;
        case MISCREG_HTSTATE:
        case MISCREG_STRAND_STS_REG:
          setReg(miscReg, val);
          break;

        default:
          panic("Invalid write to FS misc register %s\n", getMiscRegName(miscReg));
    }
}

MiscReg
MiscRegFile::readFSRegWithEffect(int miscReg, ThreadContext * tc)
{
    switch (miscReg) {
      /* Privileged registers. */
      case MISCREG_QUEUE_CPU_MONDO_HEAD:
      case MISCREG_QUEUE_CPU_MONDO_TAIL:
      case MISCREG_QUEUE_DEV_MONDO_HEAD:
      case MISCREG_QUEUE_DEV_MONDO_TAIL:
      case MISCREG_QUEUE_RES_ERROR_HEAD:
      case MISCREG_QUEUE_RES_ERROR_TAIL:
      case MISCREG_QUEUE_NRES_ERROR_HEAD:
      case MISCREG_QUEUE_NRES_ERROR_TAIL:
      case MISCREG_SOFTINT:
      case MISCREG_TICK_CMPR:
      case MISCREG_STICK_CMPR:
      case MISCREG_PIL:
      case MISCREG_HPSTATE:
      case MISCREG_HINTP:
      case MISCREG_HTSTATE:
      case MISCREG_STRAND_STS_REG:
      case MISCREG_HSTICK_CMPR:
        return readReg(miscReg) ;

      case MISCREG_HTBA:
        return readReg(miscReg) & ULL(~0x7FFF);
      case MISCREG_HVER:
        return NWindows | MaxTL << 8 | MaxGL << 16;

      default:
        panic("Invalid read to FS misc register\n");
    }
}
/*
        In Niagra STICK==TICK so this isn't needed
        case MISCREG_STICK:
          SparcSystem *sys;
          sys = dynamic_cast<SparcSystem*>(tc->getSystemPtr());
          assert(sys != NULL);
          return curTick/Clock::Int::ns - sys->sysTick | (stick & ~(mask(63)));
*/



void
MiscRegFile::processTickCompare(ThreadContext *tc)
{
    panic("tick compare not implemented\n");
}

void
MiscRegFile::processSTickCompare(ThreadContext *tc)
{
    panic("tick compare not implemented\n");
}

void
MiscRegFile::processHSTickCompare(ThreadContext *tc)
{
    panic("tick compare not implemented\n");
}

