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

#include "arch/sparc/kernel_stats.hh"
#include "arch/sparc/miscregfile.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "sim/system.hh"

using namespace SparcISA;


void
MiscRegFile::checkSoftInt(ThreadContext *tc)
{
    // If PIL < 14, copy over the tm and sm bits
    if (pil < 14 && softint & 0x10000)
        tc->getCpuPtr()->post_interrupt(IT_SOFT_INT,16);
    else
        tc->getCpuPtr()->clear_interrupt(IT_SOFT_INT,16);
    if (pil < 14 && softint & 0x1)
        tc->getCpuPtr()->post_interrupt(IT_SOFT_INT,0);
    else
        tc->getCpuPtr()->clear_interrupt(IT_SOFT_INT,0);

    // Copy over any of the other bits that are set
    for (int bit = 15; bit > 0; --bit) {
        if (1 << bit & softint && bit > pil)
            tc->getCpuPtr()->post_interrupt(IT_SOFT_INT,bit);
        else
            tc->getCpuPtr()->clear_interrupt(IT_SOFT_INT,bit);
    }
}


void
MiscRegFile::setFSReg(int miscReg, const MiscReg &val, ThreadContext *tc)
{
    int64_t time;
    switch (miscReg) {
        /* Full system only ASRs */
      case MISCREG_SOFTINT:
        setRegNoEffect(miscReg, val);;
        checkSoftInt(tc);
        break;
      case MISCREG_SOFTINT_CLR:
        return setReg(MISCREG_SOFTINT, ~val & softint, tc);
      case MISCREG_SOFTINT_SET:
        return setReg(MISCREG_SOFTINT, val | softint, tc);

      case MISCREG_TICK_CMPR:
        if (tickCompare == NULL)
            tickCompare = new TickCompareEvent(this, tc);
        setRegNoEffect(miscReg, val);
        if ((tick_cmpr & ~mask(63)) && tickCompare->scheduled())
            tickCompare->deschedule();
        time = (tick_cmpr & mask(63)) - (tick & mask(63));
        if (!(tick_cmpr & ~mask(63)) && time > 0) {
            if (tickCompare->scheduled())
                tickCompare->deschedule();
            tickCompare->schedule(time * tc->getCpuPtr()->cycles(1));
        }
        panic("writing to TICK compare register %#X\n", val);
        break;

      case MISCREG_STICK_CMPR:
        if (sTickCompare == NULL)
            sTickCompare = new STickCompareEvent(this, tc);
        setRegNoEffect(miscReg, val);
        if ((stick_cmpr & ~mask(63)) && sTickCompare->scheduled())
            sTickCompare->deschedule();
        time = ((int64_t)(stick_cmpr & mask(63)) - (int64_t)stick) -
            tc->getCpuPtr()->instCount();
        if (!(stick_cmpr & ~mask(63)) && time > 0) {
            if (sTickCompare->scheduled())
                sTickCompare->deschedule();
            sTickCompare->schedule(time * tc->getCpuPtr()->cycles(1) + curTick);
        }
        DPRINTF(Timer, "writing to sTICK compare register value %#X\n", val);
        break;

      case MISCREG_PSTATE:
        setRegNoEffect(miscReg, val);

      case MISCREG_PIL:
        setRegNoEffect(miscReg, val);
        checkSoftInt(tc);
        break;

      case MISCREG_HVER:
        panic("Shouldn't be writing HVER\n");

      case MISCREG_HINTP:
        setRegNoEffect(miscReg, val);
        if (hintp)
            tc->getCpuPtr()->post_interrupt(IT_HINTP,0);
        else
            tc->getCpuPtr()->clear_interrupt(IT_HINTP,0);
        break;

      case MISCREG_HTBA:
        // clear lower 7 bits on writes.
        setRegNoEffect(miscReg, val & ULL(~0x7FFF));
        break;

      case MISCREG_QUEUE_CPU_MONDO_HEAD:
      case MISCREG_QUEUE_CPU_MONDO_TAIL:
        setRegNoEffect(miscReg, val);
        if (cpu_mondo_head != cpu_mondo_tail)
            tc->getCpuPtr()->post_interrupt(IT_CPU_MONDO,0);
        else
            tc->getCpuPtr()->clear_interrupt(IT_CPU_MONDO,0);
        break;
      case MISCREG_QUEUE_DEV_MONDO_HEAD:
      case MISCREG_QUEUE_DEV_MONDO_TAIL:
        setRegNoEffect(miscReg, val);
        if (dev_mondo_head != dev_mondo_tail)
            tc->getCpuPtr()->post_interrupt(IT_DEV_MONDO,0);
        else
            tc->getCpuPtr()->clear_interrupt(IT_DEV_MONDO,0);
        break;
      case MISCREG_QUEUE_RES_ERROR_HEAD:
      case MISCREG_QUEUE_RES_ERROR_TAIL:
        setRegNoEffect(miscReg, val);
        if (res_error_head != res_error_tail)
            tc->getCpuPtr()->post_interrupt(IT_RES_ERROR,0);
        else
            tc->getCpuPtr()->clear_interrupt(IT_RES_ERROR,0);
        break;
      case MISCREG_QUEUE_NRES_ERROR_HEAD:
      case MISCREG_QUEUE_NRES_ERROR_TAIL:
        setRegNoEffect(miscReg, val);
        // This one doesn't have an interrupt to report to the guest OS
        break;

      case MISCREG_HSTICK_CMPR:
        if (hSTickCompare == NULL)
            hSTickCompare = new HSTickCompareEvent(this, tc);
        setRegNoEffect(miscReg, val);
        if ((hstick_cmpr & ~mask(63)) && hSTickCompare->scheduled())
            hSTickCompare->deschedule();
        time = ((int64_t)(hstick_cmpr & mask(63)) - (int64_t)stick) -
            tc->getCpuPtr()->instCount();
        if (!(hstick_cmpr & ~mask(63)) && time > 0) {
            if (hSTickCompare->scheduled())
                hSTickCompare->deschedule();
            hSTickCompare->schedule(curTick + time * tc->getCpuPtr()->cycles(1));
        }
        DPRINTF(Timer, "writing to hsTICK compare register value %#X\n", val);
        break;

      case MISCREG_HPSTATE:
        // T1000 spec says impl. dependent val must always be 1
        setRegNoEffect(miscReg, val | HPSTATE::id);
#if FULL_SYSTEM
        if (hpstate & HPSTATE::tlz && tl == 0 && !(hpstate & HPSTATE::hpriv))
            tc->getCpuPtr()->post_interrupt(IT_TRAP_LEVEL_ZERO,0);
        else
            tc->getCpuPtr()->clear_interrupt(IT_TRAP_LEVEL_ZERO,0);
#endif
        break;
      case MISCREG_HTSTATE:
        setRegNoEffect(miscReg, val);
        break;

      case MISCREG_STRAND_STS_REG:
        if (bits(val,2,2))
            panic("No support for setting spec_en bit\n");
        setRegNoEffect(miscReg, bits(val,0,0));
        if (!bits(val,0,0)) {
            // Time to go to sleep
            tc->suspend();
            if (tc->getKernelStats())
                tc->getKernelStats()->quiesce();
            }
        break;

      default:
        panic("Invalid write to FS misc register %s\n", getMiscRegName(miscReg));
    }
}

MiscReg
MiscRegFile::readFSReg(int miscReg, ThreadContext * tc)
{
    uint64_t temp;

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
      case MISCREG_HSTICK_CMPR:
        return readRegNoEffect(miscReg) ;

      case MISCREG_HTBA:
        return readRegNoEffect(miscReg) & ULL(~0x7FFF);
      case MISCREG_HVER:
        return NWindows | MaxTL << 8 | MaxGL << 16;

      case MISCREG_STRAND_STS_REG:
        System *sys;
        int x;
        sys = tc->getSystemPtr();

        temp = readRegNoEffect(miscReg) & (STS::active | STS::speculative);
        // Check that the CPU array is fully populated (by calling getNumCPus())
        assert(sys->getNumCPUs() > tc->readCpuId());

        temp |= tc->readCpuId()  << STS::shft_id;

        for (x = tc->readCpuId() & ~3; x < sys->threadContexts.size(); x++) {
            switch (sys->threadContexts[x]->status()) {
              case ThreadContext::Active:
                temp |= STS::st_run << (STS::shft_fsm0 -
                        ((x & 0x3) * (STS::shft_fsm0-STS::shft_fsm1)));
                break;
              case ThreadContext::Suspended:
                // should this be idle?
                temp |= STS::st_idle << (STS::shft_fsm0 -
                        ((x & 0x3) * (STS::shft_fsm0-STS::shft_fsm1)));
                break;
              case ThreadContext::Halted:
                temp |= STS::st_halt << (STS::shft_fsm0 -
                        ((x & 0x3) * (STS::shft_fsm0-STS::shft_fsm1)));
                break;
              default:
                panic("What state are we in?!\n");
            } // switch
        } // for

        return temp;
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
    // since our microcode instructions take two cycles we need to check if
    // we're actually at the correct cycle or we need to wait a little while
    // more
    int ticks;
    ticks = ((int64_t)(stick_cmpr & mask(63)) - (int64_t)stick) -
        tc->getCpuPtr()->instCount();
    assert(ticks >= 0 && "stick compare missed interrupt cycle");

    if (ticks == 0) {
        DPRINTF(Timer, "STick compare cycle reached at %#x\n",
                (stick_cmpr & mask(63)));
        if (!(tc->readMiscRegNoEffect(MISCREG_STICK_CMPR) & (ULL(1) << 63))) {
            setReg(MISCREG_SOFTINT, softint | (ULL(1) << 16), tc);
        }
    } else
        sTickCompare->schedule(ticks * tc->getCpuPtr()->cycles(1) + curTick);
}

void
MiscRegFile::processHSTickCompare(ThreadContext *tc)
{
    // since our microcode instructions take two cycles we need to check if
    // we're actually at the correct cycle or we need to wait a little while
    // more
    int ticks;
    ticks = ((int64_t)(hstick_cmpr & mask(63)) - (int64_t)stick) -
        tc->getCpuPtr()->instCount();
    assert(ticks >= 0 && "hstick compare missed interrupt cycle");

    if (ticks == 0) {
        DPRINTF(Timer, "HSTick compare cycle reached at %#x\n",
                (stick_cmpr & mask(63)));
        if (!(tc->readMiscRegNoEffect(MISCREG_HSTICK_CMPR) & (ULL(1) << 63))) {
            setReg(MISCREG_HINTP, 1, tc);
        }
        // Need to do something to cause interrupt to happen here !!! @todo
    } else
        hSTickCompare->schedule(ticks * tc->getCpuPtr()->cycles(1) + curTick);
}

