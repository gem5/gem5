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
 * Authors: Steve Reinhardt
 */

#ifndef __ARCH_MIPS_LOCKED_MEM_HH__
#define __ARCH_MIPS_LOCKED_MEM_HH__

/**
 * @file
 *
 * ISA-specific helper functions for locked memory accesses.
 */

#include "arch/isa_traits.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "mem/request.hh"


namespace MipsISA
{
template <class XC>
inline void
handleLockedRead(XC *xc, Request *req)
{
    unsigned tid = req->getThreadNum();
    xc->setMiscReg(LLAddr, req->getPaddr() & ~0xf, tid);
    xc->setMiscReg(LLFlag, true, tid);
    DPRINTF(LLSC, "[tid:%i]: Load-Link Flag Set & Load-Link Address set to %x.\n",
            tid, req->getPaddr() & ~0xf);
}


template <class XC>
inline bool
handleLockedWrite(XC *xc, Request *req)
{
    unsigned tid = req->getThreadNum();

    if (req->isUncacheable()) {
        // Funky Turbolaser mailbox access...don't update
        // result register (see stq_c in decoder.isa)
        req->setExtraData(2);
    } else {
        // standard store conditional
        bool lock_flag = xc->readMiscReg(LLFlag, tid);
        Addr lock_addr = xc->readMiscReg(LLAddr, tid);

        if (!lock_flag || (req->getPaddr() & ~0xf) != lock_addr) {
            // Lock flag not set or addr mismatch in CPU;
            // don't even bother sending to memory system
            req->setExtraData(0);
            xc->setMiscReg(LLFlag, false, tid);

            // the rest of this code is not architectural;
            // it's just a debugging aid to help detect
            // livelock by warning on long sequences of failed
            // store conditionals
            int stCondFailures = xc->readStCondFailures();
            stCondFailures++;
            xc->setStCondFailures(stCondFailures);
            if (stCondFailures % 10 == 0) {
                warn("%i: cpu %d: %d consecutive "
                     "store conditional failures\n",
                     curTick, xc->readCpuId(), stCondFailures);
            }

            if (stCondFailures == 5000) {
                panic("Max (5000) Store Conditional Fails Reached. Check Code For Deadlock.\n");
            }

            if (!lock_flag){
                DPRINTF(LLSC, "[tid:%i]: Lock Flag Set, Store Conditional Failed.\n",
                        tid);
            } else if ((req->getPaddr() & ~0xf) != lock_addr) {
                DPRINTF(LLSC, "[tid:%i]: Load-Link Address Mismatch, Store Conditional Failed.\n",
                        tid);
            }
            // store conditional failed already, so don't issue it to mem
            return false;
        }
    }

    return true;
}


} // namespace MipsISA

#endif
