/*
 * Copyright © 2007 MIPS Technologies, Inc.  All Rights Reserved
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
 * work (e.g., Copyright © <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. (“MIPS”) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED “AS IS.”  MIPS MAKES NO WARRANTIES AND
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
 * Authors: Steven K. Reinhardt
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
    xc->setMiscRegNoEffect(LLAddr, req->getPaddr() & ~0xf, tid);
    xc->setMiscRegNoEffect(LLFlag, true, tid);
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
        bool lock_flag = xc->readMiscRegNoEffect(LLFlag, tid);
        Addr lock_addr = xc->readMiscRegNoEffect(LLAddr, tid);

        if (!lock_flag || (req->getPaddr() & ~0xf) != lock_addr) {
            // Lock flag not set or addr mismatch in CPU;
            // don't even bother sending to memory system
            req->setExtraData(0);
            xc->setMiscRegNoEffect(LLFlag, false, tid);

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
