/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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

#ifndef __ARCH_ALPHA_LOCKED_MEM_HH__
#define __ARCH_ALPHA_LOCKED_MEM_HH__

/**
 * @file
 *
 * ISA-specific helper functions for locked memory accesses.
 *
 * Note that these functions are not embedded in the ISA description
 * because they operate on the *physical* address rather than the
 * virtual address.  In the current M5 design, the physical address is
 * not accessible from the ISA description, only from the CPU model.
 * Thus the CPU is responsible for calling back to the ISA (here)
 * after the address translation has been performed to allow the ISA
 * to do these manipulations based on the physical address.
 */

#include "arch/alpha/registers.hh"
#include "base/misc.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace AlphaISA {

template <class XC>
inline void
handleLockedSnoop(XC *xc, PacketPtr pkt, Addr cacheBlockMask)
{
    // If we see a snoop come into the CPU and we currently have an LLSC
    // operation pending we need to clear the lock flag if it is to the same
    // cache line.

    if (!xc->readMiscReg(MISCREG_LOCKFLAG))
        return;

    Addr locked_addr = xc->readMiscReg(MISCREG_LOCKADDR) & cacheBlockMask;
    Addr snoop_addr = pkt->getAddr() & cacheBlockMask;

    if (locked_addr == snoop_addr)
        xc->setMiscReg(MISCREG_LOCKFLAG, false);
}


template <class XC>
inline void
handleLockedRead(XC *xc, Request *req)
{
    xc->setMiscReg(MISCREG_LOCKADDR, req->getPaddr() & ~0xf);
    xc->setMiscReg(MISCREG_LOCKFLAG, true);
}

template <class XC>
inline void
handleLockedSnoopHit(XC *xc)
{
}

template <class XC>
inline bool
handleLockedWrite(XC *xc, Request *req, Addr cacheBlockMask)
{
    if (req->isUncacheable()) {
        // Funky Turbolaser mailbox access...don't update
        // result register (see stq_c in decoder.isa)
        req->setExtraData(2);
    } else {
        // standard store conditional
        bool lock_flag = xc->readMiscReg(MISCREG_LOCKFLAG);
        Addr lock_addr = xc->readMiscReg(MISCREG_LOCKADDR);
        if (!lock_flag || (req->getPaddr() & ~0xf) != lock_addr) {
            // Lock flag not set or addr mismatch in CPU;
            // don't even bother sending to memory system
            req->setExtraData(0);
            xc->setMiscReg(MISCREG_LOCKFLAG, false);
            // the rest of this code is not architectural;
            // it's just a debugging aid to help detect
            // livelock by warning on long sequences of failed
            // store conditionals
            int stCondFailures = xc->readStCondFailures();
            stCondFailures++;
            xc->setStCondFailures(stCondFailures);
            if (stCondFailures % 100000 == 0) {
                warn("context %d: %d consecutive "
                     "store conditional failures\n",
                     xc->contextId(), stCondFailures);
            }

            // store conditional failed already, so don't issue it to mem
            return false;
        }
    }

    return true;
}

} // namespace AlphaISA

#endif // __ARCH_ALPHA_LOCKED_MEM_HH__
