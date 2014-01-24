/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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
 *          Steve Reinhardt
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_LOCKED_MEM_HH__
#define __ARCH_ARM_LOCKED_MEM_HH__

/**
 * @file
 *
 * ISA-specific helper functions for locked memory accesses.
 */

#include "arch/arm/miscregs.hh"
#include "arch/arm/isa_traits.hh"
#include "debug/LLSC.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

namespace ArmISA
{
template <class XC>
inline void
handleLockedSnoop(XC *xc, PacketPtr pkt, Addr cacheBlockMask)
{
    DPRINTF(LLSC,"%s:  handleing snoop for address: %#x locked: %d\n",
            xc->getCpuPtr()->name(),pkt->getAddr(),
            xc->readMiscReg(MISCREG_LOCKFLAG));
    if (!xc->readMiscReg(MISCREG_LOCKFLAG))
        return;

    Addr locked_addr = xc->readMiscReg(MISCREG_LOCKADDR) & cacheBlockMask;
    // If no caches are attached, the snoop address always needs to be masked
    Addr snoop_addr = pkt->getAddr() & cacheBlockMask;

    DPRINTF(LLSC,"%s:  handleing snoop for address: %#x locked addr: %#x\n",
            xc->getCpuPtr()->name(),snoop_addr, locked_addr);
    if (locked_addr == snoop_addr) {
        DPRINTF(LLSC,"%s: address match, clearing lock and signaling sev\n",
                xc->getCpuPtr()->name());
        xc->setMiscReg(MISCREG_LOCKFLAG, false);
        // Implement ARMv8 WFE/SEV semantics
        xc->setMiscReg(MISCREG_SEV_MAILBOX, true);
        xc->getCpuPtr()->wakeup();
    }
}

template <class XC>
inline void
handleLockedRead(XC *xc, Request *req)
{
    xc->setMiscReg(MISCREG_LOCKADDR, req->getPaddr());
    xc->setMiscReg(MISCREG_LOCKFLAG, true);
    DPRINTF(LLSC,"%s: Placing address %#x in monitor\n", xc->getCpuPtr()->name(),
                 req->getPaddr());
}

template <class XC>
inline void
handleLockedSnoopHit(XC *xc)
{
    DPRINTF(LLSC,"%s:  handling snoop lock hit address: %#x\n",
            xc->getCpuPtr()->name(), xc->readMiscReg(MISCREG_LOCKADDR));
        xc->setMiscReg(MISCREG_LOCKFLAG, false);
        xc->setMiscReg(MISCREG_SEV_MAILBOX, true);
}

template <class XC>
inline bool
handleLockedWrite(XC *xc, Request *req, Addr cacheBlockMask)
{
    if (req->isSwap())
        return true;

    DPRINTF(LLSC,"%s: handling locked write for  address %#x in monitor\n",
            xc->getCpuPtr()->name(), req->getPaddr());
    // Verify that the lock flag is still set and the address
    // is correct
    bool lock_flag = xc->readMiscReg(MISCREG_LOCKFLAG);
    Addr lock_addr = xc->readMiscReg(MISCREG_LOCKADDR) & cacheBlockMask;
    if (!lock_flag || (req->getPaddr() & cacheBlockMask) != lock_addr) {
        // Lock flag not set or addr mismatch in CPU;
        // don't even bother sending to memory system
        req->setExtraData(0);
        xc->setMiscReg(MISCREG_LOCKFLAG, false);
        DPRINTF(LLSC,"%s: clearing lock flag in handle locked write\n",
                xc->getCpuPtr()->name());
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
    return true;
}


} // namespace ArmISA

#endif
