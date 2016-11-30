/*  
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
 * All rights reserved.
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
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
 * Copyright (c) 2016 The University of Virginia
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
 *          Alec Roelke
 */
#ifndef __ARCH_RISCV_LOCKED_MEM_HH__
#define __ARCH_RISCV_LOCKED_MEM_HH__

#include "arch/registers.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "debug/LLSC.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

/*
 * ISA-specific helper functions for locked memory accesses.
 */
namespace RiscvISA
{
static bool lock_flag = false;
static Addr lock_addr = 0;

template <class XC>
inline void handleLockedSnoop(XC *xc, PacketPtr pkt, Addr cacheBlockMask)
{
    if (!lock_flag)
        return;

    DPRINTF(LLSC, "Locked snoop on address %x.\n",
            pkt->getAddr()&cacheBlockMask);

    Addr snoop_addr = pkt->getAddr()&cacheBlockMask;

    if ((lock_addr&cacheBlockMask) == snoop_addr)
        lock_flag = false;
}


template <class XC>
inline void handleLockedRead(XC *xc, Request *req)
{
    lock_addr = req->getPaddr()&~0xF;
    lock_flag = true;
    DPRINTF(LLSC, "[cid:%i]: "
            "Load-Link Flag Set & Load-Link Address set to %x.\n",
            req->contextId(), req->getPaddr()&~0xF);
}

template <class XC>
inline void handleLockedSnoopHit(XC *xc)
{}

template <class XC>
inline bool handleLockedWrite(XC *xc, Request *req, Addr cacheBlockMask)
{
    if (req->isUncacheable()) {
        // Funky Turbolaser mailbox access...don't update
        // result register (see stq_c in decoder.isa)
        req->setExtraData(2);
    } else {
        // standard store conditional
        if (!lock_flag || (req->getPaddr()&~0xF) != lock_addr) {
            // Lock flag not set or addr mismatch in CPU;
            // don't even bother sending to memory system
            req->setExtraData(0);
            lock_flag = false;

            // the rest of this code is not architectural;
            // it's just a debugging aid to help detect
            // livelock by warning on long sequences of failed
            // store conditionals
            int stCondFailures = xc->readStCondFailures();
            stCondFailures++;
            xc->setStCondFailures(stCondFailures);
            if (stCondFailures % 100000 == 0) {
                warn("%i:"" context %d:"
                        " %d consecutive store conditional failures\n",
                        curTick(), xc->contextId(), stCondFailures);
            }

            if (!lock_flag){
                DPRINTF(LLSC, "[cid:%i]:"
                        " Lock Flag Set, Store Conditional Failed.\n",
                        req->contextId());
            } else if ((req->getPaddr() & ~0xf) != lock_addr) {
                DPRINTF(LLSC, "[cid:%i]: Load-Link Address Mismatch, "
                        "Store Conditional Failed.\n", req->contextId());
            }
            // store conditional failed already, so don't issue it to mem
            return false;
        }
    }

    return true;
}

} // namespace RiscvISA

#endif // __ARCH_RISCV_LOCKED_MEM_HH__
