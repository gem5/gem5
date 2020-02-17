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
 */

#ifndef __ARCH_RISCV_LOCKED_MEM_HH__
#define __ARCH_RISCV_LOCKED_MEM_HH__

#include <stack>
#include <unordered_map>

#include "arch/registers.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/LLSC.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

/*
 * ISA-specific helper functions for locked memory accesses.
 */
namespace RiscvISA
{

const int WARN_FAILURE = 10000;

// RISC-V allows multiple locks per hart, but each SC has to unlock the most
// recent one, so we use a stack here.
extern std::unordered_map<int, std::stack<Addr>> locked_addrs;

template <class XC> inline void
handleLockedSnoop(XC *xc, PacketPtr pkt, Addr cacheBlockMask)
{
    std::stack<Addr>& locked_addr_stack = locked_addrs[xc->contextId()];

    if (locked_addr_stack.empty())
        return;
    Addr snoop_addr = pkt->getAddr() & cacheBlockMask;
    DPRINTF(LLSC, "Locked snoop on address %x.\n", snoop_addr);
    if ((locked_addr_stack.top() & cacheBlockMask) == snoop_addr)
        locked_addr_stack.pop();
}


template <class XC> inline void
handleLockedRead(XC *xc, const RequestPtr &req)
{
    std::stack<Addr>& locked_addr_stack = locked_addrs[xc->contextId()];

    locked_addr_stack.push(req->getPaddr() & ~0xF);
    DPRINTF(LLSC, "[cid:%d]: Reserved address %x.\n",
            req->contextId(), req->getPaddr() & ~0xF);
}

template <class XC> inline void
handleLockedSnoopHit(XC *xc)
{}

template <class XC> inline bool
handleLockedWrite(XC *xc, const RequestPtr &req, Addr cacheBlockMask)
{
    std::stack<Addr>& locked_addr_stack = locked_addrs[xc->contextId()];

    // Normally RISC-V uses zero to indicate success and nonzero to indicate
    // failure (right now only 1 is reserved), but in gem5 zero indicates
    // failure and one indicates success, so here we conform to that (it should
    // be switched in the instruction's implementation)

    DPRINTF(LLSC, "[cid:%d]: locked_addrs empty? %s.\n", req->contextId(),
            locked_addr_stack.empty() ? "yes" : "no");
    if (!locked_addr_stack.empty()) {
        DPRINTF(LLSC, "[cid:%d]: addr = %x.\n", req->contextId(),
                req->getPaddr() & ~0xF);
        DPRINTF(LLSC, "[cid:%d]: last locked addr = %x.\n", req->contextId(),
                locked_addr_stack.top());
    }
    if (locked_addr_stack.empty()
            || locked_addr_stack.top() != ((req->getPaddr() & ~0xF))) {
        req->setExtraData(0);
        int stCondFailures = xc->readStCondFailures();
        xc->setStCondFailures(++stCondFailures);
        if (stCondFailures % WARN_FAILURE == 0) {
            warn("%i: context %d: %d consecutive SC failures.\n",
                    curTick(), xc->contextId(), stCondFailures);
        }
        return false;
    }
    if (req->isUncacheable()) {
        req->setExtraData(2);
    }
    return true;
}

template <class XC>
inline void
globalClearExclusive(XC *xc)
{
    xc->getCpuPtr()->wakeup(xc->threadId());
}

} // namespace RiscvISA

#endif // __ARCH_RISCV_LOCKED_MEM_HH__
