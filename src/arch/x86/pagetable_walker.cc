/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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
 */

#include "arch/x86/pagetable.hh"
#include "arch/x86/pagetable_walker.hh"
#include "arch/x86/tlb.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"
#include "sim/system.hh"

namespace X86ISA {

// Unfortunately, the placement of the base field in a page table entry is
// very erratic and would make a mess here. It might be moved here at some
// point in the future.
BitUnion64(PageTableEntry)
    Bitfield<63> nx;
    Bitfield<11, 9> avl;
    Bitfield<8> g;
    Bitfield<7> ps;
    Bitfield<6> d;
    Bitfield<5> a;
    Bitfield<4> pcd;
    Bitfield<3> pwt;
    Bitfield<2> u;
    Bitfield<1> w;
    Bitfield<0> p;
EndBitUnion(PageTableEntry)

Fault
Walker::doNext(PacketPtr &write)
{
    assert(state != Ready && state != Waiting);
    write = NULL;
    PageTableEntry pte;
    if (size == 8)
        pte = read->get<uint64_t>();
    else
        pte = read->get<uint32_t>();
    VAddr vaddr = entry.vaddr;
    bool uncacheable = pte.pcd;
    Addr nextRead = 0;
    bool doWrite = false;
    bool badNX = pte.nx && mode == BaseTLB::Execute && enableNX;
    switch(state) {
      case LongPML4:
        DPRINTF(PageTableWalker,
                "Got long mode PML4 entry %#016x.\n", (uint64_t)pte);
        nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.longl3 * size;
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (badNX || !pte.p) {
            stop();
            return pageFault(pte.p);
        }
        entry.noExec = pte.nx;
        nextState = LongPDP;
        break;
      case LongPDP:
        DPRINTF(PageTableWalker,
                "Got long mode PDP entry %#016x.\n", (uint64_t)pte);
        nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.longl2 * size;
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            stop();
            return pageFault(pte.p);
        }
        nextState = LongPD;
        break;
      case LongPD:
        DPRINTF(PageTableWalker,
                "Got long mode PD entry %#016x.\n", (uint64_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            stop();
            return pageFault(pte.p);
        }
        if (!pte.ps) {
            // 4 KB page
            entry.size = 4 * (1 << 10);
            nextRead =
                ((uint64_t)pte & (mask(40) << 12)) + vaddr.longl1 * size;
            nextState = LongPTE;
            break;
        } else {
            // 2 MB page
            entry.size = 2 * (1 << 20);
            entry.paddr = (uint64_t)pte & (mask(31) << 21);
            entry.uncacheable = uncacheable;
            entry.global = pte.g;
            entry.patBit = bits(pte, 12);
            entry.vaddr = entry.vaddr & ~((2 * (1 << 20)) - 1);
            tlb->insert(entry.vaddr, entry);
            stop();
            return NoFault;
        }
      case LongPTE:
        DPRINTF(PageTableWalker,
                "Got long mode PTE entry %#016x.\n", (uint64_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            stop();
            return pageFault(pte.p);
        }
        entry.paddr = (uint64_t)pte & (mask(40) << 12);
        entry.uncacheable = uncacheable;
        entry.global = pte.g;
        entry.patBit = bits(pte, 12);
        entry.vaddr = entry.vaddr & ~((4 * (1 << 10)) - 1);
        tlb->insert(entry.vaddr, entry);
        stop();
        return NoFault;
      case PAEPDP:
        DPRINTF(PageTableWalker,
                "Got legacy mode PAE PDP entry %#08x.\n", (uint32_t)pte);
        nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.pael2 * size;
        if (!pte.p) {
            stop();
            return pageFault(pte.p);
        }
        nextState = PAEPD;
        break;
      case PAEPD:
        DPRINTF(PageTableWalker,
                "Got legacy mode PAE PD entry %#08x.\n", (uint32_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (badNX || !pte.p) {
            stop();
            return pageFault(pte.p);
        }
        if (!pte.ps) {
            // 4 KB page
            entry.size = 4 * (1 << 10);
            nextRead = ((uint64_t)pte & (mask(40) << 12)) + vaddr.pael1 * size;
            nextState = PAEPTE;
            break;
        } else {
            // 2 MB page
            entry.size = 2 * (1 << 20);
            entry.paddr = (uint64_t)pte & (mask(31) << 21);
            entry.uncacheable = uncacheable;
            entry.global = pte.g;
            entry.patBit = bits(pte, 12);
            entry.vaddr = entry.vaddr & ~((2 * (1 << 20)) - 1);
            tlb->insert(entry.vaddr, entry);
            stop();
            return NoFault;
        }
      case PAEPTE:
        DPRINTF(PageTableWalker,
                "Got legacy mode PAE PTE entry %#08x.\n", (uint32_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = entry.writable && pte.w;
        entry.user = entry.user && pte.u;
        if (badNX || !pte.p) {
            stop();
            return pageFault(pte.p);
        }
        entry.paddr = (uint64_t)pte & (mask(40) << 12);
        entry.uncacheable = uncacheable;
        entry.global = pte.g;
        entry.patBit = bits(pte, 7);
        entry.vaddr = entry.vaddr & ~((4 * (1 << 10)) - 1);
        tlb->insert(entry.vaddr, entry);
        stop();
        return NoFault;
      case PSEPD:
        DPRINTF(PageTableWalker,
                "Got legacy mode PSE PD entry %#08x.\n", (uint32_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (!pte.p) {
            stop();
            return pageFault(pte.p);
        }
        if (!pte.ps) {
            // 4 KB page
            entry.size = 4 * (1 << 10);
            nextRead =
                ((uint64_t)pte & (mask(20) << 12)) + vaddr.norml2 * size;
            nextState = PTE;
            break;
        } else {
            // 4 MB page
            entry.size = 4 * (1 << 20);
            entry.paddr = bits(pte, 20, 13) << 32 | bits(pte, 31, 22) << 22;
            entry.uncacheable = uncacheable;
            entry.global = pte.g;
            entry.patBit = bits(pte, 12);
            entry.vaddr = entry.vaddr & ~((4 * (1 << 20)) - 1);
            tlb->insert(entry.vaddr, entry);
            stop();
            return NoFault;
        }
      case PD:
        DPRINTF(PageTableWalker,
                "Got legacy mode PD entry %#08x.\n", (uint32_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (!pte.p) {
            stop();
            return pageFault(pte.p);
        }
        // 4 KB page
        entry.size = 4 * (1 << 10);
        nextRead = ((uint64_t)pte & (mask(20) << 12)) + vaddr.norml2 * size;
        nextState = PTE;
        break;
      case PTE:
        DPRINTF(PageTableWalker,
                "Got legacy mode PTE entry %#08x.\n", (uint32_t)pte);
        doWrite = !pte.a;
        pte.a = 1;
        entry.writable = pte.w;
        entry.user = pte.u;
        if (!pte.p) {
            stop();
            return pageFault(pte.p);
        }
        entry.paddr = (uint64_t)pte & (mask(20) << 12);
        entry.uncacheable = uncacheable;
        entry.global = pte.g;
        entry.patBit = bits(pte, 7);
        entry.vaddr = entry.vaddr & ~((4 * (1 << 10)) - 1);
        tlb->insert(entry.vaddr, entry);
        stop();
        return NoFault;
      default:
        panic("Unknown page table walker state %d!\n");
    }
    PacketPtr oldRead = read;
    //If we didn't return, we're setting up another read.
    Request::Flags flags = oldRead->req->getFlags();
    flags.set(Request::UNCACHEABLE, uncacheable);
    RequestPtr request =
        new Request(nextRead, oldRead->getSize(), flags);
    read = new Packet(request, MemCmd::ReadExReq, Packet::Broadcast);
    read->allocate();
    //If we need to write, adjust the read packet to write the modified value
    //back to memory.
    if (doWrite) {
        write = oldRead;
        write->set<uint64_t>(pte);
        write->cmd = MemCmd::WriteReq;
        write->setDest(Packet::Broadcast);
    } else {
        write = NULL;
        delete oldRead->req;
        delete oldRead;
    }
    return NoFault;
}

Fault
Walker::start(ThreadContext * _tc, BaseTLB::Translation *_translation,
              RequestPtr _req, BaseTLB::Mode _mode)
{
    assert(state == Ready);
    tc = _tc;
    req = _req;
    Addr vaddr = req->getVaddr();
    mode = _mode;
    translation = _translation;

    VAddr addr = vaddr;

    //Figure out what we're doing.
    CR3 cr3 = tc->readMiscRegNoEffect(MISCREG_CR3);
    Addr top = 0;
    // Check if we're in long mode or not
    Efer efer = tc->readMiscRegNoEffect(MISCREG_EFER);
    size = 8;
    if (efer.lma) {
        // Do long mode.
        state = LongPML4;
        top = (cr3.longPdtb << 12) + addr.longl4 * size;
        enableNX = efer.nxe;
    } else {
        // We're in some flavor of legacy mode.
        CR4 cr4 = tc->readMiscRegNoEffect(MISCREG_CR4);
        if (cr4.pae) {
            // Do legacy PAE.
            state = PAEPDP;
            top = (cr3.paePdtb << 5) + addr.pael3 * size;
            enableNX = efer.nxe;
        } else {
            size = 4;
            top = (cr3.pdtb << 12) + addr.norml2 * size;
            if (cr4.pse) {
                // Do legacy PSE.
                state = PSEPD;
            } else {
                // Do legacy non PSE.
                state = PD;
            }
            enableNX = false;
        }
    }

    nextState = Ready;
    entry.vaddr = vaddr;

    Request::Flags flags = Request::PHYSICAL;
    if (cr3.pcd)
        flags.set(Request::UNCACHEABLE);
    RequestPtr request = new Request(top, size, flags);
    read = new Packet(request, MemCmd::ReadExReq, Packet::Broadcast);
    read->allocate();
    Enums::MemoryMode memMode = sys->getMemoryMode();
    if (memMode == Enums::timing) {
        nextState = state;
        state = Waiting;
        timingFault = NoFault;
        sendPackets();
    } else if (memMode == Enums::atomic) {
        Fault fault;
        do {
            port.sendAtomic(read);
            PacketPtr write = NULL;
            fault = doNext(write);
            assert(fault == NoFault || read == NULL);
            state = nextState;
            nextState = Ready;
            if (write)
                port.sendAtomic(write);
        } while(read);
        state = Ready;
        nextState = Waiting;
        return fault;
    } else {
        panic("Unrecognized memory system mode.\n");
    }
    return NoFault;
}

bool
Walker::WalkerPort::recvTiming(PacketPtr pkt)
{
    return walker->recvTiming(pkt);
}

bool
Walker::recvTiming(PacketPtr pkt)
{
    if (pkt->isResponse() && !pkt->wasNacked()) {
        assert(inflight);
        assert(state == Waiting);
        assert(!read);
        inflight--;
        if (pkt->isRead()) {
            state = nextState;
            nextState = Ready;
            PacketPtr write = NULL;
            read = pkt;
            timingFault = doNext(write);
            state = Waiting;
            assert(timingFault == NoFault || read == NULL);
            if (write) {
                writes.push_back(write);
            }
            sendPackets();
        } else {
            sendPackets();
        }
        if (inflight == 0 && read == NULL && writes.size() == 0) {
            state = Ready;
            nextState = Waiting;
            if (timingFault == NoFault) {
                /*
                 * Finish the translation. Now that we now the right entry is
                 * in the TLB, this should work with no memory accesses.
                 * There could be new faults unrelated to the table walk like
                 * permissions violations, so we'll need the return value as
                 * well.
                 */
                bool delayedResponse;
                Fault fault = tlb->translate(req, tc, NULL, mode,
                        delayedResponse, true);
                assert(!delayedResponse);
                // Let the CPU continue.
                translation->finish(fault, req, tc, mode);
            } else {
                // There was a fault during the walk. Let the CPU know.
                translation->finish(timingFault, req, tc, mode);
            }
        }
    } else if (pkt->wasNacked()) {
        pkt->reinitNacked();
        if (!port.sendTiming(pkt)) {
            inflight--;
            retrying = true;
            if (pkt->isWrite()) {
                writes.push_back(pkt);
            } else {
                assert(!read);
                read = pkt;
            }
        }
    }
    return true;
}

Tick
Walker::WalkerPort::recvAtomic(PacketPtr pkt)
{
    return 0;
}

void
Walker::WalkerPort::recvFunctional(PacketPtr pkt)
{
    return;
}

void
Walker::WalkerPort::recvStatusChange(Status status)
{
    if (status == RangeChange) {
        if (!snoopRangeSent) {
            snoopRangeSent = true;
            sendStatusChange(Port::RangeChange);
        }
        return;
    }

    panic("Unexpected recvStatusChange.\n");
}

void
Walker::WalkerPort::recvRetry()
{
    walker->recvRetry();
}

void
Walker::recvRetry()
{
    retrying = false;
    sendPackets();
}

void
Walker::sendPackets()
{
    //If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    //Reads always have priority
    if (read) {
        PacketPtr pkt = read;
        read = NULL;
        inflight++;
        if (!port.sendTiming(pkt)) {
            retrying = true;
            read = pkt;
            inflight--;
            return;
        }
    }
    //Send off as many of the writes as we can.
    while (writes.size()) {
        PacketPtr write = writes.back();
        writes.pop_back();
        inflight++;
        if (!port.sendTiming(write)) {
            retrying = true;
            writes.push_back(write);
            inflight--;
            return;
        }
    }
}

Port *
Walker::getPort(const std::string &if_name, int idx)
{
    if (if_name == "port")
        return &port;
    else
        panic("No page table walker port named %s!\n", if_name);
}

Fault
Walker::pageFault(bool present)
{
    DPRINTF(PageTableWalker, "Raising page fault.\n");
    HandyM5Reg m5reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
    if (mode == BaseTLB::Execute && !enableNX)
        mode = BaseTLB::Read;
    return new PageFault(entry.vaddr, present, mode, m5reg.cpl == 3, false);
}

}

X86ISA::Walker *
X86PagetableWalkerParams::create()
{
    return new X86ISA::Walker(this);
}
