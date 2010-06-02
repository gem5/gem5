/*
 * Copyright (c) 2010 ARM Limited
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

#include "arch/arm/faults.hh"
#include "arch/arm/table_walker.hh"
#include "arch/arm/tlb.hh"
#include "dev/io_device.hh"
#include "cpu/thread_context.hh"


using namespace ArmISA;

TableWalker::TableWalker(const Params *p)
    : MemObject(p), port(NULL), tlb(NULL), tc(NULL), req(NULL),
      doL1DescEvent(this), doL2DescEvent(this)
{}

TableWalker::~TableWalker()
{
    ;
}


unsigned int
drain(Event *de)
{
    panic("Not implemented\n");
}

Port*
TableWalker::getPort(const std::string &if_name, int idx)
{
    if (if_name == "port") {
        if (port != NULL)
            fatal("%s: port already connected to %s",
                  name(), port->getPeer()->name());
        System *sys = params()->sys;
        Tick minb = params()->min_backoff;
        Tick maxb = params()->max_backoff;
        port = new DmaPort(this, sys, minb, maxb);
        return port;
    }
    return NULL;
}

Fault
TableWalker::walk(RequestPtr _req, ThreadContext *_tc, uint8_t _cid, TLB::Mode mode,
            TLB::Translation *_trans, bool _timing)
{
    // Right now 1 CPU == 1 TLB == 1 TLB walker
    // In the future we might want to change this as multiple
    // threads/contexts could share a walker and/or a TLB
    if (tc || req)
        panic("Overlapping TLB walks attempted\n");

    tc = _tc;
    transState = _trans;
    req = _req;
    fault = NoFault;
    contextId = _cid;
    timing = _timing;

    // XXX These should be cached or grabbed from cached copies in
    // the TLB, all these miscreg reads are expensive
    vaddr = req->getVaddr() & ~PcModeMask;
    sctlr = tc->readMiscReg(MISCREG_SCTLR);
    cpsr = tc->readMiscReg(MISCREG_CPSR);
    N = tc->readMiscReg(MISCREG_TTBCR);
    Addr ttbr = 0;

    isFetch = (mode == TLB::Execute);
    isWrite = (mode == TLB::Write);
    isPriv = (cpsr.mode != MODE_USER);

    // If translation isn't enabled, we shouldn't be here
    assert(sctlr.m);

    if (N == 0 || mbits(vaddr, 31, 32-N)) {
        ttbr = tc->readMiscReg(MISCREG_TTBR0);
    } else {
        ttbr = tc->readMiscReg(MISCREG_TTBR0);
        N = 0;
    }

    Addr l1desc_addr = mbits(ttbr, 31, 14-N) | (bits(vaddr,31-N,20) << 2);
    DPRINTF(TLB, "Begining table walk for address %#x at descriptor %#x\n",
            vaddr, l1desc_addr);


    // Trickbox address check
    fault = tlb->walkTrickBoxCheck(l1desc_addr, vaddr, sizeof(uint32_t),
            isFetch, 0, true);
    if (fault) {
       tc = NULL;
       req = NULL;
       return fault;
    }

    if (timing) {
        port->dmaAction(MemCmd::ReadReq, l1desc_addr, sizeof(uint32_t),
                &doL1DescEvent, (uint8_t*)&l1Desc.data, (Tick)0);
    } else {
        port->dmaAction(MemCmd::ReadReq, l1desc_addr, sizeof(uint32_t),
                NULL, (uint8_t*)&l1Desc.data, (Tick)0);
        doL1Descriptor();
    }

    return fault;
}

void
TableWalker::memAttrs(TlbEntry &te, uint8_t texcb)
{

    if (sctlr.tre == 0) {
        switch(texcb) {
          case 0:
          case 1:
          case 4:
          case 8:
            te.nonCacheable = true;
            break;
          case 16:
            if (bits(texcb, 1,0) == 0 || bits(texcb, 3,2) == 0)
                te.nonCacheable = true;
            break;
        }
    } else {
        PRRR prrr = tc->readMiscReg(MISCREG_PRRR);
        NMRR nmrr = tc->readMiscReg(MISCREG_NMRR);
        switch(bits(texcb, 2,0)) {
          case 0:
            if (nmrr.ir0 == 0 || nmrr.or0 == 0 || prrr.tr0 != 0x2)
                te.nonCacheable = true;
            break;
          case 1:
            if (nmrr.ir1 == 0 || nmrr.or1 == 0 || prrr.tr1 != 0x2)
                te.nonCacheable = true;
            break;
          case 2:
            if (nmrr.ir2 == 0 || nmrr.or2 == 0 || prrr.tr2 != 0x2)
                te.nonCacheable = true;
            break;
          case 3:
            if (nmrr.ir3 == 0 || nmrr.or3 == 0 || prrr.tr3 != 0x2)
                te.nonCacheable = true;
            break;
          case 4:
            if (nmrr.ir4 == 0 || nmrr.or4 == 0 || prrr.tr4 != 0x2)
                te.nonCacheable = true;
            break;
          case 5:
            if (nmrr.ir5 == 0 || nmrr.or5 == 0 || prrr.tr5 != 0x2)
                te.nonCacheable = true;
            break;
          case 6:
            panic("Imp defined type\n");
          case 7:
            if (nmrr.ir7 == 0 || nmrr.or7 == 0 || prrr.tr7 != 0x2)
                te.nonCacheable = true;
            break;
        }
    }
}

void
TableWalker::doL1Descriptor()
{
    DPRINTF(TLB, "L1 descriptor for %#x is %#x\n", vaddr, l1Desc.data);
    TlbEntry te;

    switch (l1Desc.type()) {
      case L1Descriptor::Ignore:
      case L1Descriptor::Reserved:
        tc = NULL;
        req = NULL;
        fault = new DataAbort(vaddr, NULL, isWrite, ArmFault::Translation0);
        return;
      case L1Descriptor::Section:
        if (sctlr.afe && bits(l1Desc.ap(), 0) == 0)
            panic("Haven't implemented AFE\n");

        if (l1Desc.supersection()) {
            panic("Haven't implemented supersections\n");
        }
        te.N = 20;
        te.pfn = l1Desc.pfn();
        te.size = (1<<te.N) - 1;
        te.global = !l1Desc.global();
        te.valid = true;
        te.vpn = vaddr >> te.N;
        te.sNp = true;
        te.xn = l1Desc.xn();
        te.ap =  l1Desc.ap();
        te.domain = l1Desc.domain();
        te.asid = contextId;
        memAttrs(te, l1Desc.texcb());

        DPRINTF(TLB, "Inserting Section Descriptor into TLB\n");
        DPRINTF(TLB, " - N%d pfn:%#x size: %#x global:%d valid: %d\n",
                te.N, te.pfn, te.size, te.global, te.valid);
        DPRINTF(TLB, " - vpn:%#x sNp: %d xn:%d ap:%d domain: %d asid:%d\n",
                te.vpn, te.sNp, te.xn, te.ap, te.domain, te.asid);
        DPRINTF(TLB, " - domain from l1 desc: %d data: %#x bits:%d\n",
                l1Desc.domain(), l1Desc.data, (l1Desc.data >> 5) & 0xF );

        tc = NULL;
        req = NULL;
        tlb->insert(vaddr, te);

        return;
      case L1Descriptor::PageTable:
        Addr l2desc_addr;
        l2desc_addr = l1Desc.l2Addr() | (bits(vaddr, 19,12) << 2);
        DPRINTF(TLB, "L1 descriptor points to page table at: %#x\n", l2desc_addr);

        // Trickbox address check
        fault = tlb->walkTrickBoxCheck(l2desc_addr, vaddr, sizeof(uint32_t),
                isFetch, l1Desc.domain(), false);
        if (fault) {
           tc = NULL;
           req = NULL;
           return;
        }


        if (timing) {
            port->dmaAction(MemCmd::ReadReq, l2desc_addr, sizeof(uint32_t),
                    &doL2DescEvent, (uint8_t*)&l2Desc.data, 0);
        } else {
            port->dmaAction(MemCmd::ReadReq, l2desc_addr, sizeof(uint32_t),
                    NULL, (uint8_t*)&l2Desc.data, 0);
            doL2Descriptor();
        }
        return;
      default:
        panic("A new type in a 2 bit field?\n");
    }
}

void
TableWalker::doL2Descriptor()
{
    DPRINTF(TLB, "L2 descriptor for %#x is %#x\n", vaddr, l2Desc.data);
    TlbEntry te;

    if (sctlr.afe && bits(l1Desc.ap(), 0) == 0)
        panic("Haven't implemented AFE\n");

    if (l2Desc.invalid()) {
        DPRINTF(TLB, "L2 descriptor invalid, causing fault\n");
        tc = NULL;
        req = NULL;
        fault = new DataAbort(vaddr, l1Desc.domain(), isWrite, ArmFault::Translation1);
        return;
    }

    if (l2Desc.large()) {
      te.N = 16;
      te.pfn = l2Desc.pfn();
    } else {
      te.N = 12;
      te.pfn = l2Desc.pfn();
    }

    te.valid = true;
    te.size =  (1 << te.N) - 1;
    te.asid = contextId;
    te.sNp = false;
    te.vpn = vaddr >> te.N;
    te.global = l2Desc.global();
    te.xn = l2Desc.xn();
    te.ap = l2Desc.ap();
    te.domain = l1Desc.domain();
    memAttrs(te, l2Desc.texcb());

    tc = NULL;
    req = NULL;
    tlb->insert(vaddr, te);
}

ArmISA::TableWalker *
ArmTableWalkerParams::create()
{
    return new ArmISA::TableWalker(this);
}

