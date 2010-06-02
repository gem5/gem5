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
TableWalker::walk(RequestPtr _req, ThreadContext *_tc, uint8_t _cid, TLB::Mode _mode,
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
    mode = _mode;

    /** @todo These should be cached or grabbed from cached copies in
     the TLB, all these miscreg reads are expensive */
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

    DPRINTF(TLB, "Begining table walk for address %#x, TTBCR: %#x, bits:%#x\n",
            vaddr, N, mbits(vaddr, 31, 32-N));

    if (N == 0 || !mbits(vaddr, 31, 32-N)) {
        DPRINTF(TLB, " - Selecting TTBR0\n");
        ttbr = tc->readMiscReg(MISCREG_TTBR0);
    } else {
        DPRINTF(TLB, " - Selecting TTBR1\n");
        ttbr = tc->readMiscReg(MISCREG_TTBR1);
        N = 0;
    }

    Addr l1desc_addr = mbits(ttbr, 31, 14-N) | (bits(vaddr,31-N,20) << 2);
    DPRINTF(TLB, " - Descriptor at address %#x\n", l1desc_addr);


    // Trickbox address check
    fault = tlb->walkTrickBoxCheck(l1desc_addr, vaddr, sizeof(uint32_t),
            isFetch, isWrite, 0, true);
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
TableWalker::memAttrs(TlbEntry &te, uint8_t texcb, bool s)
{
    DPRINTF(TLBVerbose, "memAttrs texcb:%d s:%d\n", texcb, s);
    te.shareable = false; // default value
    bool outer_shareable = false;
    if (sctlr.tre == 0) {
        switch(texcb) {
          case 0: // Stongly-ordered
            te.nonCacheable = true;
            te.mtype = TlbEntry::StronglyOrdered;
            te.shareable = true;
            te.innerAttrs = 1;
            te.outerAttrs = 0;
            break;
          case 1: // Shareable Device
            te.nonCacheable = true;
            te.mtype = TlbEntry::Device;
            te.shareable = true;
            te.innerAttrs = 3;
            te.outerAttrs = 0;
            break;
          case 2: // Outer and Inner Write-Through, no Write-Allocate
            te.mtype = TlbEntry::Normal;
            te.shareable = s;
            te.innerAttrs = 6;
            te.outerAttrs = bits(texcb, 1, 0);
            break;
          case 3: // Outer and Inner Write-Back, no Write-Allocate
            te.mtype = TlbEntry::Normal;
            te.shareable = s;
            te.innerAttrs = 7;
            te.outerAttrs = bits(texcb, 1, 0);
            break;
          case 4: // Outer and Inner Non-cacheable
            te.nonCacheable = true;
            te.mtype = TlbEntry::Normal;
            te.shareable = s;
            te.innerAttrs = 0;
            te.outerAttrs = bits(texcb, 1, 0);
            break;
          case 5: // Reserved
            break;
          case 6: // Implementation Defined
            break;
          case 7: // Outer and Inner Write-Back, Write-Allocate
            te.mtype = TlbEntry::Normal;
            te.shareable = s;
            te.innerAttrs = 5;
            te.outerAttrs = 1;
            break;
          case 8: // Non-shareable Device
            te.nonCacheable = true;
            te.mtype = TlbEntry::Device;
            te.shareable = false;
            te.innerAttrs = 3;
            te.outerAttrs = 0;
            break;
          case 9 ... 15:  // Reserved
            break;
          case 16 ... 31: // Cacheable Memory
            te.mtype = TlbEntry::Normal;
            te.shareable = s;
            if (bits(texcb, 1,0) == 0 || bits(texcb, 3,2) == 0)
                te.nonCacheable = true;
            te.innerAttrs = bits(texcb, 1, 0);
            te.outerAttrs = bits(texcb, 3, 2);
            break;
          default:
            panic("More than 32 states for 5 bits?\n");
        }
    } else {
        PRRR prrr = tc->readMiscReg(MISCREG_PRRR);
        NMRR nmrr = tc->readMiscReg(MISCREG_NMRR);
        DPRINTF(TLBVerbose, "memAttrs PRRR:%08x NMRR:%08x\n", prrr, nmrr);
        uint8_t curr_tr, curr_ir, curr_or;
        switch(bits(texcb, 2,0)) {
          case 0:
            curr_tr = prrr.tr0;
            curr_ir = nmrr.ir0;
            curr_or = nmrr.or0;
            outer_shareable = (prrr.nos0 == 0);
            break;
          case 1:
            curr_tr = prrr.tr1;
            curr_ir = nmrr.ir1;
            curr_or = nmrr.or1;
            outer_shareable = (prrr.nos1 == 0);
            break;
          case 2:
            curr_tr = prrr.tr2;
            curr_ir = nmrr.ir2;
            curr_or = nmrr.or2;
            outer_shareable = (prrr.nos2 == 0);
            break;
          case 3:
            curr_tr = prrr.tr3;
            curr_ir = nmrr.ir3;
            curr_or = nmrr.or3;
            outer_shareable = (prrr.nos3 == 0);
            break;
          case 4:
            curr_tr = prrr.tr4;
            curr_ir = nmrr.ir4;
            curr_or = nmrr.or4;
            outer_shareable = (prrr.nos4 == 0);
            break;
          case 5:
            curr_tr = prrr.tr5;
            curr_ir = nmrr.ir5;
            curr_or = nmrr.or5;
            outer_shareable = (prrr.nos5 == 0);
            break;
          case 6:
            panic("Imp defined type\n");
          case 7:
            curr_tr = prrr.tr7;
            curr_ir = nmrr.ir7;
            curr_or = nmrr.or7;
            outer_shareable = (prrr.nos7 == 0);
            break;
        }

        switch(curr_tr) {
          case 0:
            DPRINTF(TLBVerbose, "StronglyOrdered\n");
            te.mtype = TlbEntry::StronglyOrdered;
            te.nonCacheable = true;
            te.innerAttrs = 1;
            te.outerAttrs = 0;
            te.shareable = true;
            break;
          case 1:
            DPRINTF(TLBVerbose, "Device ds1:%d ds0:%d s:%d\n",
                    prrr.ds1, prrr.ds0, s);
            te.mtype = TlbEntry::Device;
            te.nonCacheable = true;
            te.innerAttrs = 3;
            te.outerAttrs = 0;
            if (prrr.ds1 && s)
                te.shareable = true;
            if (prrr.ds0 && !s)
                te.shareable = true;
            break;
          case 2:
            DPRINTF(TLBVerbose, "Normal ns1:%d ns0:%d s:%d\n",
                    prrr.ns1, prrr.ns0, s);
            te.mtype = TlbEntry::Normal;
            if (prrr.ns1 && s)
                te.shareable = true;
            if (prrr.ns0 && !s)
                te.shareable = true;
            //te.shareable = outer_shareable;
            break;
          case 3:
            panic("Reserved type");
        }

        if (te.mtype == TlbEntry::Normal){
            switch(curr_ir) {
              case 0:
                te.nonCacheable = true;
                te.innerAttrs = 0;
                break;
              case 1:
                te.innerAttrs = 5;
                break;
              case 2:
                te.innerAttrs = 6;
                break;
              case 3:
                te.innerAttrs = 7;
                break;
            }

            switch(curr_or) {
              case 0:
                te.nonCacheable = true;
                te.outerAttrs = 0;
                break;
              case 1:
                te.outerAttrs = 1;
                break;
              case 2:
                te.outerAttrs = 2;
                break;
              case 3:
                te.outerAttrs = 3;
                break;
            }
        }
    }

    /** Formatting for Physical Address Register (PAR)
     *  Only including lower bits (TLB info here)
     *  PAR:
     *  PA [31:12]
     *  Reserved [11]
     *  TLB info [10:1]
     *      NOS  [10] (Not Outer Sharable)
     *      NS   [9]  (Non-Secure)
     *      --   [8]  (Implementation Defined)
     *      SH   [7]  (Sharable)
     *      Inner[6:4](Inner memory attributes)
     *      Outer[3:2](Outer memory attributes)
     *      SS   [1]  (SuperSection)
     *      F    [0]  (Fault, Fault Status in [6:1] if faulted)
     */
    te.attributes = (
                ((outer_shareable ? 0:1) << 10) |
                // TODO: NS Bit
                ((te.shareable ? 1:0) << 7) |
                (te.innerAttrs << 4) |
                (te.outerAttrs << 2)
                // TODO: Supersection bit
                // TODO: Fault bit
                );


}

void
TableWalker::doL1Descriptor()
{
    DPRINTF(TLB, "L1 descriptor for %#x is %#x\n", vaddr, l1Desc.data);
    TlbEntry te;

    switch (l1Desc.type()) {
      case L1Descriptor::Ignore:
      case L1Descriptor::Reserved:
        if (!delayed) {
            tc = NULL;
            req = NULL;
        }
        DPRINTF(TLB, "L1 Descriptor Reserved/Ignore, causing fault\n");
        if (isFetch)
            fault = new PrefetchAbort(vaddr, ArmFault::Translation0);
        else
            fault = new DataAbort(vaddr, NULL, isWrite,
                                  ArmFault::Translation0);
        return;
      case L1Descriptor::Section:
        if (sctlr.afe && bits(l1Desc.ap(), 0) == 0) {
            /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is
              * enabled if set, do l1.Desc.setAp0() instead of generating
              * AccessFlag0
              */

            fault = new DataAbort(vaddr, NULL, isWrite,
                                    ArmFault::AccessFlag0);
        }

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
        memAttrs(te, l1Desc.texcb(), l1Desc.shareable());

        DPRINTF(TLB, "Inserting Section Descriptor into TLB\n");
        DPRINTF(TLB, " - N%d pfn:%#x size: %#x global:%d valid: %d\n",
                te.N, te.pfn, te.size, te.global, te.valid);
        DPRINTF(TLB, " - vpn:%#x sNp: %d xn:%d ap:%d domain: %d asid:%d\n",
                te.vpn, te.sNp, te.xn, te.ap, te.domain, te.asid);
        DPRINTF(TLB, " - domain from l1 desc: %d data: %#x bits:%d\n",
                l1Desc.domain(), l1Desc.data, (l1Desc.data >> 5) & 0xF );

        if (!timing) {
            tc = NULL;
            req = NULL;
        }
        tlb->insert(vaddr, te);

        return;
      case L1Descriptor::PageTable:
        Addr l2desc_addr;
        l2desc_addr = l1Desc.l2Addr() | (bits(vaddr, 19,12) << 2);
        DPRINTF(TLB, "L1 descriptor points to page table at: %#x\n",
                l2desc_addr);

        // Trickbox address check
        fault = tlb->walkTrickBoxCheck(l2desc_addr, vaddr, sizeof(uint32_t),
                isFetch, isWrite, l1Desc.domain(), false);
        if (fault) {
            if (!timing) {
                tc = NULL;
                req = NULL;
            }
            return;
        }


        if (timing) {
            delayed = true;
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

    if (l2Desc.invalid()) {
        DPRINTF(TLB, "L2 descriptor invalid, causing fault\n");
        if (!delayed) {
            tc = NULL;
            req = NULL;
        }
        if (isFetch)
            fault = new PrefetchAbort(vaddr, ArmFault::Translation1);
        else
            fault = new DataAbort(vaddr, l1Desc.domain(), isWrite,
                                    ArmFault::Translation1);
        return;
    }

    if (sctlr.afe && bits(l2Desc.ap(), 0) == 0) {
        /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is enabled
          * if set, do l2.Desc.setAp0() instead of generating AccessFlag0
          */

        fault = new DataAbort(vaddr, NULL, isWrite, ArmFault::AccessFlag1);
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
    memAttrs(te, l2Desc.texcb(), l2Desc.shareable());

    if (!delayed) {
        tc = NULL;
        req = NULL;
    }
    tlb->insert(vaddr, te);
}

void
TableWalker::doL1DescriptorWrapper()
{
    delayed = false;

    DPRINTF(TLBVerbose, "calling doL1Descriptor\n");
    doL1Descriptor();

    // Check if fault was generated
    if (fault != NoFault) {
        transState->finish(fault, req, tc, mode);

        req = NULL;
        tc = NULL;
        delayed = false;
    }
    else if (!delayed) {
        DPRINTF(TLBVerbose, "calling translateTiming again\n");
        fault = tlb->translateTiming(req, tc, transState, mode);

        req = NULL;
        tc = NULL;
        delayed = false;
    }
}

void
TableWalker::doL2DescriptorWrapper()
{
    assert(delayed);

    DPRINTF(TLBVerbose, "calling doL2Descriptor\n");
    doL2Descriptor();

    // Check if fault was generated
    if (fault != NoFault) {
        transState->finish(fault, req, tc, mode);
    }
    else {
        DPRINTF(TLBVerbose, "calling translateTiming again\n");
        fault = tlb->translateTiming(req, tc, transState, mode);
    }

    req = NULL;
    tc = NULL;
    delayed = false;
}

ArmISA::TableWalker *
ArmTableWalkerParams::create()
{
    return new ArmISA::TableWalker(this);
}

