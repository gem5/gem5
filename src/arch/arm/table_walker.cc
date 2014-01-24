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
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Checkpoint.hh"
#include "debug/Drain.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "sim/system.hh"

using namespace ArmISA;

TableWalker::TableWalker(const Params *p)
    : MemObject(p), port(this, params()->sys), drainManager(NULL),
      tlb(NULL), currState(NULL), pending(false),
      masterId(p->sys->getMasterId(name())),
      numSquashable(p->num_squash_per_cycle),
      doL1DescEvent(this), doL2DescEvent(this), doProcessEvent(this)
{
    sctlr = 0;
}

TableWalker::~TableWalker()
{
    ;
}

void
TableWalker::completeDrain()
{
    if (drainManager && stateQueueL1.empty() && stateQueueL2.empty() &&
        pendingQueue.empty()) {
        setDrainState(Drainable::Drained);
        DPRINTF(Drain, "TableWalker done draining, processing drain event\n");
        drainManager->signalDrainDone();
        drainManager = NULL;
    }
}

unsigned int
TableWalker::drain(DrainManager *dm)
{
    unsigned int count = port.drain(dm);

    if (stateQueueL1.empty() && stateQueueL2.empty() &&
        pendingQueue.empty()) {
        setDrainState(Drainable::Drained);
        DPRINTF(Drain, "TableWalker free, no need to drain\n");

        // table walker is drained, but its ports may still need to be drained
        return count;
    } else {
        drainManager = dm;
        setDrainState(Drainable::Draining);
        DPRINTF(Drain, "TableWalker not drained\n");

        // return port drain count plus the table walker itself needs to drain
        return count + 1;

    }
}

void
TableWalker::drainResume()
{
    Drainable::drainResume();
    if (params()->sys->isTimingMode() && currState) {
        delete currState;
        currState = NULL;
    }
}

BaseMasterPort&
TableWalker::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    }
    return MemObject::getMasterPort(if_name, idx);
}

Fault
TableWalker::walk(RequestPtr _req, ThreadContext *_tc, uint8_t _cid, TLB::Mode _mode,
            TLB::Translation *_trans, bool _timing, bool _functional)
{
    assert(!(_functional && _timing));
    if (!currState) {
        // For atomic mode, a new WalkerState instance should be only created
        // once per TLB. For timing mode, a new instance is generated for every
        // TLB miss.
        DPRINTF(TLBVerbose, "creating new instance of WalkerState\n");

        currState = new WalkerState();
        currState->tableWalker = this;
    } else if (_timing) {
        // This is a translation that was completed and then faulted again
        // because some underlying parameters that affect the translation
        // changed out from under us (e.g. asid). It will either be a
        // misprediction, in which case nothing will happen or we'll use
        // this fault to re-execute the faulting instruction which should clean
        // up everything.
        if (currState->vaddr == _req->getVaddr()) {
            return new ReExec;
        }
        panic("currState should always be empty in timing mode!\n");
    }

    currState->tc = _tc;
    currState->transState = _trans;
    currState->req = _req;
    currState->fault = NoFault;
    currState->contextId = _cid;
    currState->timing = _timing;
    currState->functional = _functional;
    currState->mode = _mode;

    /** @todo These should be cached or grabbed from cached copies in
     the TLB, all these miscreg reads are expensive */
    currState->vaddr = currState->req->getVaddr();
    currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR);
    sctlr = currState->sctlr;
    currState->N = currState->tc->readMiscReg(MISCREG_TTBCR);

    currState->isFetch = (currState->mode == TLB::Execute);
    currState->isWrite = (currState->mode == TLB::Write);


    if (!currState->timing)
        return processWalk();

    if (pending || pendingQueue.size()) {
        pendingQueue.push_back(currState);
        currState = NULL;
    } else {
        pending = true;
        return processWalk();
    }

    return NoFault;
}

void
TableWalker::processWalkWrapper()
{
    assert(!currState);
    assert(pendingQueue.size());
    currState = pendingQueue.front();

    // Check if a previous walk filled this request already
    TlbEntry* te = tlb->lookup(currState->vaddr, currState->contextId, true);

    // Check if we still need to have a walk for this request. If the requesting
    // instruction has been squashed, or a previous walk has filled the TLB with
    // a match, we just want to get rid of the walk. The latter could happen
    // when there are multiple outstanding misses to a single page and a
    // previous request has been successfully translated.
    if (!currState->transState->squashed() && !te) {
        // We've got a valid request, lets process it
        pending = true;
        pendingQueue.pop_front();
        processWalk();
        return;
    }


    // If the instruction that we were translating for has been
    // squashed we shouldn't bother.
    unsigned num_squashed = 0;
    ThreadContext *tc = currState->tc;
    while ((num_squashed < numSquashable) && currState &&
           (currState->transState->squashed() || te)) {
        pendingQueue.pop_front();
        num_squashed++;

        DPRINTF(TLB, "Squashing table walk for address %#x\n", currState->vaddr);

        if (currState->transState->squashed()) {
            // finish the translation which will delete the translation object
            currState->transState->finish(new UnimpFault("Squashed Inst"),
                    currState->req, currState->tc, currState->mode);
        } else {
            // translate the request now that we know it will work
            currState->fault = tlb->translateTiming(currState->req, currState->tc,
                                      currState->transState, currState->mode);
        }

        // delete the current request
        delete currState;

        // peak at the next one
        if (pendingQueue.size()) {
            currState = pendingQueue.front();
            te = tlb->lookup(currState->vaddr, currState->contextId, true);
        } else {
            // Terminate the loop, nothing more to do
            currState = NULL;
        }
    }

    // if we've still got pending translations schedule more work
    nextWalk(tc);
    currState = NULL;
    completeDrain();
}

Fault
TableWalker::processWalk()
{
    Addr ttbr = 0;

    // If translation isn't enabled, we shouldn't be here
    assert(currState->sctlr.m);

    DPRINTF(TLB, "Begining table walk for address %#x, TTBCR: %#x, bits:%#x\n",
            currState->vaddr, currState->N, mbits(currState->vaddr, 31,
            32-currState->N));

    if (currState->N == 0 || !mbits(currState->vaddr, 31, 32-currState->N)) {
        DPRINTF(TLB, " - Selecting TTBR0\n");
        ttbr = currState->tc->readMiscReg(MISCREG_TTBR0);
    } else {
        DPRINTF(TLB, " - Selecting TTBR1\n");
        ttbr = currState->tc->readMiscReg(MISCREG_TTBR1);
        currState->N = 0;
    }

    Addr l1desc_addr = mbits(ttbr, 31, 14-currState->N) |
                       (bits(currState->vaddr,31-currState->N,20) << 2);
    DPRINTF(TLB, " - Descriptor at address %#x\n", l1desc_addr);


    // Trickbox address check
    Fault f;
    f = tlb->walkTrickBoxCheck(l1desc_addr, currState->vaddr, sizeof(uint32_t),
            currState->isFetch, currState->isWrite, 0, true);
    if (f) {
        DPRINTF(TLB, "Trickbox check caused fault on %#x\n", currState->vaddr);
        if (currState->timing) {
            pending = false;
            nextWalk(currState->tc);
            currState = NULL;
        } else {
            currState->tc = NULL;
            currState->req = NULL;
        }
        return f;
    }

    Request::Flags flag = 0;
    if (currState->sctlr.c == 0) {
        flag = Request::UNCACHEABLE;
    }

    if (currState->timing) {
        port.dmaAction(MemCmd::ReadReq, l1desc_addr, sizeof(uint32_t),
                       &doL1DescEvent, (uint8_t*)&currState->l1Desc.data,
                       currState->tc->getCpuPtr()->clockPeriod(), flag);
        DPRINTF(TLBVerbose, "Adding to walker fifo: queue size before "
                "adding: %d\n",
                stateQueueL1.size());
        stateQueueL1.push_back(currState);
        currState = NULL;
    } else if (!currState->functional) {
        port.dmaAction(MemCmd::ReadReq, l1desc_addr, sizeof(uint32_t),
                       NULL, (uint8_t*)&currState->l1Desc.data,
                       currState->tc->getCpuPtr()->clockPeriod(), flag);
        doL1Descriptor();
        f = currState->fault;
    } else {
        RequestPtr req = new Request(l1desc_addr, sizeof(uint32_t), flag, masterId);
        req->taskId(ContextSwitchTaskId::DMA);
        PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
        pkt->dataStatic((uint8_t*)&currState->l1Desc.data);
        port.sendFunctional(pkt);
        doL1Descriptor();
        delete req;
        delete pkt;
        f = currState->fault;
    }

    return f;
}

void
TableWalker::memAttrs(ThreadContext *tc, TlbEntry &te, SCTLR sctlr,
                      uint8_t texcb, bool s)
{
    // Note: tc and sctlr local variables are hiding tc and sctrl class
    // variables
    DPRINTF(TLBVerbose, "memAttrs texcb:%d s:%d\n", texcb, s);
    te.shareable = false; // default value
    te.nonCacheable = false;
    bool outer_shareable = false;
    if (sctlr.tre == 0 || ((sctlr.tre == 1) && (sctlr.m == 0))) {
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
            panic("Reserved texcb value!\n");
            break;
          case 6: // Implementation Defined
            panic("Implementation-defined texcb value!\n");
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
            panic("Reserved texcb value!\n");
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
        assert(tc);
        PRRR prrr = tc->readMiscReg(MISCREG_PRRR);
        NMRR nmrr = tc->readMiscReg(MISCREG_NMRR);
        DPRINTF(TLBVerbose, "memAttrs PRRR:%08x NMRR:%08x\n", prrr, nmrr);
        uint8_t curr_tr = 0, curr_ir = 0, curr_or = 0;
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
    DPRINTF(TLBVerbose, "memAttrs: shareable: %d, innerAttrs: %d, \
            outerAttrs: %d\n",
            te.shareable, te.innerAttrs, te.outerAttrs);

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
    DPRINTF(TLB, "L1 descriptor for %#x is %#x\n",
            currState->vaddr, currState->l1Desc.data);
    TlbEntry te;

    switch (currState->l1Desc.type()) {
      case L1Descriptor::Ignore:
      case L1Descriptor::Reserved:
        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }
        DPRINTF(TLB, "L1 Descriptor Reserved/Ignore, causing fault\n");
        if (currState->isFetch)
            currState->fault =
                new PrefetchAbort(currState->vaddr, ArmFault::Translation0);
        else
            currState->fault =
                new DataAbort(currState->vaddr, 0, currState->isWrite,
                                  ArmFault::Translation0);
        return;
      case L1Descriptor::Section:
        if (currState->sctlr.afe && bits(currState->l1Desc.ap(), 0) == 0) {
            /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is
              * enabled if set, do l1.Desc.setAp0() instead of generating
              * AccessFlag0
              */

            currState->fault = new DataAbort(currState->vaddr,
                                    currState->l1Desc.domain(), currState->isWrite,
                                    ArmFault::AccessFlag0);
        }
        if (currState->l1Desc.supersection()) {
            panic("Haven't implemented supersections\n");
        }
        te.N = 20;
        te.pfn = currState->l1Desc.pfn();
        te.size = (1<<te.N) - 1;
        te.global = !currState->l1Desc.global();
        te.valid = true;
        te.vpn = currState->vaddr >> te.N;
        te.sNp = true;
        te.xn = currState->l1Desc.xn();
        te.ap = currState->l1Desc.ap();
        te.domain = currState->l1Desc.domain();
        te.asid = currState->contextId;
        memAttrs(currState->tc, te, currState->sctlr,
                currState->l1Desc.texcb(), currState->l1Desc.shareable());

        DPRINTF(TLB, "Inserting Section Descriptor into TLB\n");
        DPRINTF(TLB, " - N:%d pfn:%#x size: %#x global:%d valid: %d\n",
                te.N, te.pfn, te.size, te.global, te.valid);
        DPRINTF(TLB, " - vpn:%#x sNp: %d xn:%d ap:%d domain: %d asid:%d nc:%d\n",
                te.vpn, te.sNp, te.xn, te.ap, te.domain, te.asid,
                te.nonCacheable);
        DPRINTF(TLB, " - domain from l1 desc: %d data: %#x bits:%d\n",
                currState->l1Desc.domain(), currState->l1Desc.data,
                (currState->l1Desc.data >> 5) & 0xF );

        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }
        tlb->insert(currState->vaddr, te);

        return;
      case L1Descriptor::PageTable:
        Addr l2desc_addr;
        l2desc_addr = currState->l1Desc.l2Addr() |
                      (bits(currState->vaddr, 19,12) << 2);
        DPRINTF(TLB, "L1 descriptor points to page table at: %#x\n",
                l2desc_addr);

        // Trickbox address check
        currState->fault = tlb->walkTrickBoxCheck(l2desc_addr, currState->vaddr,
                sizeof(uint32_t), currState->isFetch, currState->isWrite,
                currState->l1Desc.domain(), false);

        if (currState->fault) {
            if (!currState->timing) {
                currState->tc = NULL;
                currState->req = NULL;
            }
            return;
        }


        if (currState->timing) {
            currState->delayed = true;
            port.dmaAction(MemCmd::ReadReq, l2desc_addr, sizeof(uint32_t),
                           &doL2DescEvent, (uint8_t*)&currState->l2Desc.data,
                           currState->tc->getCpuPtr()->clockPeriod());
        } else if (!currState->functional) {
            port.dmaAction(MemCmd::ReadReq, l2desc_addr, sizeof(uint32_t),
                           NULL, (uint8_t*)&currState->l2Desc.data,
                           currState->tc->getCpuPtr()->clockPeriod());
            doL2Descriptor();
        } else {
            RequestPtr req = new Request(l2desc_addr, sizeof(uint32_t), 0,
                                         masterId);
            req->taskId(ContextSwitchTaskId::DMA);
            PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
            pkt->dataStatic((uint8_t*)&currState->l2Desc.data);
            port.sendFunctional(pkt);
            doL2Descriptor();
            delete req;
            delete pkt;
        }
        return;
      default:
        panic("A new type in a 2 bit field?\n");
    }
}

void
TableWalker::doL2Descriptor()
{
    DPRINTF(TLB, "L2 descriptor for %#x is %#x\n",
            currState->vaddr, currState->l2Desc.data);
    TlbEntry te;

    if (currState->l2Desc.invalid()) {
        DPRINTF(TLB, "L2 descriptor invalid, causing fault\n");
        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }
        if (currState->isFetch)
            currState->fault =
                new PrefetchAbort(currState->vaddr, ArmFault::Translation1);
        else
            currState->fault =
                new DataAbort(currState->vaddr, currState->l1Desc.domain(),
                              currState->isWrite, ArmFault::Translation1);
        return;
    }

    if (currState->sctlr.afe && bits(currState->l2Desc.ap(), 0) == 0) {
        /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is enabled
          * if set, do l2.Desc.setAp0() instead of generating AccessFlag0
          */

        currState->fault =
            new DataAbort(currState->vaddr, 0, currState->isWrite,
                          ArmFault::AccessFlag1);

    }

    if (currState->l2Desc.large()) {
      te.N = 16;
      te.pfn = currState->l2Desc.pfn();
    } else {
      te.N = 12;
      te.pfn = currState->l2Desc.pfn();
    }

    te.valid = true;
    te.size =  (1 << te.N) - 1;
    te.asid = currState->contextId;
    te.sNp = false;
    te.vpn = currState->vaddr >> te.N;
    te.global = currState->l2Desc.global();
    te.xn = currState->l2Desc.xn();
    te.ap = currState->l2Desc.ap();
    te.domain = currState->l1Desc.domain();
    memAttrs(currState->tc, te, currState->sctlr, currState->l2Desc.texcb(),
             currState->l2Desc.shareable());

    if (!currState->timing) {
        currState->tc = NULL;
        currState->req = NULL;
    }
    tlb->insert(currState->vaddr, te);
}

void
TableWalker::doL1DescriptorWrapper()
{
    currState = stateQueueL1.front();
    currState->delayed = false;

    DPRINTF(TLBVerbose, "L1 Desc object host addr: %p\n",&currState->l1Desc.data);
    DPRINTF(TLBVerbose, "L1 Desc object      data: %08x\n",currState->l1Desc.data);

    DPRINTF(TLBVerbose, "calling doL1Descriptor for vaddr:%#x\n", currState->vaddr);
    doL1Descriptor();

    stateQueueL1.pop_front();
    completeDrain();
    // Check if fault was generated
    if (currState->fault != NoFault) {
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    }
    else if (!currState->delayed) {
        // delay is not set so there is no L2 to do
        DPRINTF(TLBVerbose, "calling translateTiming again\n");
        currState->fault = tlb->translateTiming(currState->req, currState->tc,
                                       currState->transState, currState->mode);

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    } else {
        // need to do L2 descriptor
        stateQueueL2.push_back(currState);
    }
    currState = NULL;
}

void
TableWalker::doL2DescriptorWrapper()
{
    currState = stateQueueL2.front();
    assert(currState->delayed);

    DPRINTF(TLBVerbose, "calling doL2Descriptor for vaddr:%#x\n",
            currState->vaddr);
    doL2Descriptor();

    // Check if fault was generated
    if (currState->fault != NoFault) {
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);
    }
    else {
        DPRINTF(TLBVerbose, "calling translateTiming again\n");
        currState->fault = tlb->translateTiming(currState->req, currState->tc,
                                      currState->transState, currState->mode);
    }


    stateQueueL2.pop_front();
    completeDrain();
    pending = false;
    nextWalk(currState->tc);

    currState->req = NULL;
    currState->tc = NULL;
    currState->delayed = false;

    delete currState;
    currState = NULL;
}

void
TableWalker::nextWalk(ThreadContext *tc)
{
    if (pendingQueue.size())
        schedule(doProcessEvent, clockEdge(Cycles(1)));
}



ArmISA::TableWalker *
ArmTableWalkerParams::create()
{
    return new ArmISA::TableWalker(this);
}

