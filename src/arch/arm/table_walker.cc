/*
 * Copyright (c) 2010, 2012-2015 ARM Limited
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
 *          Giacomo Gabrielli
 */
#include "arch/arm/table_walker.hh"

#include <memory>

#include "arch/arm/faults.hh"
#include "arch/arm/stage2_mmu.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Checkpoint.hh"
#include "debug/Drain.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "dev/dma_device.hh"
#include "sim/system.hh"

using namespace ArmISA;

TableWalker::TableWalker(const Params *p)
    : MemObject(p),
      stage2Mmu(NULL), port(NULL), masterId(Request::invldMasterId),
      isStage2(p->is_stage2), tlb(NULL),
      currState(NULL), pending(false),
      numSquashable(p->num_squash_per_cycle),
      pendingReqs(0),
      pendingChangeTick(curTick()),
      doL1DescEvent(this), doL2DescEvent(this),
      doL0LongDescEvent(this), doL1LongDescEvent(this), doL2LongDescEvent(this),
      doL3LongDescEvent(this),
      doProcessEvent(this)
{
    sctlr = 0;

    // Cache system-level properties
    if (FullSystem) {
        ArmSystem *armSys = dynamic_cast<ArmSystem *>(p->sys);
        assert(armSys);
        haveSecurity = armSys->haveSecurity();
        _haveLPAE = armSys->haveLPAE();
        _haveVirtualization = armSys->haveVirtualization();
        physAddrRange = armSys->physAddrRange();
        _haveLargeAsid64 = armSys->haveLargeAsid64();
    } else {
        haveSecurity = _haveLPAE = _haveVirtualization = false;
        _haveLargeAsid64 = false;
        physAddrRange = 32;
    }

}

TableWalker::~TableWalker()
{
    ;
}

void
TableWalker::setMMU(Stage2MMU *m, MasterID master_id)
{
    stage2Mmu = m;
    port = &m->getPort();
    masterId = master_id;
}

void
TableWalker::init()
{
    fatal_if(!stage2Mmu, "Table walker must have a valid stage-2 MMU\n");
    fatal_if(!port, "Table walker must have a valid port\n");
    fatal_if(!tlb, "Table walker must have a valid TLB\n");
}

BaseMasterPort&
TableWalker::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port") {
        if (!isStage2) {
            return *port;
        } else {
            fatal("Cannot access table walker port through stage-two walker\n");
        }
    }
    return MemObject::getMasterPort(if_name, idx);
}

TableWalker::WalkerState::WalkerState() :
    tc(nullptr), aarch64(false), el(EL0), physAddrRange(0), req(nullptr),
    asid(0), vmid(0), isHyp(false), transState(nullptr),
    vaddr(0), vaddr_tainted(0), isWrite(false), isFetch(false), isSecure(false),
    secureLookup(false), rwTable(false), userTable(false), xnTable(false),
    pxnTable(false), stage2Req(false), doingStage2(false),
    stage2Tran(nullptr), timing(false), functional(false),
    mode(BaseTLB::Read), tranType(TLB::NormalTran), l2Desc(l1Desc),
    delayed(false), tableWalker(nullptr)
{
}

void
TableWalker::completeDrain()
{
    if (drainState() == DrainState::Draining &&
        stateQueues[L1].empty() && stateQueues[L2].empty() &&
        pendingQueue.empty()) {

        DPRINTF(Drain, "TableWalker done draining, processing drain event\n");
        signalDrainDone();
    }
}

DrainState
TableWalker::drain()
{
    bool state_queues_not_empty = false;

    for (int i = 0; i < MAX_LOOKUP_LEVELS; ++i) {
        if (!stateQueues[i].empty()) {
            state_queues_not_empty = true;
            break;
        }
    }

    if (state_queues_not_empty || pendingQueue.size()) {
        DPRINTF(Drain, "TableWalker not drained\n");
        return DrainState::Draining;
    } else {
        DPRINTF(Drain, "TableWalker free, no need to drain\n");
        return DrainState::Drained;
    }
}

void
TableWalker::drainResume()
{
    if (params()->sys->isTimingMode() && currState) {
        delete currState;
        currState = NULL;
        pendingChange();
    }
}

Fault
TableWalker::walk(RequestPtr _req, ThreadContext *_tc, uint16_t _asid,
                  uint8_t _vmid, bool _isHyp, TLB::Mode _mode,
                  TLB::Translation *_trans, bool _timing, bool _functional,
                  bool secure, TLB::ArmTranslationType tranType)
{
    assert(!(_functional && _timing));
    ++statWalks;

    WalkerState *savedCurrState = NULL;

    if (!currState && !_functional) {
        // For atomic mode, a new WalkerState instance should be only created
        // once per TLB. For timing mode, a new instance is generated for every
        // TLB miss.
        DPRINTF(TLBVerbose, "creating new instance of WalkerState\n");

        currState = new WalkerState();
        currState->tableWalker = this;
    } else if (_functional) {
        // If we are mixing functional mode with timing (or even
        // atomic), we need to to be careful and clean up after
        // ourselves to not risk getting into an inconsistent state.
        DPRINTF(TLBVerbose, "creating functional instance of WalkerState\n");
        savedCurrState = currState;
        currState = new WalkerState();
        currState->tableWalker = this;
    } else if (_timing) {
        // This is a translation that was completed and then faulted again
        // because some underlying parameters that affect the translation
        // changed out from under us (e.g. asid). It will either be a
        // misprediction, in which case nothing will happen or we'll use
        // this fault to re-execute the faulting instruction which should clean
        // up everything.
        if (currState->vaddr_tainted == _req->getVaddr()) {
            ++statSquashedBefore;
            return std::make_shared<ReExec>();
        }
    }
    pendingChange();

    currState->startTime = curTick();
    currState->tc = _tc;
    currState->aarch64 = opModeIs64(currOpMode(_tc));
    currState->el = currEL(_tc);
    currState->transState = _trans;
    currState->req = _req;
    currState->fault = NoFault;
    currState->asid = _asid;
    currState->vmid = _vmid;
    currState->isHyp = _isHyp;
    currState->timing = _timing;
    currState->functional = _functional;
    currState->mode = _mode;
    currState->tranType = tranType;
    currState->isSecure = secure;
    currState->physAddrRange = physAddrRange;

    /** @todo These should be cached or grabbed from cached copies in
     the TLB, all these miscreg reads are expensive */
    currState->vaddr_tainted = currState->req->getVaddr();
    if (currState->aarch64)
        currState->vaddr = purifyTaggedAddr(currState->vaddr_tainted,
                                            currState->tc, currState->el);
    else
        currState->vaddr = currState->vaddr_tainted;

    if (currState->aarch64) {
        switch (currState->el) {
          case EL0:
          case EL1:
            currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL1);
            currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL1);
            break;
          // @todo: uncomment this to enable Virtualization
          // case EL2:
          //   assert(haveVirtualization);
          //   currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL2);
          //   currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL2);
          //   break;
          case EL3:
            assert(haveSecurity);
            currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL3);
            currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL3);
            break;
          default:
            panic("Invalid exception level");
            break;
        }
    } else {
        currState->sctlr = currState->tc->readMiscReg(flattenMiscRegNsBanked(
            MISCREG_SCTLR, currState->tc, !currState->isSecure));
        currState->ttbcr = currState->tc->readMiscReg(flattenMiscRegNsBanked(
            MISCREG_TTBCR, currState->tc, !currState->isSecure));
        currState->htcr  = currState->tc->readMiscReg(MISCREG_HTCR);
        currState->hcr   = currState->tc->readMiscReg(MISCREG_HCR);
        currState->vtcr  = currState->tc->readMiscReg(MISCREG_VTCR);
    }
    sctlr = currState->sctlr;

    currState->isFetch = (currState->mode == TLB::Execute);
    currState->isWrite = (currState->mode == TLB::Write);

    statRequestOrigin[REQUESTED][currState->isFetch]++;

    // We only do a second stage of translation if we're not secure, or in
    // hyp mode, the second stage MMU is enabled, and this table walker
    // instance is the first stage.
    currState->doingStage2 = false;
    // @todo: for now disable this in AArch64 (HCR is not set)
    currState->stage2Req = !currState->aarch64 && currState->hcr.vm &&
                           !isStage2 && !currState->isSecure && !currState->isHyp;

    bool long_desc_format = currState->aarch64 ||
                            (_haveLPAE && currState->ttbcr.eae) ||
                            _isHyp || isStage2;

    if (long_desc_format) {
        // Helper variables used for hierarchical permissions
        currState->secureLookup = currState->isSecure;
        currState->rwTable = true;
        currState->userTable = true;
        currState->xnTable = false;
        currState->pxnTable = false;

        ++statWalksLongDescriptor;
    } else {
        ++statWalksShortDescriptor;
    }

    if (!currState->timing) {
        Fault fault = NoFault;
        if (currState->aarch64)
            fault = processWalkAArch64();
        else if (long_desc_format)
            fault = processWalkLPAE();
        else
            fault = processWalk();

        // If this was a functional non-timing access restore state to
        // how we found it.
        if (currState->functional) {
            delete currState;
            currState = savedCurrState;
        }
        return fault;
    }

    if (pending || pendingQueue.size()) {
        pendingQueue.push_back(currState);
        currState = NULL;
        pendingChange();
    } else {
        pending = true;
        pendingChange();
        if (currState->aarch64)
            return processWalkAArch64();
        else if (long_desc_format)
            return processWalkLPAE();
        else
            return processWalk();
    }

    return NoFault;
}

void
TableWalker::processWalkWrapper()
{
    assert(!currState);
    assert(pendingQueue.size());
    pendingChange();
    currState = pendingQueue.front();

    ExceptionLevel target_el = EL0;
    if (currState->aarch64)
        target_el = currEL(currState->tc);
    else
        target_el = EL1;

    // Check if a previous walk filled this request already
    // @TODO Should this always be the TLB or should we look in the stage2 TLB?
    TlbEntry* te = tlb->lookup(currState->vaddr, currState->asid,
            currState->vmid, currState->isHyp, currState->isSecure, true, false,
            target_el);

    // Check if we still need to have a walk for this request. If the requesting
    // instruction has been squashed, or a previous walk has filled the TLB with
    // a match, we just want to get rid of the walk. The latter could happen
    // when there are multiple outstanding misses to a single page and a
    // previous request has been successfully translated.
    if (!currState->transState->squashed() && !te) {
        // We've got a valid request, lets process it
        pending = true;
        pendingQueue.pop_front();
        // Keep currState in case one of the processWalk... calls NULLs it
        WalkerState *curr_state_copy = currState;
        Fault f;
        if (currState->aarch64)
            f = processWalkAArch64();
        else if ((_haveLPAE && currState->ttbcr.eae) || currState->isHyp || isStage2)
            f = processWalkLPAE();
        else
            f = processWalk();

        if (f != NoFault) {
            curr_state_copy->transState->finish(f, curr_state_copy->req,
                    curr_state_copy->tc, curr_state_copy->mode);

            delete curr_state_copy;
        }
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
        statSquashedBefore++;

        DPRINTF(TLB, "Squashing table walk for address %#x\n",
                      currState->vaddr_tainted);

        if (currState->transState->squashed()) {
            // finish the translation which will delete the translation object
            currState->transState->finish(
                std::make_shared<UnimpFault>("Squashed Inst"),
                currState->req, currState->tc, currState->mode);
        } else {
            // translate the request now that we know it will work
            statWalkServiceTime.sample(curTick() - currState->startTime);
            tlb->translateTiming(currState->req, currState->tc,
                        currState->transState, currState->mode);

        }

        // delete the current request
        delete currState;

        // peak at the next one
        if (pendingQueue.size()) {
            currState = pendingQueue.front();
            te = tlb->lookup(currState->vaddr, currState->asid,
                currState->vmid, currState->isHyp, currState->isSecure, true,
                false, target_el);
        } else {
            // Terminate the loop, nothing more to do
            currState = NULL;
        }
    }
    pendingChange();

    // if we still have pending translations, schedule more work
    nextWalk(tc);
    currState = NULL;
}

Fault
TableWalker::processWalk()
{
    Addr ttbr = 0;

    // If translation isn't enabled, we shouldn't be here
    assert(currState->sctlr.m || isStage2);

    DPRINTF(TLB, "Beginning table walk for address %#x, TTBCR: %#x, bits:%#x\n",
            currState->vaddr_tainted, currState->ttbcr, mbits(currState->vaddr, 31,
                                                      32 - currState->ttbcr.n));

    statWalkWaitTime.sample(curTick() - currState->startTime);

    if (currState->ttbcr.n == 0 || !mbits(currState->vaddr, 31,
                                          32 - currState->ttbcr.n)) {
        DPRINTF(TLB, " - Selecting TTBR0\n");
        // Check if table walk is allowed when Security Extensions are enabled
        if (haveSecurity && currState->ttbcr.pd0) {
            if (currState->isFetch)
                return std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + L1,
                    isStage2,
                    ArmFault::VmsaTran);
            else
                return std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, currState->isWrite,
                    ArmFault::TranslationLL + L1, isStage2,
                    ArmFault::VmsaTran);
        }
        ttbr = currState->tc->readMiscReg(flattenMiscRegNsBanked(
            MISCREG_TTBR0, currState->tc, !currState->isSecure));
    } else {
        DPRINTF(TLB, " - Selecting TTBR1\n");
        // Check if table walk is allowed when Security Extensions are enabled
        if (haveSecurity && currState->ttbcr.pd1) {
            if (currState->isFetch)
                return std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + L1,
                    isStage2,
                    ArmFault::VmsaTran);
            else
                return std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess, currState->isWrite,
                    ArmFault::TranslationLL + L1, isStage2,
                    ArmFault::VmsaTran);
        }
        ttbr = currState->tc->readMiscReg(flattenMiscRegNsBanked(
            MISCREG_TTBR1, currState->tc, !currState->isSecure));
        currState->ttbcr.n = 0;
    }

    Addr l1desc_addr = mbits(ttbr, 31, 14 - currState->ttbcr.n) |
        (bits(currState->vaddr, 31 - currState->ttbcr.n, 20) << 2);
    DPRINTF(TLB, " - Descriptor at address %#x (%s)\n", l1desc_addr,
            currState->isSecure ? "s" : "ns");

    // Trickbox address check
    Fault f;
    f = tlb->walkTrickBoxCheck(l1desc_addr, currState->isSecure,
            currState->vaddr, sizeof(uint32_t), currState->isFetch,
            currState->isWrite, TlbEntry::DomainType::NoAccess, L1);
    if (f) {
        DPRINTF(TLB, "Trickbox check caused fault on %#x\n", currState->vaddr_tainted);
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

    Request::Flags flag = Request::PT_WALK;
    if (currState->sctlr.c == 0) {
        flag.set(Request::UNCACHEABLE);
    }

    bool delayed;
    delayed = fetchDescriptor(l1desc_addr, (uint8_t*)&currState->l1Desc.data,
                              sizeof(uint32_t), flag, L1, &doL1DescEvent,
                              &TableWalker::doL1Descriptor);
    if (!delayed) {
       f = currState->fault;
    }

    return f;
}

Fault
TableWalker::processWalkLPAE()
{
    Addr ttbr, ttbr0_max, ttbr1_min, desc_addr;
    int tsz, n;
    LookupLevel start_lookup_level = L1;

    DPRINTF(TLB, "Beginning table walk for address %#x, TTBCR: %#x\n",
            currState->vaddr_tainted, currState->ttbcr);

    statWalkWaitTime.sample(curTick() - currState->startTime);

    Request::Flags flag = Request::PT_WALK;
    if (currState->isSecure)
        flag.set(Request::SECURE);

    // work out which base address register to use, if in hyp mode we always
    // use HTTBR
    if (isStage2) {
        DPRINTF(TLB, " - Selecting VTTBR (long-desc.)\n");
        ttbr = currState->tc->readMiscReg(MISCREG_VTTBR);
        tsz  = sext<4>(currState->vtcr.t0sz);
        start_lookup_level = currState->vtcr.sl0 ? L1 : L2;
    } else if (currState->isHyp) {
        DPRINTF(TLB, " - Selecting HTTBR (long-desc.)\n");
        ttbr = currState->tc->readMiscReg(MISCREG_HTTBR);
        tsz  = currState->htcr.t0sz;
    } else {
        assert(_haveLPAE && currState->ttbcr.eae);

        // Determine boundaries of TTBR0/1 regions
        if (currState->ttbcr.t0sz)
            ttbr0_max = (1ULL << (32 - currState->ttbcr.t0sz)) - 1;
        else if (currState->ttbcr.t1sz)
            ttbr0_max = (1ULL << 32) -
                (1ULL << (32 - currState->ttbcr.t1sz)) - 1;
        else
            ttbr0_max = (1ULL << 32) - 1;
        if (currState->ttbcr.t1sz)
            ttbr1_min = (1ULL << 32) - (1ULL << (32 - currState->ttbcr.t1sz));
        else
            ttbr1_min = (1ULL << (32 - currState->ttbcr.t0sz));

        // The following code snippet selects the appropriate translation table base
        // address (TTBR0 or TTBR1) and the appropriate starting lookup level
        // depending on the address range supported by the translation table (ARM
        // ARM issue C B3.6.4)
        if (currState->vaddr <= ttbr0_max) {
            DPRINTF(TLB, " - Selecting TTBR0 (long-desc.)\n");
            // Check if table walk is allowed
            if (currState->ttbcr.epd0) {
                if (currState->isFetch)
                    return std::make_shared<PrefetchAbort>(
                        currState->vaddr_tainted,
                        ArmFault::TranslationLL + L1,
                        isStage2,
                        ArmFault::LpaeTran);
                else
                    return std::make_shared<DataAbort>(
                        currState->vaddr_tainted,
                        TlbEntry::DomainType::NoAccess,
                        currState->isWrite,
                        ArmFault::TranslationLL + L1,
                        isStage2,
                        ArmFault::LpaeTran);
            }
            ttbr = currState->tc->readMiscReg(flattenMiscRegNsBanked(
                MISCREG_TTBR0, currState->tc, !currState->isSecure));
            tsz = currState->ttbcr.t0sz;
            if (ttbr0_max < (1ULL << 30))  // Upper limit < 1 GB
                start_lookup_level = L2;
        } else if (currState->vaddr >= ttbr1_min) {
            DPRINTF(TLB, " - Selecting TTBR1 (long-desc.)\n");
            // Check if table walk is allowed
            if (currState->ttbcr.epd1) {
                if (currState->isFetch)
                    return std::make_shared<PrefetchAbort>(
                        currState->vaddr_tainted,
                        ArmFault::TranslationLL + L1,
                        isStage2,
                        ArmFault::LpaeTran);
                else
                    return std::make_shared<DataAbort>(
                        currState->vaddr_tainted,
                        TlbEntry::DomainType::NoAccess,
                        currState->isWrite,
                        ArmFault::TranslationLL + L1,
                        isStage2,
                        ArmFault::LpaeTran);
            }
            ttbr = currState->tc->readMiscReg(flattenMiscRegNsBanked(
                MISCREG_TTBR1, currState->tc, !currState->isSecure));
            tsz = currState->ttbcr.t1sz;
            if (ttbr1_min >= (1ULL << 31) + (1ULL << 30))  // Lower limit >= 3 GB
                start_lookup_level = L2;
        } else {
            // Out of boundaries -> translation fault
            if (currState->isFetch)
                return std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + L1,
                    isStage2,
                    ArmFault::LpaeTran);
            else
                return std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    currState->isWrite, ArmFault::TranslationLL + L1,
                    isStage2, ArmFault::LpaeTran);
        }

    }

    // Perform lookup (ARM ARM issue C B3.6.6)
    if (start_lookup_level == L1) {
        n = 5 - tsz;
        desc_addr = mbits(ttbr, 39, n) |
            (bits(currState->vaddr, n + 26, 30) << 3);
        DPRINTF(TLB, " - Descriptor at address %#x (%s) (long-desc.)\n",
                desc_addr, currState->isSecure ? "s" : "ns");
    } else {
        // Skip first-level lookup
        n = (tsz >= 2 ? 14 - tsz : 12);
        desc_addr = mbits(ttbr, 39, n) |
            (bits(currState->vaddr, n + 17, 21) << 3);
        DPRINTF(TLB, " - Descriptor at address %#x (%s) (long-desc.)\n",
                desc_addr, currState->isSecure ? "s" : "ns");
    }

    // Trickbox address check
    Fault f = tlb->walkTrickBoxCheck(desc_addr, currState->isSecure,
                        currState->vaddr, sizeof(uint64_t), currState->isFetch,
                        currState->isWrite, TlbEntry::DomainType::NoAccess,
                        start_lookup_level);
    if (f) {
        DPRINTF(TLB, "Trickbox check caused fault on %#x\n", currState->vaddr_tainted);
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

    if (currState->sctlr.c == 0) {
        flag.set(Request::UNCACHEABLE);
    }

    if (currState->isSecure)
        flag.set(Request::SECURE);

    currState->longDesc.lookupLevel = start_lookup_level;
    currState->longDesc.aarch64 = false;
    currState->longDesc.grainSize = Grain4KB;

    Event *event = start_lookup_level == L1 ? (Event *) &doL1LongDescEvent
                                            : (Event *) &doL2LongDescEvent;

    bool delayed = fetchDescriptor(desc_addr, (uint8_t*)&currState->longDesc.data,
                                   sizeof(uint64_t), flag, start_lookup_level,
                                   event, &TableWalker::doLongDescriptor);
    if (!delayed) {
        f = currState->fault;
    }

    return f;
}

unsigned
TableWalker::adjustTableSizeAArch64(unsigned tsz)
{
    if (tsz < 25)
        return 25;
    if (tsz > 48)
        return 48;
    return tsz;
}

bool
TableWalker::checkAddrSizeFaultAArch64(Addr addr, int currPhysAddrRange)
{
    return (currPhysAddrRange != MaxPhysAddrRange &&
            bits(addr, MaxPhysAddrRange - 1, currPhysAddrRange));
}

Fault
TableWalker::processWalkAArch64()
{
    assert(currState->aarch64);

    DPRINTF(TLB, "Beginning table walk for address %#llx, TCR: %#llx\n",
            currState->vaddr_tainted, currState->tcr);

    static const GrainSize GrainMapDefault[] =
      { Grain4KB, Grain64KB, Grain16KB, ReservedGrain };
    static const GrainSize GrainMap_EL1_tg1[] =
      { ReservedGrain, Grain16KB, Grain4KB, Grain64KB };

    statWalkWaitTime.sample(curTick() - currState->startTime);

    // Determine TTBR, table size, granule size and phys. address range
    Addr ttbr = 0;
    int tsz = 0, ps = 0;
    GrainSize tg = Grain4KB; // grain size computed from tg* field
    bool fault = false;
    switch (currState->el) {
      case EL0:
      case EL1:
        switch (bits(currState->vaddr, 63,48)) {
          case 0:
            DPRINTF(TLB, " - Selecting TTBR0 (AArch64)\n");
            ttbr = currState->tc->readMiscReg(MISCREG_TTBR0_EL1);
            tsz = adjustTableSizeAArch64(64 - currState->tcr.t0sz);
            tg = GrainMapDefault[currState->tcr.tg0];
            if (bits(currState->vaddr, 63, tsz) != 0x0 ||
                currState->tcr.epd0)
              fault = true;
            break;
          case 0xffff:
            DPRINTF(TLB, " - Selecting TTBR1 (AArch64)\n");
            ttbr = currState->tc->readMiscReg(MISCREG_TTBR1_EL1);
            tsz = adjustTableSizeAArch64(64 - currState->tcr.t1sz);
            tg = GrainMap_EL1_tg1[currState->tcr.tg1];
            if (bits(currState->vaddr, 63, tsz) != mask(64-tsz) ||
                currState->tcr.epd1)
              fault = true;
            break;
          default:
            // top two bytes must be all 0s or all 1s, else invalid addr
            fault = true;
        }
        ps = currState->tcr.ips;
        break;
      case EL2:
      case EL3:
        switch(bits(currState->vaddr, 63,48)) {
            case 0:
                DPRINTF(TLB, " - Selecting TTBR0 (AArch64)\n");
                if (currState->el == EL2)
                    ttbr = currState->tc->readMiscReg(MISCREG_TTBR0_EL2);
                else
                    ttbr = currState->tc->readMiscReg(MISCREG_TTBR0_EL3);
                tsz = adjustTableSizeAArch64(64 - currState->tcr.t0sz);
                tg = GrainMapDefault[currState->tcr.tg0];
                break;
            default:
                // invalid addr if top two bytes are not all 0s
                fault = true;
        }
        ps = currState->tcr.ips;
        break;
    }

    if (fault) {
        Fault f;
        if (currState->isFetch)
            f =  std::make_shared<PrefetchAbort>(
                currState->vaddr_tainted,
                ArmFault::TranslationLL + L0, isStage2,
                ArmFault::LpaeTran);
        else
            f = std::make_shared<DataAbort>(
                currState->vaddr_tainted,
                TlbEntry::DomainType::NoAccess,
                currState->isWrite,
                ArmFault::TranslationLL + L0,
                isStage2, ArmFault::LpaeTran);

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

    if (tg == ReservedGrain) {
        warn_once("Reserved granule size requested; gem5's IMPLEMENTATION "
                  "DEFINED behavior takes this to mean 4KB granules\n");
        tg = Grain4KB;
    }

    int stride = tg - 3;
    LookupLevel start_lookup_level = MAX_LOOKUP_LEVELS;

    // Determine starting lookup level
    // See aarch64/translation/walk in Appendix G: ARMv8 Pseudocode Library
    // in ARM DDI 0487A.  These table values correspond to the cascading tests
    // to compute the lookup level and are of the form
    // (grain_size + N*stride), for N = {1, 2, 3}.
    // A value of 64 will never succeed and a value of 0 will always succeed.
    {
        struct GrainMap {
            GrainSize grain_size;
            unsigned lookup_level_cutoff[MAX_LOOKUP_LEVELS];
        };
        static const GrainMap GM[] = {
            { Grain4KB,  { 39, 30,  0, 0 } },
            { Grain16KB, { 47, 36, 25, 0 } },
            { Grain64KB, { 64, 42, 29, 0 } }
        };

        const unsigned *lookup = NULL; // points to a lookup_level_cutoff

        for (unsigned i = 0; i < 3; ++i) { // choose entry of GM[]
            if (tg == GM[i].grain_size) {
                lookup = GM[i].lookup_level_cutoff;
                break;
            }
        }
        assert(lookup);

        for (int L = L0; L != MAX_LOOKUP_LEVELS; ++L) {
            if (tsz > lookup[L]) {
                start_lookup_level = (LookupLevel) L;
                break;
            }
        }
        panic_if(start_lookup_level == MAX_LOOKUP_LEVELS,
                 "Table walker couldn't find lookup level\n");
    }

    // Determine table base address
    int base_addr_lo = 3 + tsz - stride * (3 - start_lookup_level) - tg;
    Addr base_addr = mbits(ttbr, 47, base_addr_lo);

    // Determine physical address size and raise an Address Size Fault if
    // necessary
    int pa_range = decodePhysAddrRange64(ps);
    // Clamp to lower limit
    if (pa_range > physAddrRange)
        currState->physAddrRange = physAddrRange;
    else
        currState->physAddrRange = pa_range;
    if (checkAddrSizeFaultAArch64(base_addr, currState->physAddrRange)) {
        DPRINTF(TLB, "Address size fault before any lookup\n");
        Fault f;
        if (currState->isFetch)
            f = std::make_shared<PrefetchAbort>(
                currState->vaddr_tainted,
                ArmFault::AddressSizeLL + start_lookup_level,
                isStage2,
                ArmFault::LpaeTran);
        else
            f = std::make_shared<DataAbort>(
                currState->vaddr_tainted,
                TlbEntry::DomainType::NoAccess,
                currState->isWrite,
                ArmFault::AddressSizeLL + start_lookup_level,
                isStage2,
                ArmFault::LpaeTran);


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

    // Determine descriptor address
    Addr desc_addr = base_addr |
        (bits(currState->vaddr, tsz - 1,
              stride * (3 - start_lookup_level) + tg) << 3);

    // Trickbox address check
    Fault f = tlb->walkTrickBoxCheck(desc_addr, currState->isSecure,
                        currState->vaddr, sizeof(uint64_t), currState->isFetch,
                        currState->isWrite, TlbEntry::DomainType::NoAccess,
                        start_lookup_level);
    if (f) {
        DPRINTF(TLB, "Trickbox check caused fault on %#x\n", currState->vaddr_tainted);
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

    Request::Flags flag = Request::PT_WALK;
    if (currState->sctlr.c == 0) {
        flag.set(Request::UNCACHEABLE);
    }

    currState->longDesc.lookupLevel = start_lookup_level;
    currState->longDesc.aarch64 = true;
    currState->longDesc.grainSize = tg;

    if (currState->timing) {
        Event *event;
        switch (start_lookup_level) {
          case L0:
            event = (Event *) &doL0LongDescEvent;
            break;
          case L1:
            event = (Event *) &doL1LongDescEvent;
            break;
          case L2:
            event = (Event *) &doL2LongDescEvent;
            break;
          case L3:
            event = (Event *) &doL3LongDescEvent;
            break;
          default:
            panic("Invalid table lookup level");
            break;
        }
        port->dmaAction(MemCmd::ReadReq, desc_addr, sizeof(uint64_t),
                       event, (uint8_t*) &currState->longDesc.data,
                       currState->tc->getCpuPtr()->clockPeriod(), flag);
        DPRINTF(TLBVerbose,
                "Adding to walker fifo: queue size before adding: %d\n",
                stateQueues[start_lookup_level].size());
        stateQueues[start_lookup_level].push_back(currState);
        currState = NULL;
    } else if (!currState->functional) {
        port->dmaAction(MemCmd::ReadReq, desc_addr, sizeof(uint64_t),
                       NULL, (uint8_t*) &currState->longDesc.data,
                       currState->tc->getCpuPtr()->clockPeriod(), flag);
        doLongDescriptor();
        f = currState->fault;
    } else {
        RequestPtr req = new Request(desc_addr, sizeof(uint64_t), flag,
                                     masterId);
        PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
        pkt->dataStatic((uint8_t*) &currState->longDesc.data);
        port->sendFunctional(pkt);
        doLongDescriptor();
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
    te.outerShareable = false;
    if (sctlr.tre == 0 || ((sctlr.tre == 1) && (sctlr.m == 0))) {
        switch(texcb) {
          case 0: // Stongly-ordered
            te.nonCacheable = true;
            te.mtype = TlbEntry::MemoryType::StronglyOrdered;
            te.shareable = true;
            te.innerAttrs = 1;
            te.outerAttrs = 0;
            break;
          case 1: // Shareable Device
            te.nonCacheable = true;
            te.mtype = TlbEntry::MemoryType::Device;
            te.shareable = true;
            te.innerAttrs = 3;
            te.outerAttrs = 0;
            break;
          case 2: // Outer and Inner Write-Through, no Write-Allocate
            te.mtype = TlbEntry::MemoryType::Normal;
            te.shareable = s;
            te.innerAttrs = 6;
            te.outerAttrs = bits(texcb, 1, 0);
            break;
          case 3: // Outer and Inner Write-Back, no Write-Allocate
            te.mtype = TlbEntry::MemoryType::Normal;
            te.shareable = s;
            te.innerAttrs = 7;
            te.outerAttrs = bits(texcb, 1, 0);
            break;
          case 4: // Outer and Inner Non-cacheable
            te.nonCacheable = true;
            te.mtype = TlbEntry::MemoryType::Normal;
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
            te.mtype = TlbEntry::MemoryType::Normal;
            te.shareable = s;
            te.innerAttrs = 5;
            te.outerAttrs = 1;
            break;
          case 8: // Non-shareable Device
            te.nonCacheable = true;
            te.mtype = TlbEntry::MemoryType::Device;
            te.shareable = false;
            te.innerAttrs = 3;
            te.outerAttrs = 0;
            break;
          case 9 ... 15:  // Reserved
            panic("Reserved texcb value!\n");
            break;
          case 16 ... 31: // Cacheable Memory
            te.mtype = TlbEntry::MemoryType::Normal;
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
        PRRR prrr = tc->readMiscReg(flattenMiscRegNsBanked(MISCREG_PRRR,
                                    currState->tc, !currState->isSecure));
        NMRR nmrr = tc->readMiscReg(flattenMiscRegNsBanked(MISCREG_NMRR,
                                    currState->tc, !currState->isSecure));
        DPRINTF(TLBVerbose, "memAttrs PRRR:%08x NMRR:%08x\n", prrr, nmrr);
        uint8_t curr_tr = 0, curr_ir = 0, curr_or = 0;
        switch(bits(texcb, 2,0)) {
          case 0:
            curr_tr = prrr.tr0;
            curr_ir = nmrr.ir0;
            curr_or = nmrr.or0;
            te.outerShareable = (prrr.nos0 == 0);
            break;
          case 1:
            curr_tr = prrr.tr1;
            curr_ir = nmrr.ir1;
            curr_or = nmrr.or1;
            te.outerShareable = (prrr.nos1 == 0);
            break;
          case 2:
            curr_tr = prrr.tr2;
            curr_ir = nmrr.ir2;
            curr_or = nmrr.or2;
            te.outerShareable = (prrr.nos2 == 0);
            break;
          case 3:
            curr_tr = prrr.tr3;
            curr_ir = nmrr.ir3;
            curr_or = nmrr.or3;
            te.outerShareable = (prrr.nos3 == 0);
            break;
          case 4:
            curr_tr = prrr.tr4;
            curr_ir = nmrr.ir4;
            curr_or = nmrr.or4;
            te.outerShareable = (prrr.nos4 == 0);
            break;
          case 5:
            curr_tr = prrr.tr5;
            curr_ir = nmrr.ir5;
            curr_or = nmrr.or5;
            te.outerShareable = (prrr.nos5 == 0);
            break;
          case 6:
            panic("Imp defined type\n");
          case 7:
            curr_tr = prrr.tr7;
            curr_ir = nmrr.ir7;
            curr_or = nmrr.or7;
            te.outerShareable = (prrr.nos7 == 0);
            break;
        }

        switch(curr_tr) {
          case 0:
            DPRINTF(TLBVerbose, "StronglyOrdered\n");
            te.mtype = TlbEntry::MemoryType::StronglyOrdered;
            te.nonCacheable = true;
            te.innerAttrs = 1;
            te.outerAttrs = 0;
            te.shareable = true;
            break;
          case 1:
            DPRINTF(TLBVerbose, "Device ds1:%d ds0:%d s:%d\n",
                    prrr.ds1, prrr.ds0, s);
            te.mtype = TlbEntry::MemoryType::Device;
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
            te.mtype = TlbEntry::MemoryType::Normal;
            if (prrr.ns1 && s)
                te.shareable = true;
            if (prrr.ns0 && !s)
                te.shareable = true;
            break;
          case 3:
            panic("Reserved type");
        }

        if (te.mtype == TlbEntry::MemoryType::Normal){
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
    DPRINTF(TLBVerbose, "memAttrs: shareable: %d, innerAttrs: %d, "
            "outerAttrs: %d\n",
            te.shareable, te.innerAttrs, te.outerAttrs);
    te.setAttributes(false);
}

void
TableWalker::memAttrsLPAE(ThreadContext *tc, TlbEntry &te,
    LongDescriptor &lDescriptor)
{
    assert(_haveLPAE);

    uint8_t attr;
    uint8_t sh = lDescriptor.sh();
    // Different format and source of attributes if this is a stage 2
    // translation
    if (isStage2) {
        attr = lDescriptor.memAttr();
        uint8_t attr_3_2 = (attr >> 2) & 0x3;
        uint8_t attr_1_0 =  attr       & 0x3;

        DPRINTF(TLBVerbose, "memAttrsLPAE MemAttr:%#x sh:%#x\n", attr, sh);

        if (attr_3_2 == 0) {
            te.mtype        = attr_1_0 == 0 ? TlbEntry::MemoryType::StronglyOrdered
                                            : TlbEntry::MemoryType::Device;
            te.outerAttrs   = 0;
            te.innerAttrs   = attr_1_0 == 0 ? 1 : 3;
            te.nonCacheable = true;
        } else {
            te.mtype        = TlbEntry::MemoryType::Normal;
            te.outerAttrs   = attr_3_2 == 1 ? 0 :
                              attr_3_2 == 2 ? 2 : 1;
            te.innerAttrs   = attr_1_0 == 1 ? 0 :
                              attr_1_0 == 2 ? 6 : 5;
            te.nonCacheable = (attr_3_2 == 1) || (attr_1_0 == 1);
        }
    } else {
        uint8_t attrIndx = lDescriptor.attrIndx();

        // LPAE always uses remapping of memory attributes, irrespective of the
        // value of SCTLR.TRE
        MiscRegIndex reg = attrIndx & 0x4 ? MISCREG_MAIR1 : MISCREG_MAIR0;
        int reg_as_int = flattenMiscRegNsBanked(reg, currState->tc,
                                                !currState->isSecure);
        uint32_t mair = currState->tc->readMiscReg(reg_as_int);
        attr = (mair >> (8 * (attrIndx % 4))) & 0xff;
        uint8_t attr_7_4 = bits(attr, 7, 4);
        uint8_t attr_3_0 = bits(attr, 3, 0);
        DPRINTF(TLBVerbose, "memAttrsLPAE AttrIndx:%#x sh:%#x, attr %#x\n", attrIndx, sh, attr);

        // Note: the memory subsystem only cares about the 'cacheable' memory
        // attribute. The other attributes are only used to fill the PAR register
        // accordingly to provide the illusion of full support
        te.nonCacheable = false;

        switch (attr_7_4) {
          case 0x0:
            // Strongly-ordered or Device memory
            if (attr_3_0 == 0x0)
                te.mtype = TlbEntry::MemoryType::StronglyOrdered;
            else if (attr_3_0 == 0x4)
                te.mtype = TlbEntry::MemoryType::Device;
            else
                panic("Unpredictable behavior\n");
            te.nonCacheable = true;
            te.outerAttrs   = 0;
            break;
          case 0x4:
            // Normal memory, Outer Non-cacheable
            te.mtype = TlbEntry::MemoryType::Normal;
            te.outerAttrs = 0;
            if (attr_3_0 == 0x4)
                // Inner Non-cacheable
                te.nonCacheable = true;
            else if (attr_3_0 < 0x8)
                panic("Unpredictable behavior\n");
            break;
          case 0x8:
          case 0x9:
          case 0xa:
          case 0xb:
          case 0xc:
          case 0xd:
          case 0xe:
          case 0xf:
            if (attr_7_4 & 0x4) {
                te.outerAttrs = (attr_7_4 & 1) ? 1 : 3;
            } else {
                te.outerAttrs = 0x2;
            }
            // Normal memory, Outer Cacheable
            te.mtype = TlbEntry::MemoryType::Normal;
            if (attr_3_0 != 0x4 && attr_3_0 < 0x8)
                panic("Unpredictable behavior\n");
            break;
          default:
            panic("Unpredictable behavior\n");
            break;
        }

        switch (attr_3_0) {
          case 0x0:
            te.innerAttrs = 0x1;
            break;
          case 0x4:
            te.innerAttrs = attr_7_4 == 0 ? 0x3 : 0;
            break;
          case 0x8:
          case 0x9:
          case 0xA:
          case 0xB:
            te.innerAttrs = 6;
            break;
          case 0xC:
          case 0xD:
          case 0xE:
          case 0xF:
            te.innerAttrs = attr_3_0 & 1 ? 0x5 : 0x7;
            break;
          default:
            panic("Unpredictable behavior\n");
            break;
        }
    }

    te.outerShareable = sh == 2;
    te.shareable       = (sh & 0x2) ? true : false;
    te.setAttributes(true);
    te.attributes |= (uint64_t) attr << 56;
}

void
TableWalker::memAttrsAArch64(ThreadContext *tc, TlbEntry &te, uint8_t attrIndx,
                             uint8_t sh)
{
    DPRINTF(TLBVerbose, "memAttrsAArch64 AttrIndx:%#x sh:%#x\n", attrIndx, sh);

    // Select MAIR
    uint64_t mair;
    switch (currState->el) {
      case EL0:
      case EL1:
        mair = tc->readMiscReg(MISCREG_MAIR_EL1);
        break;
      case EL2:
        mair = tc->readMiscReg(MISCREG_MAIR_EL2);
        break;
      case EL3:
        mair = tc->readMiscReg(MISCREG_MAIR_EL3);
        break;
      default:
        panic("Invalid exception level");
        break;
    }

    // Select attributes
    uint8_t attr = bits(mair, 8 * attrIndx + 7, 8 * attrIndx);
    uint8_t attr_lo = bits(attr, 3, 0);
    uint8_t attr_hi = bits(attr, 7, 4);

    // Memory type
    te.mtype = attr_hi == 0 ? TlbEntry::MemoryType::Device : TlbEntry::MemoryType::Normal;

    // Cacheability
    te.nonCacheable = false;
    if (te.mtype == TlbEntry::MemoryType::Device ||  // Device memory
        attr_hi == 0x8 ||  // Normal memory, Outer Non-cacheable
        attr_lo == 0x8) {  // Normal memory, Inner Non-cacheable
        te.nonCacheable = true;
    }

    te.shareable       = sh == 2;
    te.outerShareable = (sh & 0x2) ? true : false;
    // Attributes formatted according to the 64-bit PAR
    te.attributes = ((uint64_t) attr << 56) |
        (1 << 11) |     // LPAE bit
        (te.ns << 9) |  // NS bit
        (sh << 7);
}

void
TableWalker::doL1Descriptor()
{
    if (currState->fault != NoFault) {
        return;
    }

    DPRINTF(TLB, "L1 descriptor for %#x is %#x\n",
            currState->vaddr_tainted, currState->l1Desc.data);
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
                std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + L1,
                    isStage2,
                    ArmFault::VmsaTran);
        else
            currState->fault =
                std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    currState->isWrite,
                    ArmFault::TranslationLL + L1, isStage2,
                    ArmFault::VmsaTran);
        return;
      case L1Descriptor::Section:
        if (currState->sctlr.afe && bits(currState->l1Desc.ap(), 0) == 0) {
            /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is
              * enabled if set, do l1.Desc.setAp0() instead of generating
              * AccessFlag0
              */

            currState->fault = std::make_shared<DataAbort>(
                currState->vaddr_tainted,
                currState->l1Desc.domain(),
                currState->isWrite,
                ArmFault::AccessFlagLL + L1,
                isStage2,
                ArmFault::VmsaTran);
        }
        if (currState->l1Desc.supersection()) {
            panic("Haven't implemented supersections\n");
        }
        insertTableEntry(currState->l1Desc, false);
        return;
      case L1Descriptor::PageTable:
        {
            Addr l2desc_addr;
            l2desc_addr = currState->l1Desc.l2Addr() |
                (bits(currState->vaddr, 19, 12) << 2);
            DPRINTF(TLB, "L1 descriptor points to page table at: %#x (%s)\n",
                    l2desc_addr, currState->isSecure ? "s" : "ns");

            // Trickbox address check
            currState->fault = tlb->walkTrickBoxCheck(
                l2desc_addr, currState->isSecure, currState->vaddr,
                sizeof(uint32_t), currState->isFetch, currState->isWrite,
                currState->l1Desc.domain(), L2);

            if (currState->fault) {
                if (!currState->timing) {
                    currState->tc = NULL;
                    currState->req = NULL;
                }
                return;
            }

            Request::Flags flag = Request::PT_WALK;
            if (currState->isSecure)
                flag.set(Request::SECURE);

            bool delayed;
            delayed = fetchDescriptor(l2desc_addr,
                                      (uint8_t*)&currState->l2Desc.data,
                                      sizeof(uint32_t), flag, -1, &doL2DescEvent,
                                      &TableWalker::doL2Descriptor);
            if (delayed) {
                currState->delayed = true;
            }

            return;
        }
      default:
        panic("A new type in a 2 bit field?\n");
    }
}

void
TableWalker::doLongDescriptor()
{
    if (currState->fault != NoFault) {
        return;
    }

    DPRINTF(TLB, "L%d descriptor for %#llx is %#llx (%s)\n",
            currState->longDesc.lookupLevel, currState->vaddr_tainted,
            currState->longDesc.data,
            currState->aarch64 ? "AArch64" : "long-desc.");

    if ((currState->longDesc.type() == LongDescriptor::Block) ||
        (currState->longDesc.type() == LongDescriptor::Page)) {
        DPRINTF(TLBVerbose, "Analyzing L%d descriptor: %#llx, pxn: %d, "
                "xn: %d, ap: %d, af: %d, type: %d\n",
                currState->longDesc.lookupLevel,
                currState->longDesc.data,
                currState->longDesc.pxn(),
                currState->longDesc.xn(),
                currState->longDesc.ap(),
                currState->longDesc.af(),
                currState->longDesc.type());
    } else {
        DPRINTF(TLBVerbose, "Analyzing L%d descriptor: %#llx, type: %d\n",
                currState->longDesc.lookupLevel,
                currState->longDesc.data,
                currState->longDesc.type());
    }

    TlbEntry te;

    switch (currState->longDesc.type()) {
      case LongDescriptor::Invalid:
        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }

        DPRINTF(TLB, "L%d descriptor Invalid, causing fault type %d\n",
                currState->longDesc.lookupLevel,
                ArmFault::TranslationLL + currState->longDesc.lookupLevel);
        if (currState->isFetch)
            currState->fault = std::make_shared<PrefetchAbort>(
                currState->vaddr_tainted,
                ArmFault::TranslationLL + currState->longDesc.lookupLevel,
                isStage2,
                ArmFault::LpaeTran);
        else
            currState->fault = std::make_shared<DataAbort>(
                currState->vaddr_tainted,
                TlbEntry::DomainType::NoAccess,
                currState->isWrite,
                ArmFault::TranslationLL + currState->longDesc.lookupLevel,
                isStage2,
                ArmFault::LpaeTran);
        return;
      case LongDescriptor::Block:
      case LongDescriptor::Page:
        {
            bool fault = false;
            bool aff = false;
            // Check for address size fault
            if (checkAddrSizeFaultAArch64(
                    mbits(currState->longDesc.data, MaxPhysAddrRange - 1,
                          currState->longDesc.offsetBits()),
                    currState->physAddrRange)) {
                fault = true;
                DPRINTF(TLB, "L%d descriptor causing Address Size Fault\n",
                        currState->longDesc.lookupLevel);
            // Check for access fault
            } else if (currState->longDesc.af() == 0) {
                fault = true;
                DPRINTF(TLB, "L%d descriptor causing Access Fault\n",
                        currState->longDesc.lookupLevel);
                aff = true;
            }
            if (fault) {
                if (currState->isFetch)
                    currState->fault = std::make_shared<PrefetchAbort>(
                        currState->vaddr_tainted,
                        (aff ? ArmFault::AccessFlagLL : ArmFault::AddressSizeLL) +
                        currState->longDesc.lookupLevel,
                        isStage2,
                        ArmFault::LpaeTran);
                else
                    currState->fault = std::make_shared<DataAbort>(
                        currState->vaddr_tainted,
                        TlbEntry::DomainType::NoAccess, currState->isWrite,
                        (aff ? ArmFault::AccessFlagLL : ArmFault::AddressSizeLL) +
                        currState->longDesc.lookupLevel,
                        isStage2,
                        ArmFault::LpaeTran);
            } else {
                insertTableEntry(currState->longDesc, true);
            }
        }
        return;
      case LongDescriptor::Table:
        {
            // Set hierarchical permission flags
            currState->secureLookup = currState->secureLookup &&
                currState->longDesc.secureTable();
            currState->rwTable = currState->rwTable &&
                currState->longDesc.rwTable();
            currState->userTable = currState->userTable &&
                currState->longDesc.userTable();
            currState->xnTable = currState->xnTable ||
                currState->longDesc.xnTable();
            currState->pxnTable = currState->pxnTable ||
                currState->longDesc.pxnTable();

            // Set up next level lookup
            Addr next_desc_addr = currState->longDesc.nextDescAddr(
                currState->vaddr);

            DPRINTF(TLB, "L%d descriptor points to L%d descriptor at: %#x (%s)\n",
                    currState->longDesc.lookupLevel,
                    currState->longDesc.lookupLevel + 1,
                    next_desc_addr,
                    currState->secureLookup ? "s" : "ns");

            // Check for address size fault
            if (currState->aarch64 && checkAddrSizeFaultAArch64(
                    next_desc_addr, currState->physAddrRange)) {
                DPRINTF(TLB, "L%d descriptor causing Address Size Fault\n",
                        currState->longDesc.lookupLevel);
                if (currState->isFetch)
                    currState->fault = std::make_shared<PrefetchAbort>(
                        currState->vaddr_tainted,
                        ArmFault::AddressSizeLL
                        + currState->longDesc.lookupLevel,
                        isStage2,
                        ArmFault::LpaeTran);
                else
                    currState->fault = std::make_shared<DataAbort>(
                        currState->vaddr_tainted,
                        TlbEntry::DomainType::NoAccess, currState->isWrite,
                        ArmFault::AddressSizeLL
                        + currState->longDesc.lookupLevel,
                        isStage2,
                        ArmFault::LpaeTran);
                return;
            }

            // Trickbox address check
            currState->fault = tlb->walkTrickBoxCheck(
                            next_desc_addr, currState->vaddr,
                            currState->vaddr, sizeof(uint64_t),
                            currState->isFetch, currState->isWrite,
                            TlbEntry::DomainType::Client,
                            toLookupLevel(currState->longDesc.lookupLevel +1));

            if (currState->fault) {
                if (!currState->timing) {
                    currState->tc = NULL;
                    currState->req = NULL;
                }
                return;
            }

            Request::Flags flag = Request::PT_WALK;
            if (currState->secureLookup)
                flag.set(Request::SECURE);

            currState->longDesc.lookupLevel =
                (LookupLevel) (currState->longDesc.lookupLevel + 1);
            Event *event = NULL;
            switch (currState->longDesc.lookupLevel) {
              case L1:
                assert(currState->aarch64);
                event = &doL1LongDescEvent;
                break;
              case L2:
                event = &doL2LongDescEvent;
                break;
              case L3:
                event = &doL3LongDescEvent;
                break;
              default:
                panic("Wrong lookup level in table walk\n");
                break;
            }

            bool delayed;
            delayed = fetchDescriptor(next_desc_addr, (uint8_t*)&currState->longDesc.data,
                                      sizeof(uint64_t), flag, -1, event,
                                      &TableWalker::doLongDescriptor);
            if (delayed) {
                 currState->delayed = true;
            }
        }
        return;
      default:
        panic("A new type in a 2 bit field?\n");
    }
}

void
TableWalker::doL2Descriptor()
{
    if (currState->fault != NoFault) {
        return;
    }

    DPRINTF(TLB, "L2 descriptor for %#x is %#x\n",
            currState->vaddr_tainted, currState->l2Desc.data);
    TlbEntry te;

    if (currState->l2Desc.invalid()) {
        DPRINTF(TLB, "L2 descriptor invalid, causing fault\n");
        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }
        if (currState->isFetch)
            currState->fault = std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + L2,
                    isStage2,
                    ArmFault::VmsaTran);
        else
            currState->fault = std::make_shared<DataAbort>(
                currState->vaddr_tainted, currState->l1Desc.domain(),
                currState->isWrite, ArmFault::TranslationLL + L2,
                isStage2,
                ArmFault::VmsaTran);
        return;
    }

    if (currState->sctlr.afe && bits(currState->l2Desc.ap(), 0) == 0) {
        /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is enabled
          * if set, do l2.Desc.setAp0() instead of generating AccessFlag0
          */
         DPRINTF(TLB, "Generating access fault at L2, afe: %d, ap: %d\n",
                 currState->sctlr.afe, currState->l2Desc.ap());

        currState->fault = std::make_shared<DataAbort>(
            currState->vaddr_tainted,
            TlbEntry::DomainType::NoAccess, currState->isWrite,
            ArmFault::AccessFlagLL + L2, isStage2,
            ArmFault::VmsaTran);
    }

    insertTableEntry(currState->l2Desc, false);
}

void
TableWalker::doL1DescriptorWrapper()
{
    currState = stateQueues[L1].front();
    currState->delayed = false;
    // if there's a stage2 translation object we don't need it any more
    if (currState->stage2Tran) {
        delete currState->stage2Tran;
        currState->stage2Tran = NULL;
    }


    DPRINTF(TLBVerbose, "L1 Desc object host addr: %p\n",&currState->l1Desc.data);
    DPRINTF(TLBVerbose, "L1 Desc object      data: %08x\n",currState->l1Desc.data);

    DPRINTF(TLBVerbose, "calling doL1Descriptor for vaddr:%#x\n", currState->vaddr_tainted);
    doL1Descriptor();

    stateQueues[L1].pop_front();
    // Check if fault was generated
    if (currState->fault != NoFault) {
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);
        statWalksShortTerminatedAtLevel[0]++;

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    }
    else if (!currState->delayed) {
        // delay is not set so there is no L2 to do
        // Don't finish the translation if a stage 2 look up is underway
        if (!currState->doingStage2) {
            statWalkServiceTime.sample(curTick() - currState->startTime);
            DPRINTF(TLBVerbose, "calling translateTiming again\n");
            currState->fault = tlb->translateTiming(currState->req, currState->tc,
                currState->transState, currState->mode);
            statWalksShortTerminatedAtLevel[0]++;
        }

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    } else {
        // need to do L2 descriptor
        stateQueues[L2].push_back(currState);
    }
    currState = NULL;
}

void
TableWalker::doL2DescriptorWrapper()
{
    currState = stateQueues[L2].front();
    assert(currState->delayed);
    // if there's a stage2 translation object we don't need it any more
    if (currState->stage2Tran) {
        delete currState->stage2Tran;
        currState->stage2Tran = NULL;
    }

    DPRINTF(TLBVerbose, "calling doL2Descriptor for vaddr:%#x\n",
            currState->vaddr_tainted);
    doL2Descriptor();

    // Check if fault was generated
    if (currState->fault != NoFault) {
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);
        statWalksShortTerminatedAtLevel[1]++;
    }
    else {
        // Don't finish the translation if a stage 2 look up is underway
        if (!currState->doingStage2) {
            statWalkServiceTime.sample(curTick() - currState->startTime);
            DPRINTF(TLBVerbose, "calling translateTiming again\n");
            currState->fault = tlb->translateTiming(currState->req,
                currState->tc, currState->transState, currState->mode);
            statWalksShortTerminatedAtLevel[1]++;
        }
    }


    stateQueues[L2].pop_front();
    pending = false;
    nextWalk(currState->tc);

    currState->req = NULL;
    currState->tc = NULL;
    currState->delayed = false;

    delete currState;
    currState = NULL;
}

void
TableWalker::doL0LongDescriptorWrapper()
{
    doLongDescriptorWrapper(L0);
}

void
TableWalker::doL1LongDescriptorWrapper()
{
    doLongDescriptorWrapper(L1);
}

void
TableWalker::doL2LongDescriptorWrapper()
{
    doLongDescriptorWrapper(L2);
}

void
TableWalker::doL3LongDescriptorWrapper()
{
    doLongDescriptorWrapper(L3);
}

void
TableWalker::doLongDescriptorWrapper(LookupLevel curr_lookup_level)
{
    currState = stateQueues[curr_lookup_level].front();
    assert(curr_lookup_level == currState->longDesc.lookupLevel);
    currState->delayed = false;

    // if there's a stage2 translation object we don't need it any more
    if (currState->stage2Tran) {
        delete currState->stage2Tran;
        currState->stage2Tran = NULL;
    }

    DPRINTF(TLBVerbose, "calling doLongDescriptor for vaddr:%#x\n",
            currState->vaddr_tainted);
    doLongDescriptor();

    stateQueues[curr_lookup_level].pop_front();

    if (currState->fault != NoFault) {
        // A fault was generated
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    } else if (!currState->delayed) {
        // No additional lookups required
        // Don't finish the translation if a stage 2 look up is underway
        if (!currState->doingStage2) {
            DPRINTF(TLBVerbose, "calling translateTiming again\n");
            statWalkServiceTime.sample(curTick() - currState->startTime);
            currState->fault = tlb->translateTiming(currState->req, currState->tc,
                                                    currState->transState,
                                                    currState->mode);
            statWalksLongTerminatedAtLevel[(unsigned) curr_lookup_level]++;
        }

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    } else {
        if (curr_lookup_level >= MAX_LOOKUP_LEVELS - 1)
            panic("Max. number of lookups already reached in table walk\n");
        // Need to perform additional lookups
        stateQueues[currState->longDesc.lookupLevel].push_back(currState);
    }
    currState = NULL;
}


void
TableWalker::nextWalk(ThreadContext *tc)
{
    if (pendingQueue.size())
        schedule(doProcessEvent, clockEdge(Cycles(1)));
    else
        completeDrain();
}

bool
TableWalker::fetchDescriptor(Addr descAddr, uint8_t *data, int numBytes,
    Request::Flags flags, int queueIndex, Event *event,
    void (TableWalker::*doDescriptor)())
{
    bool isTiming = currState->timing;

    // do the requests for the page table descriptors have to go through the
    // second stage MMU
    if (currState->stage2Req) {
        Fault fault;
        flags = flags | TLB::MustBeOne;

        if (isTiming) {
            Stage2MMU::Stage2Translation *tran = new
                Stage2MMU::Stage2Translation(*stage2Mmu, data, event,
                                             currState->vaddr);
            currState->stage2Tran = tran;
            stage2Mmu->readDataTimed(currState->tc, descAddr, tran, numBytes,
                                     flags);
            fault = tran->fault;
        } else {
            fault = stage2Mmu->readDataUntimed(currState->tc,
                currState->vaddr, descAddr, data, numBytes, flags,
                currState->functional);
        }

        if (fault != NoFault) {
            currState->fault = fault;
        }
        if (isTiming) {
            if (queueIndex >= 0) {
                DPRINTF(TLBVerbose, "Adding to walker fifo: queue size before adding: %d\n",
                        stateQueues[queueIndex].size());
                stateQueues[queueIndex].push_back(currState);
                currState = NULL;
            }
        } else {
            (this->*doDescriptor)();
        }
    } else {
        if (isTiming) {
            port->dmaAction(MemCmd::ReadReq, descAddr, numBytes, event, data,
                           currState->tc->getCpuPtr()->clockPeriod(),flags);
            if (queueIndex >= 0) {
                DPRINTF(TLBVerbose, "Adding to walker fifo: queue size before adding: %d\n",
                        stateQueues[queueIndex].size());
                stateQueues[queueIndex].push_back(currState);
                currState = NULL;
            }
        } else if (!currState->functional) {
            port->dmaAction(MemCmd::ReadReq, descAddr, numBytes, NULL, data,
                           currState->tc->getCpuPtr()->clockPeriod(), flags);
            (this->*doDescriptor)();
        } else {
            RequestPtr req = new Request(descAddr, numBytes, flags, masterId);
            req->taskId(ContextSwitchTaskId::DMA);
            PacketPtr  pkt = new Packet(req, MemCmd::ReadReq);
            pkt->dataStatic(data);
            port->sendFunctional(pkt);
            (this->*doDescriptor)();
            delete req;
            delete pkt;
        }
    }
    return (isTiming);
}

void
TableWalker::insertTableEntry(DescriptorBase &descriptor, bool longDescriptor)
{
    TlbEntry te;

    // Create and fill a new page table entry
    te.valid          = true;
    te.longDescFormat = longDescriptor;
    te.isHyp          = currState->isHyp;
    te.asid           = currState->asid;
    te.vmid           = currState->vmid;
    te.N              = descriptor.offsetBits();
    te.vpn            = currState->vaddr >> te.N;
    te.size           = (1<<te.N) - 1;
    te.pfn            = descriptor.pfn();
    te.domain         = descriptor.domain();
    te.lookupLevel    = descriptor.lookupLevel;
    te.ns             = !descriptor.secure(haveSecurity, currState) || isStage2;
    te.nstid          = !currState->isSecure;
    te.xn             = descriptor.xn();
    if (currState->aarch64)
        te.el         = currState->el;
    else
        te.el         = 1;

    statPageSizes[pageSizeNtoStatBin(te.N)]++;
    statRequestOrigin[COMPLETED][currState->isFetch]++;

    // ASID has no meaning for stage 2 TLB entries, so mark all stage 2 entries
    // as global
    te.global         = descriptor.global(currState) || isStage2;
    if (longDescriptor) {
        LongDescriptor lDescriptor =
            dynamic_cast<LongDescriptor &>(descriptor);

        te.xn |= currState->xnTable;
        te.pxn = currState->pxnTable || lDescriptor.pxn();
        if (isStage2) {
            // this is actually the HAP field, but its stored in the same bit
            // possitions as the AP field in a stage 1 translation.
            te.hap = lDescriptor.ap();
        } else {
           te.ap = ((!currState->rwTable || descriptor.ap() >> 1) << 1) |
               (currState->userTable && (descriptor.ap() & 0x1));
        }
        if (currState->aarch64)
            memAttrsAArch64(currState->tc, te, currState->longDesc.attrIndx(),
                            currState->longDesc.sh());
        else
            memAttrsLPAE(currState->tc, te, lDescriptor);
    } else {
        te.ap = descriptor.ap();
        memAttrs(currState->tc, te, currState->sctlr, descriptor.texcb(),
                 descriptor.shareable());
    }

    // Debug output
    DPRINTF(TLB, descriptor.dbgHeader().c_str());
    DPRINTF(TLB, " - N:%d pfn:%#x size:%#x global:%d valid:%d\n",
            te.N, te.pfn, te.size, te.global, te.valid);
    DPRINTF(TLB, " - vpn:%#x xn:%d pxn:%d ap:%d domain:%d asid:%d "
            "vmid:%d hyp:%d nc:%d ns:%d\n", te.vpn, te.xn, te.pxn,
            te.ap, static_cast<uint8_t>(te.domain), te.asid, te.vmid, te.isHyp,
            te.nonCacheable, te.ns);
    DPRINTF(TLB, " - domain from L%d desc:%d data:%#x\n",
            descriptor.lookupLevel, static_cast<uint8_t>(descriptor.domain()),
            descriptor.getRawData());

    // Insert the entry into the TLB
    tlb->insert(currState->vaddr, te);
    if (!currState->timing) {
        currState->tc  = NULL;
        currState->req = NULL;
    }
}

ArmISA::TableWalker *
ArmTableWalkerParams::create()
{
    return new ArmISA::TableWalker(this);
}

LookupLevel
TableWalker::toLookupLevel(uint8_t lookup_level_as_int)
{
    switch (lookup_level_as_int) {
      case L1:
        return L1;
      case L2:
        return L2;
      case L3:
        return L3;
      default:
        panic("Invalid lookup level conversion");
    }
}

/* this method keeps track of the table walker queue's residency, so
 * needs to be called whenever requests start and complete. */
void
TableWalker::pendingChange()
{
    unsigned n = pendingQueue.size();
    if ((currState != NULL) && (currState != pendingQueue.front())) {
        ++n;
    }

    if (n != pendingReqs) {
        Tick now = curTick();
        statPendingWalks.sample(pendingReqs, now - pendingChangeTick);
        pendingReqs = n;
        pendingChangeTick = now;
    }
}

uint8_t
TableWalker::pageSizeNtoStatBin(uint8_t N)
{
    /* for statPageSizes */
    switch(N) {
        case 12: return 0; // 4K
        case 14: return 1; // 16K (using 16K granule in v8-64)
        case 16: return 2; // 64K
        case 20: return 3; // 1M
        case 21: return 4; // 2M-LPAE
        case 24: return 5; // 16M
        case 25: return 6; // 32M (using 16K granule in v8-64)
        case 29: return 7; // 512M (using 64K granule in v8-64)
        case 30: return 8; // 1G-LPAE
        default:
            panic("unknown page size");
            return 255;
    }
}

void
TableWalker::regStats()
{
    statWalks
        .name(name() + ".walks")
        .desc("Table walker walks requested")
        ;

    statWalksShortDescriptor
        .name(name() + ".walksShort")
        .desc("Table walker walks initiated with short descriptors")
        .flags(Stats::nozero)
        ;

    statWalksLongDescriptor
        .name(name() + ".walksLong")
        .desc("Table walker walks initiated with long descriptors")
        .flags(Stats::nozero)
        ;

    statWalksShortTerminatedAtLevel
        .init(2)
        .name(name() + ".walksShortTerminationLevel")
        .desc("Level at which table walker walks "
              "with short descriptors terminate")
        .flags(Stats::nozero)
        ;
    statWalksShortTerminatedAtLevel.subname(0, "Level1");
    statWalksShortTerminatedAtLevel.subname(1, "Level2");

    statWalksLongTerminatedAtLevel
        .init(4)
        .name(name() + ".walksLongTerminationLevel")
        .desc("Level at which table walker walks "
              "with long descriptors terminate")
        .flags(Stats::nozero)
        ;
    statWalksLongTerminatedAtLevel.subname(0, "Level0");
    statWalksLongTerminatedAtLevel.subname(1, "Level1");
    statWalksLongTerminatedAtLevel.subname(2, "Level2");
    statWalksLongTerminatedAtLevel.subname(3, "Level3");

    statSquashedBefore
        .name(name() + ".walksSquashedBefore")
        .desc("Table walks squashed before starting")
        .flags(Stats::nozero)
        ;

    statSquashedAfter
        .name(name() + ".walksSquashedAfter")
        .desc("Table walks squashed after completion")
        .flags(Stats::nozero)
        ;

    statWalkWaitTime
        .init(16)
        .name(name() + ".walkWaitTime")
        .desc("Table walker wait (enqueue to first request) latency")
        .flags(Stats::pdf | Stats::nozero | Stats::nonan)
        ;

    statWalkServiceTime
        .init(16)
        .name(name() + ".walkCompletionTime")
        .desc("Table walker service (enqueue to completion) latency")
        .flags(Stats::pdf | Stats::nozero | Stats::nonan)
        ;

    statPendingWalks
        .init(16)
        .name(name() + ".walksPending")
        .desc("Table walker pending requests distribution")
        .flags(Stats::pdf | Stats::dist | Stats::nozero | Stats::nonan)
        ;

    statPageSizes // see DDI 0487A D4-1661
        .init(9)
        .name(name() + ".walkPageSizes")
        .desc("Table walker page sizes translated")
        .flags(Stats::total | Stats::pdf | Stats::dist | Stats::nozero)
        ;
    statPageSizes.subname(0, "4K");
    statPageSizes.subname(1, "16K");
    statPageSizes.subname(2, "64K");
    statPageSizes.subname(3, "1M");
    statPageSizes.subname(4, "2M");
    statPageSizes.subname(5, "16M");
    statPageSizes.subname(6, "32M");
    statPageSizes.subname(7, "512M");
    statPageSizes.subname(8, "1G");

    statRequestOrigin
        .init(2,2) // Instruction/Data, requests/completed
        .name(name() + ".walkRequestOrigin")
        .desc("Table walker requests started/completed, data/inst")
        .flags(Stats::total)
        ;
    statRequestOrigin.subname(0,"Requested");
    statRequestOrigin.subname(1,"Completed");
    statRequestOrigin.ysubname(0,"Data");
    statRequestOrigin.ysubname(1,"Inst");
}
