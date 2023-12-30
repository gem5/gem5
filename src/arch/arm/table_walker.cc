/*
 * Copyright (c) 2010, 2012-2019, 2021-2023 Arm Limited
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
 */
#include "arch/arm/table_walker.hh"

#include <cassert>
#include <memory>

#include "arch/arm/faults.hh"
#include "arch/arm/mmu.hh"
#include "arch/arm/pagetable.hh"
#include "arch/arm/system.hh"
#include "arch/arm/tlb.hh"
#include "base/compiler.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Checkpoint.hh"
#include "debug/Drain.hh"
#include "debug/PageTableWalker.hh"
#include "debug/TLB.hh"
#include "debug/TLBVerbose.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace ArmISA;

TableWalker::TableWalker(const Params &p)
    : ClockedObject(p),
      requestorId(p.sys->getRequestorId(this)),
      port(new Port(*this, requestorId)),
      isStage2(p.is_stage2), tlb(NULL),
      currState(NULL), pending(false),
      numSquashable(p.num_squash_per_cycle),
      release(nullptr),
      stats(this),
      pendingReqs(0),
      pendingChangeTick(curTick()),
      doL1DescEvent([this]{ doL1DescriptorWrapper(); }, name()),
      doL2DescEvent([this]{ doL2DescriptorWrapper(); }, name()),
      doL0LongDescEvent([this]{ doL0LongDescriptorWrapper(); }, name()),
      doL1LongDescEvent([this]{ doL1LongDescriptorWrapper(); }, name()),
      doL2LongDescEvent([this]{ doL2LongDescriptorWrapper(); }, name()),
      doL3LongDescEvent([this]{ doL3LongDescriptorWrapper(); }, name()),
      LongDescEventByLevel { &doL0LongDescEvent, &doL1LongDescEvent,
                             &doL2LongDescEvent, &doL3LongDescEvent },
      doProcessEvent([this]{ processWalkWrapper(); }, name())
{
    sctlr = 0;

    // Cache system-level properties
    if (FullSystem) {
        ArmSystem *arm_sys = dynamic_cast<ArmSystem *>(p.sys);
        assert(arm_sys);
        _physAddrRange = arm_sys->physAddrRange();
        _haveLargeAsid64 = arm_sys->haveLargeAsid64();
    } else {
        _haveLargeAsid64 = false;
        _physAddrRange = 48;
    }

}

TableWalker::~TableWalker()
{
    ;
}

TableWalker::Port &
TableWalker::getTableWalkerPort()
{
    return static_cast<Port&>(getPort("port"));
}

Port &
TableWalker::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port") {
        return *port;
    }
    return ClockedObject::getPort(if_name, idx);
}

void
TableWalker::setMmu(MMU *_mmu)
{
    mmu = _mmu;
    release = mmu->release();
}

TableWalker::WalkerState::WalkerState() :
    tc(nullptr), aarch64(false), el(EL0), physAddrRange(0), req(nullptr),
    asid(0), vmid(0), transState(nullptr),
    vaddr(0), vaddr_tainted(0),
    sctlr(0), scr(0), cpsr(0), tcr(0),
    htcr(0), hcr(0), vtcr(0),
    isWrite(false), isFetch(false), isSecure(false),
    isUncacheable(false),
    secureLookup(false), rwTable(false), userTable(false), xnTable(false),
    pxnTable(false), hpd(false), stage2Req(false),
    stage2Tran(nullptr), timing(false), functional(false),
    mode(BaseMMU::Read), tranType(MMU::NormalTran), l2Desc(l1Desc),
    delayed(false), tableWalker(nullptr)
{
}

TableWalker::Port::Port(TableWalker& _walker, RequestorID id)
  : QueuedRequestPort(_walker.name() + ".port", reqQueue, snoopRespQueue),
    owner{_walker},
    reqQueue(_walker, *this),
    snoopRespQueue(_walker, *this),
    requestorId(id)
{
}

PacketPtr
TableWalker::Port::createPacket(
    Addr desc_addr, int size,
    uint8_t *data, Request::Flags flags, Tick delay,
    Event *event)
{
    RequestPtr req = std::make_shared<Request>(
        desc_addr, size, flags, requestorId);
    req->taskId(context_switch_task_id::DMA);

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->dataStatic(data);

    auto state = new TableWalkerState;
    state->event = event;
    state->delay = delay;

    pkt->senderState = state;
    return pkt;
}

void
TableWalker::Port::sendFunctionalReq(
    Addr desc_addr, int size,
    uint8_t *data, Request::Flags flags)
{
    auto pkt = createPacket(desc_addr, size, data, flags, 0, nullptr);

    sendFunctional(pkt);

    handleRespPacket(pkt);
}

void
TableWalker::Port::sendAtomicReq(
    Addr desc_addr, int size,
    uint8_t *data, Request::Flags flags, Tick delay)
{
    auto pkt = createPacket(desc_addr, size, data, flags, delay, nullptr);

    Tick lat = sendAtomic(pkt);

    handleRespPacket(pkt, lat);
}

void
TableWalker::Port::sendTimingReq(
    Addr desc_addr, int size,
    uint8_t *data, Request::Flags flags, Tick delay,
    Event *event)
{
    auto pkt = createPacket(desc_addr, size, data, flags, delay, event);

    schedTimingReq(pkt, curTick());
}

bool
TableWalker::Port::recvTimingResp(PacketPtr pkt)
{
    // We shouldn't ever get a cacheable block in Modified state.
    assert(pkt->req->isUncacheable() ||
           !(pkt->cacheResponding() && !pkt->hasSharers()));

    handleRespPacket(pkt);

    return true;
}

void
TableWalker::Port::handleRespPacket(PacketPtr pkt, Tick delay)
{
    // Should always see a response with a sender state.
    assert(pkt->isResponse());

    // Get the DMA sender state.
    auto *state = dynamic_cast<TableWalkerState*>(pkt->senderState);
    assert(state);

    handleResp(state, pkt->getAddr(), pkt->req->getSize(), delay);

    delete pkt;
}

void
TableWalker::Port::handleResp(TableWalkerState *state, Addr addr,
                              Addr size, Tick delay)
{
    if (state->event) {
        owner.schedule(state->event, curTick() + delay);
    }
    delete state;
}

void
TableWalker::completeDrain()
{
    if (drainState() == DrainState::Draining &&
        stateQueues[LookupLevel::L0].empty() &&
        stateQueues[LookupLevel::L1].empty() &&
        stateQueues[LookupLevel::L2].empty() &&
        stateQueues[LookupLevel::L3].empty() &&
        pendingQueue.empty()) {

        DPRINTF(Drain, "TableWalker done draining, processing drain event\n");
        signalDrainDone();
    }
}

DrainState
TableWalker::drain()
{
    bool state_queues_not_empty = false;

    for (int i = 0; i < LookupLevel::Num_ArmLookupLevel; ++i) {
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
    if (params().sys->isTimingMode() && currState) {
        delete currState;
        currState = NULL;
        pendingChange();
    }
}

Fault
TableWalker::walk(const RequestPtr &_req, ThreadContext *_tc, uint16_t _asid,
                  vmid_t _vmid, MMU::Mode _mode,
                  MMU::Translation *_trans, bool _timing, bool _functional,
                  bool secure, MMU::ArmTranslationType tranType,
                  bool _stage2Req, const TlbEntry *walk_entry)
{
    assert(!(_functional && _timing));
    ++stats.walks;

    WalkerState *savedCurrState = NULL;

    if (!currState && !_functional) {
        // For atomic mode, a new WalkerState instance should be only created
        // once per TLB. For timing mode, a new instance is generated for every
        // TLB miss.
        DPRINTF(PageTableWalker, "creating new instance of WalkerState\n");

        currState = new WalkerState();
        currState->tableWalker = this;
    } else if (_functional) {
        // If we are mixing functional mode with timing (or even
        // atomic), we need to to be careful and clean up after
        // ourselves to not risk getting into an inconsistent state.
        DPRINTF(PageTableWalker,
                "creating functional instance of WalkerState\n");
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
            ++stats.squashedBefore;
            return std::make_shared<ReExec>();
        }
    }
    pendingChange();

    currState->startTime = curTick();
    currState->tc = _tc;
    // ARM DDI 0487A.f (ARMv8 ARM) pg J8-5672
    // aarch32/translation/translation/AArch32.TranslateAddress dictates
    // even AArch32 EL0 will use AArch64 translation if EL1 is in AArch64.
    if (isStage2) {
        currState->el = EL1;
        currState->aarch64 = ELIs64(_tc, EL2);
    } else {
        currState->el =
            MMU::tranTypeEL(_tc->readMiscReg(MISCREG_CPSR),
                _tc->readMiscReg(MISCREG_SCR_EL3),
                tranType);
        currState->aarch64 =
            ELIs64(_tc, currState->el == EL0 ? EL1 : currState->el);
    }
    currState->transState = _trans;
    currState->req = _req;
    if (walk_entry) {
        currState->walkEntry = *walk_entry;
    } else {
        currState->walkEntry = TlbEntry();
    }
    currState->fault = NoFault;
    currState->asid = _asid;
    currState->vmid = _vmid;
    currState->timing = _timing;
    currState->functional = _functional;
    currState->mode = _mode;
    currState->tranType = tranType;
    currState->isSecure = secure;
    currState->physAddrRange = _physAddrRange;

    /** @todo These should be cached or grabbed from cached copies in
     the TLB, all these miscreg reads are expensive */
    currState->vaddr_tainted = currState->req->getVaddr();
    if (currState->aarch64)
        currState->vaddr = purifyTaggedAddr(currState->vaddr_tainted,
                                            currState->tc, currState->el,
                                            currState->mode==BaseMMU::Execute);
    else
        currState->vaddr = currState->vaddr_tainted;

    if (currState->aarch64) {
        currState->hcr = currState->tc->readMiscReg(MISCREG_HCR_EL2);
        if (isStage2) {
            currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL1);
            if (currState->secureLookup) {
                currState->vtcr =
                    currState->tc->readMiscReg(MISCREG_VSTCR_EL2);
            } else {
                currState->vtcr =
                    currState->tc->readMiscReg(MISCREG_VTCR_EL2);
            }
        } else switch (currState->el) {
          case EL0:
            if (HaveExt(currState->tc, ArmExtension::FEAT_VHE) &&
                  currState->hcr.tge == 1 && currState->hcr.e2h ==1) {
              currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL2);
              currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL2);
            } else {
              currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL1);
              currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL1);
            }
            break;
          case EL1:
            currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL1);
            currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL1);
            break;
          case EL2:
            assert(release->has(ArmExtension::VIRTUALIZATION));
            currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL2);
            currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL2);
            break;
          case EL3:
            assert(release->has(ArmExtension::SECURITY));
            currState->sctlr = currState->tc->readMiscReg(MISCREG_SCTLR_EL3);
            currState->tcr = currState->tc->readMiscReg(MISCREG_TCR_EL3);
            break;
          default:
            panic("Invalid exception level");
            break;
        }
    } else {
        currState->sctlr = currState->tc->readMiscReg(snsBankedIndex(
            MISCREG_SCTLR, currState->tc, !currState->isSecure));
        currState->ttbcr = currState->tc->readMiscReg(snsBankedIndex(
            MISCREG_TTBCR, currState->tc, !currState->isSecure));
        currState->htcr  = currState->tc->readMiscReg(MISCREG_HTCR);
        currState->hcr   = currState->tc->readMiscReg(MISCREG_HCR);
        currState->vtcr  = currState->tc->readMiscReg(MISCREG_VTCR);
    }
    sctlr = currState->sctlr;

    currState->isFetch = (currState->mode == BaseMMU::Execute);
    currState->isWrite = (currState->mode == BaseMMU::Write);

    stats.requestOrigin[REQUESTED][currState->isFetch]++;

    currState->stage2Req = _stage2Req && !isStage2;

    bool hyp = currState->el == EL2;
    bool long_desc_format = currState->aarch64 || hyp || isStage2 ||
                            longDescFormatInUse(currState->tc);

    if (long_desc_format) {
        // Helper variables used for hierarchical permissions
        currState->secureLookup = currState->isSecure;
        currState->rwTable = true;
        currState->userTable = true;
        currState->xnTable = false;
        currState->pxnTable = false;

        ++stats.walksLongDescriptor;
    } else {
        ++stats.walksShortDescriptor;
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

    // Check if a previous walk filled this request already
    // @TODO Should this always be the TLB or should we look in the stage2 TLB?
    TlbEntry* te = mmu->lookup(currState->vaddr, currState->asid,
        currState->vmid, currState->isSecure, true, false,
        currState->el, false, isStage2, currState->mode);

    // Check if we still need to have a walk for this request. If the requesting
    // instruction has been squashed, or a previous walk has filled the TLB with
    // a match, we just want to get rid of the walk. The latter could happen
    // when there are multiple outstanding misses to a single page and a
    // previous request has been successfully translated.
    if (!currState->transState->squashed() && (!te || te->partial)) {
        // We've got a valid request, lets process it
        pending = true;
        pendingQueue.pop_front();
        // Keep currState in case one of the processWalk... calls NULLs it

        if (te && te->partial) {
            currState->walkEntry = *te;
        }
        WalkerState *curr_state_copy = currState;
        Fault f;
        if (currState->aarch64)
            f = processWalkAArch64();
        else if (bool hyp = currState->el == EL2;
                 longDescFormatInUse(currState->tc) ||
                 hyp || isStage2)
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
           (currState->transState->squashed() ||
            (te && !te->partial))) {
        pendingQueue.pop_front();
        num_squashed++;
        stats.squashedBefore++;

        DPRINTF(TLB, "Squashing table walk for address %#x\n",
                      currState->vaddr_tainted);

        if (currState->transState->squashed()) {
            // finish the translation which will delete the translation object
            currState->transState->finish(
                std::make_shared<UnimpFault>("Squashed Inst"),
                currState->req, currState->tc, currState->mode);
        } else {
            // translate the request now that we know it will work
            stats.walkServiceTime.sample(curTick() - currState->startTime);
            mmu->translateTiming(currState->req, currState->tc,
                currState->transState, currState->mode,
                currState->tranType, isStage2);
        }

        // delete the current request
        delete currState;

        // peak at the next one
        if (pendingQueue.size()) {
            currState = pendingQueue.front();
            te = mmu->lookup(currState->vaddr, currState->asid,
                currState->vmid, currState->isSecure, true,
                false, currState->el, false, isStage2, currState->mode);
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

    // For short descriptors, translation configs are held in
    // TTBR1.
    RegVal ttbr1 = currState->tc->readMiscReg(snsBankedIndex(
        MISCREG_TTBR1, currState->tc, !currState->isSecure));

    const auto irgn0_mask = 0x1;
    const auto irgn1_mask = 0x40;
    currState->isUncacheable = (ttbr1 & (irgn0_mask | irgn1_mask)) == 0;

    // If translation isn't enabled, we shouldn't be here
    assert(currState->sctlr.m || isStage2);
    const bool is_atomic = currState->req->isAtomic();
    const bool have_security = release->has(ArmExtension::SECURITY);

    DPRINTF(TLB, "Beginning table walk for address %#x, TTBCR: %#x, bits:%#x\n",
            currState->vaddr_tainted, currState->ttbcr, mbits(currState->vaddr, 31,
                                                      32 - currState->ttbcr.n));

    stats.walkWaitTime.sample(curTick() - currState->startTime);

    if (currState->ttbcr.n == 0 || !mbits(currState->vaddr, 31,
                                          32 - currState->ttbcr.n)) {
        DPRINTF(TLB, " - Selecting TTBR0\n");
        // Check if table walk is allowed when Security Extensions are enabled
        if (have_security && currState->ttbcr.pd0) {
            if (currState->isFetch)
                return std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1,
                    isStage2,
                    ArmFault::VmsaTran);
            else
                return std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : currState->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    ArmFault::VmsaTran);
        }
        ttbr = currState->tc->readMiscReg(snsBankedIndex(
            MISCREG_TTBR0, currState->tc, !currState->isSecure));
    } else {
        DPRINTF(TLB, " - Selecting TTBR1\n");
        // Check if table walk is allowed when Security Extensions are enabled
        if (have_security && currState->ttbcr.pd1) {
            if (currState->isFetch)
                return std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1,
                    isStage2,
                    ArmFault::VmsaTran);
            else
                return std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : currState->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    ArmFault::VmsaTran);
        }
        ttbr = ttbr1;
        currState->ttbcr.n = 0;
    }

    Addr l1desc_addr = mbits(ttbr, 31, 14 - currState->ttbcr.n) |
        (bits(currState->vaddr, 31 - currState->ttbcr.n, 20) << 2);
    DPRINTF(TLB, " - Descriptor at address %#x (%s)\n", l1desc_addr,
            currState->isSecure ? "s" : "ns");

    // Trickbox address check
    Fault f;
    f = testWalk(l1desc_addr, sizeof(uint32_t),
                 TlbEntry::DomainType::NoAccess, LookupLevel::L1, isStage2);
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
    if (currState->sctlr.c == 0 || currState->isUncacheable) {
        flag.set(Request::UNCACHEABLE);
    }

    if (currState->isSecure) {
        flag.set(Request::SECURE);
    }

    bool delayed;
    delayed = fetchDescriptor(l1desc_addr, (uint8_t*)&currState->l1Desc.data,
                              sizeof(uint32_t), flag, LookupLevel::L1,
                              &doL1DescEvent,
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
    LookupLevel start_lookup_level = LookupLevel::L1;

    DPRINTF(TLB, "Beginning table walk for address %#x, TTBCR: %#x\n",
            currState->vaddr_tainted, currState->ttbcr);

    stats.walkWaitTime.sample(curTick() - currState->startTime);

    Request::Flags flag = Request::PT_WALK;
    if (currState->isSecure)
        flag.set(Request::SECURE);

    // work out which base address register to use, if in hyp mode we always
    // use HTTBR
    if (isStage2) {
        DPRINTF(TLB, " - Selecting VTTBR (long-desc.)\n");
        ttbr = currState->tc->readMiscReg(MISCREG_VTTBR);
        tsz  = sext<4>(currState->vtcr.t0sz);
        start_lookup_level = currState->vtcr.sl0 ?
            LookupLevel::L1 : LookupLevel::L2;
        currState->isUncacheable = currState->vtcr.irgn0 == 0;
    } else if (currState->el == EL2) {
        DPRINTF(TLB, " - Selecting HTTBR (long-desc.)\n");
        ttbr = currState->tc->readMiscReg(MISCREG_HTTBR);
        tsz  = currState->htcr.t0sz;
        currState->isUncacheable = currState->htcr.irgn0 == 0;
    } else {
        assert(longDescFormatInUse(currState->tc));

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

        const bool is_atomic = currState->req->isAtomic();

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
                        ArmFault::TranslationLL + LookupLevel::L1,
                        isStage2,
                        ArmFault::LpaeTran);
                else
                    return std::make_shared<DataAbort>(
                        currState->vaddr_tainted,
                        TlbEntry::DomainType::NoAccess,
                        is_atomic ? false : currState->isWrite,
                        ArmFault::TranslationLL + LookupLevel::L1,
                        isStage2,
                        ArmFault::LpaeTran);
            }
            ttbr = currState->tc->readMiscReg(snsBankedIndex(
                MISCREG_TTBR0, currState->tc, !currState->isSecure));
            tsz = currState->ttbcr.t0sz;
            currState->isUncacheable = currState->ttbcr.irgn0 == 0;
            if (ttbr0_max < (1ULL << 30))  // Upper limit < 1 GiB
                start_lookup_level = LookupLevel::L2;
        } else if (currState->vaddr >= ttbr1_min) {
            DPRINTF(TLB, " - Selecting TTBR1 (long-desc.)\n");
            // Check if table walk is allowed
            if (currState->ttbcr.epd1) {
                if (currState->isFetch)
                    return std::make_shared<PrefetchAbort>(
                        currState->vaddr_tainted,
                        ArmFault::TranslationLL + LookupLevel::L1,
                        isStage2,
                        ArmFault::LpaeTran);
                else
                    return std::make_shared<DataAbort>(
                        currState->vaddr_tainted,
                        TlbEntry::DomainType::NoAccess,
                        is_atomic ? false : currState->isWrite,
                        ArmFault::TranslationLL + LookupLevel::L1,
                        isStage2,
                        ArmFault::LpaeTran);
            }
            ttbr = currState->tc->readMiscReg(snsBankedIndex(
                MISCREG_TTBR1, currState->tc, !currState->isSecure));
            tsz = currState->ttbcr.t1sz;
            currState->isUncacheable = currState->ttbcr.irgn1 == 0;
            // Lower limit >= 3 GiB
            if (ttbr1_min >= (1ULL << 31) + (1ULL << 30))
                start_lookup_level = LookupLevel::L2;
        } else {
            // Out of boundaries -> translation fault
            if (currState->isFetch)
                return std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1,
                    isStage2,
                    ArmFault::LpaeTran);
            else
                return std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : currState->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1,
                    isStage2, ArmFault::LpaeTran);
        }

    }

    // Perform lookup (ARM ARM issue C B3.6.6)
    if (start_lookup_level == LookupLevel::L1) {
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
    Fault f = testWalk(desc_addr, sizeof(uint64_t),
                       TlbEntry::DomainType::NoAccess, start_lookup_level,
                       isStage2);
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

    if (currState->sctlr.c == 0 || currState->isUncacheable) {
        flag.set(Request::UNCACHEABLE);
    }

    currState->longDesc.lookupLevel = start_lookup_level;
    currState->longDesc.aarch64 = false;
    currState->longDesc.grainSize = Grain4KB;

    bool delayed = fetchDescriptor(desc_addr, (uint8_t*)&currState->longDesc.data,
                                   sizeof(uint64_t), flag, start_lookup_level,
                                   LongDescEventByLevel[start_lookup_level],
                                   &TableWalker::doLongDescriptor);
    if (!delayed) {
        f = currState->fault;
    }

    return f;
}

bool
TableWalker::checkVAddrSizeFaultAArch64(Addr addr, int top_bit,
    GrainSize tg, int tsz, bool low_range)
{
    // The effective maximum input size is 48 if ARMv8.2-LVA is not
    // supported or if the translation granule that is in use is 4KB or
    // 16KB in size. When ARMv8.2-LVA is supported, for the 64KB
    // translation granule size only, the effective minimum value of
    // 52.
    const bool have_lva = HaveExt(currState->tc, ArmExtension::FEAT_LVA);
    int in_max = (have_lva && tg == Grain64KB) ? 52 : 48;
    int in_min = 64 - (tg == Grain64KB ? 47 : 48);

    return tsz > in_max || tsz < in_min || (low_range ?
        bits(currState->vaddr, top_bit, tsz) != 0x0 :
        bits(currState->vaddr, top_bit, tsz) != mask(top_bit - tsz + 1));
}

bool
TableWalker::checkAddrSizeFaultAArch64(Addr addr, int pa_range)
{
    return (pa_range != _physAddrRange &&
            bits(addr, _physAddrRange - 1, pa_range));
}

Fault
TableWalker::processWalkAArch64()
{
    assert(currState->aarch64);

    DPRINTF(TLB, "Beginning table walk for address %#llx, TCR: %#llx\n",
            currState->vaddr_tainted, currState->tcr);

    stats.walkWaitTime.sample(curTick() - currState->startTime);

    // Determine TTBR, table size, granule size and phys. address range
    Addr ttbr = 0;
    int tsz = 0, ps = 0;
    GrainSize tg = Grain4KB; // grain size computed from tg* field
    bool fault = false;

    int top_bit = computeAddrTop(currState->tc,
        bits(currState->vaddr, 55),
        currState->mode==BaseMMU::Execute,
        currState->tcr,
        currState->el);

    bool vaddr_fault = false;
    switch (currState->el) {
      case EL0:
        {
            Addr ttbr0;
            Addr ttbr1;
            if (HaveExt(currState->tc, ArmExtension::FEAT_VHE) &&
                    currState->hcr.tge==1 && currState->hcr.e2h == 1) {
                // VHE code for EL2&0 regime
                ttbr0 = currState->tc->readMiscReg(MISCREG_TTBR0_EL2);
                ttbr1 = currState->tc->readMiscReg(MISCREG_TTBR1_EL2);
            } else {
                ttbr0 = currState->tc->readMiscReg(MISCREG_TTBR0_EL1);
                ttbr1 = currState->tc->readMiscReg(MISCREG_TTBR1_EL1);
            }
            switch (bits(currState->vaddr, 63,48)) {
              case 0:
                DPRINTF(TLB, " - Selecting TTBR0 (AArch64)\n");
                ttbr = ttbr0;
                tsz = 64 - currState->tcr.t0sz;
                tg = GrainMap_tg0[currState->tcr.tg0];
                currState->hpd = currState->tcr.hpd0;
                currState->isUncacheable = currState->tcr.irgn0 == 0;
                vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                    top_bit, tg, tsz, true);

                if (vaddr_fault || currState->tcr.epd0)
                    fault = true;
                break;
              case 0xffff:
                DPRINTF(TLB, " - Selecting TTBR1 (AArch64)\n");
                ttbr = ttbr1;
                tsz = 64 - currState->tcr.t1sz;
                tg = GrainMap_tg1[currState->tcr.tg1];
                currState->hpd = currState->tcr.hpd1;
                currState->isUncacheable = currState->tcr.irgn1 == 0;
                vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                    top_bit, tg, tsz, false);

                if (vaddr_fault || currState->tcr.epd1)
                    fault = true;
                break;
              default:
                // top two bytes must be all 0s or all 1s, else invalid addr
                fault = true;
            }
            ps = currState->tcr.ips;
        }
        break;
      case EL1:
        if (isStage2) {
            if (currState->secureLookup) {
                DPRINTF(TLB, " - Selecting VSTTBR_EL2 (AArch64 stage 2)\n");
                ttbr = currState->tc->readMiscReg(MISCREG_VSTTBR_EL2);
            } else {
                DPRINTF(TLB, " - Selecting VTTBR_EL2 (AArch64 stage 2)\n");
                ttbr = currState->tc->readMiscReg(MISCREG_VTTBR_EL2);
            }
            tsz = 64 - currState->vtcr.t0sz64;
            tg = GrainMap_tg0[currState->vtcr.tg0];

            ps = currState->vtcr.ps;
            currState->isUncacheable = currState->vtcr.irgn0 == 0;
        } else {
            switch (bits(currState->vaddr, top_bit)) {
              case 0:
                DPRINTF(TLB, " - Selecting TTBR0_EL1 (AArch64)\n");
                ttbr = currState->tc->readMiscReg(MISCREG_TTBR0_EL1);
                tsz = 64 - currState->tcr.t0sz;
                tg = GrainMap_tg0[currState->tcr.tg0];
                currState->hpd = currState->tcr.hpd0;
                currState->isUncacheable = currState->tcr.irgn0 == 0;
                vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                    top_bit, tg, tsz, true);

                if (vaddr_fault || currState->tcr.epd0)
                    fault = true;
                break;
              case 0x1:
                DPRINTF(TLB, " - Selecting TTBR1_EL1 (AArch64)\n");
                ttbr = currState->tc->readMiscReg(MISCREG_TTBR1_EL1);
                tsz = 64 - currState->tcr.t1sz;
                tg = GrainMap_tg1[currState->tcr.tg1];
                currState->hpd = currState->tcr.hpd1;
                currState->isUncacheable = currState->tcr.irgn1 == 0;
                vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                    top_bit, tg, tsz, false);

                if (vaddr_fault || currState->tcr.epd1)
                    fault = true;
                break;
              default:
                // top two bytes must be all 0s or all 1s, else invalid addr
                fault = true;
            }
            ps = currState->tcr.ips;
        }
        break;
      case EL2:
        switch(bits(currState->vaddr, top_bit)) {
          case 0:
            DPRINTF(TLB, " - Selecting TTBR0_EL2 (AArch64)\n");
            ttbr = currState->tc->readMiscReg(MISCREG_TTBR0_EL2);
            tsz = 64 - currState->tcr.t0sz;
            tg = GrainMap_tg0[currState->tcr.tg0];
            currState->hpd = currState->hcr.e2h ?
                currState->tcr.hpd0 : currState->tcr.hpd;
            currState->isUncacheable = currState->tcr.irgn0 == 0;
            vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                top_bit, tg, tsz, true);

            if (vaddr_fault || (currState->hcr.e2h && currState->tcr.epd0))
                fault = true;
            break;

          case 0x1:
            DPRINTF(TLB, " - Selecting TTBR1_EL2 (AArch64)\n");
            ttbr = currState->tc->readMiscReg(MISCREG_TTBR1_EL2);
            tsz = 64 - currState->tcr.t1sz;
            tg = GrainMap_tg1[currState->tcr.tg1];
            currState->hpd = currState->tcr.hpd1;
            currState->isUncacheable = currState->tcr.irgn1 == 0;
            vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                top_bit, tg, tsz, false);

            if (vaddr_fault || !currState->hcr.e2h || currState->tcr.epd1)
                fault = true;
            break;

           default:
              // invalid addr if top two bytes are not all 0s
              fault = true;
        }
        ps = currState->hcr.e2h ? currState->tcr.ips: currState->tcr.ps;
        break;
      case EL3:
        switch(bits(currState->vaddr, top_bit)) {
          case 0:
            DPRINTF(TLB, " - Selecting TTBR0_EL3 (AArch64)\n");
            ttbr = currState->tc->readMiscReg(MISCREG_TTBR0_EL3);
            tsz = 64 - currState->tcr.t0sz;
            tg = GrainMap_tg0[currState->tcr.tg0];
            currState->hpd = currState->tcr.hpd;
            currState->isUncacheable = currState->tcr.irgn0 == 0;
            vaddr_fault = checkVAddrSizeFaultAArch64(currState->vaddr,
                top_bit, tg, tsz, true);

            if (vaddr_fault)
                fault = true;
            break;
          default:
            // invalid addr if top two bytes are not all 0s
            fault = true;
        }
        ps = currState->tcr.ps;
        break;
    }

    const bool is_atomic = currState->req->isAtomic();

    if (fault) {
        Fault f;
        if (currState->isFetch)
            f =  std::make_shared<PrefetchAbort>(
                currState->vaddr_tainted,
                ArmFault::TranslationLL + LookupLevel::L0, isStage2,
                ArmFault::LpaeTran);
        else
            f = std::make_shared<DataAbort>(
                currState->vaddr_tainted,
                TlbEntry::DomainType::NoAccess,
                is_atomic ? false : currState->isWrite,
                ArmFault::TranslationLL + LookupLevel::L0,
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

    // Clamp to lower limit
    int pa_range = decodePhysAddrRange64(ps);
    if (pa_range > _physAddrRange) {
        currState->physAddrRange = _physAddrRange;
    } else {
        currState->physAddrRange = pa_range;
    }

    auto [table_addr, desc_addr, start_lookup_level] = walkAddresses(
        ttbr, tg, tsz, pa_range);

    // Determine physical address size and raise an Address Size Fault if
    // necessary
    if (checkAddrSizeFaultAArch64(table_addr, currState->physAddrRange)) {
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
                is_atomic ? false : currState->isWrite,
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

    // Trickbox address check
    Fault f = testWalk(desc_addr, sizeof(uint64_t),
                       TlbEntry::DomainType::NoAccess, start_lookup_level, isStage2);
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
    if (currState->sctlr.c == 0 || currState->isUncacheable) {
        flag.set(Request::UNCACHEABLE);
    }

    if (currState->isSecure) {
        flag.set(Request::SECURE);
    }

    currState->longDesc.lookupLevel = start_lookup_level;
    currState->longDesc.aarch64 = true;
    currState->longDesc.grainSize = tg;
    currState->longDesc.physAddrRange = _physAddrRange;

    if (currState->timing) {
        fetchDescriptor(desc_addr, (uint8_t*) &currState->longDesc.data,
                        sizeof(uint64_t), flag, start_lookup_level,
                        LongDescEventByLevel[start_lookup_level], NULL);
    } else {
        fetchDescriptor(desc_addr, (uint8_t*)&currState->longDesc.data,
                        sizeof(uint64_t), flag, -1, NULL,
                        &TableWalker::doLongDescriptor);
        f = currState->fault;
    }

    return f;
}

std::tuple<Addr, Addr, TableWalker::LookupLevel>
TableWalker::walkAddresses(Addr ttbr, GrainSize tg, int tsz, int pa_range)
{
    const auto* ptops = getPageTableOps(tg);

    LookupLevel first_level = LookupLevel::Num_ArmLookupLevel;
    Addr table_addr = 0;
    Addr desc_addr = 0;

    if (currState->walkEntry.valid) {
        // WalkCache hit
        TlbEntry* entry = &currState->walkEntry;
        DPRINTF(PageTableWalker,
                "Walk Cache hit: va=%#x, level=%d, table address=%#x\n",
                currState->vaddr, entry->lookupLevel, entry->pfn);

        currState->xnTable = entry->xn;
        currState->pxnTable = entry->pxn;
        currState->rwTable = bits(entry->ap, 1);
        currState->userTable = bits(entry->ap, 0);

        table_addr = entry->pfn;
        first_level = (LookupLevel)(entry->lookupLevel + 1);
    } else {
        // WalkCache miss
        first_level = isStage2 ?
            ptops->firstS2Level(currState->vtcr.sl0) :
            ptops->firstLevel(64 - tsz);
        panic_if(first_level == LookupLevel::Num_ArmLookupLevel,
                 "Table walker couldn't find lookup level\n");

        int stride = tg - 3;
        int base_addr_lo = 3 + tsz - stride * (3 - first_level) - tg;

        if (pa_range == 52) {
            int z = (base_addr_lo < 6) ? 6 : base_addr_lo;
            table_addr = mbits(ttbr, 47, z);
            table_addr |= (bits(ttbr, 5, 2) << 48);
        } else {
            table_addr = mbits(ttbr, 47, base_addr_lo);
        }
    }

    desc_addr = table_addr + ptops->index(currState->vaddr, first_level, tsz);

    return std::make_tuple(table_addr, desc_addr, first_level);
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
        PRRR prrr = tc->readMiscReg(snsBankedIndex(MISCREG_PRRR,
                                    currState->tc, !currState->isSecure));
        NMRR nmrr = tc->readMiscReg(snsBankedIndex(MISCREG_NMRR,
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
    LongDescriptor &l_descriptor)
{
    assert(release->has(ArmExtension::LPAE));

    uint8_t attr;
    uint8_t sh = l_descriptor.sh();
    // Different format and source of attributes if this is a stage 2
    // translation
    if (isStage2) {
        attr = l_descriptor.memAttr();
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
        uint8_t attrIndx = l_descriptor.attrIndx();

        // LPAE always uses remapping of memory attributes, irrespective of the
        // value of SCTLR.TRE
        MiscRegIndex reg = attrIndx & 0x4 ? MISCREG_MAIR1 : MISCREG_MAIR0;
        int reg_as_int = snsBankedIndex(reg, currState->tc,
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
TableWalker::memAttrsAArch64(ThreadContext *tc, TlbEntry &te,
                             LongDescriptor &l_descriptor)
{
    uint8_t attr;
    uint8_t attr_hi;
    uint8_t attr_lo;
    uint8_t sh = l_descriptor.sh();

    if (isStage2) {
        attr = l_descriptor.memAttr();
        uint8_t attr_hi = (attr >> 2) & 0x3;
        uint8_t attr_lo =  attr       & 0x3;

        DPRINTF(TLBVerbose, "memAttrsAArch64 MemAttr:%#x sh:%#x\n", attr, sh);

        if (attr_hi == 0) {
            te.mtype        = attr_lo == 0 ? TlbEntry::MemoryType::StronglyOrdered
                                            : TlbEntry::MemoryType::Device;
            te.outerAttrs   = 0;
            te.innerAttrs   = attr_lo == 0 ? 1 : 3;
            te.nonCacheable = true;
        } else {
            te.mtype        = TlbEntry::MemoryType::Normal;
            te.outerAttrs   = attr_hi == 1 ? 0 :
                              attr_hi == 2 ? 2 : 1;
            te.innerAttrs   = attr_lo == 1 ? 0 :
                              attr_lo == 2 ? 6 : 5;
            // Treat write-through memory as uncacheable, this is safe
            // but for performance reasons not optimal.
            te.nonCacheable = (attr_hi == 1) || (attr_hi == 2) ||
                (attr_lo == 1) || (attr_lo == 2);
        }
    } else {
        uint8_t attrIndx = l_descriptor.attrIndx();

        DPRINTF(TLBVerbose, "memAttrsAArch64 AttrIndx:%#x sh:%#x\n", attrIndx, sh);
        ExceptionLevel regime =  s1TranslationRegime(tc, currState->el);

        // Select MAIR
        uint64_t mair;
        switch (regime) {
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
        attr = bits(mair, 8 * attrIndx + 7, 8 * attrIndx);
        attr_lo = bits(attr, 3, 0);
        attr_hi = bits(attr, 7, 4);

        // Memory type
        te.mtype = attr_hi == 0 ? TlbEntry::MemoryType::Device : TlbEntry::MemoryType::Normal;

        // Cacheability
        te.nonCacheable = false;
        if (te.mtype == TlbEntry::MemoryType::Device) {  // Device memory
            te.nonCacheable = true;
        }
        // Treat write-through memory as uncacheable, this is safe
        // but for performance reasons not optimal.
        switch (attr_hi) {
          case 0x1 ... 0x3: // Normal Memory, Outer Write-through transient
          case 0x4:         // Normal memory, Outer Non-cacheable
          case 0x8 ... 0xb: // Normal Memory, Outer Write-through non-transient
            te.nonCacheable = true;
        }
        switch (attr_lo) {
          case 0x1 ... 0x3: // Normal Memory, Inner Write-through transient
          case 0x9 ... 0xb: // Normal Memory, Inner Write-through non-transient
            warn_if(!attr_hi, "Unpredictable behavior");
            [[fallthrough]];
          case 0x4:         // Device-nGnRE memory or
                            // Normal memory, Inner Non-cacheable
          case 0x8:         // Device-nGRE memory or
                            // Normal memory, Inner Write-through non-transient
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
}

void
TableWalker::doL1Descriptor()
{
    if (currState->fault != NoFault) {
        return;
    }

    currState->l1Desc.data = htog(currState->l1Desc.data,
                                  byteOrder(currState->tc));

    DPRINTF(TLB, "L1 descriptor for %#x is %#x\n",
            currState->vaddr_tainted, currState->l1Desc.data);
    TlbEntry te;

    const bool is_atomic = currState->req->isAtomic();

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
                    ArmFault::TranslationLL + LookupLevel::L1,
                    isStage2,
                    ArmFault::VmsaTran);
        else
            currState->fault =
                std::make_shared<DataAbort>(
                    currState->vaddr_tainted,
                    TlbEntry::DomainType::NoAccess,
                    is_atomic ? false : currState->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
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
                is_atomic ? false : currState->isWrite,
                ArmFault::AccessFlagLL + LookupLevel::L1,
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
            currState->fault = testWalk(l2desc_addr, sizeof(uint32_t),
                                        currState->l1Desc.domain(),
                                        LookupLevel::L2, isStage2);

            if (currState->fault) {
                if (!currState->timing) {
                    currState->tc = NULL;
                    currState->req = NULL;
                }
                return;
            }

            Request::Flags flag = Request::PT_WALK;

            if (currState->sctlr.c == 0 || currState->isUncacheable) {
                flag.set(Request::UNCACHEABLE);
            }

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

Fault
TableWalker::generateLongDescFault(ArmFault::FaultSource src)
{
    if (currState->isFetch) {
        return std::make_shared<PrefetchAbort>(
            currState->vaddr_tainted,
            src + currState->longDesc.lookupLevel,
            isStage2,
            ArmFault::LpaeTran);
    } else {
        return std::make_shared<DataAbort>(
            currState->vaddr_tainted,
            TlbEntry::DomainType::NoAccess,
            currState->req->isAtomic() ? false : currState->isWrite,
            src + currState->longDesc.lookupLevel,
            isStage2,
            ArmFault::LpaeTran);
    }
}

void
TableWalker::doLongDescriptor()
{
    if (currState->fault != NoFault) {
        return;
    }

    currState->longDesc.data = htog(currState->longDesc.data,
                                    byteOrder(currState->tc));

    DPRINTF(TLB, "L%d descriptor for %#llx is %#llx (%s)\n",
            currState->longDesc.lookupLevel, currState->vaddr_tainted,
            currState->longDesc.data,
            currState->aarch64 ? "AArch64" : "long-desc.");

    if ((currState->longDesc.type() == LongDescriptor::Block) ||
        (currState->longDesc.type() == LongDescriptor::Page)) {
        DPRINTF(PageTableWalker, "Analyzing L%d descriptor: %#llx, pxn: %d, "
                "xn: %d, ap: %d, af: %d, type: %d\n",
                currState->longDesc.lookupLevel,
                currState->longDesc.data,
                currState->longDesc.pxn(),
                currState->longDesc.xn(),
                currState->longDesc.ap(),
                currState->longDesc.af(),
                currState->longDesc.type());
    } else {
        DPRINTF(PageTableWalker, "Analyzing L%d descriptor: %#llx, type: %d\n",
                currState->longDesc.lookupLevel,
                currState->longDesc.data,
                currState->longDesc.type());
    }

    TlbEntry te;

    switch (currState->longDesc.type()) {
      case LongDescriptor::Invalid:
        DPRINTF(TLB, "L%d descriptor Invalid, causing fault type %d\n",
                currState->longDesc.lookupLevel,
                ArmFault::TranslationLL + currState->longDesc.lookupLevel);

        currState->fault = generateLongDescFault(ArmFault::TranslationLL);
        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }
        return;

      case LongDescriptor::Block:
      case LongDescriptor::Page:
        {
            auto fault_source = ArmFault::FaultSourceInvalid;
            // Check for address size fault
            if (checkAddrSizeFaultAArch64(currState->longDesc.paddr(),
                currState->physAddrRange)) {

                DPRINTF(TLB, "L%d descriptor causing Address Size Fault\n",
                        currState->longDesc.lookupLevel);
                fault_source = ArmFault::AddressSizeLL;

            // Check for access fault
            } else if (currState->longDesc.af() == 0) {

                DPRINTF(TLB, "L%d descriptor causing Access Fault\n",
                        currState->longDesc.lookupLevel);
                fault_source = ArmFault::AccessFlagLL;
            }

            if (fault_source != ArmFault::FaultSourceInvalid) {
                currState->fault = generateLongDescFault(fault_source);
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
                (currState->longDesc.rwTable() || currState->hpd);
            currState->userTable = currState->userTable &&
                (currState->longDesc.userTable() || currState->hpd);
            currState->xnTable = currState->xnTable ||
                (currState->longDesc.xnTable() && !currState->hpd);
            currState->pxnTable = currState->pxnTable ||
                (currState->longDesc.pxnTable() && !currState->hpd);

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

                currState->fault = generateLongDescFault(
                    ArmFault::AddressSizeLL);
                return;
            }

            // Trickbox address check
            currState->fault = testWalk(
                next_desc_addr, sizeof(uint64_t), TlbEntry::DomainType::Client,
                toLookupLevel(currState->longDesc.lookupLevel +1), isStage2);

            if (currState->fault) {
                if (!currState->timing) {
                    currState->tc = NULL;
                    currState->req = NULL;
                }
                return;
            }

            if (mmu->hasWalkCache()) {
                insertPartialTableEntry(currState->longDesc);
            }


            Request::Flags flag = Request::PT_WALK;
            if (currState->secureLookup)
                flag.set(Request::SECURE);

            if (currState->sctlr.c == 0 || currState->isUncacheable) {
                flag.set(Request::UNCACHEABLE);
            }

            LookupLevel L = currState->longDesc.lookupLevel =
                (LookupLevel) (currState->longDesc.lookupLevel + 1);
            Event *event = NULL;
            switch (L) {
              case LookupLevel::L1:
                assert(currState->aarch64);
              case LookupLevel::L2:
              case LookupLevel::L3:
                event = LongDescEventByLevel[L];
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

    currState->l2Desc.data = htog(currState->l2Desc.data,
                                  byteOrder(currState->tc));

    DPRINTF(TLB, "L2 descriptor for %#x is %#x\n",
            currState->vaddr_tainted, currState->l2Desc.data);
    TlbEntry te;

    const bool is_atomic = currState->req->isAtomic();

    if (currState->l2Desc.invalid()) {
        DPRINTF(TLB, "L2 descriptor invalid, causing fault\n");
        if (!currState->timing) {
            currState->tc = NULL;
            currState->req = NULL;
        }
        if (currState->isFetch)
            currState->fault = std::make_shared<PrefetchAbort>(
                    currState->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L2,
                    isStage2,
                    ArmFault::VmsaTran);
        else
            currState->fault = std::make_shared<DataAbort>(
                currState->vaddr_tainted, currState->l1Desc.domain(),
                is_atomic ? false : currState->isWrite,
                ArmFault::TranslationLL + LookupLevel::L2,
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
            TlbEntry::DomainType::NoAccess,
            is_atomic ? false : currState->isWrite,
            ArmFault::AccessFlagLL + LookupLevel::L2, isStage2,
            ArmFault::VmsaTran);
    }

    insertTableEntry(currState->l2Desc, false);
}

void
TableWalker::doL1DescriptorWrapper()
{
    currState = stateQueues[LookupLevel::L1].front();
    currState->delayed = false;
    // if there's a stage2 translation object we don't need it any more
    if (currState->stage2Tran) {
        delete currState->stage2Tran;
        currState->stage2Tran = NULL;
    }


    DPRINTF(PageTableWalker, "L1 Desc object host addr: %p\n",
            &currState->l1Desc.data);
    DPRINTF(PageTableWalker, "L1 Desc object      data: %08x\n",
            currState->l1Desc.data);

    DPRINTF(PageTableWalker, "calling doL1Descriptor for vaddr:%#x\n",
            currState->vaddr_tainted);
    doL1Descriptor();

    stateQueues[LookupLevel::L1].pop_front();
    // Check if fault was generated
    if (currState->fault != NoFault) {
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);
        stats.walksShortTerminatedAtLevel[0]++;

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
        stats.walkServiceTime.sample(curTick() - currState->startTime);
        DPRINTF(PageTableWalker, "calling translateTiming again\n");

        mmu->translateTiming(currState->req, currState->tc,
            currState->transState, currState->mode,
            currState->tranType, isStage2);

        stats.walksShortTerminatedAtLevel[0]++;

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    } else {
        // need to do L2 descriptor
        stateQueues[LookupLevel::L2].push_back(currState);
    }
    currState = NULL;
}

void
TableWalker::doL2DescriptorWrapper()
{
    currState = stateQueues[LookupLevel::L2].front();
    assert(currState->delayed);
    // if there's a stage2 translation object we don't need it any more
    if (currState->stage2Tran) {
        delete currState->stage2Tran;
        currState->stage2Tran = NULL;
    }

    DPRINTF(PageTableWalker, "calling doL2Descriptor for vaddr:%#x\n",
            currState->vaddr_tainted);
    doL2Descriptor();

    // Check if fault was generated
    if (currState->fault != NoFault) {
        currState->transState->finish(currState->fault, currState->req,
                                      currState->tc, currState->mode);
        stats.walksShortTerminatedAtLevel[1]++;
    } else {
        stats.walkServiceTime.sample(curTick() - currState->startTime);
        DPRINTF(PageTableWalker, "calling translateTiming again\n");

        mmu->translateTiming(currState->req, currState->tc,
            currState->transState, currState->mode,
            currState->tranType, isStage2);

        stats.walksShortTerminatedAtLevel[1]++;
    }


    stateQueues[LookupLevel::L2].pop_front();
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
    doLongDescriptorWrapper(LookupLevel::L0);
}

void
TableWalker::doL1LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L1);
}

void
TableWalker::doL2LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L2);
}

void
TableWalker::doL3LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L3);
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

    DPRINTF(PageTableWalker, "calling doLongDescriptor for vaddr:%#x\n",
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
        DPRINTF(PageTableWalker, "calling translateTiming again\n");
        stats.walkServiceTime.sample(curTick() - currState->startTime);

        mmu->translateTiming(currState->req, currState->tc,
            currState->transState, currState->mode,
            currState->tranType, isStage2);

        stats.walksLongTerminatedAtLevel[(unsigned) curr_lookup_level]++;

        pending = false;
        nextWalk(currState->tc);

        currState->req = NULL;
        currState->tc = NULL;
        currState->delayed = false;
        delete currState;
    } else {
        if (curr_lookup_level >= LookupLevel::Num_ArmLookupLevel - 1)
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

    DPRINTF(PageTableWalker,
            "Fetching descriptor at address: 0x%x stage2Req: %d\n",
            descAddr, currState->stage2Req);

    // If this translation has a stage 2 then we know descAddr is an IPA and
    // needs to be translated before we can access the page table. Do that
    // check here.
    if (currState->stage2Req) {
        Fault fault;

        if (isTiming) {
            auto *tran = new
                Stage2Walk(*this, data, event, currState->vaddr,
                    currState->mode, currState->tranType);
            currState->stage2Tran = tran;
            readDataTimed(currState->tc, descAddr, tran, numBytes, flags);
            fault = tran->fault;
        } else {
            fault = readDataUntimed(currState->tc,
                currState->vaddr, descAddr, data, numBytes, flags,
                currState->mode,
                currState->tranType,
                currState->functional);
        }

        if (fault != NoFault) {
            currState->fault = fault;
        }
        if (isTiming) {
            if (queueIndex >= 0) {
                DPRINTF(PageTableWalker, "Adding to walker fifo: "
                        "queue size before adding: %d\n",
                        stateQueues[queueIndex].size());
                stateQueues[queueIndex].push_back(currState);
                currState = NULL;
            }
        } else {
            (this->*doDescriptor)();
        }
    } else {
        if (isTiming) {
            port->sendTimingReq(descAddr, numBytes, data, flags,
                currState->tc->getCpuPtr()->clockPeriod(), event);

            if (queueIndex >= 0) {
                DPRINTF(PageTableWalker, "Adding to walker fifo: "
                        "queue size before adding: %d\n",
                        stateQueues[queueIndex].size());
                stateQueues[queueIndex].push_back(currState);
                currState = NULL;
            }
        } else if (!currState->functional) {
            port->sendAtomicReq(descAddr, numBytes, data, flags,
                currState->tc->getCpuPtr()->clockPeriod());

            (this->*doDescriptor)();
        } else {
            port->sendFunctionalReq(descAddr, numBytes, data, flags);
            (this->*doDescriptor)();
        }
    }
    return (isTiming);
}

void
TableWalker::insertPartialTableEntry(LongDescriptor &descriptor)
{
    const bool have_security = release->has(ArmExtension::SECURITY);
    TlbEntry te;

    // Create and fill a new page table entry
    te.valid          = true;
    te.longDescFormat = true;
    te.partial        = true;
    // The entry is global if there is no address space identifier
    // to differentiate translation contexts
    te.global         = !mmu->hasUnprivRegime(
        currState->el, currState->hcr.e2h);
    te.asid           = currState->asid;
    te.vmid           = currState->vmid;
    te.N              = descriptor.offsetBits();
    te.tg             = descriptor.grainSize;
    te.vpn            = currState->vaddr >> te.N;
    te.size           = (1ULL << te.N) - 1;
    te.pfn            = descriptor.nextTableAddr();
    te.domain         = descriptor.domain();
    te.lookupLevel    = descriptor.lookupLevel;
    te.ns             = !descriptor.secure(have_security, currState);
    te.nstid          = !currState->isSecure;
    te.type           = TypeTLB::unified;

    te.el         = currState->el;

    te.xn = currState->xnTable;
    te.pxn = currState->pxnTable;
    te.ap = (currState->rwTable << 1) | (currState->userTable);

    // Debug output
    DPRINTF(TLB, descriptor.dbgHeader().c_str());
    DPRINTF(TLB, " - N:%d pfn:%#x size:%#x global:%d valid:%d\n",
            te.N, te.pfn, te.size, te.global, te.valid);
    DPRINTF(TLB, " - vpn:%#x xn:%d pxn:%d ap:%d domain:%d asid:%d "
            "vmid:%d hyp:%d nc:%d ns:%d\n", te.vpn, te.xn, te.pxn,
            te.ap, static_cast<uint8_t>(te.domain), te.asid, te.vmid,
            te.el == EL2, te.nonCacheable, te.ns);
    DPRINTF(TLB, " - domain from L%d desc:%d data:%#x\n",
            descriptor.lookupLevel, static_cast<uint8_t>(descriptor.domain()),
            descriptor.getRawData());

    // Insert the entry into the TLBs
    tlb->multiInsert(te);
}

void
TableWalker::insertTableEntry(DescriptorBase &descriptor, bool long_descriptor)
{
    const bool have_security = release->has(ArmExtension::SECURITY);
    TlbEntry te;

    // Create and fill a new page table entry
    te.valid          = true;
    te.longDescFormat = long_descriptor;
    te.asid           = currState->asid;
    te.vmid           = currState->vmid;
    te.N              = descriptor.offsetBits();
    te.vpn            = currState->vaddr >> te.N;
    te.size           = (1<<te.N) - 1;
    te.pfn            = descriptor.pfn();
    te.domain         = descriptor.domain();
    te.lookupLevel    = descriptor.lookupLevel;
    te.ns             = !descriptor.secure(have_security, currState);
    te.nstid          = !currState->isSecure;
    te.xn             = descriptor.xn();
    te.type           = currState->mode == BaseMMU::Execute ?
        TypeTLB::instruction : TypeTLB::data;

    te.el         = currState->el;

    stats.pageSizes[pageSizeNtoStatBin(te.N)]++;
    stats.requestOrigin[COMPLETED][currState->isFetch]++;

    // ASID has no meaning for stage 2 TLB entries, so mark all stage 2 entries
    // as global
    te.global         = descriptor.global(currState) || isStage2;
    if (long_descriptor) {
        LongDescriptor l_descriptor =
            dynamic_cast<LongDescriptor &>(descriptor);

        te.tg = l_descriptor.grainSize;
        te.xn |= currState->xnTable;
        te.pxn = currState->pxnTable || l_descriptor.pxn();
        if (isStage2) {
            // this is actually the HAP field, but its stored in the same bit
            // possitions as the AP field in a stage 1 translation.
            te.hap = l_descriptor.ap();
        } else {
           te.ap = ((!currState->rwTable || descriptor.ap() >> 1) << 1) |
               (currState->userTable && (descriptor.ap() & 0x1));
        }
        if (currState->aarch64)
            memAttrsAArch64(currState->tc, te, l_descriptor);
        else
            memAttrsLPAE(currState->tc, te, l_descriptor);
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
            "vmid:%d nc:%d ns:%d\n", te.vpn, te.xn, te.pxn,
            te.ap, static_cast<uint8_t>(te.domain), te.asid, te.vmid,
            te.nonCacheable, te.ns);
    DPRINTF(TLB, " - domain from L%d desc:%d data:%#x\n",
            descriptor.lookupLevel, static_cast<uint8_t>(descriptor.domain()),
            descriptor.getRawData());

    // Insert the entry into the TLBs
    tlb->multiInsert(te);
    if (!currState->timing) {
        currState->tc  = NULL;
        currState->req = NULL;
    }
}

TableWalker::LookupLevel
TableWalker::toLookupLevel(uint8_t lookup_level_as_int)
{
    switch (lookup_level_as_int) {
      case LookupLevel::L1:
        return LookupLevel::L1;
      case LookupLevel::L2:
        return LookupLevel::L2;
      case LookupLevel::L3:
        return LookupLevel::L3;
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
        stats.pendingWalks.sample(pendingReqs, now - pendingChangeTick);
        pendingReqs = n;
        pendingChangeTick = now;
    }
}

Fault
TableWalker::testWalk(Addr pa, Addr size, TlbEntry::DomainType domain,
                      LookupLevel lookup_level, bool stage2)
{
    return mmu->testWalk(pa, size, currState->vaddr, currState->isSecure,
                         currState->mode, domain, lookup_level, stage2);
}


uint8_t
TableWalker::pageSizeNtoStatBin(uint8_t N)
{
    /* for stats.pageSizes */
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
        case 42: return 9; // 1G-LPAE
        default:
            panic("unknown page size");
            return 255;
    }
}

Fault
TableWalker::readDataUntimed(ThreadContext *tc, Addr vaddr, Addr desc_addr,
    uint8_t *data, int num_bytes, Request::Flags flags, BaseMMU::Mode mode,
    MMU::ArmTranslationType tran_type, bool functional)
{
    Fault fault;

    // translate to physical address using the second stage MMU
    auto req = std::make_shared<Request>();
    req->setVirt(desc_addr, num_bytes, flags | Request::PT_WALK,
                requestorId, 0);

    if (functional) {
        fault = mmu->translateFunctional(req, tc, BaseMMU::Read,
            tran_type, true);
    } else {
        fault = mmu->translateAtomic(req, tc, BaseMMU::Read,
            tran_type, true);
    }

    // Now do the access.
    if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
        Packet pkt = Packet(req, MemCmd::ReadReq);
        pkt.dataStatic(data);
        if (functional) {
            port->sendFunctional(&pkt);
        } else {
            port->sendAtomic(&pkt);
        }
        assert(!pkt.isError());
    }

    // If there was a fault annotate it with the flag saying the foult occured
    // while doing a translation for a stage 1 page table walk.
    if (fault != NoFault) {
        ArmFault *arm_fault = reinterpret_cast<ArmFault *>(fault.get());
        arm_fault->annotate(ArmFault::S1PTW, true);
        arm_fault->annotate(ArmFault::OVA, vaddr);
    }
    return fault;
}

void
TableWalker::readDataTimed(ThreadContext *tc, Addr desc_addr,
                           Stage2Walk *translation, int num_bytes,
                           Request::Flags flags)
{
    // translate to physical address using the second stage MMU
    translation->setVirt(
            desc_addr, num_bytes, flags | Request::PT_WALK, requestorId);
    translation->translateTiming(tc);
}

TableWalker::Stage2Walk::Stage2Walk(TableWalker &_parent,
        uint8_t *_data, Event *_event, Addr vaddr, BaseMMU::Mode _mode,
        MMU::ArmTranslationType tran_type)
    : data(_data), numBytes(0), event(_event), parent(_parent),
      oVAddr(vaddr), mode(_mode), tranType(tran_type), fault(NoFault)
{
    req = std::make_shared<Request>();
}

void
TableWalker::Stage2Walk::finish(const Fault &_fault,
                                const RequestPtr &req,
                                ThreadContext *tc, BaseMMU::Mode mode)
{
    fault = _fault;

    // If there was a fault annotate it with the flag saying the foult occured
    // while doing a translation for a stage 1 page table walk.
    if (fault != NoFault) {
        ArmFault *arm_fault = reinterpret_cast<ArmFault *>(fault.get());
        arm_fault->annotate(ArmFault::S1PTW, true);
        arm_fault->annotate(ArmFault::OVA, oVAddr);
    }

    if (_fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
        parent.getTableWalkerPort().sendTimingReq(
            req->getPaddr(), numBytes, data, req->getFlags(),
            tc->getCpuPtr()->clockPeriod(), event);
    } else {
        // We can't do the DMA access as there's been a problem, so tell the
        // event we're done
        event->process();
    }
}

void
TableWalker::Stage2Walk::translateTiming(ThreadContext *tc)
{
    parent.mmu->translateTiming(req, tc, this, mode, tranType, true);
}

TableWalker::TableWalkerStats::TableWalkerStats(statistics::Group *parent)
    : statistics::Group(parent),
    ADD_STAT(walks, statistics::units::Count::get(),
             "Table walker walks requested"),
    ADD_STAT(walksShortDescriptor, statistics::units::Count::get(),
             "Table walker walks initiated with short descriptors"),
    ADD_STAT(walksLongDescriptor, statistics::units::Count::get(),
             "Table walker walks initiated with long descriptors"),
    ADD_STAT(walksShortTerminatedAtLevel, statistics::units::Count::get(),
             "Level at which table walker walks with short descriptors "
             "terminate"),
    ADD_STAT(walksLongTerminatedAtLevel, statistics::units::Count::get(),
             "Level at which table walker walks with long descriptors "
             "terminate"),
    ADD_STAT(squashedBefore, statistics::units::Count::get(),
             "Table walks squashed before starting"),
    ADD_STAT(squashedAfter, statistics::units::Count::get(),
             "Table walks squashed after completion"),
    ADD_STAT(walkWaitTime, statistics::units::Tick::get(),
             "Table walker wait (enqueue to first request) latency"),
    ADD_STAT(walkServiceTime, statistics::units::Tick::get(),
             "Table walker service (enqueue to completion) latency"),
    ADD_STAT(pendingWalks, statistics::units::Tick::get(),
             "Table walker pending requests distribution"),
    ADD_STAT(pageSizes, statistics::units::Count::get(),
             "Table walker page sizes translated"),
    ADD_STAT(requestOrigin, statistics::units::Count::get(),
             "Table walker requests started/completed, data/inst")
{
    walksShortDescriptor
        .flags(statistics::nozero);

    walksLongDescriptor
        .flags(statistics::nozero);

    walksShortTerminatedAtLevel
        .init(2)
        .flags(statistics::nozero);

    walksShortTerminatedAtLevel.subname(0, "Level1");
    walksShortTerminatedAtLevel.subname(1, "Level2");

    walksLongTerminatedAtLevel
        .init(4)
        .flags(statistics::nozero);
    walksLongTerminatedAtLevel.subname(0, "Level0");
    walksLongTerminatedAtLevel.subname(1, "Level1");
    walksLongTerminatedAtLevel.subname(2, "Level2");
    walksLongTerminatedAtLevel.subname(3, "Level3");

    squashedBefore
        .flags(statistics::nozero);

    squashedAfter
        .flags(statistics::nozero);

    walkWaitTime
        .init(16)
        .flags(statistics::pdf | statistics::nozero | statistics::nonan);

    walkServiceTime
        .init(16)
        .flags(statistics::pdf | statistics::nozero | statistics::nonan);

    pendingWalks
        .init(16)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    pageSizes // see DDI 0487A D4-1661
        .init(10)
        .flags(statistics::total | statistics::pdf | statistics::dist |
            statistics::nozero);
    pageSizes.subname(0, "4KiB");
    pageSizes.subname(1, "16KiB");
    pageSizes.subname(2, "64KiB");
    pageSizes.subname(3, "1MiB");
    pageSizes.subname(4, "2MiB");
    pageSizes.subname(5, "16MiB");
    pageSizes.subname(6, "32MiB");
    pageSizes.subname(7, "512MiB");
    pageSizes.subname(8, "1GiB");
    pageSizes.subname(9, "4TiB");

    requestOrigin
        .init(2,2) // Instruction/Data, requests/completed
        .flags(statistics::total);
    requestOrigin.subname(0,"Requested");
    requestOrigin.subname(1,"Completed");
    requestOrigin.ysubname(0,"Data");
    requestOrigin.ysubname(1,"Inst");
}

} // namespace gem5
