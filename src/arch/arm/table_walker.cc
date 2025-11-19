/*
 * Copyright (c) 2010, 2012-2019, 2021-2025 Arm Limited
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
#include "arch/arm/mpam.hh"
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
#include "params/ArmTableWalker.hh"
#include "params/ArmWalkUnit.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace ArmISA;

TableWalker::TableWalker(const Params &p)
    : ClockedObject(p),
      mmu(nullptr),
      walkUnits(p.walk_units),
      walkUnitFunctionalS1(p.walk_unit_func_s1),
      walkUnitFunctionalS2(p.walk_unit_func_s2),
      requestorId(p.sys->getRequestorId(this)),
      port(new Port(*this, requestorId)),
      stats(this)
{
    for (auto walk_unit : walkUnits) {
        walk_unit->setPort(port);
        walk_unit->setTableWalker(this);
    }
    walkUnitFunctionalS1->setPort(port);
    walkUnitFunctionalS1->setTableWalker(this);
    walkUnitFunctionalS2->setPort(port);
    walkUnitFunctionalS2->setTableWalker(this);

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

void
TableWalker::setMmu(MMU *_mmu)
{
    mmu = _mmu;
    for (auto walk_unit : walkUnits) {
        walk_unit->setMmu(mmu);
    }
    walkUnitFunctionalS1->setMmu(mmu);
    walkUnitFunctionalS2->setMmu(mmu);
}

TypeTLB
TableWalker::modeToType(BaseMMU::Mode mode) const
{
    return mode == BaseMMU::Execute ? TypeTLB::instruction : TypeTLB::data;
}

WalkUnit *
TableWalker::getAvailableWalk(BaseMMU::Mode mode, bool stage2,
                              bool functional) const
{
    if (functional) {
        return stage2 ? walkUnitFunctionalS2 : walkUnitFunctionalS1;
    }

    for (auto *walk_unit : walkUnits) {
        if ((walk_unit->type() & modeToType(mode)) &&
            walk_unit->stage2() == stage2 && walk_unit->isAvailable()) {
            return walk_unit;
        }
    }
    return nullptr;
}

WalkUnit *
TableWalker::busyOnSamePage(Addr iaddr, BaseMMU::Mode mode, bool stage2) const
{
    for (auto walk_unit : walkUnits) {
        if ((walk_unit->type() & modeToType(mode)) &&
            walk_unit->stage2() == stage2 &&
            walk_unit->busyOnSamePage(iaddr)) {
            return walk_unit;
        }
    }

    return nullptr;
}

Fault
TableWalker::walk(const RequestPtr &req, ThreadContext *tc, uint16_t asid,
                  vmid_t vmid, BaseMMU::Mode mode, BaseMMU::Translation *trans,
                  bool timing, bool functional, SecurityState ss,
                  PASpace ipaspace, MMU::ArmTranslationType tran_type,
                  bool stage2_req, bool stage2, const TlbEntry *walk_entry)
{
    assert(!(functional && timing));

    for (auto it = inCompletionWalks.rbegin(); it != inCompletionWalks.rend();
         ++it) {

        if ((*it)->vaddr_tainted == req->getVaddr() &&
            (*it)->isStage2 == stage2) {
            ++stats.squashedBefore;
            return std::make_shared<ReExec>();
        }
    }

    if (mode == BaseMMU::Execute) {
        if (!stage2) {
            ++stats.instructionWalksS1;
        } else {
            ++stats.instructionWalksS2;
        }
    } else {
        if (!stage2) {
            ++stats.dataWalksS1;
        } else {
            ++stats.dataWalksS2;
        }
    }

    DPRINTF(PageTableWalker, "creating new instance of WalkerState\n");
    WalkerState *curr_state = new WalkerState();
    curr_state->tableWalker = this;
    curr_state->startTime = curTick();
    curr_state->tc = tc;
    curr_state->isStage2 = stage2;
    curr_state->el =
        MMU::tranTypeEL(tc->readMiscReg(MISCREG_CPSR),
                        tc->readMiscReg(MISCREG_SCR_EL3), tran_type);

    if (stage2) {
        curr_state->regime = TranslationRegime::EL10;
        curr_state->aarch64 = ELIs64(tc, EL2);
        curr_state->ipaSpace = ipaspace;
    } else {
        curr_state->regime = translationRegime(tc, curr_state->el);
        curr_state->aarch64 = ELIs64(tc, translationEl(curr_state->regime));
    }
    curr_state->transState = trans;
    curr_state->req = req;
    if (walk_entry) {
        curr_state->walkEntry = *walk_entry;
    } else {
        curr_state->walkEntry = TlbEntry();
    }
    curr_state->fault = NoFault;
    curr_state->asid = asid;
    curr_state->vmid = vmid;
    curr_state->timing = timing;
    curr_state->functional = functional;
    curr_state->mode = mode;
    curr_state->tranType = tran_type;
    curr_state->ss = ss;
    curr_state->secureLookup = curr_state->ss == SecurityState::Secure;
    curr_state->physAddrRange = _physAddrRange;

    /** @todo These should be cached or grabbed from cached copies in
     the TLB, all these miscreg reads are expensive */
    curr_state->vaddr_tainted = curr_state->req->getVaddr();
    if (curr_state->aarch64) {
        curr_state->vaddr = purifyTaggedAddr(
            curr_state->vaddr_tainted, curr_state->tc, curr_state->el,
            curr_state->mode == BaseMMU::Execute);
    } else {
        curr_state->vaddr = curr_state->vaddr_tainted;
    }

    if (curr_state->aarch64) {
        curr_state->hcr = curr_state->tc->readMiscReg(MISCREG_HCR_EL2);
        if (stage2) {
            curr_state->sctlr = curr_state->tc->readMiscReg(MISCREG_SCTLR_EL1);
            if (curr_state->ss == SecurityState::Secure &&
                curr_state->ipaSpace == PASpace::Secure) {
                curr_state->vtcr =
                    curr_state->tc->readMiscReg(MISCREG_VSTCR_EL2);
            } else {
                curr_state->vtcr =
                    curr_state->tc->readMiscReg(MISCREG_VTCR_EL2);
            }
        } else {
            switch (curr_state->regime) {
                case TranslationRegime::EL10:
                    curr_state->sctlr =
                        curr_state->tc->readMiscReg(MISCREG_SCTLR_EL1);
                    curr_state->tcr =
                        curr_state->tc->readMiscReg(MISCREG_TCR_EL1);
                    break;
                case TranslationRegime::EL20:
                case TranslationRegime::EL2:
                    assert(mmu->release()->has(ArmExtension::VIRTUALIZATION));
                    curr_state->sctlr =
                        curr_state->tc->readMiscReg(MISCREG_SCTLR_EL2);
                    curr_state->tcr =
                        curr_state->tc->readMiscReg(MISCREG_TCR_EL2);
                    break;
                case TranslationRegime::EL3:
                    assert(mmu->release()->has(ArmExtension::SECURITY));
                    curr_state->sctlr =
                        curr_state->tc->readMiscReg(MISCREG_SCTLR_EL3);
                    curr_state->tcr =
                        curr_state->tc->readMiscReg(MISCREG_TCR_EL3);
                    break;
                default:
                    panic("Invalid translation regime");
                    break;
            }
        }
    } else {
        curr_state->sctlr = curr_state->tc->readMiscReg(
            snsBankedIndex(MISCREG_SCTLR, curr_state->tc,
                           curr_state->ss == SecurityState::NonSecure));
        curr_state->ttbcr = curr_state->tc->readMiscReg(
            snsBankedIndex(MISCREG_TTBCR, curr_state->tc,
                           curr_state->ss == SecurityState::NonSecure));
        curr_state->htcr = curr_state->tc->readMiscReg(MISCREG_HTCR);
        curr_state->hcr = curr_state->tc->readMiscReg(MISCREG_HCR);
        curr_state->vtcr = curr_state->tc->readMiscReg(MISCREG_VTCR);
    }
    curr_state->isFetch = (curr_state->mode == BaseMMU::Execute);
    curr_state->isWrite = (curr_state->mode == BaseMMU::Write);

    stats.requestOrigin[REQUESTED][curr_state->isFetch]++;

    curr_state->stage2Req = stage2_req && !stage2;

    bool hyp = curr_state->el == EL2;
    bool long_desc_format = curr_state->aarch64 || hyp ||
                            curr_state->isStage2 ||
                            longDescFormatInUse(curr_state->tc);

    if (long_desc_format) {
        // Helper variables used for hierarchical permissions
        curr_state->longDescData = WalkerState::LongDescData();
        curr_state->longDescData->rwTable = true;
        curr_state->longDescData->userTable = true;
        curr_state->longDescData->xnTable = false;
        curr_state->longDescData->pxnTable = false;
        ++stats.walksLongDescriptor;
    } else {
        curr_state->longDescData = std::nullopt;
        ++stats.walksShortDescriptor;
    }

    bool wait_for_other =
        curr_state->timing && busyOnSamePage(curr_state->vaddr, mode, stage2);

    auto walk_unit = getAvailableWalk(mode, stage2, functional);

    if (wait_for_other || !walk_unit) {
        // No available walk unit at the moment. Stall the table walker
        // This can only happen in timing mode
        assert(curr_state->timing);
        pendingQueue.push_back(curr_state);

        pendingChange();

        return NoFault;
    } else {
        // The walk unit is now busy servicing this walk request
        assert(walk_unit->isAvailable());
        walk_unit->isAvailable(false);

        Fault fault = walk_unit->walk(curr_state);

        // The translation walk has finished in two cases:
        // a) If a fault has been generated
        // b) If we are in atomic/functional mode (not timing)
        if (fault || !curr_state->timing) {
            // Mark the walk unit as available
            nextWalk(walk_unit);

            // Delete the walker state
            delete curr_state;
        } else {
            // Either we are using the long descriptor, which means we
            // need to extract the queue index from longDesc, or we are
            // using the short. In the latter we always start at L1
            LookupLevel curr_lookup_level =
                long_desc_format ? curr_state->longDesc.lookupLevel
                                 : LookupLevel::L1;

            walk_unit->stashCurrState(curr_state, curr_lookup_level);
        }

        return fault;
    }
}

void
WalkUnit::processWalkWrapper(WalkerState *curr_state)
{
    // Check if a previous walk filled this request already
    // @TODO Should this always be the TLB or should we look in the stage2 TLB?
    TlbEntry *te =
        mmu->lookup(curr_state->vaddr, curr_state->asid, curr_state->vmid,
                    curr_state->ss, true, false, curr_state->regime,
                    curr_state->isStage2, curr_state->mode);

    // Check if we still need to have a walk for this request. If the
    // requesting instruction has been squashed, or a previous walk has
    // filled the TLB with a match, we just want to get rid of the walk.
    // The latter could happen when there are multiple outstanding misses
    // to a single page and a previous request has been successfully
    // translated.
    if (!curr_state->transState->squashed() &&
        !mmu->isCompleteTranslation(te)) {

        Fault fault = walk(curr_state);
        if (fault != NoFault) {
            curr_state->transState->finish(fault, curr_state->req,
                                           curr_state->tc, curr_state->mode);

            // Mark the walk unit as available
            parent->nextWalk(this);

            delete curr_state;
        } else {
            bool long_desc_format =
                curr_state->aarch64 || curr_state->el == EL2 ||
                curr_state->isStage2 || longDescFormatInUse(curr_state->tc);

            // Either we are using the long descriptor, which means we
            // need to extract the queue index from longDesc, or we are
            // using the short. In the latter we always start at L1
            LookupLevel curr_lookup_level =
                long_desc_format ? curr_state->longDesc.lookupLevel
                                 : LookupLevel::L1;

            stashCurrState(curr_state, curr_lookup_level);
        }
    } else {
        parent->squashWalk(curr_state);
        parent->nextWalk(this);
    }
}

void
TableWalker::squashWalk(WalkerState *curr_state)
{
    DPRINTF(PageTableWalker, "Squashing table walk for address %#x\n",
            curr_state->vaddr_tainted);

    if (curr_state->transState->squashed()) {
        // finish the translation which will delete the translation object
        curr_state->transState->finish(
            std::make_shared<UnimpFault>("Squashed Inst"), curr_state->req,
            curr_state->tc, curr_state->mode);
    } else {
        // translate the request now that we know it will work
        stats.walkServiceTime.sample(curTick() - curr_state->startTime);
        mmu->translateTiming(curr_state->req, curr_state->tc,
                             curr_state->transState, curr_state->mode,
                             curr_state->tranType, curr_state->isStage2);
    }

    // delete the current request
    delete curr_state;
    stats.squashedBefore++;
}

void
TableWalker::nextWalk(WalkUnit *walk_unit)
{
    WalkerState *next_walk = nullptr;
    // With functional walk units we just mark the walker
    // as available without trying to schedule pending
    // walks (which are non functional by definition)
    if (!walk_unit->functional()) {
        for (auto candidate : pendingQueue) {
            // Check if the walk unit is able to service the current
            // pending walk request
            if (walk_unit->type() & modeToType(candidate->mode) &&
                walk_unit->stage2() == candidate->isStage2) {

                next_walk = candidate;
                walk_unit->scheduleWalk(next_walk, clockEdge(Cycles(1)));
                break;
            }
        }
    }

    if (!next_walk) {
        // Add the walk unit to the pool of available walks
        walk_unit->isAvailable(true);

        completeDrain();
    } else {
        // We found a match between the walk unit and a pending walk
        // let's remove the walk from the list so that it does not
        // get executed by a different walk unit
        pendingQueue.remove(next_walk);

        pendingChange();
    }
}

void
TableWalker::completeDrain()
{
    auto drain_completed = [](WalkUnit *wu) { return wu->completeDrain(); };

    if (drainState() == DrainState::Draining &&
        std::all_of(walkUnits.begin(), walkUnits.end(), drain_completed) &&
        pendingQueue.empty()) {

        DPRINTF(Drain, "TableWalker done draining, processing drain event\n");
        signalDrainDone();
    }
}

DrainState
TableWalker::drain()
{
    if (pendingQueue.size()) {
        DPRINTF(Drain, "TableWalker not drained\n");
        return DrainState::Draining;
    } else {
        DPRINTF(Drain, "TableWalker free, no need to drain\n");
        return DrainState::Drained;
    }
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

TableWalker::Port::Port(TableWalker &_walker, RequestorID id)
    : QueuedRequestPort(_walker.name() + ".port", reqQueue, snoopRespQueue),
      owner(_walker),
      reqQueue(_walker, *this),
      snoopRespQueue(_walker, *this),
      _requestorId(id)
{
}

PacketPtr
TableWalker::Port::createPacket(
    const RequestPtr &req,
    uint8_t *data, Tick delay,
    Event *event)
{
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
    const RequestPtr &req, uint8_t *data)
{
    auto pkt = createPacket(req, data, 0, nullptr);

    sendFunctional(pkt);

    handleRespPacket(pkt);
}

void
TableWalker::Port::sendAtomicReq(
    const RequestPtr &req,
    uint8_t *data, Tick delay)
{
    auto pkt = createPacket(req, data, delay, nullptr);

    Tick lat = sendAtomic(pkt);

    handleRespPacket(pkt, lat);
}

void
TableWalker::Port::sendTimingReq(
    const RequestPtr &req,
    uint8_t *data, Tick delay,
    Event *event)
{
    auto pkt = createPacket(req, data, delay, event);

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
TableWalker::setTestInterface(TlbTestInterface *test)
{
    for (auto walk_unit : walkUnits) {
        walk_unit->setTestInterface(test);
    }

    walkUnitFunctionalS1->setTestInterface(test);
    walkUnitFunctionalS2->setTestInterface(test);
}

/* this method keeps track of the table walker queue's residency, so
 * needs to be called whenever requests start and complete. */
void
TableWalker::pendingChange()
{
    if (auto num_pending = pendingQueue.size(); num_pending != pendingReqs) {
        Tick now = curTick();
        stats.pendingWalks.sample(pendingReqs, now - pendingChangeTick);
        pendingReqs = num_pending;
        pendingChangeTick = now;
    }
}

WalkUnit::WalkUnit(const Params &p)
    : ClockedObject(p),
      isStage2(p.is_stage2),
      tlb(NULL),
      walkType(p.walk_type),
      isFunctional(p.functional),
      available(true),
      doProcessEvent(this, name()),
      doL1DescEvent([this] { doL1DescriptorWrapper(); }, name()),
      doL2DescEvent([this] { doL2DescriptorWrapper(); }, name()),
      doL0LongDescEvent([this] { doL0LongDescriptorWrapper(); }, name()),
      doL1LongDescEvent([this] { doL1LongDescriptorWrapper(); }, name()),
      doL2LongDescEvent([this] { doL2LongDescriptorWrapper(); }, name()),
      doL3LongDescEvent([this] { doL3LongDescriptorWrapper(); }, name()),
      LongDescEventByLevel{&doL0LongDescEvent, &doL1LongDescEvent,
                           &doL2LongDescEvent, &doL3LongDescEvent},
      test(nullptr)
{}

WalkUnit::~WalkUnit()
{
    ;
}

TableWalker::WalkerState::WalkerState()
    : tc(nullptr),
      aarch64(false),
      regime(TranslationRegime::EL10),
      physAddrRange(0),
      req(nullptr),
      asid(0),
      vmid(0),
      transState(nullptr),
      vaddr(0),
      vaddr_tainted(0),
      sctlr(0),
      scr(0),
      cpsr(0),
      tcr(0),
      htcr(0),
      hcr(0),
      vtcr(0),
      isWrite(false),
      isFetch(false),
      ss(SecurityState::NonSecure),
      isUncacheable(false),
      longDescData(std::nullopt),
      hpd(false),
      sh(0),
      irgn(0),
      orgn(0),
      stage2Req(false),
      stage2Tran(nullptr),
      timing(false),
      functional(false),
      mode(BaseMMU::Read),
      tranType(MMU::NormalTran),
      l2Desc(l1Desc),
      delayed(false),
      tableWalker(nullptr)
{}

bool
WalkUnit::completeDrain()
{
    if (!(drainState() == DrainState::Draining)) {
        return true;
    } else if (stateQueues[LookupLevel::L0].empty() &&
               stateQueues[LookupLevel::L1].empty() &&
               stateQueues[LookupLevel::L2].empty() &&
               stateQueues[LookupLevel::L3].empty()) {

        DPRINTF(Drain, "WalkUnit done draining, processing drain event\n");
        signalDrainDone();
        return true;
    } else {
        return false;
    }
}

DrainState
WalkUnit::drain()
{
    bool state_queues_not_empty = false;

    for (int i = 0; i < LookupLevel::Num_ArmLookupLevel; ++i) {
        if (!stateQueues[i].empty()) {
            state_queues_not_empty = true;
            break;
        }
    }

    if (state_queues_not_empty) {
        DPRINTF(Drain, "WalkUnit not drained\n");
        return DrainState::Draining;
    } else {
        DPRINTF(Drain, "WalkUnit free, no need to drain\n");
        return DrainState::Drained;
    }
}

bool
WalkUnit::uncacheableWalk(WalkerState *curr_state) const
{
    bool disable_cacheability =
        isStage2 ? curr_state->hcr.cd : curr_state->sctlr.c == 0;
    return disable_cacheability || curr_state->isUncacheable;
}

void
WalkUnit::setMmu(MMU *_mmu)
{
    mmu = _mmu;
}

Fault
WalkUnit::walk(WalkerState *state)
{
    Fault fault = NoFault;

    if (state->aarch64) {
        fault = processWalkAArch64(state);
    } else if (longDescFormatInUse(state->tc) || state->el == EL2 ||
               isStage2) {
        fault = processWalkLPAE(state);
    } else {
        fault = processWalk(state);
    }

    return fault;
}

Fault
WalkUnit::processWalk(WalkerState *curr_state)
{
    Addr ttbr = 0;

    // For short descriptors, translation configs are held in
    // TTBR1.
    RegVal ttbr1 = curr_state->tc->readMiscReg(
        snsBankedIndex(MISCREG_TTBR1, curr_state->tc,
                       curr_state->ss == SecurityState::NonSecure));

    const auto irgn0_mask = 0x1;
    const auto irgn1_mask = 0x40;
    curr_state->isUncacheable = (ttbr1 & (irgn0_mask | irgn1_mask)) == 0;

    // If translation isn't enabled, we shouldn't be here
    assert(curr_state->sctlr.m || isStage2);
    const bool is_atomic = curr_state->req->isAtomic();
    const bool have_security = mmu->release()->has(ArmExtension::SECURITY);

    DPRINTF(PageTableWalker,
            "Beginning table walk for address %#x, TTBCR: %#x, bits:%#x\n",
            curr_state->vaddr_tainted, curr_state->ttbcr,
            mbits(curr_state->vaddr, 31, 32 - curr_state->ttbcr.n));

    parent->stats.walkWaitTime.sample(curTick() - curr_state->startTime);

    if (curr_state->ttbcr.n == 0 ||
        !mbits(curr_state->vaddr, 31, 32 - curr_state->ttbcr.n)) {
        DPRINTF(PageTableWalker, " - Selecting TTBR0\n");
        // Check if table walk is allowed when Security Extensions are enabled
        if (have_security && curr_state->ttbcr.pd0) {
            if (curr_state->isFetch) {
                return std::make_shared<PrefetchAbort>(
                    curr_state->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            } else {
                return std::make_shared<DataAbort>(
                    curr_state->vaddr_tainted, DomainType::NoAccess,
                    is_atomic ? false : curr_state->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            }
        }
        ttbr = curr_state->tc->readMiscReg(
            snsBankedIndex(MISCREG_TTBR0, curr_state->tc,
                           curr_state->ss == SecurityState::NonSecure));
    } else {
        DPRINTF(PageTableWalker, " - Selecting TTBR1\n");
        // Check if table walk is allowed when Security Extensions are enabled
        if (have_security && curr_state->ttbcr.pd1) {
            if (curr_state->isFetch) {
                return std::make_shared<PrefetchAbort>(
                    curr_state->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            } else {
                return std::make_shared<DataAbort>(
                    curr_state->vaddr_tainted, DomainType::NoAccess,
                    is_atomic ? false : curr_state->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            }
        }
        ttbr = ttbr1;
        curr_state->ttbcr.n = 0;
    }

    Addr l1desc_addr =
        mbits(ttbr, 31, 14 - curr_state->ttbcr.n) |
        (bits(curr_state->vaddr, 31 - curr_state->ttbcr.n, 20) << 2);
    DPRINTF(PageTableWalker, " - Descriptor at address %#x (%s)\n",
            l1desc_addr, curr_state->ss == SecurityState::Secure ? "s" : "ns");

    Request::Flags flag = Request::PT_WALK;
    if (uncacheableWalk(curr_state)) {
        flag.set(Request::UNCACHEABLE);
    }

    if (curr_state->secureLookup) {
        flag.set(Request::SECURE);
    }

    fetchDescriptor(l1desc_addr, curr_state->l1Desc, sizeof(uint32_t), flag,
                    LookupLevel::L1, &doL1DescEvent, &WalkUnit::doL1Descriptor,
                    curr_state);

    return curr_state->fault;
}

Fault
WalkUnit::processWalkLPAE(WalkerState *curr_state)
{
    Addr ttbr, ttbr0_max, ttbr1_min, desc_addr;
    int tsz, n;
    LookupLevel start_lookup_level = LookupLevel::L1;

    DPRINTF(PageTableWalker,
            "Beginning table walk for address %#x, TTBCR: %#x\n",
            curr_state->vaddr_tainted, curr_state->ttbcr);

    parent->stats.walkWaitTime.sample(curTick() - curr_state->startTime);

    Request::Flags flag = Request::PT_WALK;
    if (curr_state->secureLookup) {
        flag.set(Request::SECURE);
    }

    // work out which base address register to use, if in hyp mode we always
    // use HTTBR
    if (isStage2) {
        DPRINTF(PageTableWalker, " - Selecting VTTBR (long-desc.)\n");
        ttbr = curr_state->tc->readMiscReg(MISCREG_VTTBR);
        tsz = sext<4>(curr_state->vtcr.t0sz);
        start_lookup_level =
            curr_state->vtcr.sl0 ? LookupLevel::L1 : LookupLevel::L2;
        curr_state->isUncacheable = curr_state->vtcr.irgn0 == 0;
    } else if (curr_state->el == EL2) {
        DPRINTF(PageTableWalker, " - Selecting HTTBR (long-desc.)\n");
        ttbr = curr_state->tc->readMiscReg(MISCREG_HTTBR);
        tsz = curr_state->htcr.t0sz;
        curr_state->isUncacheable = curr_state->htcr.irgn0 == 0;
    } else {
        assert(longDescFormatInUse(curr_state->tc));

        // Determine boundaries of TTBR0/1 regions
        if (curr_state->ttbcr.t0sz) {
            ttbr0_max = (1ULL << (32 - curr_state->ttbcr.t0sz)) - 1;
        } else if (curr_state->ttbcr.t1sz) {
            ttbr0_max =
                (1ULL << 32) - (1ULL << (32 - curr_state->ttbcr.t1sz)) - 1;
        } else {
            ttbr0_max = (1ULL << 32) - 1;
        }
        if (curr_state->ttbcr.t1sz) {
            ttbr1_min = (1ULL << 32) - (1ULL << (32 - curr_state->ttbcr.t1sz));
        } else {
            ttbr1_min = (1ULL << (32 - curr_state->ttbcr.t0sz));
        }

        const bool is_atomic = curr_state->req->isAtomic();

        // The following code snippet selects the appropriate translation table base
        // address (TTBR0 or TTBR1) and the appropriate starting lookup level
        // depending on the address range supported by the translation table (ARM
        // ARM issue C B3.6.4)
        if (curr_state->vaddr <= ttbr0_max) {
            DPRINTF(PageTableWalker, " - Selecting TTBR0 (long-desc.)\n");
            // Check if table walk is allowed
            if (curr_state->ttbcr.epd0) {
                if (curr_state->isFetch) {
                    return std::make_shared<PrefetchAbort>(
                        curr_state->vaddr_tainted,
                        ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                        TranMethod::LpaeTran);
                } else {
                    return std::make_shared<DataAbort>(
                        curr_state->vaddr_tainted, DomainType::NoAccess,
                        is_atomic ? false : curr_state->isWrite,
                        ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                        TranMethod::LpaeTran);
                }
            }
            ttbr = curr_state->tc->readMiscReg(
                snsBankedIndex(MISCREG_TTBR0, curr_state->tc,
                               curr_state->ss == SecurityState::NonSecure));
            tsz = curr_state->ttbcr.t0sz;
            curr_state->isUncacheable = curr_state->ttbcr.irgn0 == 0;
            if (ttbr0_max < (1ULL << 30))  // Upper limit < 1 GiB
                start_lookup_level = LookupLevel::L2;
        } else if (curr_state->vaddr >= ttbr1_min) {
            DPRINTF(PageTableWalker, " - Selecting TTBR1 (long-desc.)\n");
            // Check if table walk is allowed
            if (curr_state->ttbcr.epd1) {
                if (curr_state->isFetch) {
                    return std::make_shared<PrefetchAbort>(
                        curr_state->vaddr_tainted,
                        ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                        TranMethod::LpaeTran);
                } else {
                    return std::make_shared<DataAbort>(
                        curr_state->vaddr_tainted, DomainType::NoAccess,
                        is_atomic ? false : curr_state->isWrite,
                        ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                        TranMethod::LpaeTran);
                }
            }
            ttbr = curr_state->tc->readMiscReg(
                snsBankedIndex(MISCREG_TTBR1, curr_state->tc,
                               curr_state->ss == SecurityState::NonSecure));
            tsz = curr_state->ttbcr.t1sz;
            curr_state->isUncacheable = curr_state->ttbcr.irgn1 == 0;
            // Lower limit >= 3 GiB
            if (ttbr1_min >= (1ULL << 31) + (1ULL << 30))
                start_lookup_level = LookupLevel::L2;
        } else {
            // Out of boundaries -> translation fault
            if (curr_state->isFetch) {
                return std::make_shared<PrefetchAbort>(
                    curr_state->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::LpaeTran);
            } else {
                return std::make_shared<DataAbort>(
                    curr_state->vaddr_tainted, DomainType::NoAccess,
                    is_atomic ? false : curr_state->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::LpaeTran);
            }
        }
    }

    // Perform lookup (ARM ARM issue C B3.6.6)
    if (start_lookup_level == LookupLevel::L1) {
        n = 5 - tsz;
        desc_addr =
            mbits(ttbr, 39, n) | (bits(curr_state->vaddr, n + 26, 30) << 3);
        DPRINTF(PageTableWalker,
                " - Descriptor at address %#x (%s) (long-desc.)\n", desc_addr,
                curr_state->ss == SecurityState::Secure ? "s" : "ns");
    } else {
        // Skip first-level lookup
        n = (tsz >= 2 ? 14 - tsz : 12);
        desc_addr =
            mbits(ttbr, 39, n) | (bits(curr_state->vaddr, n + 17, 21) << 3);
        DPRINTF(PageTableWalker,
                " - Descriptor at address %#x (%s) (long-desc.)\n", desc_addr,
                curr_state->ss == SecurityState::Secure ? "s" : "ns");
    }

    if (uncacheableWalk(curr_state)) {
        flag.set(Request::UNCACHEABLE);
    }

    curr_state->longDesc.lookupLevel = start_lookup_level;
    curr_state->longDesc.aarch64 = false;
    curr_state->longDesc.grainSize = Grain4KB;
    curr_state->longDesc.isStage2 = isStage2;

    fetchDescriptor(desc_addr, curr_state->longDesc, sizeof(uint64_t), flag,
                    start_lookup_level,
                    LongDescEventByLevel[start_lookup_level],
                    &WalkUnit::doLongDescriptor, curr_state);

    return curr_state->fault;
}

Addr
WalkUnit::s1MinTxSz(WalkerState *curr_state, GrainSize tg) const
{
    // The effective maximum input size is 48 if ARMv8.2-LVA is not
    // supported or if the translation granule that is in use is 4KB or
    // 16KB in size. When ARMv8.2-LVA is supported, for the 64KB
    // translation granule size only, the effective minimum value of
    // 52.
    if (HaveExt(curr_state->tc, ArmExtension::FEAT_LVA) && tg == Grain64KB) {
        return 12;
    } else {
        return 16;
    }
}

Addr
WalkUnit::maxTxSz(WalkerState *curr_state, GrainSize tg) const
{
    if (HaveExt(curr_state->tc, ArmExtension::FEAT_TTST)) {
        switch (tg) {
          case Grain4KB: return 48;
          case Grain16KB: return 48;
          case Grain64KB: return 47;
          default:
            // If the value is programmed to either a reserved value or a size
            // that has not been implemented, then the hardware will treat the
            // field as if it has been programmed to an IMPLEMENTATION DEFINED
            // choice
            warn_once("Invalid grain size\n");
            return 48;
        }
    }
    return 39;
}

bool
WalkUnit::s1TxSzFault(WalkerState *curr_state, GrainSize tg, int tsz) const
{
    Addr min_txsz = s1MinTxSz(curr_state, tg);
    Addr max_txsz = maxTxSz(curr_state, tg);

    return tsz > max_txsz || tsz < min_txsz;
}

bool
WalkUnit::checkVAOutOfRange(WalkerState *curr_state, int top_bit, int tsz,
                            bool low_range)
{
    return low_range ? bits(curr_state->vaddr, top_bit, tsz) != 0x0
                     : bits(curr_state->vaddr, top_bit, tsz) !=
                           mask(top_bit - tsz + 1);
}

bool
WalkUnit::checkAddrSizeFaultAArch64(Addr addr, int pa_range)
{
    return (pa_range != parent->physAddrRange() &&
            bits(addr, parent->physAddrRange() - 1, pa_range));
}

Fault
WalkUnit::processWalkAArch64(WalkerState *curr_state)
{
    assert(curr_state->aarch64);

    DPRINTF(PageTableWalker,
            "Beginning table walk for address %#llx, TCR: %#llx\n",
            curr_state->vaddr_tainted, curr_state->tcr);

    parent->stats.walkWaitTime.sample(curTick() - curr_state->startTime);

    // Determine TTBR, table size, granule size and phys. address range
    Addr ttbr = 0;
    int tsz = 0, ps = 0;
    GrainSize tg = Grain4KB; // grain size computed from tg* field
    bool fault = false;

    int top_bit = computeAddrTop(curr_state->tc, bits(curr_state->vaddr, 55),
                                 curr_state->mode == BaseMMU::Execute,
                                 curr_state->tcr, curr_state->el);

    bool vaddr_fault = false;
    switch (curr_state->regime) {
        case TranslationRegime::EL10:
            if (isStage2) {
                if (curr_state->ss == SecurityState::Secure &&
                    curr_state->ipaSpace == PASpace::Secure) {
                    // Secure EL1&0 Secure IPA
                    DPRINTF(PageTableWalker,
                            " - Selecting VSTTBR_EL2 (AArch64 stage 2)\n");
                    ttbr = curr_state->tc->readMiscReg(MISCREG_VSTTBR_EL2);
                    curr_state->secureLookup = !curr_state->vtcr.sw;
                } else {
                    // Secure EL1&0 NonSecure IPA or NonSecure EL1&0
                    DPRINTF(PageTableWalker,
                            " - Selecting VTTBR_EL2 (AArch64 stage 2)\n");
                    ttbr = curr_state->tc->readMiscReg(MISCREG_VTTBR_EL2);
                    curr_state->secureLookup =
                        curr_state->ss == SecurityState::Secure
                            ? !curr_state->vtcr.nsw
                            :      // Secure EL1&0 NonSecure IPA
                            false; // NonSecure EL1&0
                }
                tsz = 64 - curr_state->vtcr.t0sz64;
                tg = GrainMap_tg0[curr_state->vtcr.tg0];

                ps = curr_state->vtcr.ps;
                curr_state->sh = curr_state->vtcr.sh0;
                curr_state->irgn = curr_state->vtcr.irgn0;
                curr_state->orgn = curr_state->vtcr.orgn0;
            } else {
                switch (bits(curr_state->vaddr, top_bit)) {
                    case 0:
                        DPRINTF(PageTableWalker,
                                " - Selecting TTBR0_EL1 (AArch64)\n");
                        ttbr = curr_state->tc->readMiscReg(MISCREG_TTBR0_EL1);
                        tsz = 64 - curr_state->tcr.t0sz;
                        tg = GrainMap_tg0[curr_state->tcr.tg0];
                        curr_state->hpd = curr_state->tcr.hpd0;
                        curr_state->sh = curr_state->tcr.sh0;
                        curr_state->irgn = curr_state->tcr.irgn0;
                        curr_state->orgn = curr_state->tcr.orgn0;
                        vaddr_fault =
                            s1TxSzFault(curr_state, tg,
                                        curr_state->tcr.t0sz) ||
                            checkVAOutOfRange(curr_state, top_bit, tsz, true);

                        if (vaddr_fault || curr_state->tcr.epd0) {
                            fault = true;
                        }
                        break;
                    case 0x1:
                        DPRINTF(PageTableWalker,
                                " - Selecting TTBR1_EL1 (AArch64)\n");
                        ttbr = curr_state->tc->readMiscReg(MISCREG_TTBR1_EL1);
                        tsz = 64 - curr_state->tcr.t1sz;
                        tg = GrainMap_tg1[curr_state->tcr.tg1];
                        curr_state->hpd = curr_state->tcr.hpd1;
                        curr_state->sh = curr_state->tcr.sh1;
                        curr_state->irgn = curr_state->tcr.irgn1;
                        curr_state->orgn = curr_state->tcr.orgn1;
                        vaddr_fault =
                            s1TxSzFault(curr_state, tg,
                                        curr_state->tcr.t1sz) ||
                            checkVAOutOfRange(curr_state, top_bit, tsz, false);

                        if (vaddr_fault || curr_state->tcr.epd1) {
                            fault = true;
                        }
                        break;
                    default:
                        // top two bytes must be all 0s or all 1s, else invalid
                        // addr
                        fault = true;
                }
                ps = curr_state->tcr.ips;
            }
            break;
        case TranslationRegime::EL2:
        case TranslationRegime::EL20:
            switch (bits(curr_state->vaddr, top_bit)) {
                case 0:
                    DPRINTF(PageTableWalker,
                            " - Selecting TTBR0_EL2 (AArch64)\n");
                    ttbr = curr_state->tc->readMiscReg(MISCREG_TTBR0_EL2);
                    tsz = 64 - curr_state->tcr.t0sz;
                    tg = GrainMap_tg0[curr_state->tcr.tg0];
                    curr_state->hpd = curr_state->hcr.e2h
                                          ? curr_state->tcr.hpd0
                                          : curr_state->tcr.hpd;
                    curr_state->sh = curr_state->tcr.sh0;
                    curr_state->irgn = curr_state->tcr.irgn0;
                    curr_state->orgn = curr_state->tcr.orgn0;
                    vaddr_fault =
                        s1TxSzFault(curr_state, tg, curr_state->tcr.t0sz) ||
                        checkVAOutOfRange(curr_state, top_bit, tsz, true);

                    if (vaddr_fault ||
                        (curr_state->hcr.e2h && curr_state->tcr.epd0)) {
                        fault = true;
                    }
                    break;

                case 0x1:
                    DPRINTF(PageTableWalker,
                            " - Selecting TTBR1_EL2 (AArch64)\n");
                    ttbr = curr_state->tc->readMiscReg(MISCREG_TTBR1_EL2);
                    tsz = 64 - curr_state->tcr.t1sz;
                    tg = GrainMap_tg1[curr_state->tcr.tg1];
                    curr_state->hpd = curr_state->tcr.hpd1;
                    curr_state->sh = curr_state->tcr.sh1;
                    curr_state->irgn = curr_state->tcr.irgn1;
                    curr_state->orgn = curr_state->tcr.orgn1;
                    vaddr_fault =
                        s1TxSzFault(curr_state, tg, curr_state->tcr.t1sz) ||
                        checkVAOutOfRange(curr_state, top_bit, tsz, false);

                    if (vaddr_fault || !curr_state->hcr.e2h ||
                        curr_state->tcr.epd1) {
                        fault = true;
                    }
                    break;

                default:
                    // invalid addr if top two bytes are not all 0s
                    fault = true;
            }
            ps =
                curr_state->hcr.e2h ? curr_state->tcr.ips : curr_state->tcr.ps;
            break;
        case TranslationRegime::EL3:
            switch (bits(curr_state->vaddr, top_bit)) {
                case 0:
                    DPRINTF(PageTableWalker,
                            " - Selecting TTBR0_EL3 (AArch64)\n");
                    ttbr = curr_state->tc->readMiscReg(MISCREG_TTBR0_EL3);
                    tsz = 64 - curr_state->tcr.t0sz;
                    tg = GrainMap_tg0[curr_state->tcr.tg0];
                    curr_state->hpd = curr_state->tcr.hpd;
                    curr_state->sh = curr_state->tcr.sh0;
                    curr_state->irgn = curr_state->tcr.irgn0;
                    curr_state->orgn = curr_state->tcr.orgn0;
                    vaddr_fault =
                        s1TxSzFault(curr_state, tg, curr_state->tcr.t0sz) ||
                        checkVAOutOfRange(curr_state, top_bit, tsz, true);

                    if (vaddr_fault) {
                        fault = true;
                    }
                    break;
                default:
                    // invalid addr if top two bytes are not all 0s
                    fault = true;
            }
            ps = curr_state->tcr.ps;
            break;
    }

    curr_state->isUncacheable = curr_state->irgn == 0 || curr_state->orgn == 0;

    const bool is_atomic = curr_state->req->isAtomic();

    if (fault) {
        if (curr_state->isFetch) {
            return std::make_shared<PrefetchAbort>(
                curr_state->vaddr_tainted,
                ArmFault::TranslationLL + LookupLevel::L0, isStage2,
                TranMethod::LpaeTran);
        } else {
            return std::make_shared<DataAbort>(
                curr_state->vaddr_tainted, DomainType::NoAccess,
                is_atomic ? false : curr_state->isWrite,
                ArmFault::TranslationLL + LookupLevel::L0, isStage2,
                TranMethod::LpaeTran);
        }
    }

    if (tg == ReservedGrain) {
        warn_once("Reserved granule size requested; gem5's IMPLEMENTATION "
                  "DEFINED behavior takes this to mean 4KB granules\n");
        tg = Grain4KB;
    }

    // Clamp to lower limit
    int pa_range = decodePhysAddrRange64(ps);
    if (pa_range > parent->physAddrRange()) {
        curr_state->physAddrRange = parent->physAddrRange();
    } else {
        curr_state->physAddrRange = pa_range;
    }

    auto [table_addr, desc_addr, start_lookup_level] =
        walkAddresses(curr_state, ttbr, tg, tsz, pa_range);

    // Determine physical address size and raise an Address Size Fault if
    // necessary
    if (checkAddrSizeFaultAArch64(table_addr, curr_state->physAddrRange)) {
        DPRINTF(PageTableWalker, "Address size fault before any lookup\n");
        if (curr_state->isFetch) {
            return std::make_shared<PrefetchAbort>(
                curr_state->vaddr_tainted,
                ArmFault::AddressSizeLL + start_lookup_level, isStage2,
                TranMethod::LpaeTran);
        } else {
            return std::make_shared<DataAbort>(
                curr_state->vaddr_tainted, DomainType::NoAccess,
                is_atomic ? false : curr_state->isWrite,
                ArmFault::AddressSizeLL + start_lookup_level, isStage2,
                TranMethod::LpaeTran);
        }
    }

    Request::Flags flag = Request::PT_WALK;
    if (uncacheableWalk(curr_state)) {
        flag.set(Request::UNCACHEABLE);
    }

    if (curr_state->secureLookup) {
        flag.set(Request::SECURE);
    }

    curr_state->longDesc.lookupLevel = start_lookup_level;
    curr_state->longDesc.aarch64 = true;
    curr_state->longDesc.grainSize = tg;
    curr_state->longDesc.physAddrRange = parent->physAddrRange();
    curr_state->longDesc.isStage2 = isStage2;

    assert(start_lookup_level < LookupLevel::Num_ArmLookupLevel);

    fetchDescriptor(desc_addr, curr_state->longDesc, sizeof(uint64_t), flag,
                    start_lookup_level,
                    LongDescEventByLevel[start_lookup_level],
                    &WalkUnit::doLongDescriptor, curr_state);

    return curr_state->fault;
}

std::tuple<Addr, Addr, WalkUnit::LookupLevel>
WalkUnit::walkAddresses(WalkerState *curr_state, Addr ttbr, GrainSize tg,
                        int tsz, int pa_range)
{
    const auto* ptops = getPageTableOps(tg);

    LookupLevel first_level = LookupLevel::Num_ArmLookupLevel;
    Addr table_addr = 0;
    Addr desc_addr = 0;

    if (curr_state->walkEntry.valid) {
        // WalkCache hit
        TlbEntry *entry = &curr_state->walkEntry;
        DPRINTF(PageTableWalker,
                "Walk Cache hit: va=%#x, level=%d, table address=%#x\n",
                curr_state->vaddr, entry->lookupLevel, entry->pfn);

        if (curr_state->longDescData.has_value()) {
            curr_state->longDescData->xnTable = entry->xn;
            curr_state->longDescData->pxnTable = entry->pxn;
            curr_state->longDescData->rwTable = bits(entry->ap, 1);
            curr_state->longDescData->userTable = bits(entry->ap, 0);
        }

        table_addr = entry->pfn;
        first_level = (LookupLevel)(entry->lookupLevel + 1);
    } else {
        // WalkCache miss
        first_level = isStage2 ? ptops->firstS2Level(curr_state->vtcr.sl0)
                               : ptops->firstLevel(64 - tsz);
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

    desc_addr = table_addr + ptops->index(curr_state->vaddr, first_level, tsz);

    return std::make_tuple(table_addr, desc_addr, first_level);
}

void
WalkUnit::memAttrs(WalkerState *curr_state, TlbEntry &te, uint8_t texcb,
                   bool s)
{
    // Note: tc and sctlr local variables are hiding tc and sctrl class
    // variables
    auto &sctlr = curr_state->sctlr;
    auto &tc = curr_state->tc;

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
        PRRR prrr = tc->readMiscReg(
            snsBankedIndex(MISCREG_PRRR, curr_state->tc,
                           curr_state->ss == SecurityState::NonSecure));
        NMRR nmrr = tc->readMiscReg(
            snsBankedIndex(MISCREG_NMRR, curr_state->tc,
                           curr_state->ss == SecurityState::NonSecure));
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
WalkUnit::memAttrsLPAE(WalkerState *curr_state, TlbEntry &te,
                       LongDescriptor &l_descriptor)
{
    assert(mmu->release()->has(ArmExtension::LPAE));

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
        int reg_as_int = snsBankedIndex(
            reg, curr_state->tc, curr_state->ss == SecurityState::NonSecure);
        uint32_t mair = curr_state->tc->readMiscReg(reg_as_int);
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

bool
WalkUnit::uncacheableFromAttrs(uint8_t attrs)
{
    return !bits(attrs, 2) || // Write-through
        attrs == 0b0100;      // NonCacheable
}

void
WalkUnit::memAttrsAArch64(WalkerState *curr_state, TlbEntry &te,
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

            // To be used when merging stage1 and astage 2 attributes
            te.xs = !l_descriptor.fnxs();
        }
    } else {
        uint8_t attrIndx = l_descriptor.attrIndx();

        DPRINTF(TLBVerbose, "memAttrsAArch64 AttrIndx:%#x sh:%#x\n", attrIndx, sh);

        // Select MAIR
        uint64_t mair;
        switch (curr_state->regime) {
            case TranslationRegime::EL10:
                mair = curr_state->tc->readMiscReg(MISCREG_MAIR_EL1);
                break;
            case TranslationRegime::EL20:
            case TranslationRegime::EL2:
                mair = curr_state->tc->readMiscReg(MISCREG_MAIR_EL2);
                break;
            case TranslationRegime::EL3:
                mair = curr_state->tc->readMiscReg(MISCREG_MAIR_EL3);
                break;
            default:
                panic("Invalid exception level");
                break;
        }

        // Select attributes
        attr = bits(mair, 8 * attrIndx + 7, 8 * attrIndx);
        attr_lo = bits(attr, 3, 0);
        attr_hi = bits(attr, 7, 4);

        // Treat write-through memory as uncacheable, this is safe
        // but for performance reasons not optimal.
        switch (attr) {
          case 0b00000000 ... 0b00001111: // Device Memory
            te.mtype = TlbEntry::MemoryType::Device;
            te.nonCacheable = true;
            te.xs = !bits(attr, 0);
            break;
          case 0b01000000: // Normal memory, Non-cacheable
            te.mtype = TlbEntry::MemoryType::Normal;
            te.nonCacheable = true;
            te.xs = false;
            break;
          case 0b10100000: // Normal memory, Write-through
            te.mtype = TlbEntry::MemoryType::Normal;
            te.nonCacheable = true;
            te.xs = false;
            break;
          default:
            te.mtype = TlbEntry::MemoryType::Normal;
            te.nonCacheable = uncacheableFromAttrs(attr_hi) ||
                              uncacheableFromAttrs(attr_lo);
            // XS is 0 only for write-back regions (cacheable)
            te.xs = te.nonCacheable;
            break;
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
WalkUnit::memAttrsWalkAArch64(WalkerState *curr_state, TlbEntry &te)
{
    te.mtype = TlbEntry::MemoryType::Normal;
    if (uncacheableWalk(curr_state)) {
        te.shareable = 3;
        te.outerAttrs = 0;
        te.innerAttrs = 0;
        te.nonCacheable = true;
    } else {
        te.shareable = curr_state->sh;
        te.outerAttrs = curr_state->orgn;
        te.innerAttrs = curr_state->irgn;
        te.nonCacheable = (te.outerAttrs == 0 || te.outerAttrs == 2) &&
            (te.innerAttrs == 0 || te.innerAttrs == 2);
    }

    // XS is 0 only for write-back regions (cacheable)
    te.xs = te.nonCacheable;
}

void
WalkUnit::doL1Descriptor(WalkerState *curr_state)
{
    if (curr_state->fault != NoFault) {
        return;
    }

    curr_state->l1Desc.data =
        htog(curr_state->l1Desc.data, byteOrder(curr_state->tc));

    DPRINTF(PageTableWalker, "L1 descriptor for %#x is %#x\n",
            curr_state->vaddr_tainted, curr_state->l1Desc.data);
    TlbEntry te;

    const bool is_atomic = curr_state->req->isAtomic();

    switch (curr_state->l1Desc.type()) {
        case L1Descriptor::Ignore:
        case L1Descriptor::Reserved:
            DPRINTF(PageTableWalker,
                    "L1 Descriptor Reserved/Ignore, causing fault\n");
            if (curr_state->isFetch) {
                curr_state->fault = std::make_shared<PrefetchAbort>(
                    curr_state->vaddr_tainted,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            } else {
                curr_state->fault = std::make_shared<DataAbort>(
                    curr_state->vaddr_tainted, DomainType::NoAccess,
                    is_atomic ? false : curr_state->isWrite,
                    ArmFault::TranslationLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            }
            return;
        case L1Descriptor::Section:
            if (curr_state->sctlr.afe &&
                bits(curr_state->l1Desc.ap(), 0) == 0) {
                /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is
                 * enabled if set, do l1.Desc.setAp0() instead of generating
                 * AccessFlag0
                 */

                curr_state->fault = std::make_shared<DataAbort>(
                    curr_state->vaddr_tainted, curr_state->l1Desc.domain(),
                    is_atomic ? false : curr_state->isWrite,
                    ArmFault::AccessFlagLL + LookupLevel::L1, isStage2,
                    TranMethod::VmsaTran);
            }
            if (curr_state->l1Desc.supersection()) {
                panic("Haven't implemented supersections\n");
            }
            insertTableEntry(curr_state, curr_state->l1Desc, false);
            return;
        case L1Descriptor::PageTable: {
            Addr l2desc_addr;
            l2desc_addr = curr_state->l1Desc.l2Addr() |
                          (bits(curr_state->vaddr, 19, 12) << 2);
            DPRINTF(PageTableWalker,
                    "L1 descriptor points to page table at: %#x (%s)\n",
                    l2desc_addr,
                    curr_state->ss == SecurityState::Secure ? "s" : "ns");

            Request::Flags flag = Request::PT_WALK;

            if (curr_state->sctlr.c == 0 || curr_state->isUncacheable) {
                flag.set(Request::UNCACHEABLE);
            }

            if (curr_state->secureLookup) {
                flag.set(Request::SECURE);
            }

            fetchDescriptor(l2desc_addr, curr_state->l2Desc, sizeof(uint32_t),
                            flag, LookupLevel::L2, &doL2DescEvent,
                            &WalkUnit::doL2Descriptor, curr_state);

            curr_state->delayed = curr_state->timing;

            return;
        }
      default:
        panic("A new type in a 2 bit field?\n");
    }
}

Fault
WalkUnit::generateLongDescFault(WalkerState *curr_state,
                                ArmFault::FaultSource src)
{
    if (curr_state->isFetch) {
        return std::make_shared<PrefetchAbort>(
            curr_state->vaddr_tainted, src + curr_state->longDesc.lookupLevel,
            isStage2, TranMethod::LpaeTran);
    } else {
        return std::make_shared<DataAbort>(
            curr_state->vaddr_tainted, DomainType::NoAccess,
            curr_state->req->isAtomic() ? false : curr_state->isWrite,
            src + curr_state->longDesc.lookupLevel, isStage2,
            TranMethod::LpaeTran);
    }
}

void
WalkUnit::scheduleWalk(WalkerState *next_walk, Tick when)
{
    doProcessEvent.setState(next_walk);
    schedule(doProcessEvent, when);
}

void
WalkUnit::doLongDescriptor(WalkerState *curr_state)
{
    if (curr_state->fault != NoFault) {
        return;
    }

    curr_state->longDesc.data =
        htog(curr_state->longDesc.data, byteOrder(curr_state->tc));

    DPRINTF(PageTableWalker, "L%d descriptor for %#llx is %#llx (%s)\n",
            curr_state->longDesc.lookupLevel, curr_state->vaddr_tainted,
            curr_state->longDesc.data,
            curr_state->aarch64 ? "AArch64" : "long-desc.");

    if ((curr_state->longDesc.type() == LongDescriptor::Block) ||
        (curr_state->longDesc.type() == LongDescriptor::Page)) {
        DPRINTF(PageTableWalker,
                "Analyzing L%d descriptor: %#llx, pxn: %d, "
                "xn: %d, ap: %d, piindex: %d, af: %d, type: %d\n",
                curr_state->longDesc.lookupLevel, curr_state->longDesc.data,
                curr_state->longDesc.pxn(), curr_state->longDesc.xn(),
                curr_state->longDesc.ap(), curr_state->longDesc.piindex(),
                curr_state->longDesc.af(), curr_state->longDesc.type());
    } else {
        DPRINTF(PageTableWalker, "Analyzing L%d descriptor: %#llx, type: %d\n",
                curr_state->longDesc.lookupLevel, curr_state->longDesc.data,
                curr_state->longDesc.type());
    }

    TlbEntry te;

    switch (curr_state->longDesc.type()) {
        case LongDescriptor::Invalid:
            DPRINTF(PageTableWalker,
                    "L%d descriptor Invalid, causing fault type %d\n",
                    curr_state->longDesc.lookupLevel,
                    ArmFault::TranslationLL +
                        curr_state->longDesc.lookupLevel);

            curr_state->fault =
                generateLongDescFault(curr_state, ArmFault::TranslationLL);
            return;

        case LongDescriptor::Block:
        case LongDescriptor::Page: {
            auto fault_source = ArmFault::FaultSourceInvalid;
            // Check for address size fault
            if (checkAddrSizeFaultAArch64(curr_state->longDesc.paddr(),
                                          curr_state->physAddrRange)) {

                DPRINTF(PageTableWalker,
                        "L%d descriptor causing Address Size Fault\n",
                        curr_state->longDesc.lookupLevel);
                fault_source = ArmFault::AddressSizeLL;

            // Check for access fault
            } else if (curr_state->longDesc.af() == 0) {

                DPRINTF(PageTableWalker,
                        "L%d descriptor causing Access Fault\n",
                        curr_state->longDesc.lookupLevel);
                fault_source = ArmFault::AccessFlagLL;
            }

            if (fault_source != ArmFault::FaultSourceInvalid) {
                curr_state->fault =
                    generateLongDescFault(curr_state, fault_source);
            } else {
                insertTableEntry(curr_state, curr_state->longDesc, true);
            }
        }
            return;
        case LongDescriptor::Table: {
            // Set hierarchical permission flags
            if (!isStage2) {
                curr_state->secureLookup = curr_state->secureLookup &&
                                           curr_state->longDesc.secureTable();
            }
            curr_state->longDescData->rwTable =
                curr_state->longDescData->rwTable &&
                (curr_state->longDesc.rwTable() || curr_state->hpd);
            curr_state->longDescData->userTable =
                curr_state->longDescData->userTable &&
                (curr_state->longDesc.userTable() || curr_state->hpd);
            curr_state->longDescData->xnTable =
                curr_state->longDescData->xnTable ||
                (curr_state->longDesc.xnTable() && !curr_state->hpd);
            curr_state->longDescData->pxnTable =
                curr_state->longDescData->pxnTable ||
                (curr_state->longDesc.pxnTable() && !curr_state->hpd);

            // Set up next level lookup
            Addr next_desc_addr =
                curr_state->longDesc.nextDescAddr(curr_state->vaddr);

            DPRINTF(PageTableWalker,
                    "L%d descriptor points to L%d descriptor at: %#x (%s)\n",
                    curr_state->longDesc.lookupLevel,
                    curr_state->longDesc.lookupLevel + 1, next_desc_addr,
                    curr_state->secureLookup ? "s" : "ns");

            // Check for address size fault
            if (curr_state->aarch64 &&
                checkAddrSizeFaultAArch64(next_desc_addr,
                                          curr_state->physAddrRange)) {
                DPRINTF(PageTableWalker,
                        "L%d descriptor causing Address Size Fault\n",
                        curr_state->longDesc.lookupLevel);

                curr_state->fault =
                    generateLongDescFault(curr_state, ArmFault::AddressSizeLL);
                return;
            }

            if (mmu->hasWalkCache()) {
                insertPartialTableEntry(curr_state);
            }

            Request::Flags flag = Request::PT_WALK;
            if (curr_state->secureLookup) {
                flag.set(Request::SECURE);
            }

            if (curr_state->sctlr.c == 0 || curr_state->isUncacheable) {
                flag.set(Request::UNCACHEABLE);
            }

            LookupLevel L = curr_state->longDesc.lookupLevel =
                (LookupLevel)(curr_state->longDesc.lookupLevel + 1);
            Event *event = NULL;
            switch (L) {
              case LookupLevel::L1:
                  assert(curr_state->aarch64);
                  [[fallthrough]];
              case LookupLevel::L2:
              case LookupLevel::L3:
                event = LongDescEventByLevel[L];
                break;
              default:
                panic("Wrong lookup level in table walk\n");
                break;
            }

            fetchDescriptor(next_desc_addr, curr_state->longDesc,
                            sizeof(uint64_t), flag, L, event,
                            &WalkUnit::doLongDescriptor, curr_state);

            curr_state->delayed = curr_state->timing;
        }
            return;
        default:
            panic("A new type in a 2 bit field?\n");
    }
}

void
WalkUnit::doL2Descriptor(WalkerState *curr_state)
{
    if (curr_state->fault != NoFault) {
        return;
    }

    curr_state->l2Desc.data =
        htog(curr_state->l2Desc.data, byteOrder(curr_state->tc));

    DPRINTF(PageTableWalker, "L2 descriptor for %#x is %#x\n",
            curr_state->vaddr_tainted, curr_state->l2Desc.data);
    TlbEntry te;

    const bool is_atomic = curr_state->req->isAtomic();

    if (curr_state->l2Desc.invalid()) {
        DPRINTF(PageTableWalker, "L2 descriptor invalid, causing fault\n");
        if (curr_state->isFetch) {
            curr_state->fault = std::make_shared<PrefetchAbort>(
                curr_state->vaddr_tainted,
                ArmFault::TranslationLL + LookupLevel::L2, isStage2,
                TranMethod::VmsaTran);
        } else {
            curr_state->fault = std::make_shared<DataAbort>(
                curr_state->vaddr_tainted, curr_state->l1Desc.domain(),
                is_atomic ? false : curr_state->isWrite,
                ArmFault::TranslationLL + LookupLevel::L2, isStage2,
                TranMethod::VmsaTran);
        }
        return;
    }

    if (curr_state->sctlr.afe && bits(curr_state->l2Desc.ap(), 0) == 0) {
        /** @todo: check sctlr.ha (bit[17]) if Hardware Access Flag is enabled
          * if set, do l2.Desc.setAp0() instead of generating AccessFlag0
          */
        DPRINTF(PageTableWalker,
                "Generating access fault at L2, afe: %d, ap: %d\n",
                curr_state->sctlr.afe, curr_state->l2Desc.ap());

        curr_state->fault = std::make_shared<DataAbort>(
            curr_state->vaddr_tainted, DomainType::NoAccess,
            is_atomic ? false : curr_state->isWrite,
            ArmFault::AccessFlagLL + LookupLevel::L2, isStage2,
            TranMethod::VmsaTran);
    }

    insertTableEntry(curr_state, curr_state->l2Desc, false);
}

void
WalkUnit::doL1DescriptorWrapper()
{
    WalkerState *curr_state = stateQueues[LookupLevel::L1].front();
    curr_state->delayed = false;
    // if there's a stage2 translation object we don't need it any more
    if (curr_state->stage2Tran) {
        delete curr_state->stage2Tran;
        curr_state->stage2Tran = NULL;
    }

    DPRINTF(PageTableWalker, "L1 Desc object host addr: %p\n",
            &curr_state->l1Desc.data);
    DPRINTF(PageTableWalker, "L1 Desc object      data: %08x\n",
            curr_state->l1Desc.data);

    DPRINTF(PageTableWalker, "calling doL1Descriptor for vaddr:%#x\n",
            curr_state->vaddr_tainted);
    doL1Descriptor(curr_state);

    stateQueues[LookupLevel::L1].pop_front();
    // Check if fault was generated
    if (curr_state->fault != NoFault) {
        curr_state->transState->finish(curr_state->fault, curr_state->req,
                                       curr_state->tc, curr_state->mode);
        parent->stats.walksShortTerminatedAtLevel[0]++;

        parent->nextWalk(this);

        curr_state->req = NULL;
        curr_state->tc = NULL;
        curr_state->delayed = false;
        delete curr_state;
    } else if (!curr_state->delayed) {
        // delay is not set so there is no L2 to do
        // Don't finish the translation if a stage 2 look up is underway
        parent->stats.walkServiceTime.sample(curTick() -
                                             curr_state->startTime);

        DPRINTF(PageTableWalker, "calling translateTiming again\n");

        parent->inCompletionWalks.push_back(curr_state);

        mmu->translateTiming(curr_state->req, curr_state->tc,
                             curr_state->transState, curr_state->mode,
                             curr_state->tranType, isStage2);

        parent->inCompletionWalks.pop_back();

        parent->stats.walksShortTerminatedAtLevel[0]++;

        parent->nextWalk(this);

        curr_state->req = NULL;
        curr_state->tc = NULL;
        curr_state->delayed = false;
        delete curr_state;
    } else {
        // need to do L2 descriptor
        stashCurrState(curr_state, LookupLevel::L2);
    }
}

void
WalkUnit::doL2DescriptorWrapper()
{
    WalkerState *curr_state = stateQueues[LookupLevel::L2].front();
    assert(curr_state->delayed);

    // if there's a stage2 translation object we don't need it any more
    if (curr_state->stage2Tran) {
        delete curr_state->stage2Tran;
        curr_state->stage2Tran = NULL;
    }

    DPRINTF(PageTableWalker, "calling doL2Descriptor for vaddr:%#x\n",
            curr_state->vaddr_tainted);
    doL2Descriptor(curr_state);

    // Check if fault was generated
    if (curr_state->fault != NoFault) {
        curr_state->transState->finish(curr_state->fault, curr_state->req,
                                       curr_state->tc, curr_state->mode);
        parent->stats.walksShortTerminatedAtLevel[1]++;
    } else {
        parent->stats.walkServiceTime.sample(curTick() -
                                             curr_state->startTime);

        DPRINTF(PageTableWalker, "calling translateTiming again\n");

        parent->inCompletionWalks.push_back(curr_state);

        mmu->translateTiming(curr_state->req, curr_state->tc,
                             curr_state->transState, curr_state->mode,
                             curr_state->tranType, isStage2);

        parent->inCompletionWalks.pop_back();

        parent->stats.walksShortTerminatedAtLevel[1]++;
    }

    stateQueues[LookupLevel::L2].pop_front();
    parent->nextWalk(this);

    curr_state->req = NULL;
    curr_state->tc = NULL;
    curr_state->delayed = false;

    delete curr_state;
}

void
WalkUnit::doL0LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L0);
}

void
WalkUnit::doL1LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L1);
}

void
WalkUnit::doL2LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L2);
}

void
WalkUnit::doL3LongDescriptorWrapper()
{
    doLongDescriptorWrapper(LookupLevel::L3);
}

void
WalkUnit::doLongDescriptorWrapper(LookupLevel curr_lookup_level)
{
    WalkerState *curr_state = stateQueues[curr_lookup_level].front();
    assert(curr_lookup_level == curr_state->longDesc.lookupLevel);
    curr_state->delayed = false;

    // if there's a stage2 translation object we don't need it any more
    if (curr_state->stage2Tran) {
        delete curr_state->stage2Tran;
        curr_state->stage2Tran = NULL;
    }

    DPRINTF(PageTableWalker, "calling doLongDescriptor for vaddr:%#x\n",
            curr_state->vaddr_tainted);
    doLongDescriptor(curr_state);

    stateQueues[curr_lookup_level].pop_front();

    if (curr_state->fault != NoFault) {
        // A fault was generated
        curr_state->transState->finish(curr_state->fault, curr_state->req,
                                       curr_state->tc, curr_state->mode);

        parent->nextWalk(this);

        curr_state->req = NULL;
        curr_state->tc = NULL;
        curr_state->delayed = false;
        delete curr_state;
    } else if (!curr_state->delayed) {
        // No additional lookups required
        parent->stats.walkServiceTime.sample(curTick() -
                                             curr_state->startTime);

        DPRINTF(PageTableWalker, "calling translateTiming again\n");

        parent->inCompletionWalks.push_back(curr_state);

        mmu->translateTiming(curr_state->req, curr_state->tc,
                             curr_state->transState, curr_state->mode,
                             curr_state->tranType, isStage2);

        parent->inCompletionWalks.pop_back();

        parent->stats.walksLongTerminatedAtLevel[curr_lookup_level]++;

        parent->nextWalk(this);

        curr_state->req = NULL;
        curr_state->tc = NULL;
        curr_state->delayed = false;
        delete curr_state;
    } else {
        if (curr_lookup_level >= LookupLevel::Num_ArmLookupLevel - 1)
            panic("Max. number of lookups already reached in table walk\n");
        // Need to perform additional lookups
        stashCurrState(curr_state, curr_state->longDesc.lookupLevel);
    }
}

void
WalkUnit::fetchDescriptor(
    Addr desc_addr, DescriptorBase &descriptor, int num_bytes,
    Request::Flags flags, LookupLevel lookup_level, Event *event,
    void (WalkUnit::*doDescriptor)(WalkerState *curr_state),
    WalkerState *curr_state)
{
    uint8_t *data = descriptor.getRawPtr();

    DPRINTF(PageTableWalker,
            "Fetching descriptor at address: 0x%x stage2Req: %d\n", desc_addr,
            curr_state->stage2Req);

    // If this translation has a stage 2 then we know desc_addr is an IPA and
    // needs to be translated before we can access the page table. Do that
    // check here.
    if (curr_state->stage2Req) {
        Fault fault;

        if (curr_state->timing) {
            auto *tran =
                new Stage2Walk(*this, data, event, curr_state->vaddr,
                               curr_state->mode, curr_state->tranType, port);
            curr_state->stage2Tran = tran;
            readDataTimed(curr_state->tc, desc_addr, tran, num_bytes, flags);
            fault = tran->fault;

            if (fault != NoFault) {
                curr_state->fault = fault;
            }
        } else {
            fault =
                readDataUntimed(curr_state->tc, curr_state->vaddr, desc_addr,
                                data, num_bytes, flags, curr_state->mode,
                                curr_state->tranType, curr_state->functional);

            if (fault != NoFault) {
                curr_state->fault = fault;
            }

            (this->*doDescriptor)(curr_state);
        }
    } else {
        RequestPtr req = std::make_shared<Request>(desc_addr, num_bytes, flags,
                                                   port->requestorId());
        req->taskId(context_switch_task_id::DMA);

        mpamTagTableWalk(curr_state, req);

        Fault fault =
            testWalk(curr_state, req, descriptor.domain(), lookup_level);

        if (fault != NoFault) {
            curr_state->fault = fault;
            return;
        }

        if (curr_state->timing) {
            port->sendTimingReq(
                req, data, curr_state->tc->getCpuPtr()->clockPeriod(), event);

        } else if (!curr_state->functional) {
            port->sendAtomicReq(req, data,
                                curr_state->tc->getCpuPtr()->clockPeriod());

            (this->*doDescriptor)(curr_state);
        } else {
            port->sendFunctionalReq(req, data);
            (this->*doDescriptor)(curr_state);
        }
    }
}

void
WalkUnit::stashCurrState(WalkerState *curr_state, int queue_idx)
{
    DPRINTF(PageTableWalker, "Adding to walker fifo: "
            "queue size before adding: %d\n",
            stateQueues[queue_idx].size());
    stateQueues[queue_idx].push_back(curr_state);
}

bool
WalkUnit::busyOnSamePage(Addr iaddr) const
{
    for (int lvl = 0; lvl < LookupLevel::Num_ArmLookupLevel; lvl++) {
        panic_if(stateQueues[lvl].size() > 1, "How is this possible\n");
        for (auto state : stateQueues[lvl]) {
            if (auto tg = state->longDesc.grainSize;
                state->vaddr >> tg == iaddr >> tg) {
                return true;
            }
        }
    }
    return false;
}

void
WalkUnit::insertPartialTableEntry(WalkerState *curr_state)
{
    LongDescriptor &descriptor = curr_state->longDesc;
    const bool have_security = mmu->release()->has(ArmExtension::SECURITY);
    TlbEntry te;

    // Create and fill a new page table entry
    te.valid          = true;
    te.longDescFormat = true;
    te.partial        = true;
    // The entry is global if there is no address space identifier
    // to differentiate translation contexts
    // clang-format off
    te.global         = !mmu->hasUnprivRegime(curr_state->regime);
    te.asid           = curr_state->asid;
    te.vmid           = curr_state->vmid;
    te.N              = descriptor.offsetBits();
    te.tg             = descriptor.grainSize;
    te.vpn            = curr_state->vaddr >> te.N;
    te.size           = (1ULL << te.N) - 1;
    te.pfn            = descriptor.nextTableAddr();
    te.domain         = descriptor.domain();
    te.lookupLevel    = descriptor.lookupLevel;
    te.ns             = !descriptor.secure(have_security, curr_state);
    te.ss             = curr_state->ss;
    te.ipaSpace       = curr_state->ipaSpace; // Used by stage2 entries only
    te.type           = TypeTLB::unified;

    te.regime         = curr_state->regime;
    // clang-format on

    te.xn = curr_state->longDescData->xnTable;
    te.pxn = curr_state->longDescData->pxnTable;
    te.ap = (curr_state->longDescData->rwTable << 1) |
            (curr_state->longDescData->userTable);

    memAttrsWalkAArch64(curr_state, te);

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
    mmu->insert(te, curr_state->mode, isStage2);
}

void
WalkUnit::insertTableEntry(WalkerState *curr_state, DescriptorBase &descriptor,
                           bool long_descriptor)
{
    const bool have_security = mmu->release()->has(ArmExtension::SECURITY);
    TlbEntry te;

    // Create and fill a new page table entry
    // clang-format off
    te.valid          = true;
    te.longDescFormat = long_descriptor;
    te.asid           = curr_state->asid;
    te.vmid           = curr_state->vmid;
    te.N              = descriptor.offsetBits();
    te.vpn            = curr_state->vaddr >> te.N;
    te.size           = (1<<te.N) - 1;
    te.pfn            = descriptor.pfn();
    te.domain         = descriptor.domain();
    te.lookupLevel    = descriptor.lookupLevel;
    te.ns             = !descriptor.secure(have_security, curr_state);
    te.ss             = curr_state->ss;
    te.ipaSpace       = curr_state->ipaSpace; // Used by stage2 entries only
    te.xn             = descriptor.xn();
    te.type = curr_state->mode == BaseMMU::Execute ? TypeTLB::instruction
                                                   : TypeTLB::data;
    // clang-format on

    te.regime = curr_state->regime;

    parent->stats.pageSizes[pageSizeNtoStatBin(te.N)]++;
    parent->stats.requestOrigin[TableWalker::COMPLETED][curr_state->isFetch]++;

    // ASID has no meaning for stage 2 TLB entries, so mark all stage 2 entries
    // as global
    te.global = descriptor.global(curr_state) || curr_state->isStage2;
    if (long_descriptor) {
        LongDescriptor l_descriptor =
            dynamic_cast<LongDescriptor &>(descriptor);

        te.tg = l_descriptor.grainSize;
        te.xn |= curr_state->longDescData->xnTable;
        te.pxn = curr_state->longDescData->pxnTable || l_descriptor.pxn();
        if (isStage2) {
            // this is actually the HAP field, but its stored in the same bit
            // possitions as the AP field in a stage 1 translation.
            te.hap = l_descriptor.ap();
        } else {
            te.ap =
                ((!curr_state->longDescData->rwTable || descriptor.ap() >> 1)
                 << 1) |
                (curr_state->longDescData->userTable &&
                 (descriptor.ap() & 0x1));
            // Add index of Indirect Permission.
            te.piindex = l_descriptor.piindex();
        }
        if (curr_state->aarch64) {
            memAttrsAArch64(curr_state, te, l_descriptor);
        } else {
            memAttrsLPAE(curr_state, te, l_descriptor);
        }
    } else {
        te.ap = descriptor.ap();
        memAttrs(curr_state, te, descriptor.texcb(), descriptor.shareable());
    }

    // Debug output
    DPRINTF(TLB, descriptor.dbgHeader().c_str());
    DPRINTF(TLB, " - N:%d pfn:%#x size:%#x global:%d valid:%d\n",
            te.N, te.pfn, te.size, te.global, te.valid);
    DPRINTF(TLB, " - vpn:%#x xn:%d pxn:%d ap:%d piindex:%d domain:%d asid:%d "
            "vmid:%d nc:%d ns:%d\n", te.vpn, te.xn, te.pxn,
            te.ap, te.piindex,
            static_cast<uint8_t>(te.domain), te.asid, te.vmid,
            te.nonCacheable, te.ns);
    DPRINTF(TLB, " - domain from L%d desc:%d data:%#x\n",
            descriptor.lookupLevel, static_cast<uint8_t>(descriptor.domain()),
            descriptor.getRawData());

    // Insert the entry into the TLBs
    mmu->insert(te, curr_state->mode, isStage2);
}

WalkUnit::LookupLevel
WalkUnit::toLookupLevel(uint8_t lookup_level_as_int)
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

Fault
WalkUnit::testWalk(WalkerState *curr_state, const RequestPtr &walk_req,
                   DomainType domain, LookupLevel lookup_level)
{
    if (!test) {
        return NoFault;
    } else {
        return test->walkCheck(walk_req, curr_state->vaddr,
                               curr_state->ss == SecurityState::Secure,
                               curr_state->el != EL0, curr_state->mode, domain,
                               lookup_level);
    }
}

void
WalkUnit::setTestInterface(TlbTestInterface *ti)
{
    test = ti;
}

uint8_t
WalkUnit::pageSizeNtoStatBin(uint8_t N)
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
WalkUnit::readDataUntimed(ThreadContext *tc, Addr vaddr, Addr desc_addr,
                          uint8_t *data, int num_bytes, Request::Flags flags,
                          BaseMMU::Mode mode,
                          MMU::ArmTranslationType tran_type, bool functional)
{
    Fault fault;

    // translate to physical address using the second stage MMU
    auto req = std::make_shared<Request>();
    req->setVirt(desc_addr, num_bytes, flags | Request::PT_WALK,
                 port->requestorId(), 0);

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
WalkUnit::mpamTagTableWalk(WalkerState *curr_state, RequestPtr &req) const
{
    mpam::tagRequest(curr_state->tc, req, curr_state->isFetch);
}

void
WalkUnit::readDataTimed(ThreadContext *tc, Addr desc_addr,
                        Stage2Walk *translation, int num_bytes,
                        Request::Flags flags)
{
    // translate to physical address using the second stage MMU
    translation->setVirt(desc_addr, num_bytes, flags | Request::PT_WALK,
                         port->requestorId());
    translation->translateTiming(tc);
}

WalkUnit::Stage2Walk::Stage2Walk(WalkUnit &_parent, uint8_t *_data,
                                 Event *_event, Addr vaddr,
                                 BaseMMU::Mode _mode,
                                 MMU::ArmTranslationType tran_type,
                                 TableWalker::Port *_port)
    : data(_data),
      numBytes(0),
      event(_event),
      parent(_parent),
      oVAddr(vaddr),
      mode(_mode),
      tranType(tran_type),
      port(_port),
      fault(NoFault)
{
    req = std::make_shared<Request>();
}

void
WalkUnit::Stage2Walk::finish(const Fault &_fault, const RequestPtr &req,
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
        port->sendTimingReq(req, data, tc->getCpuPtr()->clockPeriod(), event);
    } else {
        // We can't do the DMA access as there's been a problem, so tell the
        // event we're done
        event->process();
    }
}

void
WalkUnit::Stage2Walk::translateTiming(ThreadContext *tc)
{
    parent.mmu->translateTiming(req, tc, this, mode, tranType, true);
}

TableWalker::TableWalkerStats::TableWalkerStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(instructionWalksS1, statistics::units::Count::get(),
               "Table walker stage 1 instruction walks requested"),
      ADD_STAT(instructionWalksS2, statistics::units::Count::get(),
               "Table walker stage 2 instruction walks requested"),
      ADD_STAT(dataWalksS1, statistics::units::Count::get(),
               "Table walker stage 1 data walks requested"),
      ADD_STAT(dataWalksS2, statistics::units::Count::get(),
               "Table walker stage 2 data walks requested"),
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
               "Table walker requests started/completed, data/inst"),
      ADD_STAT(walks, statistics::units::Count::get(),
               "Table walker walks requested",
               instructionWalksS1 + instructionWalksS2 + dataWalksS1 +
                   dataWalksS2)
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
