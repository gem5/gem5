/*
 * Copyright (c) 2013, 2018-2019, 2021 Arm Limited
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

#include "dev/arm/smmu_v3_transl.hh"

#include "arch/arm/pagetable.hh"
#include "debug/SMMUv3.hh"
#include "debug/SMMUv3Hazard.hh"
#include "dev/arm/amba.hh"
#include "dev/arm/smmu_v3.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace ArmISA;

SMMUTranslRequest
SMMUTranslRequest::fromPacket(PacketPtr pkt, bool ats)
{
    SMMUTranslRequest req;
    req.addr         = pkt->getAddr();
    req.size         = pkt->getSize();
    req.sid          = pkt->req->streamId();
    req.ssid         = pkt->req->hasSubstreamId() ?
        pkt->req->substreamId() : 0;
    req.isWrite      = pkt->isWrite();
    req.isPrefetch   = false;
    req.isAtsRequest = ats;
    req.pkt          = pkt;

    return req;
}

SMMUTranslRequest
SMMUTranslRequest::prefetch(Addr addr, uint32_t sid, uint32_t ssid)
{
    SMMUTranslRequest req;
    req.addr         = addr;
    req.size         = 0;
    req.sid          = sid;
    req.ssid         = ssid;
    req.isWrite      = false;
    req.isPrefetch   = true;
    req.isAtsRequest = false;
    req.pkt          = NULL;

    return req;
}

SMMUTranslationProcess::SMMUTranslationProcess(const std::string &name,
    SMMUv3 &_smmu, SMMUv3DeviceInterface &_ifc)
  :
    SMMUProcess(name, _smmu),
    ifc(_ifc)
{
    // Decrease number of pending translation slots on the device interface
    assert(ifc.xlateSlotsRemaining > 0);
    ifc.xlateSlotsRemaining--;

    ifc.pendingMemAccesses++;
    reinit();
}

SMMUTranslationProcess::~SMMUTranslationProcess()
{
    // Increase number of pending translation slots on the device interface
    assert(ifc.pendingMemAccesses > 0);
    ifc.pendingMemAccesses--;

    // If no more SMMU memory accesses are pending,
    // signal SMMU Device Interface as drained
    if (ifc.pendingMemAccesses == 0) {
        ifc.signalDrainDone();
    }
}

void
SMMUTranslationProcess::beginTransaction(const SMMUTranslRequest &req)
{
    request = req;

    reinit();
}

void
SMMUTranslationProcess::resumeTransaction()
{
    assert(smmu.system.isTimingMode());

    assert(!"Stalls are broken");

    Tick resumeTick = curTick();

    (void) resumeTick;
    DPRINTF(SMMUv3, "Resume at tick = %d. Fault duration = %d (%.3fus)\n",
        resumeTick, resumeTick-faultTick, (resumeTick-faultTick) / 1e6);

    beginTransaction(request);

    smmu.runProcessTiming(this, request.pkt);
}

void
SMMUTranslationProcess::main(Yield &yield)
{
    // Hack:
    // The coroutine starts running as soon as it's created.
    // But we need to wait for request data esp. in atomic mode.
    SMMUAction a;
    a.type = ACTION_INITIAL_NOP;
    a.pkt = NULL;
    yield(a);

    const Addr next4k = (request.addr + 0x1000ULL) & ~0xfffULL;

    if ((request.addr + request.size) > next4k)
        panic("Transaction crosses 4k boundary (addr=%#x size=%#x)!\n",
                request.addr, request.size);


    unsigned numResponderBeats = request.isWrite ?
        (request.size + (ifc.portWidth - 1)) / ifc.portWidth : 1;

    doSemaphoreDown(yield, ifc.devicePortSem);
    doDelay(yield, Cycles(numResponderBeats));
    doSemaphoreUp(ifc.devicePortSem);


    recvTick = curTick();

    if (!(smmu.regs.cr0 & CR0_SMMUEN_MASK)) {
        // SMMU disabled
        doDelay(yield, Cycles(1));
        completeTransaction(yield, bypass(request.addr));
        return;
    }

    TranslResult tr;
    bool wasPrefetched = false;

    if (request.isPrefetch) {
        // Abort prefetch if:
        //   - there's already a transaction looking up the same 4k page, OR
        //   - requested address is already in the TLB.
        if (hazard4kCheck() || ifcTLBLookup(yield, tr, wasPrefetched))
            completePrefetch(yield); // this never returns

        hazard4kRegister();

        tr = smmuTranslation(yield);

        if (tr.fault == FAULT_NONE)
            ifcTLBUpdate(yield, tr);

        hazard4kRelease();

        completePrefetch(yield);
    } else {
        hazardIdRegister();

        if (!microTLBLookup(yield, tr)) {
            bool hit = ifcTLBLookup(yield, tr, wasPrefetched);
            if (!hit) {
                while (!hit && hazard4kCheck()) {
                    hazard4kHold(yield);
                    hit = ifcTLBLookup(yield, tr, wasPrefetched);
                }
            }

            // Issue prefetch if:
            //   - there was a TLB hit and the entry was prefetched, OR
            //   - TLB miss was successfully serviced
            if (hit) {
                if (wasPrefetched)
                    issuePrefetch(next4k);
            } else {
                hazard4kRegister();

                tr = smmuTranslation(yield);

                if (tr.fault == FAULT_NONE) {
                    ifcTLBUpdate(yield, tr);

                    issuePrefetch(next4k);
                }

                hazard4kRelease();
            }

            if (tr.fault == FAULT_NONE)
                microTLBUpdate(yield, tr);
        }

        hazardIdHold(yield);
        hazardIdRelease();

        if (tr.fault != FAULT_NONE)
            panic("Translation Fault (addr=%#x, size=%#x, sid=%d, ssid=%d, "
                    "isWrite=%d, isPrefetch=%d, isAtsRequest=%d)\n",
                    request.addr, request.size, request.sid, request.ssid,
                    request.isWrite, request.isPrefetch, request.isAtsRequest);

        completeTransaction(yield, tr);
    }
}

SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::bypass(Addr addr) const
{
    TranslResult tr;
    tr.fault = FAULT_NONE;
    tr.addr = addr;
    tr.addrMask = 0;
    tr.writable = 1;

    return tr;
}

SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::smmuTranslation(Yield &yield)
{
    TranslResult tr;

    // Need SMMU credit to proceed
    doSemaphoreDown(yield, smmu.transSem);

    // Simulate pipelined IFC->SMMU link
    doSemaphoreDown(yield, smmu.ifcSmmuSem);
    doDelay(yield, Cycles(1)); // serialize transactions
    doSemaphoreUp(smmu.ifcSmmuSem);
    doDelay(yield, smmu.ifcSmmuLat - Cycles(1)); // remaining pipeline delay

    bool haveConfig = true;
    if (!configCacheLookup(yield, context)) {
        if (findConfig(yield, context, tr)) {
            configCacheUpdate(yield, context);
        } else {
            haveConfig = false;
        }
    }

    if (haveConfig && !smmuTLBLookup(yield, tr)) {
        // SMMU main TLB miss

        // Need PTW slot to proceed
        doSemaphoreDown(yield, smmu.ptwSem);

        // Page table walk
        Tick ptwStartTick = curTick();

        if (context.stage1Enable) {
            tr = translateStage1And2(yield, request.addr);
        } else if (context.stage2Enable) {
            tr = translateStage2(yield, request.addr, true);
        } else {
            tr = bypass(request.addr);
        }

        if (context.stage1Enable || context.stage2Enable)
            smmu.stats.ptwTimeDist.sample(curTick() - ptwStartTick);

        // Free PTW slot
        doSemaphoreUp(smmu.ptwSem);

        if (tr.fault == FAULT_NONE)
            smmuTLBUpdate(yield, tr);
    }

    // Simulate pipelined SMMU->RESPONSE INTERFACE link
    doSemaphoreDown(yield, smmu.smmuIfcSem);
    doDelay(yield, Cycles(1)); // serialize transactions
    doSemaphoreUp(smmu.smmuIfcSem);
    doDelay(yield, smmu.smmuIfcLat - Cycles(1)); // remaining pipeline delay

    // return SMMU credit
    doSemaphoreUp(smmu.transSem);

    return tr;
}

bool
SMMUTranslationProcess::microTLBLookup(Yield &yield, TranslResult &tr)
{
    if (!ifc.microTLBEnable)
        return false;

    doSemaphoreDown(yield, ifc.microTLBSem);
    doDelay(yield, ifc.microTLBLat);
    const SMMUTLB::Entry *e =
        ifc.microTLB->lookup(request.sid, request.ssid, request.addr);
    doSemaphoreUp(ifc.microTLBSem);

    if (!e) {
        DPRINTF(SMMUv3, "micro TLB miss vaddr=%#x sid=%#x ssid=%#x\n",
            request.addr, request.sid, request.ssid);

        return false;
    }

    DPRINTF(SMMUv3,
        "micro TLB hit vaddr=%#x amask=%#x sid=%#x ssid=%#x paddr=%#x\n",
        request.addr, e->vaMask, request.sid, request.ssid, e->pa);

    tr.fault = FAULT_NONE;
    tr.addr = e->pa + (request.addr & ~e->vaMask);;
    tr.addrMask = e->vaMask;
    tr.writable = e->permissions;

    return true;
}

bool
SMMUTranslationProcess::ifcTLBLookup(Yield &yield, TranslResult &tr,
                                     bool &wasPrefetched)
{
    if (!ifc.mainTLBEnable)
        return false;

    doSemaphoreDown(yield, ifc.mainTLBSem);
    doDelay(yield, ifc.mainTLBLat);
    const SMMUTLB::Entry *e =
        ifc.mainTLB->lookup(request.sid, request.ssid, request.addr);
    doSemaphoreUp(ifc.mainTLBSem);

    if (!e) {
        DPRINTF(SMMUv3,
                "RESPONSE Interface TLB miss vaddr=%#x sid=%#x ssid=%#x\n",
                request.addr, request.sid, request.ssid);

        return false;
    }

    DPRINTF(SMMUv3,
            "RESPONSE Interface TLB hit vaddr=%#x amask=%#x sid=%#x ssid=%#x "
            "paddr=%#x\n", request.addr, e->vaMask, request.sid,
            request.ssid, e->pa);

    tr.fault = FAULT_NONE;
    tr.addr = e->pa + (request.addr & ~e->vaMask);;
    tr.addrMask = e->vaMask;
    tr.writable = e->permissions;
    wasPrefetched = e->prefetched;

    return true;
}

bool
SMMUTranslationProcess::smmuTLBLookup(Yield &yield, TranslResult &tr)
{
    if (!smmu.tlbEnable)
        return false;

    doSemaphoreDown(yield, smmu.tlbSem);
    doDelay(yield, smmu.tlbLat);
    const ARMArchTLB::Entry *e =
        smmu.tlb.lookup(request.addr, context.asid, context.vmid);
    doSemaphoreUp(smmu.tlbSem);

    if (!e) {
        DPRINTF(SMMUv3, "SMMU TLB miss vaddr=%#x asid=%#x vmid=%#x\n",
            request.addr, context.asid, context.vmid);

        return false;
    }

    DPRINTF(SMMUv3,
            "SMMU TLB hit vaddr=%#x amask=%#x asid=%#x vmid=%#x paddr=%#x\n",
            request.addr, e->vaMask, context.asid, context.vmid, e->pa);

    tr.fault = FAULT_NONE;
    tr.addr = e->pa + (request.addr & ~e->vaMask);;
    tr.addrMask = e->vaMask;
    tr.writable = e->permissions;

    return true;
}

void
SMMUTranslationProcess::microTLBUpdate(Yield &yield,
                                       const TranslResult &tr)
{
    assert(tr.fault == FAULT_NONE);

    if (!ifc.microTLBEnable)
        return;

    SMMUTLB::Entry e;
    e.valid = true;
    e.prefetched = false;
    e.sid = request.sid;
    e.ssid = request.ssid;
    e.vaMask = tr.addrMask;
    e.va = request.addr & e.vaMask;
    e.pa = tr.addr & e.vaMask;
    e.permissions = tr.writable;
    e.asid = context.asid;
    e.vmid = context.vmid;

    doSemaphoreDown(yield, ifc.microTLBSem);

    DPRINTF(SMMUv3,
        "micro TLB upd vaddr=%#x amask=%#x paddr=%#x sid=%#x ssid=%#x\n",
        e.va, e.vaMask, e.pa, e.sid, e.ssid);

    ifc.microTLB->store(e, SMMUTLB::ALLOC_ANY_WAY);

    doSemaphoreUp(ifc.microTLBSem);
}

void
SMMUTranslationProcess::ifcTLBUpdate(Yield &yield,
                                     const TranslResult &tr)
{
    assert(tr.fault == FAULT_NONE);

    if (!ifc.mainTLBEnable)
        return;

    SMMUTLB::Entry e;
    e.valid = true;
    e.prefetched = request.isPrefetch;
    e.sid = request.sid;
    e.ssid = request.ssid;
    e.vaMask = tr.addrMask;
    e.va = request.addr & e.vaMask;
    e.pa = tr.addr & e.vaMask;
    e.permissions = tr.writable;
    e.asid = context.asid;
    e.vmid = context.vmid;

    SMMUTLB::AllocPolicy alloc = SMMUTLB::ALLOC_ANY_WAY;
    if (ifc.prefetchEnable && ifc.prefetchReserveLastWay)
        alloc = request.isPrefetch ?
            SMMUTLB::ALLOC_LAST_WAY : SMMUTLB::ALLOC_ANY_BUT_LAST_WAY;

    doSemaphoreDown(yield, ifc.mainTLBSem);

    DPRINTF(SMMUv3,
            "RESPONSE Interface upd vaddr=%#x amask=%#x paddr=%#x sid=%#x "
            "ssid=%#x\n", e.va, e.vaMask, e.pa, e.sid, e.ssid);

    ifc.mainTLB->store(e, alloc);

    doSemaphoreUp(ifc.mainTLBSem);
}

void
SMMUTranslationProcess::smmuTLBUpdate(Yield &yield,
                                      const TranslResult &tr)
{
    assert(tr.fault == FAULT_NONE);

    if (!smmu.tlbEnable)
        return;

    ARMArchTLB::Entry e;
    e.valid = true;
    e.vaMask = tr.addrMask;
    e.va = request.addr & e.vaMask;
    e.asid = context.asid;
    e.vmid = context.vmid;
    e.pa = tr.addr & e.vaMask;
    e.permissions = tr.writable;

    doSemaphoreDown(yield, smmu.tlbSem);

    DPRINTF(SMMUv3,
            "SMMU TLB upd vaddr=%#x amask=%#x paddr=%#x asid=%#x vmid=%#x\n",
            e.va, e.vaMask, e.pa, e.asid, e.vmid);

    smmu.tlb.store(e);

    doSemaphoreUp(smmu.tlbSem);
}

bool
SMMUTranslationProcess::configCacheLookup(Yield &yield, TranslContext &tc)
{
    if (!smmu.configCacheEnable)
        return false;

    doSemaphoreDown(yield, smmu.configSem);
    doDelay(yield, smmu.configLat);
    const ConfigCache::Entry *e =
        smmu.configCache.lookup(request.sid, request.ssid);
    doSemaphoreUp(smmu.configSem);

    if (!e) {
        DPRINTF(SMMUv3, "Config miss sid=%#x ssid=%#x\n",
                request.sid, request.ssid);

        return false;
    }

    DPRINTF(SMMUv3, "Config hit sid=%#x ssid=%#x ttb=%#08x asid=%#x\n",
            request.sid, request.ssid, e->ttb0, e->asid);

    tc.stage1Enable = e->stage1_en;
    tc.stage2Enable = e->stage2_en;

    tc.ttb0 = e->ttb0;
    tc.ttb1 = e->ttb1;
    tc.asid = e->asid;
    tc.httb = e->httb;
    tc.vmid = e->vmid;

    tc.stage1TranslGranule = e->stage1_tg;
    tc.stage2TranslGranule = e->stage2_tg;

    tc.t0sz = e->t0sz;
    tc.s2t0sz = e->s2t0sz;

    return true;
}

void
SMMUTranslationProcess::configCacheUpdate(Yield &yield,
                                          const TranslContext &tc)
{
    if (!smmu.configCacheEnable)
        return;

    ConfigCache::Entry e;
    e.valid = true;
    e.sid = request.sid;
    e.ssid = request.ssid;
    e.stage1_en = tc.stage1Enable;
    e.stage2_en = tc.stage2Enable;
    e.ttb0 = tc.ttb0;
    e.ttb1 = tc.ttb1;
    e.asid = tc.asid;
    e.httb = tc.httb;
    e.vmid = tc.vmid;
    e.stage1_tg = tc.stage1TranslGranule;
    e.stage2_tg = tc.stage2TranslGranule;
    e.t0sz = tc.t0sz;
    e.s2t0sz = tc.s2t0sz;

    doSemaphoreDown(yield, smmu.configSem);

    DPRINTF(SMMUv3, "Config upd  sid=%#x ssid=%#x\n", e.sid, e.ssid);

    smmu.configCache.store(e);

    doSemaphoreUp(smmu.configSem);
}

bool
SMMUTranslationProcess::findConfig(Yield &yield,
                                   TranslContext &tc,
                                   TranslResult &tr)
{
    tc.stage1Enable = false;
    tc.stage2Enable = false;

    StreamTableEntry ste;
    doReadSTE(yield, ste, request.sid);

    switch (ste.dw0.config) {
        case STE_CONFIG_BYPASS:
            break;

        case STE_CONFIG_STAGE1_ONLY:
            tc.stage1Enable = true;
            break;

        case STE_CONFIG_STAGE2_ONLY:
            tc.stage2Enable = true;
            break;

        case STE_CONFIG_STAGE1_AND_2:
            tc.stage1Enable = true;
            tc.stage2Enable = true;
            break;

        default:
            panic("Bad or unimplemented STE config %d\n",
                ste.dw0.config);
    }


    // Establish stage 2 context first since
    // Context Descriptors can be in IPA space.
    if (tc.stage2Enable) {
        tc.httb = ste.dw3.s2ttb << STE_S2TTB_SHIFT;
        tc.vmid = ste.dw2.s2vmid;
        tc.stage2TranslGranule = ste.dw2.s2tg;
        tc.s2t0sz = ste.dw2.s2t0sz;
    } else {
        tc.httb = 0xdeadbeef;
        tc.vmid = 0;
        tc.stage2TranslGranule = TRANS_GRANULE_INVALID;
        tc.s2t0sz = 0;
    }


    // Now fetch stage 1 config.
    if (context.stage1Enable) {
        ContextDescriptor cd;
        doReadCD(yield, cd, ste, request.sid, request.ssid);

        tc.ttb0 = cd.dw1.ttb0 << CD_TTB_SHIFT;
        tc.ttb1 = cd.dw2.ttb1 << CD_TTB_SHIFT;
        tc.asid = cd.dw0.asid;
        tc.stage1TranslGranule = cd.dw0.tg0;
        tc.t0sz = cd.dw0.t0sz;
    } else {
        tc.ttb0 = 0xcafebabe;
        tc.ttb1 = 0xcafed00d;
        tc.asid = 0;
        tc.stage1TranslGranule = TRANS_GRANULE_INVALID;
        tc.t0sz = 0;
    }

    return true;
}

void
SMMUTranslationProcess::walkCacheLookup(
        Yield &yield,
        const WalkCache::Entry *&walkEntry,
        Addr addr, uint16_t asid, uint16_t vmid,
        unsigned stage, unsigned level)
{
    const char *indent = stage==2 ? "  " : "";
    (void) indent; // this is only used in DPRINTFs

    const auto tg = stage == 1 ?
        GrainMap_tg0[context.stage1TranslGranule] :
        GrainMap_tg0[context.stage2TranslGranule];

    const auto *pt_ops = getPageTableOps(tg);

    unsigned walkCacheLevels =
        smmu.walkCacheEnable ?
            (stage == 1 ? smmu.walkCacheS1Levels : smmu.walkCacheS2Levels) :
            0;

    if ((1 << level) & walkCacheLevels) {
        doSemaphoreDown(yield, smmu.walkSem);
        doDelay(yield, smmu.walkLat);

        walkEntry = smmu.walkCache.lookup(addr, pt_ops->walkMask(level),
                                          asid, vmid, stage, level);

        if (walkEntry) {
            DPRINTF(SMMUv3, "%sWalkCache hit  va=%#x asid=%#x vmid=%#x "
                            "base=%#x (S%d, L%d)\n",
                    indent, addr, asid, vmid, walkEntry->pa, stage, level);
        } else {
            DPRINTF(SMMUv3, "%sWalkCache miss va=%#x asid=%#x vmid=%#x "
                            "(S%d, L%d)\n",
                    indent, addr, asid, vmid, stage, level);
        }

        doSemaphoreUp(smmu.walkSem);
    }
}

void
SMMUTranslationProcess::walkCacheUpdate(Yield &yield, Addr va,
                                        Addr vaMask, Addr pa,
                                        unsigned stage, unsigned level,
                                        bool leaf, uint8_t permissions)
{
    unsigned walkCacheLevels =
        stage == 1 ? smmu.walkCacheS1Levels : smmu.walkCacheS2Levels;

    if (smmu.walkCacheEnable && ((1<<level) & walkCacheLevels)) {
        WalkCache::Entry e;
        e.valid = true;
        e.va = va;
        e.vaMask = vaMask;
        e.asid = stage==1 ? context.asid : 0;
        e.vmid = context.vmid;
        e.stage = stage;
        e.level = level;
        e.leaf = leaf;
        e.pa = pa;
        e.permissions = permissions;

        doSemaphoreDown(yield, smmu.walkSem);

        DPRINTF(SMMUv3, "%sWalkCache upd  va=%#x mask=%#x asid=%#x vmid=%#x "
                        "tpa=%#x leaf=%s (S%d, L%d)\n",
                e.stage==2 ? "  " : "",
                e.va, e.vaMask, e.asid, e.vmid,
                e.pa, e.leaf, e.stage, e.level);

        smmu.walkCache.store(e);

        doSemaphoreUp(smmu.walkSem);
    }
}

/*
 * Please note:
 * This does not deal with the case where stage 1 page size
 * is larger than stage 2 page size.
 */
SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::walkStage1And2(Yield &yield, Addr addr,
                                       const PageTableOps *pt_ops,
                                       unsigned level, Addr walkPtr)
{
    PageTableOps::pte_t pte = 0;

    doSemaphoreDown(yield, smmu.cycleSem);
    doDelay(yield, Cycles(1));
    doSemaphoreUp(smmu.cycleSem);

    for (; level <= pt_ops->lastLevel(); level++) {
        Addr pte_addr = walkPtr + pt_ops->index(
            addr, level, 64 - context.t0sz);

        DPRINTF(SMMUv3, "Fetching S1 L%d PTE from pa=%#08x\n",
                level, pte_addr);

        doReadPTE(yield, addr, pte_addr, &pte, 1, level);

        DPRINTF(SMMUv3, "Got S1 L%d PTE=%#x from pa=%#08x\n",
                level, pte, pte_addr);

        doSemaphoreDown(yield, smmu.cycleSem);
        doDelay(yield, Cycles(1));
        doSemaphoreUp(smmu.cycleSem);

        bool valid = pt_ops->isValid(pte, level);
        bool leaf  = pt_ops->isLeaf(pte, level);

        if (!valid) {
            DPRINTF(SMMUv3, "S1 PTE not valid - fault\n");

            TranslResult tr;
            tr.fault = FAULT_TRANSLATION;
            return tr;
        }

        if (valid && leaf && request.isWrite &&
            !pt_ops->isWritable(pte, level, false))
        {
            DPRINTF(SMMUv3, "S1 page not writable - fault\n");

            TranslResult tr;
            tr.fault = FAULT_PERMISSION;
            return tr;
        }

        walkPtr = pt_ops->nextLevelPointer(pte, level);

        if (leaf)
            break;

        if (context.stage2Enable) {
            TranslResult s2tr = translateStage2(yield, walkPtr, false);
            if (s2tr.fault != FAULT_NONE)
                return s2tr;

            walkPtr = s2tr.addr;
        }

        walkCacheUpdate(yield, addr, pt_ops->walkMask(level), walkPtr,
                        1, level, leaf, 0);
    }

    TranslResult tr;
    tr.fault    = FAULT_NONE;
    tr.addrMask = pt_ops->pageMask(pte, level);
    tr.addr     = walkPtr + (addr & ~tr.addrMask);
    tr.writable = pt_ops->isWritable(pte, level, false);

    if (context.stage2Enable) {
        TranslResult s2tr = translateStage2(yield, tr.addr, true);
        if (s2tr.fault != FAULT_NONE)
            return s2tr;

        tr = combineTranslations(tr, s2tr);
    }

    walkCacheUpdate(yield, addr, tr.addrMask, walkPtr,
                    1, level, true, tr.writable);

    return tr;
}

SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::walkStage2(Yield &yield, Addr addr, bool final_tr,
                                   const PageTableOps *pt_ops,
                                   unsigned level, Addr walkPtr)
{
    PageTableOps::pte_t pte;

    doSemaphoreDown(yield, smmu.cycleSem);
    doDelay(yield, Cycles(1));
    doSemaphoreUp(smmu.cycleSem);

    for (; level <= pt_ops->lastLevel(); level++) {
        Addr pte_addr = walkPtr + pt_ops->index(
            addr, level, 64 - context.s2t0sz);

        DPRINTF(SMMUv3, "  Fetching S2 L%d PTE from pa=%#08x\n",
                level, pte_addr);

        doReadPTE(yield, addr, pte_addr, &pte, 2, level);

        DPRINTF(SMMUv3, "  Got S2 L%d PTE=%#x from pa=%#08x\n",
                level, pte, pte_addr);

        doSemaphoreDown(yield, smmu.cycleSem);
        doDelay(yield, Cycles(1));
        doSemaphoreUp(smmu.cycleSem);

        bool valid = pt_ops->isValid(pte, level);
        bool leaf  = pt_ops->isLeaf(pte, level);

        if (!valid) {
            DPRINTF(SMMUv3, "  S2 PTE not valid - fault\n");

            TranslResult tr;
            tr.fault = FAULT_TRANSLATION;
            return tr;
        }

        if (valid && leaf && request.isWrite &&
            !pt_ops->isWritable(pte, level, true))
        {
            DPRINTF(SMMUv3, "  S2 PTE not writable = fault\n");

            TranslResult tr;
            tr.fault = FAULT_PERMISSION;
            return tr;
        }

        walkPtr = pt_ops->nextLevelPointer(pte, level);

        if (final_tr || smmu.walkCacheNonfinalEnable)
            walkCacheUpdate(yield, addr, pt_ops->walkMask(level), walkPtr,
                            2, level, leaf,
                            leaf ? pt_ops->isWritable(pte, level, true) : 0);
        if (leaf)
            break;
    }

    TranslResult tr;
    tr.fault    = FAULT_NONE;
    tr.addrMask = pt_ops->pageMask(pte, level);
    tr.addr     = walkPtr + (addr & ~tr.addrMask);
    tr.writable = pt_ops->isWritable(pte, level, true);

    return tr;
}

SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::translateStage1And2(Yield &yield, Addr addr)
{
    const auto tg = GrainMap_tg0[context.stage1TranslGranule];
    const auto *pt_ops = getPageTableOps(tg);

    const WalkCache::Entry *walk_ep = NULL;
    unsigned level;

    // Level here is actually (level+1) so we can count down
    // to 0 using unsigned int.
    for (level = pt_ops->lastLevel() + 1;
        level > pt_ops->firstLevel(context.t0sz);
        level--)
    {
        walkCacheLookup(yield, walk_ep, addr,
                        context.asid, context.vmid, 1, level-1);

        if (walk_ep)
            break;
    }

    // Correct level (see above).
    level -= 1;

    TranslResult tr;
    if (walk_ep) {
        if (walk_ep->leaf) {
            tr.fault    = FAULT_NONE;
            tr.addr     = walk_ep->pa + (addr & ~walk_ep->vaMask);
            tr.addrMask = walk_ep->vaMask;
            tr.writable = walk_ep->permissions;
        } else {
            tr = walkStage1And2(yield, addr, pt_ops, level+1, walk_ep->pa);
        }
    } else {
        Addr table_addr = context.ttb0;
        if (context.stage2Enable) {
            TranslResult s2tr = translateStage2(yield, table_addr, false);
            if (s2tr.fault != FAULT_NONE)
                return s2tr;

            table_addr = s2tr.addr;
        }

        tr = walkStage1And2(yield, addr, pt_ops,
                            pt_ops->firstLevel(context.t0sz),
                            table_addr);
    }

    if (tr.fault == FAULT_NONE)
        DPRINTF(SMMUv3, "Translated vaddr %#x to paddr %#x\n", addr, tr.addr);

    return tr;
}

SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::translateStage2(Yield &yield, Addr addr, bool final_tr)
{
    const auto tg = GrainMap_tg0[context.stage2TranslGranule];
    const auto *pt_ops = getPageTableOps(tg);

    const IPACache::Entry *ipa_ep = NULL;
    if (smmu.ipaCacheEnable) {
        doSemaphoreDown(yield, smmu.ipaSem);
        doDelay(yield, smmu.ipaLat);
        ipa_ep = smmu.ipaCache.lookup(addr, context.vmid);
        doSemaphoreUp(smmu.ipaSem);
    }

    if (ipa_ep) {
        TranslResult tr;
        tr.fault    = FAULT_NONE;
        tr.addr     = ipa_ep->pa + (addr & ~ipa_ep->ipaMask);
        tr.addrMask = ipa_ep->ipaMask;
        tr.writable = ipa_ep->permissions;

        DPRINTF(SMMUv3, "  IPACache hit  ipa=%#x vmid=%#x pa=%#x\n",
            addr, context.vmid, tr.addr);

        return tr;
    } else if (smmu.ipaCacheEnable) {
        DPRINTF(SMMUv3, "  IPACache miss ipa=%#x vmid=%#x\n",
                addr, context.vmid);
    }

    const WalkCache::Entry *walk_ep = NULL;
    unsigned level = pt_ops->firstLevel(context.s2t0sz);

    if (final_tr || smmu.walkCacheNonfinalEnable) {
        // Level here is actually (level+1) so we can count down
        // to 0 using unsigned int.
        for (level = pt_ops->lastLevel() + 1;
            level > pt_ops->firstLevel(context.s2t0sz);
            level--)
        {
            walkCacheLookup(yield, walk_ep, addr,
                            0, context.vmid, 2, level-1);

            if (walk_ep)
                break;
        }

        // Correct level (see above).
        level -= 1;
    }

    TranslResult tr;
    if (walk_ep) {
        if (walk_ep->leaf) {
            tr.fault    = FAULT_NONE;
            tr.addr     = walk_ep->pa + (addr & ~walk_ep->vaMask);
            tr.addrMask = walk_ep->vaMask;
            tr.writable = walk_ep->permissions;
        } else {
            tr = walkStage2(yield, addr, final_tr, pt_ops,
                            level + 1, walk_ep->pa);
        }
    } else {
        tr = walkStage2(yield, addr, final_tr, pt_ops,
                        pt_ops->firstLevel(context.s2t0sz),
                        context.httb);
    }

    if (tr.fault == FAULT_NONE)
        DPRINTF(SMMUv3, "  Translated %saddr %#x to paddr %#x\n",
            context.stage1Enable ? "ip" : "v", addr, tr.addr);

    if (smmu.ipaCacheEnable) {
        IPACache::Entry e;
        e.valid = true;
        e.ipaMask = tr.addrMask;
        e.ipa = addr & e.ipaMask;
        e.pa = tr.addr & tr.addrMask;
        e.permissions = tr.writable;
        e.vmid = context.vmid;

        doSemaphoreDown(yield, smmu.ipaSem);
        smmu.ipaCache.store(e);
        doSemaphoreUp(smmu.ipaSem);
    }

    return tr;
}

SMMUTranslationProcess::TranslResult
SMMUTranslationProcess::combineTranslations(const TranslResult &s1tr,
                                            const TranslResult &s2tr) const
{
    if (s2tr.fault != FAULT_NONE)
        return s2tr;

    assert(s1tr.fault == FAULT_NONE);

    TranslResult tr;
    tr.fault    = FAULT_NONE;
    tr.addr     = s2tr.addr;
    tr.addrMask = s1tr.addrMask | s2tr.addrMask;
    tr.writable = s1tr.writable & s2tr.writable;

    return tr;
}

bool
SMMUTranslationProcess::hazard4kCheck()
{
    Addr addr4k = request.addr & ~0xfffULL;

    for (auto it = ifc.duplicateReqs.begin();
         it != ifc.duplicateReqs.end();
         ++it)
    {
        Addr other4k = (*it)->request.addr & ~0xfffULL;
        if (addr4k == other4k)
            return true;
    }

    return false;
}

void
SMMUTranslationProcess::hazard4kRegister()
{
    DPRINTF(SMMUv3Hazard, "4kReg:  p=%p a4k=%#x\n",
            this, request.addr & ~0xfffULL);

    ifc.duplicateReqs.push_back(this);
}

void
SMMUTranslationProcess::hazard4kHold(Yield &yield)
{
    Addr addr4k = request.addr & ~0xfffULL;

    bool found_hazard;

    do {
        found_hazard = false;

        for (auto it = ifc.duplicateReqs.begin();
             it!=ifc.duplicateReqs.end() && *it!=this;
             ++it)
        {
            Addr other4k = (*it)->request.addr & ~0xfffULL;

            DPRINTF(SMMUv3Hazard, "4kHold: p=%p a4k=%#x Q: p=%p a4k=%#x\n",
                    this, addr4k, *it, other4k);

            if (addr4k == other4k) {
                DPRINTF(SMMUv3Hazard,
                        "4kHold: p=%p a4k=%#x WAIT on p=%p a4k=%#x\n",
                        this, addr4k, *it, other4k);

                doWaitForSignal(yield, ifc.duplicateReqRemoved);

                DPRINTF(SMMUv3Hazard, "4kHold: p=%p a4k=%#x RESUME\n",
                        this, addr4k);

                // This is to avoid checking *it!=this after doWaitForSignal()
                // since it could have been deleted.
                found_hazard = true;
                break;
            }
        }
    } while (found_hazard);
}

void
SMMUTranslationProcess::hazard4kRelease()
{
    DPRINTF(SMMUv3Hazard, "4kRel:  p=%p a4k=%#x\n",
            this, request.addr & ~0xfffULL);

    std::list<SMMUTranslationProcess *>::iterator it;

    for (it = ifc.duplicateReqs.begin(); it != ifc.duplicateReqs.end(); ++it)
        if (*it == this)
            break;

    if (it == ifc.duplicateReqs.end())
        panic("hazard4kRelease: request not found");

    ifc.duplicateReqs.erase(it);

    doBroadcastSignal(ifc.duplicateReqRemoved);
}

void
SMMUTranslationProcess::hazardIdRegister()
{
    auto orderId = AMBA::orderId(request.pkt);

    DPRINTF(SMMUv3Hazard, "IdReg:  p=%p oid=%d\n", this, orderId);

    assert(orderId < SMMU_MAX_TRANS_ID);

    std::list<SMMUTranslationProcess *> &depReqs =
        request.isWrite ?
            ifc.dependentWrites[orderId] : ifc.dependentReads[orderId];
    depReqs.push_back(this);
}

void
SMMUTranslationProcess::hazardIdHold(Yield &yield)
{
    auto orderId = AMBA::orderId(request.pkt);

    DPRINTF(SMMUv3Hazard, "IdHold: p=%p oid=%d\n", this, orderId);

    std::list<SMMUTranslationProcess *> &depReqs =
        request.isWrite ?
            ifc.dependentWrites[orderId] : ifc.dependentReads[orderId];
    std::list<SMMUTranslationProcess *>::iterator it;

    bool found_hazard;

    do {
        found_hazard = false;

        for (auto it = depReqs.begin(); it!=depReqs.end() && *it!=this; ++it) {
            DPRINTF(SMMUv3Hazard, "IdHold: p=%p oid=%d Q: %p\n",
                    this, orderId, *it);

            if (AMBA::orderId((*it)->request.pkt) == orderId) {
                DPRINTF(SMMUv3Hazard, "IdHold: p=%p oid=%d WAIT on=%p\n",
                        this, orderId, *it);

                doWaitForSignal(yield, ifc.dependentReqRemoved);

                DPRINTF(SMMUv3Hazard, "IdHold: p=%p oid=%d RESUME\n",
                        this, orderId);

                // This is to avoid checking *it!=this after doWaitForSignal()
                // since it could have been deleted.
                found_hazard = true;
                break;
            }
        }
    } while (found_hazard);
}

void
SMMUTranslationProcess::hazardIdRelease()
{
    auto orderId = AMBA::orderId(request.pkt);

    DPRINTF(SMMUv3Hazard, "IdRel:  p=%p oid=%d\n", this, orderId);

    std::list<SMMUTranslationProcess *> &depReqs =
        request.isWrite ?
            ifc.dependentWrites[orderId] : ifc.dependentReads[orderId];
    std::list<SMMUTranslationProcess *>::iterator it;

    for (it = depReqs.begin(); it != depReqs.end(); ++it) {
        if (*it == this)
            break;
    }

    if (it == depReqs.end())
        panic("hazardIdRelease: request not found");

    depReqs.erase(it);

    doBroadcastSignal(ifc.dependentReqRemoved);
}

void
SMMUTranslationProcess::issuePrefetch(Addr addr)
{
    if (!smmu.system.isTimingMode())
        return;

    if (!ifc.prefetchEnable || ifc.xlateSlotsRemaining == 0)
        return;

    std::string proc_name = csprintf("%sprf", name());
    SMMUTranslationProcess *proc =
        new SMMUTranslationProcess(proc_name, smmu, ifc);

    proc->beginTransaction(
            SMMUTranslRequest::prefetch(addr, request.sid, request.ssid));
    proc->scheduleWakeup(smmu.clockEdge(Cycles(1)));
}

void
SMMUTranslationProcess::completeTransaction(Yield &yield,
                                            const TranslResult &tr)
{
    assert(tr.fault == FAULT_NONE);

    unsigned numRequestorBeats = request.isWrite ?
        (request.size + (smmu.requestPortWidth-1))
            / smmu.requestPortWidth :
        1;

    doSemaphoreDown(yield, smmu.requestPortSem);
    doDelay(yield, Cycles(numRequestorBeats));
    doSemaphoreUp(smmu.requestPortSem);


    smmu.stats.translationTimeDist.sample(curTick() - recvTick);
    ifc.xlateSlotsRemaining++;
    if (!request.isAtsRequest && request.isWrite)
        ifc.wrBufSlotsRemaining +=
            (request.size + (ifc.portWidth-1)) / ifc.portWidth;

    smmu.scheduleDeviceRetries();


    SMMUAction a;

    if (request.isAtsRequest) {
        a.type = ACTION_SEND_RESP_ATS;

        if (smmu.system.isAtomicMode()) {
            request.pkt->makeAtomicResponse();
        } else if (smmu.system.isTimingMode()) {
            request.pkt->makeTimingResponse();
        } else {
            panic("Not in atomic or timing mode");
        }
    } else {
        a.type = ACTION_SEND_REQ_FINAL;
        a.ifc = &ifc;
    }

    a.pkt = request.pkt;
    a.delay = 0;

    a.pkt->setAddr(tr.addr);
    a.pkt->req->setPaddr(tr.addr);

    yield(a);

    if (!request.isAtsRequest) {
        PacketPtr pkt = yield.get();
        pkt->setAddr(request.addr);

        a.type = ACTION_SEND_RESP;
        a.pkt = pkt;
        a.ifc = &ifc;
        a.delay = 0;
        yield(a);
    }
}

void
SMMUTranslationProcess::completePrefetch(Yield &yield)
{
    ifc.xlateSlotsRemaining++;

    SMMUAction a;
    a.type = ACTION_TERMINATE;
    a.pkt = NULL;
    a.ifc = &ifc;
    a.delay = 0;
    yield(a);
}

void
SMMUTranslationProcess::sendEvent(Yield &yield, const SMMUEvent &ev)
{
    int sizeMask = mask(smmu.regs.eventq_base & Q_BASE_SIZE_MASK);

    if (((smmu.regs.eventq_prod+1) & sizeMask) ==
            (smmu.regs.eventq_cons & sizeMask))
        panic("Event queue full - aborting\n");

    Addr event_addr =
        (smmu.regs.eventq_base & Q_BASE_ADDR_MASK) +
        (smmu.regs.eventq_prod & sizeMask) * sizeof(ev);

    DPRINTF(SMMUv3, "Sending event to addr=%#08x (pos=%d): type=%#x stag=%#x "
        "flags=%#x sid=%#x ssid=%#x va=%#08x ipa=%#x\n",
        event_addr, smmu.regs.eventq_prod, ev.type, ev.stag,
        ev.flags, ev.streamId, ev.substreamId, ev.va, ev.ipa);

    // This deliberately resets the overflow field in eventq_prod!
    smmu.regs.eventq_prod = (smmu.regs.eventq_prod + 1) & sizeMask;

    doWrite(yield, event_addr, &ev, sizeof(ev));

    if (!(smmu.regs.eventq_irq_cfg0 & E_BASE_ENABLE_MASK))
        panic("eventq msi not enabled\n");

    doWrite(yield, smmu.regs.eventq_irq_cfg0 & E_BASE_ADDR_MASK,
            &smmu.regs.eventq_irq_cfg1, sizeof(smmu.regs.eventq_irq_cfg1));
}

void
SMMUTranslationProcess::doReadSTE(Yield &yield,
                                  StreamTableEntry &ste,
                                  uint32_t sid)
{
    unsigned max_sid = 1 << (smmu.regs.strtab_base_cfg & ST_CFG_SIZE_MASK);
    if (sid >= max_sid)
        panic("SID %#x out of range, max=%#x", sid, max_sid);

    Addr ste_addr;

    if ((smmu.regs.strtab_base_cfg & ST_CFG_FMT_MASK) == ST_CFG_FMT_2LEVEL) {
        unsigned split =
            (smmu.regs.strtab_base_cfg & ST_CFG_SPLIT_MASK) >> ST_CFG_SPLIT_SHIFT;

        if (split!= 7 && split!=8 && split!=16)
            panic("Invalid stream table split %d", split);

        uint64_t l2_ptr;
        uint64_t l2_addr =
            (smmu.regs.strtab_base & VMT_BASE_ADDR_MASK) +
            bits(sid, 32, split) * sizeof(l2_ptr);

        DPRINTF(SMMUv3, "Read L1STE at %#x\n", l2_addr);

        doReadConfig(yield, l2_addr, &l2_ptr, sizeof(l2_ptr), sid, 0);

        DPRINTF(SMMUv3, "Got L1STE L1 at %#x: 0x%016x\n", l2_addr, l2_ptr);

        unsigned span = l2_ptr & ST_L2_SPAN_MASK;
        if (span == 0)
            panic("Invalid level 1 stream table descriptor");

        unsigned index = bits(sid, split-1, 0);
        if (index >= (1 << span))
            panic("StreamID %d out of level 1 descriptor range %d",
                  sid, 1<<span);

        ste_addr = (l2_ptr & ST_L2_ADDR_MASK) + index * sizeof(ste);

        smmu.stats.steL1Fetches++;
    } else if ((smmu.regs.strtab_base_cfg & ST_CFG_FMT_MASK)
                                                      == ST_CFG_FMT_LINEAR) {
        ste_addr =
            (smmu.regs.strtab_base & VMT_BASE_ADDR_MASK) + sid * sizeof(ste);
    } else {
        panic("Invalid stream table format");
    }

    DPRINTF(SMMUv3, "Read STE at %#x\n", ste_addr);

    doReadConfig(yield, ste_addr, &ste, sizeof(ste), sid, 0);

    DPRINTF(SMMUv3, "Got STE at %#x [0]: 0x%016x\n", ste_addr, ste.dw0);
    DPRINTF(SMMUv3, "    STE at %#x [1]: 0x%016x\n", ste_addr, ste.dw1);
    DPRINTF(SMMUv3, "    STE at %#x [2]: 0x%016x\n", ste_addr, ste.dw2);
    DPRINTF(SMMUv3, "    STE at %#x [3]: 0x%016x\n", ste_addr, ste.dw3);
    DPRINTF(SMMUv3, "    STE at %#x [4]: 0x%016x\n", ste_addr, ste._pad[0]);
    DPRINTF(SMMUv3, "    STE at %#x [5]: 0x%016x\n", ste_addr, ste._pad[1]);
    DPRINTF(SMMUv3, "    STE at %#x [6]: 0x%016x\n", ste_addr, ste._pad[2]);
    DPRINTF(SMMUv3, "    STE at %#x [7]: 0x%016x\n", ste_addr, ste._pad[3]);

    if (!ste.dw0.valid)
        panic("STE @ %#x not valid\n", ste_addr);

    smmu.stats.steFetches++;
}

void
SMMUTranslationProcess::doReadCD(Yield &yield,
                                 ContextDescriptor &cd,
                                 const StreamTableEntry &ste,
                                 uint32_t sid, uint32_t ssid)
{
    Addr cd_addr = 0;

    if (ste.dw0.s1cdmax == 0) {
        cd_addr = ste.dw0.s1ctxptr << ST_CD_ADDR_SHIFT;
    } else {
        unsigned max_ssid = 1 << ste.dw0.s1cdmax;
        if (ssid >= max_ssid)
            panic("SSID %#x out of range, max=%#x", ssid, max_ssid);

        if (ste.dw0.s1fmt==STAGE1_CFG_2L_4K ||
            ste.dw0.s1fmt==STAGE1_CFG_2L_64K)
        {
            unsigned split = ste.dw0.s1fmt==STAGE1_CFG_2L_4K ? 7 : 11;

            uint64_t l2_ptr;
            uint64_t l2_addr = (ste.dw0.s1ctxptr << ST_CD_ADDR_SHIFT) +
                bits(ssid, 24, split) * sizeof(l2_ptr);

            if (context.stage2Enable)
                l2_addr = translateStage2(yield, l2_addr, false).addr;

            DPRINTF(SMMUv3, "Read L1CD at %#x\n", l2_addr);

            doReadConfig(yield, l2_addr, &l2_ptr, sizeof(l2_ptr), sid, ssid);

            DPRINTF(SMMUv3, "Got L1CD at %#x: 0x%016x\n", l2_addr, l2_ptr);

            cd_addr = l2_ptr + bits(ssid, split-1, 0) * sizeof(cd);

            smmu.stats.cdL1Fetches++;
        } else if (ste.dw0.s1fmt == STAGE1_CFG_1L) {
            cd_addr = (ste.dw0.s1ctxptr << ST_CD_ADDR_SHIFT) + ssid*sizeof(cd);
        }
    }

    if (context.stage2Enable)
        cd_addr = translateStage2(yield, cd_addr, false).addr;

    DPRINTF(SMMUv3, "Read CD at %#x\n", cd_addr);

    doReadConfig(yield, cd_addr, &cd, sizeof(cd), sid, ssid);

    DPRINTF(SMMUv3, "Got CD at %#x [0]: 0x%016x\n", cd_addr, cd.dw0);
    DPRINTF(SMMUv3, "    CD at %#x [1]: 0x%016x\n", cd_addr, cd.dw1);
    DPRINTF(SMMUv3, "    CD at %#x [2]: 0x%016x\n", cd_addr, cd.dw2);
    DPRINTF(SMMUv3, "    CD at %#x [3]: 0x%016x\n", cd_addr, cd.mair);
    DPRINTF(SMMUv3, "    CD at %#x [4]: 0x%016x\n", cd_addr, cd.amair);
    DPRINTF(SMMUv3, "    CD at %#x [5]: 0x%016x\n", cd_addr, cd._pad[0]);
    DPRINTF(SMMUv3, "    CD at %#x [6]: 0x%016x\n", cd_addr, cd._pad[1]);
    DPRINTF(SMMUv3, "    CD at %#x [7]: 0x%016x\n", cd_addr, cd._pad[2]);


    if (!cd.dw0.valid)
        panic("CD @ %#x not valid\n", cd_addr);

    smmu.stats.cdFetches++;
}

void
SMMUTranslationProcess::doReadConfig(Yield &yield, Addr addr,
                                     void *ptr, size_t size,
                                     uint32_t sid, uint32_t ssid)
{
    doRead(yield, addr, ptr, size);
}

void
SMMUTranslationProcess::doReadPTE(Yield &yield, Addr va, Addr addr,
                                  void *ptr, unsigned stage,
                                  unsigned level)
{
    size_t pte_size = sizeof(PageTableOps::pte_t);

    Addr mask = pte_size - 1;
    Addr base = addr & ~mask;

    doRead(yield, base, ptr, pte_size);
}

} // namespace gem5
