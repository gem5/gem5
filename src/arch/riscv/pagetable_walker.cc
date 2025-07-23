/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
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
 */

#include "arch/riscv/pagetable_walker.hh"

#include <memory>

#include "arch/riscv/faults.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/tlb.hh"
#include "base/bitfield.hh"
#include "base/trie.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/PageTableWalker.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace gem5
{

namespace RiscvISA {

Fault
Walker::start(ThreadContext * _tc, BaseMMU::Translation *_translation,
    const RequestPtr &_req, BaseMMU::Mode _mode, TlbEntry* result_entry)
{
    // TODO: in timing mode, instead of blocking when there are other
    // outstanding requests, see if this request can be coalesced with
    // another one (i.e. either coalesce or start walk)
    WalkerState * newState = new WalkerState(this, _translation, _req);
    newState->initState(_tc, _mode, sys->isTimingMode());
    if (currStates.size()) {
        assert(newState->isTiming());
        DPRINTF(PageTableWalker, "Walks in progress: %d\n", currStates.size());
        currStates.push_back(newState);
        return NoFault;
    } else {
        currStates.push_back(newState);
        Fault fault = newState->walk();

        // Keep the resulting TLB entry
        // in some cases we might need to use the result
        // but not insert to the TLB, so we can't look it up if we return!
        if (result_entry)
            *result_entry = newState->entry;

        // In functional mode, always pop the state
        // In timing we must pop the state in the case of an early fault!
        if (fault != NoFault || !newState->isTiming())
        {
            currStates.pop_front();
            delete newState;
        }
        return fault;
    }
}

Fault
Walker::startFunctional(ThreadContext * _tc, Addr &addr, unsigned &logBytes,
              BaseMMU::Mode _mode)
{
    funcState.initState(_tc, _mode);
    return funcState.startFunctional(addr, logBytes);
}

bool
Walker::WalkerPort::recvTimingResp(PacketPtr pkt)
{
    return walker->recvTimingResp(pkt);
}

bool
Walker::recvTimingResp(PacketPtr pkt)
{
    WalkerSenderState * senderState =
        dynamic_cast<WalkerSenderState *>(pkt->popSenderState());
    WalkerState * senderWalk = senderState->senderWalk;
    bool walkComplete = senderWalk->recvPacket(pkt);
    delete senderState;
    if (walkComplete) {
        std::list<WalkerState *>::iterator iter;
        for (iter = currStates.begin(); iter != currStates.end(); iter++) {
            WalkerState * walkerState = *(iter);
            if (walkerState == senderWalk) {
                iter = currStates.erase(iter);
                break;
            }
        }
        delete senderWalk;
        // Since we block requests when another is outstanding, we
        // need to check if there is a waiting request to be serviced
        if (currStates.size() && !startWalkWrapperEvent.scheduled())
            // delay sending any new requests until we are finished
            // with the responses
            schedule(startWalkWrapperEvent, clockEdge());
    }
    return true;
}

void
Walker::WalkerPort::recvReqRetry()
{
    walker->recvReqRetry();
}

void
Walker::recvReqRetry()
{
    std::list<WalkerState *>::iterator iter;
    for (iter = currStates.begin(); iter != currStates.end(); iter++) {
        WalkerState * walkerState = *(iter);
        if (walkerState->isRetrying()) {
            walkerState->retry();
        }
    }
}

bool Walker::sendTiming(WalkerState* sendingState, PacketPtr pkt)
{
    WalkerSenderState* walker_state = new WalkerSenderState(sendingState);
    pkt->pushSenderState(walker_state);
    if (port.sendTimingReq(pkt)) {
        return true;
    } else {
        // undo the adding of the sender state and delete it, as we
        // will do it again the next time we attempt to send it
        pkt->popSenderState();
        delete walker_state;
        return false;
    }

}

Port &
Walker::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
Walker::WalkerState::initState(ThreadContext * _tc,
        BaseMMU::Mode _mode, bool _isTiming)
{
    assert(state == Ready);
    started = false;
    tc = _tc;
    mode = _mode;
    timing = _isTiming;
    // fetch these now in case they change during the walk
    memaccess = functional ?
        walker->tlb->getMemAccessInfo(tc, mode, (Request::ArchFlagsType)0):
        walker->tlb->getMemAccessInfo(tc, mode, req->getArchFlags());
    pmode = memaccess.priv;
    status = tc->readMiscReg(MISCREG_STATUS);
    MISA misa = tc->readMiscReg(MISCREG_ISA);

    // Find SATP
    // If no rvh or effective V = 0, base is SATP
    // otherwise base is VSATP (effective V=1)
    satp = (!misa.rvh || !memaccess.virt) ?
            tc->readMiscReg(MISCREG_SATP) :
            tc->readMiscReg(MISCREG_VSATP);

    // If effective V = 1, also read HGATP for
    // G-stage because we will perform a two-stage translation.
    hgatp = (misa.rvh && memaccess.virt) ?
            tc->readMiscReg(MISCREG_HGATP) :
            (RegVal)0;

    // TODO move this somewhere else
    // VSATP mode might be bare, but we still
    // will have to go through G-stage
    // assert(satp.mode == AddrXlateMode::SV39);

    // If functional entry.vaddr will be set
    // in start functional (req == NULL)
    entry.vaddr = functional ?
        (Addr)0 :
        req->getVaddr();

    entry.asid = satp.asid;
}

void
Walker::startWalkWrapper()
{
    unsigned num_squashed = 0;
    WalkerState *currState = currStates.front();

    // check if we get a tlb hit to skip the walk
    Addr vaddr = Addr(sext<SV39_VADDR_BITS>(currState->req->getVaddr()));
    Addr vpn = getVPNFromVAddr(vaddr, currState->satp.mode);
    TlbEntry *e = tlb->lookup(vpn, currState->satp.asid, currState->mode,
                              true);
    Fault fault = NoFault;
    if (e) {
       fault = tlb->checkPermissions(currState->tc, currState->memaccess,
                            e->vaddr, currState->mode, e->pte);
    }

    while ((num_squashed < numSquashable) && currState &&
           (currState->translation->squashed() || (e && fault == NoFault))) {
        currStates.pop_front();
        num_squashed++;

        DPRINTF(PageTableWalker, "Squashing table walk for address %#x\n",
            currState->req->getVaddr());

        // finish the translation which will delete the translation object
        if (currState->translation->squashed()) {
            currState->translation->finish(
                std::make_shared<UnimpFault>("Squashed Inst"),
                currState->req, currState->tc, currState->mode);
        } else {
            tlb->translateTiming(currState->req, currState->tc,
                                 currState->translation, currState->mode);
        }

        // delete the current request if there are no inflight packets.
        // if there is something in flight, delete when the packets are
        // received and inflight is zero.
        if (currState->numInflight() == 0) {
            delete currState;
        } else {
            currState->squash();
        }

        // check the next translation request, if it exists
        if (currStates.size()) {
            currState = currStates.front();
            vaddr = Addr(sext<SV39_VADDR_BITS>(currState->req->getVaddr()));
            Addr vpn = getVPNFromVAddr(vaddr, currState->satp.mode);
            e = tlb->lookup(vpn, currState->satp.asid, currState->mode,
                            true);
            if (e) {
                fault = tlb->checkPermissions(currState->tc,
                    currState->memaccess, e->vaddr, currState->mode, e->pte);
            }
        } else {
            currState = NULL;
        }
    }
    if (currState && !currState->wasStarted()) {
        if (!e || fault != NoFault) {
            Fault timingFault = currState->walk();
            if (timingFault != NoFault) {
                currStates.pop_front();
                delete currState;
                currState = NULL;
            }
        }
        else {
            schedule(startWalkWrapperEvent, clockEdge(Cycles(1)));
        }
    }
}

Fault
Walker::WalkerState::walkGStage(Addr guest_paddr, Addr& host_paddr)
{
    Fault fault = NoFault;
    curstage = XlateStage::GSTAGE;


    // reset gresult in case we were called again
    // in a two stage translation
    gresult.reset();
    gresult.vaddr = guest_paddr;

    gstate = Translate;
    nextgState = Ready;


    const int maxgpabits = SV39_LEVELS * SV39_LEVEL_BITS +
                    SV39X4_WIDENED_BITS + PageShift;
    Addr maxgpa = mask(maxgpabits);

    // If there is a bit beyond maxgpa, throw pf
    if (guest_paddr & ~maxgpa) {
        return pageFault();
    }

    // If there is another read packet,
    // deallocate it, gstage creates a new packet
    if (read) {
        delete read;
        read = nullptr;
    }

    Addr pte_addr = setupWalk(guest_paddr);
    read = createReqPacket(pte_addr, MemCmd::ReadReq, sizeof(PTESv39));
    glevel = SV39_LEVELS - 1;

    // TODO THE TIMING PATH IS UNTESTED
    if (timing) {
        panic("unimpl");
        nextgState = gstate;
        gstate = Waiting;
        timingFault = NoFault;
        sendPackets();
    }
    else {
        do {
            walker->port.sendAtomic(read);
            PacketPtr write = NULL;
            fault = stepWalkGStage(write);
            assert(fault == NoFault || read == NULL);
            gstate = nextgState;
            nextgState = Ready;
            if (write) {
                walker->port.sendAtomic(write);
            }
        } while (read);

        if (fault) {
            return fault;
        }

        // In GStageOnly result is in entry (which is put in TLB)
        // otherwise it's a two stage walk so result is in gresult
        // which is discarded after.
        Addr ppn = walkType == GstageOnly ? entry.paddr : gresult.paddr;
        Addr vpn = guest_paddr >> PageShift;
        Addr vpn_bits = vpn & mask(glevel * SV39_LEVEL_BITS);

        // Update gresult
        gresult.paddr = (ppn | vpn_bits);

        host_paddr = ((ppn | vpn_bits) << PageShift) |
                    (guest_paddr & mask(PageShift));

        gstate = Ready;
        nextgState = Waiting;
    }
    return fault;
}

Fault
Walker::WalkerState::walk()
{
    Fault fault = NoFault;
    assert(!started);
    started = true;
    state = Translate;
    nextState = Ready;

    // This is the vaddr to walk for
    Addr vaddr = entry.vaddr;

    // Decide the type of walk to perform
    // When memaccess is virtual, GStage is enabled
    if (satp.mode == AddrXlateMode::BARE && memaccess.virt) {
        // In this case VSATP (== satp) is bare and
        // we do G-stage translation only
        Addr paddr;
        walkType = WalkType::GstageOnly;
        fault = walkGStage(vaddr, paddr);
    }
    else if (memaccess.virt) {
        walkType = WalkType::TwoStage;
        fault = walkTwoStage(vaddr);
    }
    else {
        walkType = WalkType::OneStage;
        fault = walkOneStage(vaddr);
    }

    return fault;
}


Fault
Walker::WalkerState::walkOneStage(Addr vaddr)
{

    curstage = XlateStage::FIRST_STAGE;

    // Make sure MSBS are the same
    // riscv-privileged-20211203 page 84
    auto mask_for_msbs = mask(64 - SV39_VADDR_BITS);
    auto msbs = bits(vaddr, 63, SV39_VADDR_BITS);
    if (msbs != 0 && msbs != mask_for_msbs) {
        return pageFault();
    }

    Addr pte_addr = setupWalk(vaddr);
    level = SV39_LEVELS - 1;
    // Create physical request for first_pte_addr
    // This is a host physical address
    // In two-stage this gets discarded?
    read = createReqPacket(pte_addr,
            MemCmd::ReadReq, sizeof(PTESv39));

    if (timing)
    {
        MISA misa = tc->readMiscReg(MISCREG_ISA);
        panic_if(misa.rvh, "Timing walks are not supported with h extension");
        nextState = state;
        state = Waiting;
        timingFault = NoFault;
        // DO GSTAGE HERE IF NEEDED AND THEN DO PACKETS FOR *PTE
        sendPackets();
        return NoFault;
    }

    Fault fault = NoFault;
    do
    {
        if (functional) {
            walker->port.sendFunctional(read);
        }
        else {
            walker->port.sendAtomic(read);
        }

        PacketPtr write = NULL;
        fault = stepWalk(write);
        assert(fault == NoFault || read == NULL);
        state = nextState;
        nextState = Ready;

        // On a functional access (page table lookup), writes should
        // not happen so this pointer is ignored after stepWalk
        if (write && !functional) {
            walker->port.sendAtomic(write);
        }
    } while (read);

    state = Ready;
    nextState = Waiting;
    return fault;
}





Fault
Walker::WalkerState::walkTwoStage(Addr vaddr)
{
    curstage = XlateStage::FIRST_STAGE;

    // Make sure MSBS are the same
    // riscv-privileged-20211203 page 84
    auto mask_for_msbs = mask(64 - SV39_VADDR_BITS);
    auto msbs = bits(vaddr, 63, SV39_VADDR_BITS);
    if (msbs != 0 && msbs != mask_for_msbs) {
        return pageFault();
    }

    Addr pte_addr = setupWalk(vaddr);
    level = SV39_LEVELS - 1;
    // Create physical request for first_pte_addr
    // This is a host physical address
    // In two-stage this gets discarded?
    read = createReqPacket(pte_addr,
            MemCmd::ReadReq, sizeof(PTESv39));



    if (timing)
    {
        MISA misa = tc->readMiscReg(MISCREG_ISA);
        panic_if(misa.rvh, "Timing walks are not supported with h extension");
        nextState = state;
        state = Waiting;
        timingFault = NoFault;
        // DO GSTAGE HERE IF NEEDED AND THEN DO PACKETS FOR *PTE
        sendPackets();
        return NoFault;
    }

    Fault fault = NoFault;
    do
    {
        // This is a "virtual" access, pte_address
        // is guest physical (host virtual) so pass through
        // G-stage before making a request to physmem.
        Addr guest_paddr = pte_addr;
        Addr host_paddr;

        fault = walkGStage(guest_paddr, host_paddr);
        if (fault != NoFault) { return fault; }
        pte_addr = host_paddr;

        // Create the physmem packet to be sent
        read = createReqPacket(pte_addr,
            MemCmd::ReadReq, sizeof(PTESv39));

        // G-stage done go back to first_stage logic
        curstage = FIRST_STAGE;


        if (functional) {
            walker->port.sendFunctional(read);
        } else {
            walker->port.sendAtomic(read);
        }

        PacketPtr write = NULL;
        fault = stepWalk(write);

        // Set up next vpte_addr for GStage
        // This read packet should not be sent to mem
        // paddr contains a virtual (guest physical) address
        if (read && fault == NoFault) {
            pte_addr = read->req->getPaddr();
        }

        assert(fault == NoFault || read == NULL);
        state = nextState;
        nextState = Ready;

        // On a functional access (page table lookup), writes should
        // not happen so this pointer is ignored after stepWalk
        if (write && !functional) {
            walker->port.sendAtomic(write);
        }
    } while (read);


    if (fault != NoFault) {
        return fault;
    }

    // In 2-stage walks the TLB insert is done after an
    // additional g-stage walk

    // gpa is a host virtual address.
    // To actually get the host physical address of the page,
    // we have to pass through GStage one final time.
    fault = guestToHostPage(vaddr);

    if (fault != NoFault) {
        return fault;
    }

    if (!functional && !memaccess.bypassTLB()) {
        Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
        walker->tlb->insert(vpn, entry);
    }

    state = Ready;
    nextState = Waiting;
    return NoFault;
}


Fault
Walker::WalkerState::guestToHostPage(Addr vaddr)
{
    Addr gpa = (((entry.paddr) |
    ((vaddr >> PageShift) & mask(level*SV39_LEVEL_BITS)))
    << PageShift) | (vaddr & mask(PageShift));

    Addr host_page_address;
    Fault fault = walkGStage(gpa, host_page_address);
    if (fault != NoFault) { return fault; }

    // Final G-stage done, go back to first_stage logic
    curstage = FIRST_STAGE;

    // gpn (vaddr) -> ppn (paddr) translation is already
    // in gresult, host_page_address is not needed here
    // TLB stores ppn and pte
    // entry.logBytes = PageShift + (level * SV39_LEVEL_BITS);
    entry.logBytes = PageShift;
    entry.paddr = gresult.paddr;
    entry.vaddr &= ~((1 << entry.logBytes) - 1);

    // entry.pte contains guest pte
    // host pte is in gresult.pte from final GStage
    entry.gpte = entry.pte;
    entry.pte = gresult.pte;

    return NoFault;
}

Fault
Walker::WalkerState::startFunctional(Addr &addr, unsigned &logBytes)
{
    // Pass the addess to entry here
    // initState cannot because there is no req object
    entry.vaddr = addr;
    // just call walk
    // it takes care to do the right thing
    // when functional is true
    Fault fault = walk();
    logBytes = entry.logBytes;
    addr = entry.paddr << PageShift;

    return fault;
}


Fault
Walker::WalkerState::checkPTEPermissions(
    PTESv39 pte, WalkFlags& stepWalkFlags, int level)
{
    // If valid bit is off OR
    // the page is writable but not readable, throw pf
    if (!pte.v || (!pte.r && pte.w)) {
        stepWalkFlags.doEndWalk = true;
        return pageFault();
    }

    // If read-bit or exec-bit, then PTE is a leaf
    if (pte.r || pte.x) {
        stepWalkFlags.pteIsLeaf = true;
        stepWalkFlags.doEndWalk = true;

        Fault fault = walker->tlb->checkPermissions(tc, memaccess,
                    entry.vaddr, mode, pte, gresult.vaddr, curstage);

        if (fault != NoFault) {
            return fault;
        }

        // ppn fragments that correspond to unused
        // vpn fragments have to be all zeroes
        // Otherwise, throw a pagefault
        if (level >= 1 && pte.ppn0 != 0)
        {
            return pageFault();
        }
        else if (level == 2 && pte.ppn1 != 0)
        {
            return pageFault();
        }

        if (pte.n && (pte.ppn0 & mask(NapotShift)) != 8) {
            DPRINTF(PageTableWalker,
                "SVNAPOT PTE has wrong encoding, \
                    raising PF\n");
            fault = pageFault();
        }

        // Check if we need to write
        if (!pte.a) {
            pte.a = 1;
            stepWalkFlags.doWrite = true;
        }
        if (!pte.d && mode == BaseMMU::Write) {
            pte.d = 1;
            stepWalkFlags.doWrite = true;
        }
    }

    return NoFault;
}

Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    assert(state != Ready && state != Waiting);

    Fault fault = NoFault;
    write = NULL;
    PTESv39 pte = read->getLE<uint64_t>();
    Addr nextRead = 0;

    // walk flags are initialized to false
    WalkFlags stepWalkFlags;

    DPRINTF(PageTableWalker, "Got level%d PTE: %#x\n", level, pte);

    // step 2:
    // Performing PMA/PMP checks on physical address of PTE

    // Effective privilege mode for pmp checks for page table
    // walks is S mode according to specs
    fault = walker->pmp->pmpCheck(read->req, BaseMMU::Read,
                    RiscvISA::PrivilegeMode::PRV_S, tc, entry.vaddr);

    if (fault == NoFault) {
        fault = walker->pma->check(read->req, BaseMMU::Read, entry.vaddr);
    }

    if (fault == NoFault) {

        fault = checkPTEPermissions(pte, stepWalkFlags, level);

        if (fault == NoFault && stepWalkFlags.pteIsLeaf) {

            if (stepWalkFlags.doWrite) {

                // this read will eventually become write
                // if doWrite is True

                fault = walker->pmp->pmpCheck(read->req,
                            BaseMMU::Write, pmode, tc, entry.vaddr);

                if (fault == NoFault) {
                    fault = walker->pma->check(read->req,
                                BaseMMU::Write, entry.vaddr);
                }
            }

            // perform next step only if pmp checks pass
            if (fault == NoFault) {
                // TLB inserts are OK for single stage walks
                // For two-stage, FIRST_STAGE will reach here just once
                // but the TLB insertion is done in walkTwoStage()
                if (walkType == OneStage ||
                    (walkType == TwoStage && curstage == FIRST_STAGE))
                {
                    // Fill in TLB entry
                    // Check if N (contig bit) is set, if yes we have
                    // a 64K page mapping (SVNAPOT Extension)
                    assert(!(pte.n) || level == 0);
                    entry.pte = pte;
                    entry.paddr = (pte.n) ?
                        pte.ppn & ~mask(NapotShift) :
                        pte.ppn;

                    entry.logBytes = (pte.n) ?
                        PageShift + NapotShift :
                        PageShift + (level * SV39_LEVEL_BITS);

                    // Only truncate the address in non-two stage walks
                    // The truncation for two-stage is done in
                    // walkTwoStage()
                    if (walkType != TwoStage) {
                        entry.logBytes = PageShift +
                                        (level * SV39_LEVEL_BITS);
                        entry.vaddr &= ~((1 << entry.logBytes) - 1);
                    }

                    // put it non-writable into the TLB to detect
                    // writes and redo the page table walk in order
                    // to update the dirty flag
                    if (!pte.d && mode != BaseMMU::Write)
                        entry.pte.w = 0;

                    // Don't do TLB insert here when ending TwoStage.
                    // An additional GStage is done in walkTwoStage()
                    // and then we insert.
                    // Also don't insert on special_access
                    if (walkType != TwoStage && !memaccess.bypassTLB())
                        stepWalkFlags.doTLBInsert = true;
                }

                // Update statistics for completed page walks
                if (level == 1) {
                    walker->pagewalkerstats.num_2mb_walks++;
                }
                if (level == 0) {
                    walker->pagewalkerstats.num_4kb_walks++;
                }
                DPRINTF(PageTableWalker,
                        "#1 leaf node at level %d, with vpn %#x\n",
                        level, entry.vaddr);
            }
        }
        // PTE is not a leaf and there was no fault, decrement level
        else if (fault == NoFault) {
            Addr shift, idx;
            level--;
            if (level < 0)
            {
                stepWalkFlags.doEndWalk = true;
                fault = pageFault();
            }
            else {
                shift = (PageShift + SV39_LEVEL_BITS * level);
                idx = (entry.vaddr >> shift) & mask(SV39_LEVEL_BITS);
                nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                nextState = Translate;
            }
        }
    } else {
        stepWalkFlags.doEndWalk = true;
    }

    PacketPtr oldRead = read;
    Request::Flags flags = oldRead->req->getFlags();

    if (stepWalkFlags.doEndWalk) {
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (!functional && stepWalkFlags.doWrite &&
            !(walkType == TwoStage && curstage == FIRST_STAGE))
        {
            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
            read = NULL;
        } else {
            write = NULL;
        }

        if (stepWalkFlags.doTLBInsert) {
            if (!functional && !memaccess.bypassTLB()) {
                Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                walker->tlb->insert(vpn, entry);
            }
        }
        endWalk();
    }
    else {
        //If we didn't return, we're setting up another read.
        RequestPtr request = std::make_shared<Request>(
            nextRead, oldRead->getSize(), flags, walker->requestorId);

        delete oldRead;
        oldRead = nullptr;

        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();
    }

    return fault;
}

Fault
Walker::WalkerState::stepWalkGStage(PacketPtr &write)
{
    assert(gstate != Ready && gstate != Waiting);

    Fault fault = NoFault;
    write = NULL;
    PTESv39 pte = read->getLE<uint64_t>();
    Addr nextRead = 0;

    // walk flags are initialized to false
    WalkFlags stepWalkFlags;

    DPRINTF(PageTableWalker, "[GSTAGE]: Got level%d PTE: %#x\n", glevel, pte);

    // step 2:
    // Performing PMA/PMP checks on physical address of PTE

    // Effective privilege mode for pmp checks for page table
    // walks is S mode according to specs
    fault = walker->pmp->pmpCheck(read->req, BaseMMU::Read,
                    RiscvISA::PrivilegeMode::PRV_S, tc, entry.vaddr);

    if (fault == NoFault) {
        fault = walker->pma->check(read->req, BaseMMU::Read, entry.vaddr);
    }

    if (fault == NoFault) {

        fault = checkPTEPermissions(pte, stepWalkFlags, glevel);

        if (fault == NoFault && stepWalkFlags.pteIsLeaf) {

            if (stepWalkFlags.doWrite) {

                // this read will eventually become write
                // if doWrite is True

                fault = walker->pmp->pmpCheck(read->req,
                            BaseMMU::Write, pmode, tc, entry.vaddr);

                if (fault == NoFault) {
                    fault = walker->pma->check(read->req,
                                BaseMMU::Write, entry.vaddr);
                }
            }

            // perform next step only if pmp checks pass
            if (fault == NoFault) {
                // Only change TLB entry if walk is
                // GStageOnly. Otherwise the entry is produced
                // at the end of the two-stage walk.
                // (we do not currently store intermediate GStage
                // results)
                if (walkType == GstageOnly)
                {
                    // Check if N (contig bit) is set, if yes we have
                    // a 64K page mapping (SVNAPOT Extension)
                    assert(!(pte.n) || glevel == 0);
                    entry.pte = pte;
                    entry.paddr = (pte.n) ?
                        pte.ppn & ~mask(NapotShift) :
                        pte.ppn;

                    entry.logBytes = (pte.n) ?
                        PageShift + NapotShift :
                        PageShift + (glevel * SV39_LEVEL_BITS);

                    entry.vaddr &= ~((1 << entry.logBytes) - 1);

                    // put it non-writable into the TLB to detect
                    // writes and redo the page table walk in order
                    // to update the dirty flag.
                    if (!pte.d && mode != BaseMMU::Write)
                        entry.pte.w = 0;

                    // Also don't do TLB inserts on special_access
                    if (!memaccess.bypassTLB())
                        stepWalkFlags.doTLBInsert = true;
                }
                else {
                    gresult.logBytes = PageShift +
                                    (glevel * SV39_LEVEL_BITS);
                    gresult.paddr = pte.ppn;
                    gresult.vaddr &= ~((1 << entry.logBytes) - 1);
                    gresult.pte = pte;
                }

                // Update statistics for completed page walks
                if (glevel == 1) {
                    walker->pagewalkerstats.num_2mb_walks++;
                }
                if (glevel == 0) {
                    if (pte.n) {
                        walker->pagewalkerstats.num_64kb_walks++;
                    }
                    else {
                        walker->pagewalkerstats.num_4kb_walks++;
                    }
                }
                DPRINTF(PageTableWalker,
                        "[GSTAGE] #1 leaf node at level %d, with vpn %#x\n",
                        glevel, gresult.vaddr);
            }
        }
        else if (fault == NoFault) {
            Addr shift, idx;
            glevel--;
            if (glevel < 0)
            {
                stepWalkFlags.doEndWalk = true;
                fault = pageFault();
            }
            else {
                shift = (PageShift + SV39_LEVEL_BITS * glevel);
                idx = (gresult.vaddr >> shift) & mask(SV39_LEVEL_BITS);
                nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                nextgState = Translate;
            }
        }
    } else {
        stepWalkFlags.doEndWalk = true;
    }

    PacketPtr oldRead = read;
    Request::Flags flags = oldRead->req->getFlags();

    if (stepWalkFlags.doEndWalk) {
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (!functional && stepWalkFlags.doWrite)
        {
            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
            read = NULL;
        }
        else {
            write = NULL;
        }

        if (stepWalkFlags.doTLBInsert) {
            if (!functional && !memaccess.bypassTLB()) {
                // This TLB insertion should only be reachable
                // for GstageOnly walks. Two stage walks insert
                // in walkTwoStage.
                assert(walkType == GstageOnly);
                Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                walker->tlb->insert(vpn, entry);
            }
        }
        endWalk();
    }
    else {
        //If we didn't return, we're setting up another read.
        RequestPtr request = std::make_shared<Request>(
            nextRead, oldRead->getSize(), flags, walker->requestorId);

        delete oldRead;
        oldRead = nullptr;

        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();
    }

    return fault;
}

void
Walker::WalkerState::endWalk()
{
    nextState = Ready;
    delete read;
    read = NULL;
}

Addr
Walker::WalkerState::setupWalk(Addr vaddr)
{
    Addr shift;
    Addr idx;
    Addr pte_addr;

    if (curstage == FIRST_STAGE) {
        //vaddr = Addr(sext<SV39_VADDR_BITS>(vaddr));
        shift = PageShift + SV39_LEVEL_BITS * 2;
        idx = (vaddr >> shift) & mask(SV39_LEVEL_BITS);
        pte_addr = (satp.ppn << PageShift) + (idx * sizeof(PTESv39));

        // original vaddress for first-stage is in entry.vaddr already
    }
    else if (curstage == GSTAGE) {
        shift = PageShift + SV39_LEVEL_BITS * 2;
        idx = (vaddr >> shift) &
            mask(SV39_LEVEL_BITS + SV39X4_WIDENED_BITS); // widened
        pte_addr = ((hgatp.ppn << PageShift) & ~mask(2)) +
                    (idx * sizeof(PTESv39));

        gresult.vaddr = vaddr; // store original address for g-stage
    }
    else {
        panic("Unknown translation stage!");
    }

    return pte_addr;
}

bool
Walker::WalkerState::recvPacket(PacketPtr pkt)
{
    assert(pkt->isResponse());
    assert(inflight);
    assert(state == Waiting);
    inflight--;
    if (squashed) {
        // if were were squashed, return true once inflight is zero and
        // this WalkerState will be freed there.
        return (inflight == 0);
    }
    if (pkt->isRead()) {
        // should not have a pending read it we also had one outstanding
        assert(!read);

        // @todo someone should pay for this
        pkt->headerDelay = pkt->payloadDelay = 0;

        state = nextState;
        nextState = Ready;
        PacketPtr write = NULL;
        read = pkt;
        timingFault = stepWalk(write);
        state = Waiting;
        assert(timingFault == NoFault || read == NULL);
        if (write) {
            writes.push_back(write);
        }
        sendPackets();
    } else {
        delete pkt;

        sendPackets();
    }
    if (inflight == 0 && read == NULL && writes.size() == 0) {
        state = Ready;
        nextState = Waiting;
        if (timingFault == NoFault) {
            /*
             * Finish the translation. Now that we know the right entry is
             * in the TLB, this should work with no memory accesses.
             * There could be new faults unrelated to the table walk like
             * permissions violations, so we'll need the return value as
             * well.
             */
            Addr vaddr = req->getVaddr();
            vaddr = Addr(sext<SV39_VADDR_BITS>(vaddr));
            Addr paddr = walker->tlb->hiddenTranslateWithTLB(vaddr, satp.asid,
                                                             satp.mode, mode);

            req->setPaddr(paddr);

            // do pmp check if any checking condition is met.
            // timingFault will be NoFault if pmp checks are
            // passed, otherwise an address fault will be returned.
            timingFault = walker->pmp->pmpCheck(req, mode, pmode, tc);

            if (timingFault == NoFault) {
                timingFault = walker->pma->check(req, mode);
            }

            // Let the CPU continue.
            translation->finish(timingFault, req, tc, mode);
        } else {
            // There was a fault during the walk. Let the CPU know.
            translation->finish(timingFault, req, tc, mode);
        }
        return true;
    }

    return false;
}

void
Walker::WalkerState::sendPackets()
{
    //If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    //Reads always have priority
    if (read) {
        PacketPtr pkt = read;
        read = NULL;
        inflight++;
        if (!walker->sendTiming(this, pkt)) {
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
        if (!walker->sendTiming(this, write)) {
            retrying = true;
            writes.push_back(write);
            inflight--;
            return;
        }
    }
}

PacketPtr
Walker::WalkerState::createReqPacket(Addr paddr, MemCmd cmd, size_t bytes)
{
    Request::Flags flags = Request::PHYSICAL;
    RequestPtr request = std::make_shared<Request>(
        paddr, bytes, flags, walker->requestorId);
    PacketPtr pkt = new Packet(request, cmd);
    pkt->allocate();
    return pkt;
}

unsigned
Walker::WalkerState::numInflight() const
{
    return inflight;
}

bool
Walker::WalkerState::isRetrying()
{
    return retrying;
}

bool
Walker::WalkerState::isTiming()
{
    return timing;
}

bool
Walker::WalkerState::wasStarted()
{
    return started;
}

void
Walker::WalkerState::squash()
{
    squashed = true;
}

void
Walker::WalkerState::retry()
{
    retrying = false;
    sendPackets();
}

Fault
Walker::WalkerState::pageFault()
{
    return walker->tlb->createPagefault(entry.vaddr, mode,
                        gresult.vaddr, curstage == GSTAGE, memaccess.virt);
}

Walker::PagewalkerStats::PagewalkerStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(num_4kb_walks, statistics::units::Count::get(),
             "Completed page walks with 4KB pages"),
    ADD_STAT(num_64kb_walks, statistics::units::Count::get(),
             "Completed page walks with 64KB pages"),
    ADD_STAT(num_2mb_walks, statistics::units::Count::get(),
             "Completed page walks with 2MB pages")
{
}

} // namespace RiscvISA
} // namespace gem5
