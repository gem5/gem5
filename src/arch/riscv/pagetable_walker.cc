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

        if (!newState->isTiming())
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
    if (debug_condition()) {
        DPRINTF(PageTableWalker, "======================================\n");
        DPRINTF(PageTableWalker,
            "[WALK] : vaddr: %#x | SATP: %#x | HGATP: %#x \n",
            entry.vaddr, satp, hgatp);
    }
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
        if (!e || fault != NoFault)
            currState->walk();
        else
            schedule(startWalkWrapperEvent, clockEdge(Cycles(1)));
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

    if (debug_condition()) {
        DPRINTF(PageTableWalker, "-------------------------");
        DPRINTF(PageTableWalker,
        " [GSTAGE BEGIN]: gpa: %#x\n", guest_paddr);
    }
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
    if (timing)
    {
        panic("unimpl");
        nextgState = gstate;
        gstate = Waiting;
        timingFault = NoFault;
        sendPackets();
    }
    else
    {
        do
        {
            walker->port.sendAtomic(read);
            PacketPtr write = NULL;
            fault = stepWalk(write);
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

        if (debug_condition()) {
            DPRINTF(PageTableWalker,
            " [GSTAGE END]: gpa: %#x -> host pa: %#x\n",
            guest_paddr, host_paddr);
            DPRINTF(PageTableWalker, "-------------------------");
        }

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
    curstage = XlateStage::FIRST_STAGE;
    bool special_access = memaccess.force_virt ||
                                memaccess.hlvx ||
                                memaccess.lr;

    // initState initialized this
    // curva is not initialized at this point!
    // only after setupWalk()
    Addr vaddr = entry.vaddr;

    // There is the case where VSATP is bare and
    // we do G-stage translation only
    Addr pte_addr;
    if (satp.mode == AddrXlateMode::BARE && memaccess.virt) {
        walkType = WalkType::GstageOnly;
        Addr paddr;
        fault = walkGStage(vaddr, paddr);
        return fault;
    }
    else {
        // Make sure MSBS are the same
        // riscv-privileged-20211203 page 84
        auto mask_for_msbs = mask(64 - SV39_VADDR_BITS);
        auto msbs = bits(vaddr, 63, SV39_VADDR_BITS);
        if (msbs != 0 && msbs != mask_for_msbs) {
            return pageFault();
        }

        walkType = memaccess.virt ? WalkType::TwoStage :
                                    WalkType::OneStage;

        pte_addr = setupWalk(vaddr);
        level = SV39_LEVELS - 1;
        // Create physical request for first_pte_addr
        // This is a host physical address
        // In two-stage this gets discarded?
        read = createReqPacket(pte_addr,
                MemCmd::ReadReq, sizeof(PTESv39));
    }

    if (timing)
    {
        MISA misa = tc->readMiscReg(MISCREG_ISA);
        panic_if(misa.rvh, "Timing walks are not supported with h extension");
        nextState = state;
        state = Waiting;
        timingFault = NoFault;
        // DO GSTAGE HERE IF NEEDED AND THEN DO PACKETS FOR *PTE
        sendPackets();
    }
    else
    {
        do
        {
            // If this is a virtual access, pte_address
            // is guest physical (host virtual) so pass through
            // G-stage before making a request to physmem.
            if (walkType == WalkType::TwoStage) {
                Addr guest_paddr = pte_addr;
                Addr host_paddr;

                if (debug_condition()) {
                    DPRINTF(PageTableWalker,
                        "[TWO-STAGE] Request Gstage for %#x\n", guest_paddr);
                }

                fault = walkGStage(guest_paddr, host_paddr);
                if (fault != NoFault) { return fault; }
                pte_addr = host_paddr;
                if (debug_condition()) {
                    DPRINTF(PageTableWalker,
                        "[TWO-STAGE] Got pte_addr %#x\n", pte_addr);
                }
                // Create the physmem packet to be sent
                read = createReqPacket(pte_addr,
                    MemCmd::ReadReq, sizeof(PTESv39));

                // G-stage done go back to first_stage logic
                curstage = FIRST_STAGE;
            }


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
            if (read && walkType == TwoStage && fault == NoFault) {
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

        // In 2-stage walks the TLB insert is done after an
        // additional g-stage walk
        if (walkType == TwoStage && fault == NoFault) {
            Addr gpa = (((entry.paddr) |
            ((vaddr >> PageShift) & mask(level*SV39_LEVEL_BITS)))
            << PageShift) | (vaddr & mask(PageShift));
            Addr final_paddr;
            fault = walkGStage(gpa, final_paddr);
            if (fault != NoFault) { return fault; }

            // G-stage done go back to first_stage logic
            curstage = FIRST_STAGE;

            // gpn (vaddr) -> ppn (paddr) translation is in gresult
            // final_paddr is not needed here
            // TLB stores ppn and pte
            // entry.logBytes = PageShift + (level * SV39_LEVEL_BITS);
            entry.logBytes = PageShift;
            entry.paddr = gresult.paddr;
            entry.vaddr &= ~((1 << entry.logBytes) - 1);

            // entry.pte contains guest pte
            // host pte is in gresult.pte from final GStage
            entry.gpte = entry.pte;
            entry.pte = gresult.pte;


            if (debug_condition()) {
                DPRINTF(PageTableWalker,
                                "[TWO-STAGE] Translated %#x -> %#x\n",
                                vaddr, entry.paddr << PageShift |
                                (vaddr & mask(entry.logBytes)));
            }

            if (!functional && !special_access) {
                Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                walker->tlb->insert(vpn, entry);


                if (debug_condition()) {
                    DPRINTF(PageTableWalker,
                        "[TWO STAGE] TLB Insert vaddr: %#x, pte: %#x\n",
                            entry.vaddr , entry.pte);
                    DPRINTF(PageTableWalker,
                        "============================================\n");
                }
            }
            else {
                if (debug_condition() && functional) {
                DPRINTF(PageTableWalker,
                    "[FUNCTIONAL ACCESS] Translated %#x -> %#x\n",
                    vaddr, entry.paddr << PageShift |
                    (vaddr & mask(entry.logBytes)));
                }

                if (debug_condition() && special_access) {
                    DPRINTF(PageTableWalker,
                        "[SPECIAL ACCESS] NO TLB INSERT\n");
                }

                if (debug_condition())
                    DPRINTF(PageTableWalker,
                        "============================================\n");

            }
        }

        state = Ready;
        nextState = Waiting;

        if (debug_condition()) {
            DPRINTF(PageTableWalker, " Translated %#x -> %#x\n",
            vaddr, entry.paddr << PageShift |
            (vaddr & mask(entry.logBytes)));
        }
    }

    return fault;
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
    return walk();
}

Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    int &curlevel = curstage == GSTAGE ? glevel : level;
    [[maybe_unused]]State curstate = curstage == GSTAGE ? gstate : state;
    assert(curstate != Ready && curstate != Waiting);

    Fault fault = NoFault;
    write = NULL;
    PTESv39 pte = read->getLE<uint64_t>();
    Addr nextRead = 0;
    bool doWrite = false;
    bool doTLBInsert = false;
    bool doEndWalk = false;
    bool special_access = false
        || memaccess.force_virt
        || memaccess.hlvx
        || memaccess.lr;

    if (debug_condition()) {
        DPRINTF(PageTableWalker,
            " [%s] Got level%d PTE: %#x\n",
            (curstage == GSTAGE) ? "GSTAGE" : "1stSTAGE", curlevel, pte);
    }

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
        // step 3:
        if (!pte.v || (!pte.r && pte.w)) {
            doEndWalk = true;

            if (debug_condition()) {
                DPRINTF(PageTableWalker,
                    " [%s] PTE invalid, raising PF. pte: %#x\n",
                    ((curstage == GSTAGE) ? "GSTAGE" : "1stSTAGE"), pte);
            }
            fault = pageFault();
        }
        else {
            // step 4:
            if (pte.r || pte.x) {
                // step 5: leaf PTE
                doEndWalk = true;

                fault = walker->tlb->checkPermissions(tc, memaccess,
                            entry.vaddr, mode, pte, gresult.vaddr, curstage);

                // step 6
                if (fault == NoFault)
                {
                    if (curlevel >= 1 && pte.ppn0 != 0)
                    {

                        if (debug_condition()) DPRINTF(PageTableWalker,
                                "PTE has misaligned PPN, raising PF\n");
                        fault = pageFault();
                    }
                    else if (curlevel == 2 && pte.ppn1 != 0)
                    {
                        if (debug_condition()) DPRINTF(PageTableWalker,
                                "PTE has misaligned PPN, raising PF\n");
                        fault = pageFault();
                    }
                }

                if (fault == NoFault) {
                    // step 7
                    if (!pte.a) {
                        pte.a = 1;
                        doWrite = true;
                    }
                    if (!pte.d && mode == BaseMMU::Write) {
                        pte.d = 1;
                        doWrite = true;
                    }
                    // Performing PMA/PMP checks

                    if (doWrite) {

                        // this read will eventually become write
                        // if doWrite is True

                        fault = walker->pmp->pmpCheck(read->req,
                                    BaseMMU::Write, pmode, tc, entry.vaddr);

                        if (fault == NoFault) {
                            fault = walker->pma->check(read->req,
                                                BaseMMU::Write, entry.vaddr);
                        }

                    }
                    // perform step 8 only if pmp checks pass
                    if (fault == NoFault) {
                        // TLB inserts are OK for single stage walks
                        // For two-stage, FIRST_STAGE will reach here just once
                        // (GStage reaches here multiple times)
                        if (walkType == OneStage || walkType == GstageOnly ||
                            (walkType == TwoStage && curstage == FIRST_STAGE))
                        {
                            // step 8
                            entry.pte = pte;
                            entry.paddr = pte.ppn;


                            // DO NOT truncate the address
                            // it will be done from last GStage in walk
                            if (!(walkType == TwoStage &&
                                  curstage == FIRST_STAGE)) {
                                entry.logBytes = PageShift +
                                                (curlevel * SV39_LEVEL_BITS);
                                entry.vaddr &= ~((1 << entry.logBytes) - 1);
                                if (debug_condition())
                                    DPRINTF(PageTableWalker,
                                         " [%s] FOUND LEAF PTE %#x. "
                                         " LOGBYTES=%d\n",
                                        ((curstage == GSTAGE) ?
                                        "GSTAGE" : "1stSTAGE"),
                                        pte, entry.logBytes);
                            }

                            // put it non-writable into the TLB to detect
                            // writes and redo the page table walk in order
                            // to update the dirty flag.
                            if (!pte.d && mode != BaseMMU::Write)
                                entry.pte.w = 0;

                            // Don't do TLB insert when ending TwoStage.
                            // an additional GStage is done in walk
                            // and then we insert.
                            // At this point entry.paddr contains gpn
                            // and not ppn!!!
                            // Also don't insert on special_access
                            if (walkType != TwoStage && !special_access)
                                doTLBInsert = true;
                        }
                        else {
                            gresult.logBytes = PageShift +
                                            (curlevel * SV39_LEVEL_BITS);
                            gresult.paddr = pte.ppn;
                            gresult.vaddr &= ~((1 << entry.logBytes) - 1);
                            gresult.pte = pte;
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
            } else {
                Addr shift, idx;
                curlevel--;
                if (curlevel < 0)
                {
                    if (debug_condition()) {
                        DPRINTF(PageTableWalker,
                            "No leaf PTE found, raising PF\n");
                    }

                    doEndWalk = true;
                    fault = pageFault();
                }
                else if (curstage == GSTAGE)
                {
                    shift = (PageShift + SV39_LEVEL_BITS * curlevel);
                    idx = (gresult.vaddr >> shift) & mask(SV39_LEVEL_BITS);
                    nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                    nextgState = Translate;
                }
                else {
                    shift = (PageShift + SV39_LEVEL_BITS * curlevel);
                    idx = (entry.vaddr >> shift) & mask(SV39_LEVEL_BITS);
                    nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                    nextState = Translate;
                }
            }
        }
    } else {
        doEndWalk = true;
    }
    PacketPtr oldRead = read;
    Request::Flags flags = oldRead->req->getFlags();

    if (doEndWalk) {
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (!functional && doWrite &&
            !(walkType == TwoStage && curstage == FIRST_STAGE))
        {
            if (debug_condition()) {
                DPRINTF(PageTableWalker,
                    "Writing level%d PTE to %#x: %#x\n",
                    curlevel, oldRead->getAddr(), pte);
            }

            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
            read = NULL;
        } else {
            write = NULL;
        }

        if (doTLBInsert) {
            if (!functional && !special_access) {
                Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                walker->tlb->insert(vpn, entry);
            } else {
                if (walkType != TwoStage) {
                    if (debug_condition())
                        DPRINTF(PageTableWalker, "Translated %#x -> %#x\n",
                        entry.vaddr, entry.paddr << PageShift |
                        (entry.vaddr & mask(entry.logBytes)));
                }
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

        if (debug_condition()) DPRINTF(PageTableWalker,
                "[%s] Get level%d PTE from %#x\n",
                curstage == GSTAGE ? "GSTAGE" : "1stSTAGE",
                curlevel, nextRead);
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

    if (debug_condition()) {
        DPRINTF(PageTableWalker,
            "[%s] SETUP for address %#x\n",
            curstage == FIRST_STAGE ? "1stSTAGE" : "GSTAGE" , vaddr);

        DPRINTF(PageTableWalker,
            "[%s] Loading level %d PTE from %#x\n",
            curstage == FIRST_STAGE ? "1stSTAGE" : "GSTAGE",
            SV39_LEVELS - 1, pte_addr);
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
            Addr paddr = walker->tlb->translateWithTLB(vaddr, satp.asid,
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
    if (debug_condition()) {
        DPRINTF(PageTableWalker,
            "[%s] Raising page fault. vaddr: %#x gvaddr: %#x\n",
            curstage == FIRST_STAGE ? "1stSTAGE" : "GSTAGE",
            entry.vaddr, gresult.vaddr);
    }
    return walker->tlb->createPagefault(entry.vaddr, mode,
                        gresult.vaddr, curstage == GSTAGE, memaccess.virt);
}

bool
Walker::WalkerState::debug_condition() {
    return true;

    // You can use this method to only trigger
    // PageTableWalker debug printing under
    // specific conditions
    // e.g. only after a specific address
    // static bool latch = false;
    // return (latch |= (req->getVaddr() == 0xffffffd80fffb078));

}

Walker::PagewalkerStats::PagewalkerStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(num_4kb_walks, statistics::units::Count::get(),
             "Completed page walks with 4KB pages"),
    ADD_STAT(num_2mb_walks, statistics::units::Count::get(),
             "Completed page walks with 2MB pages")
{
}

} // namespace RiscvISA
} // namespace gem5
