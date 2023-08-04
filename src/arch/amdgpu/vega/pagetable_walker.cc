/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/amdgpu/vega/pagetable_walker.hh"

#include <memory>

#include "arch/amdgpu/vega/faults.hh"
#include "mem/abstract_mem.hh"
#include "mem/packet_access.hh"

namespace gem5
{
namespace VegaISA
{

/*
 * Functional/atomic mode methods
 */
Fault
Walker::startFunctional(Addr base, Addr &addr, unsigned &logBytes,
                        BaseMMU::Mode mode, bool &isSystem)
{
    PageTableEntry pte;
    Addr vaddr = addr;
    Fault fault = startFunctional(base, vaddr, pte, logBytes, mode);
    isSystem = pte.s;
    addr = ((pte.ppn << PageShift) + (vaddr & mask(logBytes)));

    return fault;
}

Fault
Walker::startFunctional(Addr base, Addr vaddr, PageTableEntry &pte,
                        unsigned &logBytes, BaseMMU::Mode mode)
{
    DPRINTF(GPUPTWalker, "Vega walker walker: %p funcState: %p "
            "funcState->walker %p\n",
            this, &funcState, funcState.getWalker());
    funcState.initState(mode, base, vaddr, true);
    return funcState.startFunctional(base, vaddr, pte, logBytes);
}

Fault
Walker::WalkerState::startFunctional(Addr base, Addr vaddr,
                                     PageTableEntry &pte, unsigned &logBytes)
{
    Fault fault = NoFault;
    DPRINTF(GPUPTWalker, "Vega walker starting with addr: %#lx "
            "logical: %#lx\n", vaddr, vaddr >> PageShift);

    assert(!started);
    started = true;

    do {
        DPRINTF(GPUPTWalker, "Sending functional read to %#lx\n",
                read->getAddr());

        auto devmem = walker->system->getDeviceMemory(read);
        assert(devmem);
        devmem->access(read);

        fault = stepWalk();
        assert(fault == NoFault || read == NULL);

        state = nextState;
    } while (read);

    logBytes = entry.logBytes;
    pte = entry.pte;

    return fault;
}


/*
 * Timing mode methods
 */
void
Walker::startTiming(PacketPtr pkt, Addr base, Addr vaddr, BaseMMU::Mode mode)
{
    DPRINTF(GPUPTWalker, "Vega walker starting with addr: %#lx "
            "logical: %#lx\n", vaddr, vaddr >> PageShift);

    WalkerState *newState = new WalkerState(this, pkt);

    newState->initState(mode, base, vaddr);
    currStates.push_back(newState);
    DPRINTF(GPUPTWalker, "There are %ld walker states\n", currStates.size());

    newState->startWalk();
}

void
Walker::WalkerState::initState(BaseMMU::Mode _mode, Addr baseAddr, Addr vaddr,
                               bool is_functional)
{
    DPRINTF(GPUPTWalker, "Walker::WalkerState::initState\n");
    DPRINTF(GPUPTWalker, "Walker::WalkerState::initState %p\n", this);
    DPRINTF(GPUPTWalker, "Walker::WalkerState::initState %d\n", state);
    assert(state == Ready);

    started = false;
    mode = _mode;
    timing = !is_functional;
    enableNX = true;
    dataSize = 8; // 64-bit PDEs / PTEs
    nextState = PDE2;

    DPRINTF(GPUPTWalker, "Setup walk with base %#lx\n", baseAddr);

    // First level in Vega is PDE2. Calculate the address for that PDE using
    // baseAddr and vaddr.
    state = PDE2;
    Addr logical_addr = vaddr >> PageShift;
    Addr pde2Addr = (((baseAddr >> 6) << 3) + (logical_addr >> 3*9)) << 3;
    DPRINTF(GPUPTWalker, "Walk PDE2 address is %#lx\n", pde2Addr);

    // Start populating the VegaTlbEntry response
    entry.vaddr = logical_addr;

    // Prepare the read packet that will be used at each level
    Request::Flags flags = Request::PHYSICAL;

    RequestPtr request = std::make_shared<Request>(
        pde2Addr, dataSize, flags, walker->deviceRequestorId);

    read = new Packet(request, MemCmd::ReadReq);
    read->allocate();
}

void
Walker::WalkerState::startWalk()
{
    if (!started) {
        // Read the first PDE to begin
        DPRINTF(GPUPTWalker, "Sending timing read to %#lx\n",
                read->getAddr());

        sendPackets();
        started = true;
    } else {
        // This is mostly the same as stepWalk except we update the state and
        // send the new timing read request.
        timingFault = stepWalk();
        assert(timingFault == NoFault || read == NULL);

        state = nextState;

        if (read) {
            DPRINTF(GPUPTWalker, "Sending timing read to %#lx\n",
                    read->getAddr());
            sendPackets();
        } else {
            // Set physical page address in entry
            entry.paddr = entry.pte.ppn << PageShift;
            entry.paddr += entry.vaddr & mask(entry.logBytes);

            // Insert to TLB
            assert(walker);
            assert(walker->tlb);
            walker->tlb->insert(entry.vaddr, entry);

            // Send translation return event
            walker->walkerResponse(this, entry, tlbPkt);
        }
    }
}

Fault
Walker::WalkerState::stepWalk()
{
    assert(state != Ready && state != Waiting && read);
    Fault fault = NoFault;
    PageTableEntry pte = read->getLE<uint64_t>();

    bool uncacheable = !pte.c;
    Addr nextRead = 0;
    bool doEndWalk = false;

    walkStateMachine(pte, nextRead, doEndWalk, fault);

    if (doEndWalk) {
        DPRINTF(GPUPTWalker, "ending walk\n");
        endWalk();
    } else {
        PacketPtr oldRead = read;

        //If we didn't return, we're setting up another read.
        Request::Flags flags = oldRead->req->getFlags();
        flags.set(Request::UNCACHEABLE, uncacheable);
        RequestPtr request = std::make_shared<Request>(
            nextRead, oldRead->getSize(), flags, walker->deviceRequestorId);

        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();

        delete oldRead;
    }

    return fault;
}

void
Walker::WalkerState::walkStateMachine(PageTableEntry &pte, Addr &nextRead,
                                      bool &doEndWalk, Fault &fault)
{
    Addr vaddr = entry.vaddr;
    bool badNX = pte.x && mode == BaseMMU::Execute && enableNX;
    Addr part1 = 0;
    Addr part2 = 0;
    PageDirectoryEntry pde = static_cast<PageDirectoryEntry>(pte);

    // Block fragment size can change the size of the pages pointed to while
    // moving to the next PDE. A value of 0 implies native page size. A
    // non-zero value implies the next leaf in the page table is a PTE unless
    // the F bit is set. If we see a non-zero value, set it here and print
    // for debugging.
    if (pde.blockFragmentSize) {
        DPRINTF(GPUPTWalker,
                "blockFragmentSize: %d, pde: %#016lx, state: %d\n",
                pde.blockFragmentSize, pde, state);
        blockFragmentSize = pde.blockFragmentSize;

        // At this time, only a value of 9 is used in the driver:
        // https://github.com/torvalds/linux/blob/master/drivers/gpu/drm/
        //     amd/amdgpu/gmc_v9_0.c#L1165
        assert(pde.blockFragmentSize == 9);
    }

    switch(state) {
      case PDE2:
        if (pde.p) {
            DPRINTF(GPUPTWalker, "Treating PDE2 as PTE: %#016x frag: %d\n",
                    (uint64_t)pte, pte.fragment);
            entry.pte = pte;
            int fragment = pte.fragment;
            entry.logBytes = PageShift + std::min(3*9, fragment);
            entry.vaddr <<= PageShift;
            entry.vaddr = entry.vaddr & ~mask(entry.logBytes);
            doEndWalk = true;
        }

        // Read the pde1Addr
        part1 = ((((uint64_t)pte) >> 6) << 3);
        part2 = offsetFunc(vaddr, 3*9, 2*9);
        nextRead = ((part1 + part2) << 3) & mask(48);
        DPRINTF(GPUPTWalker,
                "Got PDE2 entry %#016x. write:%s->%#016x va:%#016x\n",
                (uint64_t)pte, pte.w == 0 ? "yes" : "no", nextRead, vaddr);
        nextState = PDE1;
        break;
      case PDE1:
        if (pde.p) {
            DPRINTF(GPUPTWalker, "Treating PDE1 as PTE: %#016x frag: %d\n",
                    (uint64_t)pte, pte.fragment);
            entry.pte = pte;
            int fragment = pte.fragment;
            entry.logBytes = PageShift + std::min(2*9, fragment);
            entry.vaddr <<= PageShift;
            entry.vaddr = entry.vaddr & ~mask(entry.logBytes);
            doEndWalk = true;
        }

        // Read the pde0Addr
        part1 = ((((uint64_t)pte) >> 6) << 3);
        part2 = offsetFunc(vaddr, 2*9, 9);
        nextRead = ((part1 + part2) << 3) & mask(48);
        DPRINTF(GPUPTWalker,
                "Got PDE1 entry %#016x. write:%s->%#016x va: %#016x\n",
                (uint64_t)pte, pte.w == 0 ? "yes" : "no", nextRead, vaddr);
        nextState = PDE0;
        break;
      case PDE0:
        if (pde.p || (blockFragmentSize && !pte.f)) {
            DPRINTF(GPUPTWalker, "Treating PDE0 as PTE: %#016x frag: %d\n",
                    (uint64_t)pte, pte.fragment);
            entry.pte = pte;
            int fragment = pte.fragment;
            entry.logBytes = PageShift + std::min(9, fragment);
            entry.vaddr <<= PageShift;
            entry.vaddr = entry.vaddr & ~mask(entry.logBytes);
            doEndWalk = true;
        }
        // Read the PteAddr
        part1 = ((((uint64_t)pte) >> 6) << 3);
        if (pte.f) {
            // For F bit we want to use the blockFragmentSize in the previous
            // PDE and the blockFragmentSize in this PTE for offset function.
            part2 = offsetFunc(vaddr,
                               blockFragmentSize,
                               pde.blockFragmentSize);
        } else {
            part2 = offsetFunc(vaddr, 9, 0);
        }
        nextRead = ((part1 + part2) << 3) & mask(48);
        DPRINTF(GPUPTWalker,
                "Got PDE0 entry %#016x. write:%s->%#016x va:%#016x\n",
                (uint64_t)pte, pte.w == 0 ? "yes" : "no", nextRead, vaddr);
        nextState = PTE;
        break;
      case PTE:
        DPRINTF(GPUPTWalker,
                " PTE entry %#016x. write: %s va: %#016x\n",
                (uint64_t)pte, pte.w == 0 ? "yes" : "no", vaddr);
        entry.pte = pte;
        entry.logBytes = PageShift;
        entry.vaddr <<= PageShift;
        entry.vaddr = entry.vaddr & ~mask(entry.logBytes);
        doEndWalk = true;
        break;
      default:
        panic("Unknown page table walker state %d!\n");
    }

    if (badNX || !pte.v) {
        doEndWalk = true;
        fault = pageFault(pte.v);
        nextState = state;
    }
}

void
Walker::WalkerState::endWalk()
{
    nextState = Ready;
    delete read;
    read = NULL;
    walker->currStates.remove(this);
}

/**
 * Port related methods
 */
void
Walker::WalkerState::sendPackets()
{
    // If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    if (!walker->sendTiming(this, read)) {
        DPRINTF(GPUPTWalker, "Timing request for %#lx failed\n",
                read->getAddr());

        retrying = true;
    } else {
        DPRINTF(GPUPTWalker, "Timing request for %#lx successful\n",
                read->getAddr());
    }
}

bool Walker::sendTiming(WalkerState* sending_walker, PacketPtr pkt)
{
    auto walker_state = new WalkerSenderState(sending_walker);
    pkt->pushSenderState(walker_state);

    if (port.sendTimingReq(pkt)) {
        DPRINTF(GPUPTWalker, "Sending timing read to %#lx from walker %p\n",
                pkt->getAddr(), sending_walker);

        return true;
    } else {
        (void)pkt->popSenderState();
        delete walker_state;
    }

    return false;
}

bool
Walker::WalkerPort::recvTimingResp(PacketPtr pkt)
{
    walker->recvTimingResp(pkt);

    return true;
}

void
Walker::recvTimingResp(PacketPtr pkt)
{
    WalkerSenderState * senderState =
        safe_cast<WalkerSenderState *>(pkt->popSenderState());

    DPRINTF(GPUPTWalker, "Got response for %#lx from walker %p -- %#lx\n",
            pkt->getAddr(), senderState->senderWalk, pkt->getLE<uint64_t>());
    senderState->senderWalk->startWalk();

    delete senderState;
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

void
Walker::walkerResponse(WalkerState *state, VegaTlbEntry& entry, PacketPtr pkt)
{
    tlb->walkerResponse(entry, pkt);

    delete state;
}


/*
 *  Helper methods
 */
bool
Walker::WalkerState::isRetrying()
{
    return retrying;
}

void
Walker::WalkerState::retry()
{
    retrying = false;
    sendPackets();
}

Fault
Walker::WalkerState::pageFault(bool present)
{
    DPRINTF(GPUPTWalker, "Raising page fault.\n");
    ExceptionCode code;
    if (mode == BaseMMU::Read)
        code = ExceptionCode::LOAD_PAGE;
    else if (mode == BaseMMU::Write)
        code = ExceptionCode::STORE_PAGE;
    else
        code = ExceptionCode::INST_PAGE;
    if (mode == BaseMMU::Execute && !enableNX)
        mode = BaseMMU::Read;
    return std::make_shared<PageFault>(entry.vaddr, code, present, mode, true);
}

uint64_t
Walker::WalkerState::offsetFunc(Addr logicalAddr, int top, int lsb)
{
    assert(top < 32);
    assert(lsb < 32);
    return ((logicalAddr & ((1 << top) - 1)) >> lsb);
}


/**
 * gem5 methods
 */
Port &
Walker::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return ClockedObject::getPort(if_name, idx);
}

} // namespace VegaISA
} // namespace gem5
