/*
 * Copyright (c) 2010-2012,2017-2019 ARM Limited
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
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * All rights reserved.
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

#include "mem/abstract_mem.hh"

#include <vector>

#include "base/loader/memory_image.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "debug/LLSC.hh"
#include "debug/MemoryAccess.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

namespace gem5
{

namespace memory
{

AbstractMemory::AbstractMemory(const Params &p) :
    ClockedObject(p), range(p.range), pmemAddr(NULL),
    backdoor(params().range, nullptr,
             (MemBackdoor::Flags)(p.writeable ?
                 MemBackdoor::Readable | MemBackdoor::Writeable :
                 MemBackdoor::Readable)),
    confTableReported(p.conf_table_reported), inAddrMap(p.in_addr_map),
    kvmMap(p.kvm_map), writeable(p.writeable), _system(NULL),
    stats(*this)
{
    panic_if(!range.valid() || !range.size(),
             "Memory range %s must be valid with non-zero size.",
             range.to_string());
}

void
AbstractMemory::initState()
{
    ClockedObject::initState();

    const auto &file = params().image_file;
    if (file == "")
        return;

    auto *object = loader::createObjectFile(file, true);
    fatal_if(!object, "%s: Could not load %s.", name(), file);

    loader::debugSymbolTable.insert(*object->symtab().globals());
    loader::MemoryImage image = object->buildImage();

    AddrRange image_range(image.minAddr(), image.maxAddr());
    if (!range.contains(image_range.start())) {
        warn("%s: Moving image from %s to memory address range %s.",
                name(), image_range.to_string(), range.to_string());
        image = image.offset(range.start());
        image_range = AddrRange(image.minAddr(), image.maxAddr());
    }
    panic_if(!image_range.isSubset(range), "%s: memory image %s doesn't fit.",
             name(), file);

    PortProxy proxy([this](PacketPtr pkt) { functionalAccess(pkt); },
                    system()->cacheLineSize());

    panic_if(!image.write(proxy), "%s: Unable to write image.");
}

void
AbstractMemory::setBackingStore(uint8_t* pmem_addr)
{
    // If there was an existing backdoor, let everybody know it's going away.
    if (backdoor.ptr())
        backdoor.invalidate();

    // The back door can't handle interleaved memory.
    backdoor.ptr(range.interleaved() ? nullptr : pmem_addr);

    pmemAddr = pmem_addr;
}

AbstractMemory::MemStats::MemStats(AbstractMemory &_mem)
    : statistics::Group(&_mem), mem(_mem),
    ADD_STAT(bytesRead, statistics::units::Byte::get(),
             "Number of bytes read from this memory"),
    ADD_STAT(bytesInstRead, statistics::units::Byte::get(),
             "Number of instructions bytes read from this memory"),
    ADD_STAT(bytesWritten, statistics::units::Byte::get(),
             "Number of bytes written to this memory"),
    ADD_STAT(numReads, statistics::units::Count::get(),
             "Number of read requests responded to by this memory"),
    ADD_STAT(numWrites, statistics::units::Count::get(),
             "Number of write requests responded to by this memory"),
    ADD_STAT(numOther, statistics::units::Count::get(),
             "Number of other requests responded to by this memory"),
    ADD_STAT(bwRead, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Total read bandwidth from this memory"),
    ADD_STAT(bwInstRead,
             statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Instruction read bandwidth from this memory"),
    ADD_STAT(bwWrite, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Write bandwidth from this memory"),
    ADD_STAT(bwTotal, statistics::units::Rate<
                statistics::units::Byte, statistics::units::Second>::get(),
             "Total bandwidth to/from this memory")
{
}

void
AbstractMemory::MemStats::regStats()
{
    using namespace statistics;

    statistics::Group::regStats();

    System *sys = mem.system();
    assert(sys);
    const auto max_requestors = sys->maxRequestors();

    bytesRead
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bytesRead.subname(i, sys->getRequestorName(i));
    }

    bytesInstRead
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bytesInstRead.subname(i, sys->getRequestorName(i));
    }

    bytesWritten
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bytesWritten.subname(i, sys->getRequestorName(i));
    }

    numReads
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        numReads.subname(i, sys->getRequestorName(i));
    }

    numWrites
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        numWrites.subname(i, sys->getRequestorName(i));
    }

    numOther
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        numOther.subname(i, sys->getRequestorName(i));
    }

    bwRead
        .precision(0)
        .prereq(bytesRead)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bwRead.subname(i, sys->getRequestorName(i));
    }

    bwInstRead
        .precision(0)
        .prereq(bytesInstRead)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bwInstRead.subname(i, sys->getRequestorName(i));
    }

    bwWrite
        .precision(0)
        .prereq(bytesWritten)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bwWrite.subname(i, sys->getRequestorName(i));
    }

    bwTotal
        .precision(0)
        .prereq(bwTotal)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        bwTotal.subname(i, sys->getRequestorName(i));
    }

    bwRead = bytesRead / simSeconds;
    bwInstRead = bytesInstRead / simSeconds;
    bwWrite = bytesWritten / simSeconds;
    bwTotal = (bytesRead + bytesWritten) / simSeconds;
}

AddrRange
AbstractMemory::getAddrRange() const
{
    return range;
}

// Add load-locked to tracking list.  Should only be called if the
// operation is a load and the LLSC flag is set.
void
AbstractMemory::trackLoadLocked(PacketPtr pkt)
{
    const RequestPtr &req = pkt->req;
    Addr paddr = LockedAddr::mask(req->getPaddr());

    // first we check if we already have a locked addr for this
    // xc.  Since each xc only gets one, we just update the
    // existing record with the new address.
    std::list<LockedAddr>::iterator i;

    for (i = lockedAddrList.begin(); i != lockedAddrList.end(); ++i) {
        if (i->matchesContext(req)) {
            DPRINTF(LLSC, "Modifying lock record: context %d addr %#x\n",
                    req->contextId(), paddr);
            i->addr = paddr;
            return;
        }
    }

    // no record for this xc: need to allocate a new one
    DPRINTF(LLSC, "Adding lock record: context %d addr %#x\n",
            req->contextId(), paddr);
    lockedAddrList.push_front(LockedAddr(req));
    backdoor.invalidate();
}


// Called on *writes* only... both regular stores and
// store-conditional operations.  Check for conventional stores which
// conflict with locked addresses, and for success/failure of store
// conditionals.
bool
AbstractMemory::checkLockedAddrList(PacketPtr pkt)
{
    const RequestPtr &req = pkt->req;
    Addr paddr = LockedAddr::mask(req->getPaddr());
    bool isLLSC = pkt->isLLSC();

    // Initialize return value.  Non-conditional stores always
    // succeed.  Assume conditional stores will fail until proven
    // otherwise.
    bool allowStore = !isLLSC;

    // Iterate over list.  Note that there could be multiple matching records,
    // as more than one context could have done a load locked to this location.
    // Only remove records when we succeed in finding a record for (xc, addr);
    // then, remove all records with this address.  Failed store-conditionals do
    // not blow unrelated reservations.
    std::list<LockedAddr>::iterator i = lockedAddrList.begin();

    if (isLLSC) {
        while (i != lockedAddrList.end()) {
            if (i->addr == paddr && i->matchesContext(req)) {
                // it's a store conditional, and as far as the memory system can
                // tell, the requesting context's lock is still valid.
                DPRINTF(LLSC, "StCond success: context %d addr %#x\n",
                        req->contextId(), paddr);
                allowStore = true;
                break;
            }
            // If we didn't find a match, keep searching!  Someone else may well
            // have a reservation on this line here but we may find ours in just
            // a little while.
            i++;
        }
        req->setExtraData(allowStore ? 1 : 0);
    }
    // LLSCs that succeeded AND non-LLSC stores both fall into here:
    if (allowStore) {
        // We write address paddr.  However, there may be several entries with a
        // reservation on this address (for other contextIds) and they must all
        // be removed.
        i = lockedAddrList.begin();
        while (i != lockedAddrList.end()) {
            if (i->addr == paddr) {
                DPRINTF(LLSC, "Erasing lock record: context %d addr %#x\n",
                        i->contextId, paddr);
                ContextID owner_cid = i->contextId;
                assert(owner_cid != InvalidContextID);
                ContextID requestor_cid = req->hasContextId() ?
                                           req->contextId() :
                                           InvalidContextID;
                if (owner_cid != requestor_cid) {
                    ThreadContext* ctx = system()->threads[owner_cid];
                    ctx->getIsaPtr()->globalClearExclusive();
                }
                i = lockedAddrList.erase(i);
            } else {
                i++;
            }
        }
    }

    return allowStore;
}

#if TRACING_ON
static inline void
tracePacket(System *sys, const char *label, PacketPtr pkt)
{
    int size = pkt->getSize();
    if (size == 1 || size == 2 || size == 4 || size == 8) {
        ByteOrder byte_order = sys->getGuestByteOrder();
        DPRINTF(MemoryAccess, "%s from %s of size %i on address %#x data "
                "%#x %c\n", label, sys->getRequestorName(pkt->req->
                requestorId()), size, pkt->getAddr(),
                pkt->getUintX(byte_order),
                pkt->req->isUncacheable() ? 'U' : 'C');
        return;
    }
    DPRINTF(MemoryAccess, "%s from %s of size %i on address %#x %c\n",
            label, sys->getRequestorName(pkt->req->requestorId()),
            size, pkt->getAddr(), pkt->req->isUncacheable() ? 'U' : 'C');
    DDUMP(MemoryAccess, pkt->getConstPtr<uint8_t>(), pkt->getSize());
}

#   define TRACE_PACKET(A) tracePacket(system(), A, pkt)
#else
#   define TRACE_PACKET(A)
#endif

void
AbstractMemory::access(PacketPtr pkt)
{
    if (pkt->cacheResponding()) {
        DPRINTF(MemoryAccess, "Cache responding to %#llx: not responding\n",
                pkt->getAddr());
        return;
    }

    if (pkt->cmd == MemCmd::CleanEvict || pkt->cmd == MemCmd::WritebackClean) {
        DPRINTF(MemoryAccess, "CleanEvict  on 0x%x: not responding\n",
                pkt->getAddr());
      return;
    }

    assert(pkt->getAddrRange().isSubset(range));

    uint8_t *host_addr = toHostAddr(pkt->getAddr());

    if (pkt->cmd == MemCmd::SwapReq) {
        if (pkt->isAtomicOp()) {
            if (pmemAddr) {
                pkt->setData(host_addr);
                (*(pkt->getAtomicOp()))(host_addr);
            }
        } else {
            std::vector<uint8_t> overwrite_val(pkt->getSize());
            uint64_t condition_val64;
            uint32_t condition_val32;

            panic_if(!pmemAddr, "Swap only works if there is real memory " \
                     "(i.e. null=False)");

            bool overwrite_mem = true;
            // keep a copy of our possible write value, and copy what is at the
            // memory address into the packet
            pkt->writeData(&overwrite_val[0]);
            pkt->setData(host_addr);

            if (pkt->req->isCondSwap()) {
                if (pkt->getSize() == sizeof(uint64_t)) {
                    condition_val64 = pkt->req->getExtraData();
                    overwrite_mem = !std::memcmp(&condition_val64, host_addr,
                                                 sizeof(uint64_t));
                } else if (pkt->getSize() == sizeof(uint32_t)) {
                    condition_val32 = (uint32_t)pkt->req->getExtraData();
                    overwrite_mem = !std::memcmp(&condition_val32, host_addr,
                                                 sizeof(uint32_t));
                } else
                    panic("Invalid size for conditional read/write\n");
            }

            if (overwrite_mem)
                std::memcpy(host_addr, &overwrite_val[0], pkt->getSize());

            assert(!pkt->req->isInstFetch());
            TRACE_PACKET("Read/Write");
            stats.numOther[pkt->req->requestorId()]++;
        }
    } else if (pkt->isRead()) {
        assert(!pkt->isWrite());
        if (pkt->isLLSC()) {
            assert(!pkt->fromCache());
            // if the packet is not coming from a cache then we have
            // to do the LL/SC tracking here
            trackLoadLocked(pkt);
        }
        if (pmemAddr) {
            pkt->setData(host_addr);
        }
        TRACE_PACKET(pkt->req->isInstFetch() ? "IFetch" : "Read");
        stats.numReads[pkt->req->requestorId()]++;
        stats.bytesRead[pkt->req->requestorId()] += pkt->getSize();
        if (pkt->req->isInstFetch())
            stats.bytesInstRead[pkt->req->requestorId()] += pkt->getSize();
    } else if (pkt->isInvalidate() || pkt->isClean()) {
        assert(!pkt->isWrite());
        // in a fastmem system invalidating and/or cleaning packets
        // can be seen due to cache maintenance requests

        // no need to do anything
    } else if (pkt->isWrite()) {
        if (writeOK(pkt)) {
            if (pmemAddr) {
                pkt->writeData(host_addr);
                DPRINTF(MemoryAccess, "%s write due to %s\n",
                        __func__, pkt->print());
            }
            assert(!pkt->req->isInstFetch());
            TRACE_PACKET("Write");
            stats.numWrites[pkt->req->requestorId()]++;
            stats.bytesWritten[pkt->req->requestorId()] += pkt->getSize();
        }
    } else {
        panic("Unexpected packet %s", pkt->print());
    }

    if (pkt->needsResponse()) {
        pkt->makeResponse();
    }
}

void
AbstractMemory::functionalAccess(PacketPtr pkt)
{
    assert(pkt->getAddrRange().isSubset(range));

    uint8_t *host_addr = toHostAddr(pkt->getAddr());

    if (pkt->isRead()) {
        if (pmemAddr) {
            pkt->setData(host_addr);
        }
        TRACE_PACKET("Read");
        pkt->makeResponse();
    } else if (pkt->isWrite()) {
        if (pmemAddr) {
            pkt->writeData(host_addr);
        }
        TRACE_PACKET("Write");
        pkt->makeResponse();
    } else if (pkt->isPrint()) {
        Packet::PrintReqState *prs =
            dynamic_cast<Packet::PrintReqState*>(pkt->senderState);
        assert(prs);
        // Need to call printLabels() explicitly since we're not going
        // through printObj().
        prs->printLabels();
        // Right now we just print the single byte at the specified address.
        ccprintf(prs->os, "%s%#x\n", prs->curPrefix(), *host_addr);
    } else {
        panic("AbstractMemory: unimplemented functional command %s",
              pkt->cmdString());
    }
}

} // namespace memory
} // namespace gem5
