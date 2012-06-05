/*
 * Copyright (c) 2010-2012 ARM Limited
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
 *
 * Authors: Ron Dreslinski
 *          Ali Saidi
 *          Andreas Hansson
 */

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/user.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <cerrno>
#include <cstdio>
#include <iostream>
#include <string>

#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "debug/LLSC.hh"
#include "debug/MemoryAccess.hh"
#include "mem/abstract_mem.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

using namespace std;

AbstractMemory::AbstractMemory(const Params *p) :
    MemObject(p), range(params()->range), pmemAddr(NULL),
    confTableReported(p->conf_table_reported), inAddrMap(p->in_addr_map),
    _system(NULL)
{
    if (size() % TheISA::PageBytes != 0)
        panic("Memory Size not divisible by page size\n");

    if (params()->null)
        return;

    if (params()->file == "") {
        int map_flags = MAP_ANON | MAP_PRIVATE;
        pmemAddr = (uint8_t *)mmap(NULL, size(),
                                   PROT_READ | PROT_WRITE, map_flags, -1, 0);
    } else {
        int map_flags = MAP_PRIVATE;
        int fd = open(params()->file.c_str(), O_RDONLY);
        long _size = lseek(fd, 0, SEEK_END);
        if (_size != range.size()) {
            warn("Specified size %d does not match file %s %d\n", range.size(),
                 params()->file, _size);
            range = RangeSize(range.start, _size);
        }
        lseek(fd, 0, SEEK_SET);
        pmemAddr = (uint8_t *)mmap(NULL, roundUp(_size, sysconf(_SC_PAGESIZE)),
                                   PROT_READ | PROT_WRITE, map_flags, fd, 0);
    }

    if (pmemAddr == (void *)MAP_FAILED) {
        perror("mmap");
        if (params()->file == "")
            fatal("Could not mmap!\n");
        else
            fatal("Could not find file: %s\n", params()->file);
    }

    //If requested, initialize all the memory to 0
    if (p->zero)
        memset(pmemAddr, 0, size());
}


AbstractMemory::~AbstractMemory()
{
    if (pmemAddr)
        munmap((char*)pmemAddr, size());
}

void
AbstractMemory::regStats()
{
    using namespace Stats;

    assert(system());

    bytesRead
        .init(system()->maxMasters())
        .name(name() + ".bytes_read")
        .desc("Number of bytes read from this memory")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bytesRead.subname(i, system()->getMasterName(i));
    }
    bytesInstRead
        .init(system()->maxMasters())
        .name(name() + ".bytes_inst_read")
        .desc("Number of instructions bytes read from this memory")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bytesInstRead.subname(i, system()->getMasterName(i));
    }
    bytesWritten
        .init(system()->maxMasters())
        .name(name() + ".bytes_written")
        .desc("Number of bytes written to this memory")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bytesWritten.subname(i, system()->getMasterName(i));
    }
    numReads
        .init(system()->maxMasters())
        .name(name() + ".num_reads")
        .desc("Number of read requests responded to by this memory")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        numReads.subname(i, system()->getMasterName(i));
    }
    numWrites
        .init(system()->maxMasters())
        .name(name() + ".num_writes")
        .desc("Number of write requests responded to by this memory")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        numWrites.subname(i, system()->getMasterName(i));
    }
    numOther
        .init(system()->maxMasters())
        .name(name() + ".num_other")
        .desc("Number of other requests responded to by this memory")
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        numOther.subname(i, system()->getMasterName(i));
    }
    bwRead
        .name(name() + ".bw_read")
        .desc("Total read bandwidth from this memory (bytes/s)")
        .precision(0)
        .prereq(bytesRead)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bwRead.subname(i, system()->getMasterName(i));
    }

    bwInstRead
        .name(name() + ".bw_inst_read")
        .desc("Instruction read bandwidth from this memory (bytes/s)")
        .precision(0)
        .prereq(bytesInstRead)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bwInstRead.subname(i, system()->getMasterName(i));
    }
    bwWrite
        .name(name() + ".bw_write")
        .desc("Write bandwidth from this memory (bytes/s)")
        .precision(0)
        .prereq(bytesWritten)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bwWrite.subname(i, system()->getMasterName(i));
    }
    bwTotal
        .name(name() + ".bw_total")
        .desc("Total bandwidth to/from this memory (bytes/s)")
        .precision(0)
        .prereq(bwTotal)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < system()->maxMasters(); i++) {
        bwTotal.subname(i, system()->getMasterName(i));
    }
    bwRead = bytesRead / simSeconds;
    bwInstRead = bytesInstRead / simSeconds;
    bwWrite = bytesWritten / simSeconds;
    bwTotal = (bytesRead + bytesWritten) / simSeconds;
}

Range<Addr>
AbstractMemory::getAddrRange()
{
    return range;
}

// Add load-locked to tracking list.  Should only be called if the
// operation is a load and the LLSC flag is set.
void
AbstractMemory::trackLoadLocked(PacketPtr pkt)
{
    Request *req = pkt->req;
    Addr paddr = LockedAddr::mask(req->getPaddr());

    // first we check if we already have a locked addr for this
    // xc.  Since each xc only gets one, we just update the
    // existing record with the new address.
    list<LockedAddr>::iterator i;

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
}


// Called on *writes* only... both regular stores and
// store-conditional operations.  Check for conventional stores which
// conflict with locked addresses, and for success/failure of store
// conditionals.
bool
AbstractMemory::checkLockedAddrList(PacketPtr pkt)
{
    Request *req = pkt->req;
    Addr paddr = LockedAddr::mask(req->getPaddr());
    bool isLLSC = pkt->isLLSC();

    // Initialize return value.  Non-conditional stores always
    // succeed.  Assume conditional stores will fail until proven
    // otherwise.
    bool success = !isLLSC;

    // Iterate over list.  Note that there could be multiple matching
    // records, as more than one context could have done a load locked
    // to this location.
    list<LockedAddr>::iterator i = lockedAddrList.begin();

    while (i != lockedAddrList.end()) {

        if (i->addr == paddr) {
            // we have a matching address

            if (isLLSC && i->matchesContext(req)) {
                // it's a store conditional, and as far as the memory
                // system can tell, the requesting context's lock is
                // still valid.
                DPRINTF(LLSC, "StCond success: context %d addr %#x\n",
                        req->contextId(), paddr);
                success = true;
            }

            // Get rid of our record of this lock and advance to next
            DPRINTF(LLSC, "Erasing lock record: context %d addr %#x\n",
                    i->contextId, paddr);
            i = lockedAddrList.erase(i);
        }
        else {
            // no match: advance to next record
            ++i;
        }
    }

    if (isLLSC) {
        req->setExtraData(success ? 1 : 0);
    }

    return success;
}


#if TRACING_ON

#define CASE(A, T)                                                      \
  case sizeof(T):                                                       \
    DPRINTF(MemoryAccess,"%s of size %i on address 0x%x data 0x%x\n",   \
            A, pkt->getSize(), pkt->getAddr(), pkt->get<T>());          \
  break


#define TRACE_PACKET(A)                                                 \
    do {                                                                \
        switch (pkt->getSize()) {                                       \
          CASE(A, uint64_t);                                            \
          CASE(A, uint32_t);                                            \
          CASE(A, uint16_t);                                            \
          CASE(A, uint8_t);                                             \
          default:                                                      \
            DPRINTF(MemoryAccess, "%s of size %i on address 0x%x\n",    \
                    A, pkt->getSize(), pkt->getAddr());                 \
            DDUMP(MemoryAccess, pkt->getPtr<uint8_t>(), pkt->getSize());\
        }                                                               \
    } while (0)

#else

#define TRACE_PACKET(A)

#endif

void
AbstractMemory::access(PacketPtr pkt)
{
    assert(pkt->getAddr() >= range.start &&
           (pkt->getAddr() + pkt->getSize() - 1) <= range.end);

    if (pkt->memInhibitAsserted()) {
        DPRINTF(MemoryAccess, "mem inhibited on 0x%x: not responding\n",
                pkt->getAddr());
        return;
    }

    uint8_t *hostAddr = pmemAddr + pkt->getAddr() - range.start;

    if (pkt->cmd == MemCmd::SwapReq) {
        TheISA::IntReg overwrite_val;
        bool overwrite_mem;
        uint64_t condition_val64;
        uint32_t condition_val32;

        if (!pmemAddr)
            panic("Swap only works if there is real memory (i.e. null=False)");
        assert(sizeof(TheISA::IntReg) >= pkt->getSize());

        overwrite_mem = true;
        // keep a copy of our possible write value, and copy what is at the
        // memory address into the packet
        std::memcpy(&overwrite_val, pkt->getPtr<uint8_t>(), pkt->getSize());
        std::memcpy(pkt->getPtr<uint8_t>(), hostAddr, pkt->getSize());

        if (pkt->req->isCondSwap()) {
            if (pkt->getSize() == sizeof(uint64_t)) {
                condition_val64 = pkt->req->getExtraData();
                overwrite_mem = !std::memcmp(&condition_val64, hostAddr,
                                             sizeof(uint64_t));
            } else if (pkt->getSize() == sizeof(uint32_t)) {
                condition_val32 = (uint32_t)pkt->req->getExtraData();
                overwrite_mem = !std::memcmp(&condition_val32, hostAddr,
                                             sizeof(uint32_t));
            } else
                panic("Invalid size for conditional read/write\n");
        }

        if (overwrite_mem)
            std::memcpy(hostAddr, &overwrite_val, pkt->getSize());

        assert(!pkt->req->isInstFetch());
        TRACE_PACKET("Read/Write");
        numOther[pkt->req->masterId()]++;
    } else if (pkt->isRead()) {
        assert(!pkt->isWrite());
        if (pkt->isLLSC()) {
            trackLoadLocked(pkt);
        }
        if (pmemAddr)
            memcpy(pkt->getPtr<uint8_t>(), hostAddr, pkt->getSize());
        TRACE_PACKET(pkt->req->isInstFetch() ? "IFetch" : "Read");
        numReads[pkt->req->masterId()]++;
        bytesRead[pkt->req->masterId()] += pkt->getSize();
        if (pkt->req->isInstFetch())
            bytesInstRead[pkt->req->masterId()] += pkt->getSize();
    } else if (pkt->isWrite()) {
        if (writeOK(pkt)) {
            if (pmemAddr)
                memcpy(hostAddr, pkt->getPtr<uint8_t>(), pkt->getSize());
            assert(!pkt->req->isInstFetch());
            TRACE_PACKET("Write");
            numWrites[pkt->req->masterId()]++;
            bytesWritten[pkt->req->masterId()] += pkt->getSize();
        }
    } else if (pkt->isInvalidate()) {
        // no need to do anything
    } else {
        panic("unimplemented");
    }

    if (pkt->needsResponse()) {
        pkt->makeResponse();
    }
}

void
AbstractMemory::functionalAccess(PacketPtr pkt)
{
    assert(pkt->getAddr() >= range.start &&
           (pkt->getAddr() + pkt->getSize() - 1) <= range.end);

    uint8_t *hostAddr = pmemAddr + pkt->getAddr() - range.start;

    if (pkt->isRead()) {
        if (pmemAddr)
            memcpy(pkt->getPtr<uint8_t>(), hostAddr, pkt->getSize());
        TRACE_PACKET("Read");
        pkt->makeResponse();
    } else if (pkt->isWrite()) {
        if (pmemAddr)
            memcpy(hostAddr, pkt->getPtr<uint8_t>(), pkt->getSize());
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
        ccprintf(prs->os, "%s%#x\n", prs->curPrefix(), *hostAddr);
    } else {
        panic("AbstractMemory: unimplemented functional command %s",
              pkt->cmdString());
    }
}

void
AbstractMemory::serialize(ostream &os)
{
    if (!pmemAddr)
        return;

    gzFile compressedMem;
    string filename = name() + ".physmem";
    long _size = range.size();

    SERIALIZE_SCALAR(filename);
    SERIALIZE_SCALAR(_size);

    // write memory file
    string thefile = Checkpoint::dir() + "/" + filename.c_str();
    int fd = creat(thefile.c_str(), 0664);
    if (fd < 0) {
        perror("creat");
        fatal("Can't open physical memory checkpoint file '%s'\n", filename);
    }

    compressedMem = gzdopen(fd, "wb");
    if (compressedMem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
                filename);

    if (gzwrite(compressedMem, pmemAddr, size()) != (int)size()) {
        fatal("Write failed on physical memory checkpoint file '%s'\n",
              filename);
    }

    if (gzclose(compressedMem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

    list<LockedAddr>::iterator i = lockedAddrList.begin();

    vector<Addr> lal_addr;
    vector<int> lal_cid;
    while (i != lockedAddrList.end()) {
        lal_addr.push_back(i->addr);
        lal_cid.push_back(i->contextId);
        i++;
    }
    arrayParamOut(os, "lal_addr", lal_addr);
    arrayParamOut(os, "lal_cid", lal_cid);
}

void
AbstractMemory::unserialize(Checkpoint *cp, const string &section)
{
    if (!pmemAddr)
        return;

    gzFile compressedMem;
    long *tempPage;
    long *pmem_current;
    uint64_t curSize;
    uint32_t bytesRead;
    const uint32_t chunkSize = 16384;

    string filename;

    UNSERIALIZE_SCALAR(filename);

    filename = cp->cptDir + "/" + filename;

    // mmap memoryfile
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }

    compressedMem = gzdopen(fd, "rb");
    if (compressedMem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
                filename);

    // unmap file that was mmapped in the constructor
    // This is done here to make sure that gzip and open don't muck with our
    // nice large space of memory before we reallocate it
    munmap((char*)pmemAddr, size());

    long _size;
    UNSERIALIZE_SCALAR(_size);
    if (_size > params()->range.size())
        fatal("Memory size has changed! size %lld, param size %lld\n",
              _size, params()->range.size());

    pmemAddr = (uint8_t *)mmap(NULL, size(),
        PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);

    if (pmemAddr == (void *)MAP_FAILED) {
        perror("mmap");
        fatal("Could not mmap physical memory!\n");
    }

    curSize = 0;
    tempPage = (long*)malloc(chunkSize);
    if (tempPage == NULL)
        fatal("Unable to malloc memory to read file %s\n", filename);

    /* Only copy bytes that are non-zero, so we don't give the VM system hell */
    while (curSize < size()) {
        bytesRead = gzread(compressedMem, tempPage, chunkSize);
        if (bytesRead == 0)
            break;

        assert(bytesRead % sizeof(long) == 0);

        for (uint32_t x = 0; x < bytesRead / sizeof(long); x++)
        {
             if (*(tempPage+x) != 0) {
                 pmem_current = (long*)(pmemAddr + curSize + x * sizeof(long));
                 *pmem_current = *(tempPage+x);
             }
        }
        curSize += bytesRead;
    }

    free(tempPage);

    if (gzclose(compressedMem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

    vector<Addr> lal_addr;
    vector<int> lal_cid;
    arrayParamIn(cp, section, "lal_addr", lal_addr);
    arrayParamIn(cp, section, "lal_cid", lal_cid);
    for(int i = 0; i < lal_addr.size(); i++)
        lockedAddrList.push_front(LockedAddr(lal_addr[i], lal_cid[i]));
}
