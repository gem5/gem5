/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 * Authors: Erik Hallnor
 *          Steve Reinhardt
 */

// FIX ME: make trackBlkAddr use blocksize from actual cache, not hard coded

#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/random.hh"
#include "base/statistics.hh"
#include "cpu/testers/memtest/memtest.hh"
#include "debug/MemTest.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "mem/request.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace std;

int TESTER_ALLOCATOR=0;

bool
MemTest::CpuPort::recvTimingResp(PacketPtr pkt)
{
    memtest->completeRequest(pkt);
    return true;
}

void
MemTest::CpuPort::recvRetry()
{
    memtest->doRetry();
}

void
MemTest::sendPkt(PacketPtr pkt) {
    if (atomic) {
        cachePort.sendAtomic(pkt);
        completeRequest(pkt);
    }
    else if (!cachePort.sendTimingReq(pkt)) {
        DPRINTF(MemTest, "accessRetry setting to true\n");

        //
        // dma requests should never be retried
        //
        if (issueDmas) {
            panic("Nacked DMA requests are not supported\n");
        }
        accessRetry = true;
        retryPkt = pkt;
    } else {
        if (issueDmas) {
            dmaOutstanding = true;
        }
    }

}

MemTest::MemTest(const Params *p)
    : MemObject(p),
      tickEvent(this),
      cachePort("test", this),
      funcPort("functional", this),
      funcProxy(funcPort, p->sys->cacheLineSize()),
      retryPkt(NULL),
//      mainMem(main_mem),
//      checkMem(check_mem),
      size(p->memory_size),
      percentReads(p->percent_reads),
      percentFunctional(p->percent_functional),
      percentUncacheable(p->percent_uncacheable),
      issueDmas(p->issue_dmas),
      masterId(p->sys->getMasterId(name())),
      blockSize(p->sys->cacheLineSize()),
      progressInterval(p->progress_interval),
      nextProgressMessage(p->progress_interval),
      percentSourceUnaligned(p->percent_source_unaligned),
      percentDestUnaligned(p->percent_dest_unaligned),
      maxLoads(p->max_loads),
      atomic(p->atomic),
      suppress_func_warnings(p->suppress_func_warnings)
{
    id = TESTER_ALLOCATOR++;

    // Needs to be masked off once we know the block size.
    traceBlockAddr = p->trace_addr;
    baseAddr1 = 0x100000;
    baseAddr2 = 0x400000;
    uncacheAddr = 0x800000;

    blockAddrMask = blockSize - 1;
    traceBlockAddr = blockAddr(traceBlockAddr);

    // set up counters
    noResponseCycles = 0;
    numReads = 0;
    numWrites = 0;
    schedule(tickEvent, 0);

    accessRetry = false;
    dmaOutstanding = false;
}

BaseMasterPort &
MemTest::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "functional")
        return funcPort;
    else if (if_name == "test")
        return cachePort;
    else
        return MemObject::getMasterPort(if_name, idx);
}

void
MemTest::init()
{
    // initial memory contents for both physical memory and functional
    // memory should be 0; no need to initialize them.
}


void
MemTest::completeRequest(PacketPtr pkt)
{
    Request *req = pkt->req;

    if (issueDmas) {
        dmaOutstanding = false;
    }

    DPRINTF(MemTest, "completing %s at address %x (blk %x) %s\n",
            pkt->isWrite() ? "write" : "read",
            req->getPaddr(), blockAddr(req->getPaddr()),
            pkt->isError() ? "error" : "success");

    MemTestSenderState *state =
        dynamic_cast<MemTestSenderState *>(pkt->senderState);

    uint8_t *data = state->data;
    uint8_t *pkt_data = pkt->getPtr<uint8_t>();

    //Remove the address from the list of outstanding
    std::set<unsigned>::iterator removeAddr =
        outstandingAddrs.find(req->getPaddr());
    assert(removeAddr != outstandingAddrs.end());
    outstandingAddrs.erase(removeAddr);

    if (pkt->isError()) {
        if (!suppress_func_warnings) {
          warn("Functional %s access failed at %#x\n",
               pkt->isWrite() ? "write" : "read", req->getPaddr());
        }
    } else {
        if (pkt->isRead()) {
            if (memcmp(pkt_data, data, pkt->getSize()) != 0) {
                panic("%s: read of %x (blk %x) @ cycle %d "
                      "returns %x, expected %x\n", name(),
                      req->getPaddr(), blockAddr(req->getPaddr()), curTick(),
                      *pkt_data, *data);
            }

            numReads++;
            numReadsStat++;

            if (numReads == (uint64_t)nextProgressMessage) {
                ccprintf(cerr, "%s: completed %d read, %d write accesses @%d\n",
                         name(), numReads, numWrites, curTick());
                nextProgressMessage += progressInterval;
            }

            if (maxLoads != 0 && numReads >= maxLoads)
                exitSimLoop("maximum number of loads reached");
        } else {
            assert(pkt->isWrite());
            funcProxy.writeBlob(req->getPaddr(), pkt_data, req->getSize());
            numWrites++;
            numWritesStat++;
        }
    }

    noResponseCycles = 0;
    delete state;
    delete [] data;
    delete pkt->req;
    delete pkt;
}

void
MemTest::regStats()
{
    using namespace Stats;

    numReadsStat
        .name(name() + ".num_reads")
        .desc("number of read accesses completed")
        ;

    numWritesStat
        .name(name() + ".num_writes")
        .desc("number of write accesses completed")
        ;

    numCopiesStat
        .name(name() + ".num_copies")
        .desc("number of copy accesses completed")
        ;
}

void
MemTest::tick()
{
    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(1)));

    if (++noResponseCycles >= 500000) {
        if (issueDmas) {
            cerr << "DMA tester ";
        }
        cerr << name() << ": deadlocked at cycle " << curTick() << endl;
        fatal("");
    }

    if (accessRetry || (issueDmas && dmaOutstanding)) {
        DPRINTF(MemTest, "MemTester waiting on accessRetry or DMA response\n");
        return;
    }

    //make new request
    unsigned cmd = random_mt.random(0, 100);
    unsigned offset = random_mt.random<unsigned>(0, size - 1);
    unsigned base = random_mt.random(0, 1);
    uint64_t data = random_mt.random<uint64_t>();
    unsigned access_size = random_mt.random(0, 3);
    bool uncacheable = random_mt.random(0, 100) < percentUncacheable;

    unsigned dma_access_size = random_mt.random(0, 3);

    //If we aren't doing copies, use id as offset, and do a false sharing
    //mem tester
    //We can eliminate the lower bits of the offset, and then use the id
    //to offset within the blks
    offset = blockAddr(offset);
    offset += id;
    access_size = 0;
    dma_access_size = 0;

    Request::Flags flags;
    Addr paddr;

    if (uncacheable) {
        flags.set(Request::UNCACHEABLE);
        paddr = uncacheAddr + offset;
    } else  {
        paddr = ((base) ? baseAddr1 : baseAddr2) + offset;
    }

    // For now we only allow one outstanding request per address
    // per tester This means we assume CPU does write forwarding
    // to reads that alias something in the cpu store buffer.
    if (outstandingAddrs.find(paddr) != outstandingAddrs.end()) {
        return;
    }

    bool do_functional = (random_mt.random(0, 100) < percentFunctional) &&
        !uncacheable;
    Request *req = new Request();
    uint8_t *result = new uint8_t[8];

    if (issueDmas) {
        paddr &= ~((1 << dma_access_size) - 1);
        req->setPhys(paddr, 1 << dma_access_size, flags, masterId);
        req->setThreadContext(id,0);
    } else {
        paddr &= ~((1 << access_size) - 1);
        req->setPhys(paddr, 1 << access_size, flags, masterId);
        req->setThreadContext(id,0);
    }
    assert(req->getSize() == 1);

    if (cmd < percentReads) {
        // read
        outstandingAddrs.insert(paddr);

        // ***** NOTE FOR RON: I'm not sure how to access checkMem. - Kevin
        funcProxy.readBlob(req->getPaddr(), result, req->getSize());

        DPRINTF(MemTest,
                "id %d initiating %sread at addr %x (blk %x) expecting %x\n",
                id, do_functional ? "functional " : "", req->getPaddr(),
                blockAddr(req->getPaddr()), *result);

        PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
        pkt->dataDynamicArray(new uint8_t[req->getSize()]);
        MemTestSenderState *state = new MemTestSenderState(result);
        pkt->senderState = state;

        if (do_functional) {
            assert(pkt->needsResponse());
            pkt->setSuppressFuncError();
            cachePort.sendFunctional(pkt);
            completeRequest(pkt);
        } else {
            sendPkt(pkt);
        }
    } else {
        // write
        outstandingAddrs.insert(paddr);

        DPRINTF(MemTest, "initiating %swrite at addr %x (blk %x) value %x\n",
                do_functional ? "functional " : "", req->getPaddr(),
                blockAddr(req->getPaddr()), data & 0xff);

        PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
        uint8_t *pkt_data = new uint8_t[req->getSize()];
        pkt->dataDynamicArray(pkt_data);
        memcpy(pkt_data, &data, req->getSize());
        MemTestSenderState *state = new MemTestSenderState(result);
        pkt->senderState = state;

        if (do_functional) {
            pkt->setSuppressFuncError();
            cachePort.sendFunctional(pkt);
            completeRequest(pkt);
        } else {
            sendPkt(pkt);
        }
    }
}

void
MemTest::doRetry()
{
    if (cachePort.sendTimingReq(retryPkt)) {
        DPRINTF(MemTest, "accessRetry setting to false\n");
        accessRetry = false;
        retryPkt = NULL;
    }
}


void
MemTest::printAddr(Addr a)
{
    cachePort.printAddr(a);
}


MemTest *
MemTestParams::create()
{
    return new MemTest(this);
}
