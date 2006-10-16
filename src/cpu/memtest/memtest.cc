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
#include "base/statistics.hh"
//#include "cpu/simple_thread.hh"
#include "cpu/memtest/memtest.hh"
//#include "mem/cache/base_cache.hh"
//#include "mem/physical.hh"
#include "sim/builder.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "mem/port.hh"
#include "mem/mem_object.hh"

using namespace std;

int TESTER_ALLOCATOR=0;

bool
MemTest::CpuPort::recvTiming(Packet *pkt)
{
    memtest->completeRequest(pkt);
    return true;
}

Tick
MemTest::CpuPort::recvAtomic(Packet *pkt)
{
    panic("MemTest doesn't expect recvAtomic callback!");
    return curTick;
}

void
MemTest::CpuPort::recvFunctional(Packet *pkt)
{
    //Do nothing if we see one come through
    if (curTick != 0)//Supress warning durring initialization
        warn("Functional Writes not implemented in MemTester\n");
    //Need to find any response values that intersect and update
    return;
}

void
MemTest::CpuPort::recvStatusChange(Status status)
{
    if (status == RangeChange)
        return;

    panic("MemTest doesn't expect recvStatusChange callback!");
}

void
MemTest::CpuPort::recvRetry()
{
    memtest->doRetry();
}

void
MemTest::sendPkt(Packet *pkt) {
    if (atomic) {
        cachePort.sendAtomic(pkt);
        pkt->makeAtomicResponse();
        completeRequest(pkt);
    }
    else if (!cachePort.sendTiming(pkt)) {
        accessRetry = true;
        retryPkt = pkt;
    }

}

MemTest::MemTest(const string &name,
//		 MemInterface *_cache_interface,
//		 PhysicalMemory *main_mem,
//		 PhysicalMemory *check_mem,
                 unsigned _memorySize,
                 unsigned _percentReads,
//		 unsigned _percentCopies,
                 unsigned _percentUncacheable,
                 unsigned _progressInterval,
                 unsigned _percentSourceUnaligned,
                 unsigned _percentDestUnaligned,
                 Addr _traceAddr,
                 Counter _max_loads,
                 bool _atomic)
    : MemObject(name),
      tickEvent(this),
      cachePort("test", this),
      funcPort("functional", this),
      retryPkt(NULL),
//      mainMem(main_mem),
//      checkMem(check_mem),
      size(_memorySize),
      percentReads(_percentReads),
//      percentCopies(_percentCopies),
      percentUncacheable(_percentUncacheable),
      progressInterval(_progressInterval),
      nextProgressMessage(_progressInterval),
      percentSourceUnaligned(_percentSourceUnaligned),
      percentDestUnaligned(percentDestUnaligned),
      maxLoads(_max_loads),
      atomic(_atomic)
{
    vector<string> cmd;
    cmd.push_back("/bin/ls");
    vector<string> null_vec;
    //  thread = new SimpleThread(NULL, 0, NULL, 0, mainMem);
    curTick = 0;

    // Needs to be masked off once we know the block size.
    traceBlockAddr = _traceAddr;
    baseAddr1 = 0x100000;
    baseAddr2 = 0x400000;
    uncacheAddr = 0x800000;

    // set up counters
    noResponseCycles = 0;
    numReads = 0;
    tickEvent.schedule(0);

    id = TESTER_ALLOCATOR++;
    if (TESTER_ALLOCATOR > 8)
        panic("False sharing memtester only allows up to 8 testers");

    accessRetry = false;
}

Port *
MemTest::getPort(const std::string &if_name, int idx)
{
    if (if_name == "functional")
        return &funcPort;
    else if (if_name == "test")
        return &cachePort;
    else
        panic("No Such Port\n");
}

void
MemTest::init()
{
    // By the time init() is called, the ports should be hooked up.
    blockSize = cachePort.peerBlockSize();
    blockAddrMask = blockSize - 1;
    traceBlockAddr = blockAddr(traceBlockAddr);

    // set up intial memory contents here

    cachePort.memsetBlob(baseAddr1, 1, size);
    funcPort.memsetBlob(baseAddr1, 1, size);
    cachePort.memsetBlob(baseAddr2, 2, size);
    funcPort.memsetBlob(baseAddr2, 2, size);
    cachePort.memsetBlob(uncacheAddr, 3, size);
    funcPort.memsetBlob(uncacheAddr, 3, size);
}

static void
printData(ostream &os, uint8_t *data, int nbytes)
{
    os << hex << setfill('0');
    // assume little-endian: print bytes from highest address to lowest
    for (uint8_t *dp = data + nbytes - 1; dp >= data; --dp) {
        os << setw(2) << (unsigned)*dp;
    }
    os << dec;
}

void
MemTest::completeRequest(Packet *pkt)
{
    MemTestSenderState *state =
        dynamic_cast<MemTestSenderState *>(pkt->senderState);

    uint8_t *data = state->data;
    uint8_t *pkt_data = pkt->getPtr<uint8_t>();
    Request *req = pkt->req;

    //Remove the address from the list of outstanding
    std::set<unsigned>::iterator removeAddr = outstandingAddrs.find(req->getPaddr());
    assert(removeAddr != outstandingAddrs.end());
    outstandingAddrs.erase(removeAddr);

    switch (pkt->cmd) {
      case Packet::ReadResp:

        if (memcmp(pkt_data, data, pkt->getSize()) != 0) {
            cerr << name() << ": on read of 0x" << hex << req->getPaddr()
                 << " (0x" << hex << blockAddr(req->getPaddr()) << ")"
                 << "@ cycle " << dec << curTick
                 << ", cache returns 0x";
            printData(cerr, pkt_data, pkt->getSize());
            cerr << ", expected 0x";
            printData(cerr, data, pkt->getSize());
            cerr << endl;
            fatal("");
        }

        numReads++;
        numReadsStat++;

        if (numReads == nextProgressMessage) {
            ccprintf(cerr, "%s: completed %d read accesses @%d\n",
                     name(), numReads, curTick);
            nextProgressMessage += progressInterval;
        }

        if (numReads >= maxLoads)
            exitSimLoop("Maximum number of loads reached!");
        break;

      case Packet::WriteResp:
        numWritesStat++;
        break;
/*
      case Copy:
        //Also remove dest from outstanding list
        removeAddr = outstandingAddrs.find(req->dest);
        assert(removeAddr != outstandingAddrs.end());
        outstandingAddrs.erase(removeAddr);
        numCopiesStat++;
        break;
*/
      default:
        panic("invalid command");
    }

    if (blockAddr(req->getPaddr()) == traceBlockAddr) {
        cerr << name() << ": completed "
             << (pkt->isWrite() ? "write" : "read")
             << " access of "
             << dec << pkt->getSize() << " bytes at address 0x"
             << hex << req->getPaddr()
             << " (0x" << hex << blockAddr(req->getPaddr()) << ")"
             << ", value = 0x";
        printData(cerr, pkt_data, pkt->getSize());
        cerr << " @ cycle " << dec << curTick;

        cerr << endl;
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
        tickEvent.schedule(curTick + cycles(1));

    if (++noResponseCycles >= 500000) {
        cerr << name() << ": deadlocked at cycle " << curTick << endl;
        fatal("");
    }

    if (accessRetry) {
        return;
    }

    //make new request
    unsigned cmd = random() % 100;
    unsigned offset = random() % size;
    unsigned base = random() % 2;
    uint64_t data = random();
    unsigned access_size = random() % 4;
    unsigned cacheable = random() % 100;

    //If we aren't doing copies, use id as offset, and do a false sharing
    //mem tester
    //We can eliminate the lower bits of the offset, and then use the id
    //to offset within the blks
    offset &= ~63; //Not the low order bits
    offset += id;
    access_size = 0;

    Request *req = new Request();
    uint32_t flags = 0;
    Addr paddr;

    if (cacheable < percentUncacheable) {
        flags |= UNCACHEABLE;
        paddr = uncacheAddr + offset;
    } else {
        paddr = ((base) ? baseAddr1 : baseAddr2) + offset;
    }
    //bool probe = (random() % 2 == 1) && !req->isUncacheable();
    bool probe = false;

    paddr &= ~((1 << access_size) - 1);
    req->setPhys(paddr, 1 << access_size, flags);
    req->setThreadContext(id,0);

    uint8_t *result = new uint8_t[8];

    if (cmd < percentReads) {
        // read

        //For now we only allow one outstanding request per addreess per tester
        //This means we assume CPU does write forwarding to reads that alias something
        //in the cpu store buffer.
        if (outstandingAddrs.find(paddr) != outstandingAddrs.end()) {
            delete result;
            delete req;
            return;
        }
        else outstandingAddrs.insert(paddr);

        // ***** NOTE FOR RON: I'm not sure how to access checkMem. - Kevin
        funcPort.readBlob(req->getPaddr(), result, req->getSize());

        if (blockAddr(paddr) == traceBlockAddr) {
            cerr << name()
                 << ": initiating read "
                 << ((probe) ? "probe of " : "access of ")
                 << dec << req->getSize() << " bytes from addr 0x"
                 << hex << paddr
                 << " (0x" << hex << blockAddr(paddr) << ")"
                 << " at cycle "
                 << dec << curTick << endl;
        }

        Packet *pkt = new Packet(req, Packet::ReadReq, Packet::Broadcast);
        pkt->dataDynamicArray(new uint8_t[req->getSize()]);
        MemTestSenderState *state = new MemTestSenderState(result);
        pkt->senderState = state;

        if (probe) {
            cachePort.sendFunctional(pkt);
            completeRequest(pkt);
        } else {
//	    req->completionEvent = new MemCompleteEvent(req, result, this);
            sendPkt(pkt);
        }
    } else {
        // write

        //For now we only allow one outstanding request per addreess per tester
        //This means we assume CPU does write forwarding to reads that alias something
        //in the cpu store buffer.
        if (outstandingAddrs.find(paddr) != outstandingAddrs.end()) {
            delete [] result;
            delete req;
            return;
        }

        else outstandingAddrs.insert(paddr);

/*
        if (blockAddr(req->getPaddr()) == traceBlockAddr) {
            cerr << name() << ": initiating write "
                 << ((probe)?"probe of ":"access of ")
                 << dec << req->getSize() << " bytes (value = 0x";
            printData(cerr, data_pkt->getPtr(), req->getSize());
            cerr << ") to addr 0x"
                 << hex << req->getPaddr()
                 << " (0x" << hex << blockAddr(req->getPaddr()) << ")"
                 << " at cycle "
                 << dec << curTick << endl;
        }
*/
        Packet *pkt = new Packet(req, Packet::WriteReq, Packet::Broadcast);
        uint8_t *pkt_data = new uint8_t[req->getSize()];
        pkt->dataDynamicArray(pkt_data);
        memcpy(pkt_data, &data, req->getSize());
        MemTestSenderState *state = new MemTestSenderState(result);
        pkt->senderState = state;

        funcPort.writeBlob(req->getPaddr(), pkt_data, req->getSize());

        if (probe) {
            cachePort.sendFunctional(pkt);
            completeRequest(pkt);
        } else {
//	    req->completionEvent = new MemCompleteEvent(req, NULL, this);
            sendPkt(pkt);
        }
    }
/*    else {
        // copy
        unsigned source_align = random() % 100;
        unsigned dest_align = random() % 100;
        unsigned offset2 = random() % size;

        Addr source = ((base) ? baseAddr1 : baseAddr2) + offset;
        Addr dest = ((base) ? baseAddr2 : baseAddr1) + offset2;
        if (outstandingAddrs.find(source) != outstandingAddrs.end()) return;
        else outstandingAddrs.insert(source);
        if (outstandingAddrs.find(dest) != outstandingAddrs.end()) return;
        else outstandingAddrs.insert(dest);

        if (source_align >= percentSourceUnaligned) {
            source = blockAddr(source);
        }
        if (dest_align >= percentDestUnaligned) {
            dest = blockAddr(dest);
        }
        req->cmd = Copy;
        req->flags &= ~UNCACHEABLE;
        req->paddr = source;
        req->dest = dest;
        delete [] req->data;
        req->data = new uint8_t[blockSize];
        req->size = blockSize;
        if (source == traceBlockAddr || dest == traceBlockAddr) {
            cerr << name()
                 << ": initiating copy of "
                 << dec << req->size << " bytes from addr 0x"
                 << hex << source
                 << " (0x" << hex << blockAddr(source) << ")"
                 << " to addr 0x"
                 << hex << dest
                 << " (0x" << hex << blockAddr(dest) << ")"
                 << " at cycle "
                 << dec << curTick << endl;
        }*
        cacheInterface->access(req);
        uint8_t result[blockSize];
        checkMem->access(Read, source, &result, blockSize);
        checkMem->access(Write, dest, &result, blockSize);
    }
*/
}

void
MemTest::doRetry()
{
    if (cachePort.sendTiming(retryPkt)) {
        accessRetry = false;
        retryPkt = NULL;
    }
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(MemTest)

//    SimObjectParam<BaseCache *> cache;
//    SimObjectParam<PhysicalMemory *> main_mem;
//    SimObjectParam<PhysicalMemory *> check_mem;
    Param<unsigned> memory_size;
    Param<unsigned> percent_reads;
//    Param<unsigned> percent_copies;
    Param<unsigned> percent_uncacheable;
    Param<unsigned> progress_interval;
    Param<unsigned> percent_source_unaligned;
    Param<unsigned> percent_dest_unaligned;
    Param<Addr> trace_addr;
    Param<Counter> max_loads;
    Param<bool> atomic;

END_DECLARE_SIM_OBJECT_PARAMS(MemTest)


BEGIN_INIT_SIM_OBJECT_PARAMS(MemTest)

//    INIT_PARAM(cache, "L1 cache"),
//    INIT_PARAM(main_mem, "hierarchical memory"),
//    INIT_PARAM(check_mem, "check memory"),
    INIT_PARAM(memory_size, "memory size"),
    INIT_PARAM(percent_reads, "target read percentage"),
//    INIT_PARAM(percent_copies, "target copy percentage"),
    INIT_PARAM(percent_uncacheable, "target uncacheable percentage"),
    INIT_PARAM(progress_interval, "progress report interval (in accesses)"),
    INIT_PARAM(percent_source_unaligned,
               "percent of copy source address that are unaligned"),
    INIT_PARAM(percent_dest_unaligned,
               "percent of copy dest address that are unaligned"),
    INIT_PARAM(trace_addr, "address to trace"),
                              INIT_PARAM(max_loads, "terminate when we have reached this load count"),
    INIT_PARAM(atomic, "Is the tester testing atomic mode (or timing)")

END_INIT_SIM_OBJECT_PARAMS(MemTest)


CREATE_SIM_OBJECT(MemTest)
{
    return new MemTest(getInstanceName(), /*cache->getInterface(),*/ /*main_mem,*/
                       /*check_mem,*/ memory_size, percent_reads, /*percent_copies,*/
                       percent_uncacheable, progress_interval,
                       percent_source_unaligned, percent_dest_unaligned,
                       trace_addr, max_loads, atomic);
}

REGISTER_SIM_OBJECT("MemTest", MemTest)
