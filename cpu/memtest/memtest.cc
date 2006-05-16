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
 */

// FIX ME: make trackBlkAddr use blocksize from actual cache, not hard coded

#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/statistics.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/memtest/memtest.hh"
#include "mem/cache/base_cache.hh"
#include "sim/builder.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

using namespace std;
using namespace TheISA;

int TESTER_ALLOCATOR=0;

MemTest::MemTest(const string &name,
                 MemInterface *_cache_interface,
                 FunctionalMemory *main_mem,
                 FunctionalMemory *check_mem,
                 unsigned _memorySize,
                 unsigned _percentReads,
                 unsigned _percentCopies,
                 unsigned _percentUncacheable,
                 unsigned _progressInterval,
                 unsigned _percentSourceUnaligned,
                 unsigned _percentDestUnaligned,
                 Addr _traceAddr,
                 Counter _max_loads)
    : SimObject(name),
      tickEvent(this),
      cacheInterface(_cache_interface),
      mainMem(main_mem),
      checkMem(check_mem),
      size(_memorySize),
      percentReads(_percentReads),
      percentCopies(_percentCopies),
      percentUncacheable(_percentUncacheable),
      progressInterval(_progressInterval),
      nextProgressMessage(_progressInterval),
      percentSourceUnaligned(_percentSourceUnaligned),
      percentDestUnaligned(percentDestUnaligned),
      maxLoads(_max_loads)
{
    vector<string> cmd;
    cmd.push_back("/bin/ls");
    vector<string> null_vec;
    cpuXC = new CPUExecContext(NULL, 0, mainMem, 0);

    blockSize = cacheInterface->getBlockSize();
    blockAddrMask = blockSize - 1;
    traceBlockAddr = blockAddr(_traceAddr);

    //setup data storage with interesting values
    uint8_t *data1 = new uint8_t[size];
    uint8_t *data2 = new uint8_t[size];
    uint8_t *data3 = new uint8_t[size];
    memset(data1, 1, size);
    memset(data2, 2, size);
    memset(data3, 3, size);
    curTick = 0;

    baseAddr1 = 0x100000;
    baseAddr2 = 0x400000;
    uncacheAddr = 0x800000;

    // set up intial memory contents here
    mainMem->prot_write(baseAddr1, data1, size);
    checkMem->prot_write(baseAddr1, data1, size);
    mainMem->prot_write(baseAddr2, data2, size);
    checkMem->prot_write(baseAddr2, data2, size);
    mainMem->prot_write(uncacheAddr, data3, size);
    checkMem->prot_write(uncacheAddr, data3, size);

    delete [] data1;
    delete [] data2;
    delete [] data3;

    // set up counters
    noResponseCycles = 0;
    numReads = 0;
    tickEvent.schedule(0);

    id = TESTER_ALLOCATOR++;
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
MemTest::completeRequest(MemReqPtr &req, uint8_t *data)
{
    //Remove the address from the list of outstanding
    std::set<unsigned>::iterator removeAddr = outstandingAddrs.find(req->paddr);
    assert(removeAddr != outstandingAddrs.end());
    outstandingAddrs.erase(removeAddr);

    switch (req->cmd) {
      case Read:
        if (memcmp(req->data, data, req->size) != 0) {
            cerr << name() << ": on read of 0x" << hex << req->paddr
                 << " (0x" << hex << blockAddr(req->paddr) << ")"
                 << "@ cycle " << dec << curTick
                 << ", cache returns 0x";
            printData(cerr, req->data, req->size);
            cerr << ", expected 0x";
            printData(cerr, data, req->size);
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
            SimExit(curTick, "Maximum number of loads reached!");
        break;

      case Write:
        numWritesStat++;
        break;

      case Copy:
        //Also remove dest from outstanding list
        removeAddr = outstandingAddrs.find(req->dest);
        assert(removeAddr != outstandingAddrs.end());
        outstandingAddrs.erase(removeAddr);
        numCopiesStat++;
        break;

      default:
        panic("invalid command");
    }

    if (blockAddr(req->paddr) == traceBlockAddr) {
        cerr << name() << ": completed "
             << (req->cmd.isWrite() ? "write" : "read")
             << " access of "
             << dec << req->size << " bytes at address 0x"
             << hex << req->paddr
             << " (0x" << hex << blockAddr(req->paddr) << ")"
             << ", value = 0x";
        printData(cerr, req->data, req->size);
        cerr << " @ cycle " << dec << curTick;

        cerr << endl;
    }

    noResponseCycles = 0;
    delete [] data;
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

    if (cacheInterface->isBlocked()) {
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
    if (percentCopies == 0) {
        //We can eliminate the lower bits of the offset, and then use the id
        //to offset within the blks
        offset &= ~63; //Not the low order bits
        offset += id;
        access_size = 0;
    }

    MemReqPtr req = new MemReq();

    if (cacheable < percentUncacheable) {
        req->flags |= UNCACHEABLE;
        req->paddr = uncacheAddr + offset;
    } else {
        req->paddr = ((base) ? baseAddr1 : baseAddr2) + offset;
    }
    // bool probe = (random() % 2 == 1) && !req->isUncacheable();
    bool probe = false;

    req->size = 1 << access_size;
    req->data = new uint8_t[req->size];
    req->paddr &= ~(req->size - 1);
    req->time = curTick;
    req->xc = cpuXC->getProxy();

    if (cmd < percentReads) {
        // read

        //For now we only allow one outstanding request per addreess per tester
        //This means we assume CPU does write forwarding to reads that alias something
        //in the cpu store buffer.
        if (outstandingAddrs.find(req->paddr) != outstandingAddrs.end()) return;
        else outstandingAddrs.insert(req->paddr);

        req->cmd = Read;
        uint8_t *result = new uint8_t[8];
        checkMem->access(Read, req->paddr, result, req->size);
        if (blockAddr(req->paddr) == traceBlockAddr) {
            cerr << name()
                 << ": initiating read "
                 << ((probe) ? "probe of " : "access of ")
                 << dec << req->size << " bytes from addr 0x"
                 << hex << req->paddr
                 << " (0x" << hex << blockAddr(req->paddr) << ")"
                 << " at cycle "
                 << dec << curTick << endl;
        }
        if (probe) {
            cacheInterface->probeAndUpdate(req);
            completeRequest(req, result);
        } else {
            req->completionEvent = new MemCompleteEvent(req, result, this);
            cacheInterface->access(req);
        }
    } else if (cmd < (100 - percentCopies)){
        // write

        //For now we only allow one outstanding request per addreess per tester
        //This means we assume CPU does write forwarding to reads that alias something
        //in the cpu store buffer.
        if (outstandingAddrs.find(req->paddr) != outstandingAddrs.end()) return;
        else outstandingAddrs.insert(req->paddr);

        req->cmd = Write;
        memcpy(req->data, &data, req->size);
        checkMem->access(Write, req->paddr, req->data, req->size);
        if (blockAddr(req->paddr) == traceBlockAddr) {
            cerr << name() << ": initiating write "
                 << ((probe)?"probe of ":"access of ")
                 << dec << req->size << " bytes (value = 0x";
            printData(cerr, req->data, req->size);
            cerr << ") to addr 0x"
                 << hex << req->paddr
                 << " (0x" << hex << blockAddr(req->paddr) << ")"
                 << " at cycle "
                 << dec << curTick << endl;
        }
        if (probe) {
            cacheInterface->probeAndUpdate(req);
            completeRequest(req, NULL);
        } else {
            req->completionEvent = new MemCompleteEvent(req, NULL, this);
            cacheInterface->access(req);
        }
    } else {
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
        }
        cacheInterface->access(req);
        uint8_t result[blockSize];
        checkMem->access(Read, source, &result, blockSize);
        checkMem->access(Write, dest, &result, blockSize);
    }
}


void
MemCompleteEvent::process()
{
    tester->completeRequest(req, data);
    delete this;
}


const char *
MemCompleteEvent::description()
{
    return "memory access completion";
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(MemTest)

    SimObjectParam<BaseCache *> cache;
    SimObjectParam<FunctionalMemory *> main_mem;
    SimObjectParam<FunctionalMemory *> check_mem;
    Param<unsigned> memory_size;
    Param<unsigned> percent_reads;
    Param<unsigned> percent_copies;
    Param<unsigned> percent_uncacheable;
    Param<unsigned> progress_interval;
    Param<unsigned> percent_source_unaligned;
    Param<unsigned> percent_dest_unaligned;
    Param<Addr> trace_addr;
    Param<Counter> max_loads;

END_DECLARE_SIM_OBJECT_PARAMS(MemTest)


BEGIN_INIT_SIM_OBJECT_PARAMS(MemTest)

    INIT_PARAM(cache, "L1 cache"),
    INIT_PARAM(main_mem, "hierarchical memory"),
    INIT_PARAM(check_mem, "check memory"),
    INIT_PARAM(memory_size, "memory size"),
    INIT_PARAM(percent_reads, "target read percentage"),
    INIT_PARAM(percent_copies, "target copy percentage"),
    INIT_PARAM(percent_uncacheable, "target uncacheable percentage"),
    INIT_PARAM(progress_interval, "progress report interval (in accesses)"),
    INIT_PARAM(percent_source_unaligned,
               "percent of copy source address that are unaligned"),
    INIT_PARAM(percent_dest_unaligned,
               "percent of copy dest address that are unaligned"),
    INIT_PARAM(trace_addr, "address to trace"),
    INIT_PARAM(max_loads, "terminate when we have reached this load count")

END_INIT_SIM_OBJECT_PARAMS(MemTest)


CREATE_SIM_OBJECT(MemTest)
{
    return new MemTest(getInstanceName(), cache->getInterface(), main_mem,
                       check_mem, memory_size, percent_reads, percent_copies,
                       percent_uncacheable, progress_interval,
                       percent_source_unaligned, percent_dest_unaligned,
                       trace_addr, max_loads);
}

REGISTER_SIM_OBJECT("MemTest", MemTest)
