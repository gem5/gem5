/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

#include "cpu/memtest/memtest.hh"
#include "base/misc.hh"
#include "sim/sim_events.hh"
#include "mem/functional_mem/main_memory.hh"
#include "mem/cache/base_cache.hh"

#include "base/statistics.hh"
#include "sim/sim_stats.hh"

using namespace std;

MemTest::MemTest(const string &name,
                 MemInterface *_cache_interface,
                 FunctionalMemory *main_mem,
                 FunctionalMemory *check_mem,
                 unsigned _memorySize,
                 unsigned _percentReads,
                 unsigned _percentUncacheable,
                 unsigned _progressInterval,
                 Addr _traceAddr,
                 Counter max_loads_any_thread,
                 Counter max_loads_all_threads)
    : BaseCPU(name, 1, 0, 0, max_loads_any_thread, max_loads_all_threads),
      tickEvent(this),
      cacheInterface(_cache_interface),
      mainMem(main_mem),
      checkMem(check_mem),
      size(_memorySize),
      percentReads(_percentReads),
      percentUncacheable(_percentUncacheable),
      progressInterval(_progressInterval),
      nextProgressMessage(_progressInterval)
{
    vector<string> cmd;
    cmd.push_back("/bin/ls");
    vector<string> null_vec;
    xc = new ExecContext(this ,0,mainMem,0);

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
    numWrites = 0;
    tickEvent.schedule(0);
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
MemTest::completeRequest(MemReqPtr req, uint8_t *data)
{
    switch (req->cmd) {
      case Read:
        if (memcmp(req->data, data, req->size) != 0) {
            cerr << name() << ": on read of 0x" << hex << req->paddr
                 << " @ cycle " << dec << curTick
                 << ", cache returns 0x";
            printData(cerr, req->data, req->size);
            cerr << ", expected 0x";
            printData(cerr, data, req->size);
            cerr << endl;
            fatal("");
        }

        numReads++;

        if (numReads.value() == nextProgressMessage) {
            cerr << name() << ": completed " << numReads.value()
                 << " read accesses @ " << curTick << endl;
            nextProgressMessage += progressInterval;
        }

        comLoadEventQueue[0]->serviceEvents(numReads.value());
        break;

      case Write:
        numWrites++;
        break;


      default:
        panic("invalid command");
    }

    if (blockAddr(req->paddr) == traceBlockAddr) {
        cerr << name() << ": completed "
             << (req->cmd.isWrite() ? "write" : "read") << " access of "
             << req->size << " bytes at address 0x"
             << hex << req->paddr << ", value = 0x";
        printData(cerr, req->data, req->size);
        cerr << endl;
    }

    noResponseCycles = 0;
    delete [] data;
}


void
MemTest::regStats()
{
    using namespace Statistics;

    numReads
        .name(name() + ".num_reads")
        .desc("number of read accesses completed")
        ;

    numWrites
        .name(name() + ".num_writes")
        .desc("number of write accesses completed")
        ;

    numCopies
        .name(name() + ".num_copies")
        .desc("number of copy accesses completed")
        ;
}

void
MemTest::tick()
{
    if (!tickEvent.scheduled())
        tickEvent.schedule(curTick + 1);

    if (++noResponseCycles >= 5000) {
        cerr << name() << ": deadlocked at cycle " << curTick << endl;
        fatal("");
    }

    if (cacheInterface->isBlocked()) {
        return;
    }

    //make new request
    unsigned cmd = rand() % 100;
    unsigned offset1 = random() % size;
    unsigned base = random() % 2;
    uint64_t data = random();
    unsigned access_size = random() % 4;
    unsigned cacheable = rand() % 100;

    MemReqPtr req = new MemReq();

    if (cacheable < percentUncacheable) {
        req->flags |= UNCACHEABLE;
        req->paddr = uncacheAddr + offset1;
    } else {
        req->paddr = ((base) ? baseAddr1 : baseAddr2) + offset1;
    }

    req->size = 1 << access_size;
    req->data = new uint8_t[req->size];
    req->paddr &= ~(req->size - 1);
    req->time = curTick;
    req->xc = xc;

    if (cmd < percentReads) {
        // read
        req->cmd = Read;
        uint8_t *result = new uint8_t[8];
        checkMem->access(Read, req->paddr, result, req->size);
        if (blockAddr(req->paddr) == traceBlockAddr) {
            cerr << name() << ": initiating read of "
                 << req->size << " bytes from addr 0x"
                 << hex << req->paddr << " at cycle "
                 << dec << curTick << endl;
        }

        req->completionEvent = new MemCompleteEvent(req, result, this);
        cacheInterface->access(req);
    } else {
        // write
        req->cmd = Write;
        memcpy(req->data, &data, req->size);
        checkMem->access(Write, req->paddr, req->data, req->size);
        if (blockAddr(req->paddr) == traceBlockAddr) {
            cerr << name() << ": initiating write of "
                 << req->size << " bytes (value = 0x";
            printData(cerr, req->data, req->size);
            cerr << ") to addr 0x"
                 << hex << req->paddr << " at cycle "
                 << dec << curTick << endl;
        }
        req->completionEvent = new MemCompleteEvent(req, NULL, this);
        cacheInterface->access(req);
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
    Param<unsigned> percent_uncacheable;
    Param<unsigned> progress_interval;
    Param<Addr> trace_addr;
    Param<Counter> max_loads_any_thread;
    Param<Counter> max_loads_all_threads;

END_DECLARE_SIM_OBJECT_PARAMS(MemTest)


BEGIN_INIT_SIM_OBJECT_PARAMS(MemTest)

    INIT_PARAM(cache, "L1 cache"),
    INIT_PARAM(main_mem, "hierarchical memory"),
    INIT_PARAM(check_mem, "check memory"),
    INIT_PARAM_DFLT(memory_size, "memory size", 65536),
    INIT_PARAM_DFLT(percent_reads, "target read percentage", 65),
    INIT_PARAM_DFLT(percent_uncacheable, "target uncacheable percentage", 10),
    INIT_PARAM_DFLT(progress_interval,
                    "progress report interval (in accesses)", 1000000),
    INIT_PARAM_DFLT(trace_addr, "address to trace", 0),
    INIT_PARAM_DFLT(max_loads_any_thread,
                    "terminate when any thread reaches this load count",
                    0),
    INIT_PARAM_DFLT(max_loads_all_threads,
                    "terminate when all threads have reached this load count",
                    0)

END_INIT_SIM_OBJECT_PARAMS(MemTest)


CREATE_SIM_OBJECT(MemTest)
{
    return new MemTest(getInstanceName(), cache->getInterface(), main_mem,
                       check_mem, memory_size, percent_reads,
                       percent_uncacheable, progress_interval,
                       trace_addr, max_loads_any_thread,
                       max_loads_all_threads);
}

REGISTER_SIM_OBJECT("MemTest", MemTest)
