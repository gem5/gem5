/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 */

#ifndef __CPU_O3_LSQ_HH__
#define __CPU_O3_LSQ_HH__

#include <map>
#include <queue>

#include "cpu/o3/lsq_unit.hh"
#include "cpu/inst_seq.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"

struct DerivO3CPUParams;

template <class Impl>
class LSQ {
  public:
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::LSQUnit LSQUnit;

    /** SMT policy. */
    enum LSQPolicy {
        Dynamic,
        Partitioned,
        Threshold
    };

    /** Constructs an LSQ with the given parameters. */
    LSQ(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params);
    ~LSQ() {
        if (thread) delete [] thread;
    }

    /** Returns the name of the LSQ. */
    std::string name() const;

    /** Registers statistics of each LSQ unit. */
    void regStats();

    /** Sets the pointer to the list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;
    /** Has the LSQ drained? */
    bool isDrained() const;
    /** Takes over execution from another CPU's thread. */
    void takeOverFrom();

    /** Number of entries needed for the given amount of threads.*/
    int entryAmount(ThreadID num_threads);
    void removeEntries(ThreadID tid);
    /** Reset the max entries for each thread. */
    void resetEntries();
    /** Resize the max entries for a thread. */
    void resizeEntries(unsigned size, ThreadID tid);

    /** Ticks the LSQ. */
    void tick();
    /** Ticks a specific LSQ Unit. */
    void tick(ThreadID tid)
    { thread[tid].tick(); }

    /** Inserts a load into the LSQ. */
    void insertLoad(DynInstPtr &load_inst);
    /** Inserts a store into the LSQ. */
    void insertStore(DynInstPtr &store_inst);

    /** Executes a load. */
    Fault executeLoad(DynInstPtr &inst);

    /** Executes a store. */
    Fault executeStore(DynInstPtr &inst);

    /**
     * Commits loads up until the given sequence number for a specific thread.
     */
    void commitLoads(InstSeqNum &youngest_inst, ThreadID tid)
    { thread[tid].commitLoads(youngest_inst); }

    /**
     * Commits stores up until the given sequence number for a specific thread.
     */
    void commitStores(InstSeqNum &youngest_inst, ThreadID tid)
    { thread[tid].commitStores(youngest_inst); }

    /**
     * Attempts to write back stores until all cache ports are used or the
     * interface becomes blocked.
     */
    void writebackStores();
    /** Same as above, but only for one thread. */
    void writebackStores(ThreadID tid);

    /**
     * Squash instructions from a thread until the specified sequence number.
     */
    void squash(const InstSeqNum &squashed_num, ThreadID tid)
    { thread[tid].squash(squashed_num); }

    /** Returns whether or not there was a memory ordering violation. */
    bool violation();
    /**
     * Returns whether or not there was a memory ordering violation for a
     * specific thread.
     */
    bool violation(ThreadID tid)
    { return thread[tid].violation(); }

    /** Gets the instruction that caused the memory ordering violation. */
    DynInstPtr getMemDepViolator(ThreadID tid)
    { return thread[tid].getMemDepViolator(); }

    /** Returns the head index of the load queue for a specific thread. */
    int getLoadHead(ThreadID tid)
    { return thread[tid].getLoadHead(); }

    /** Returns the sequence number of the head of the load queue. */
    InstSeqNum getLoadHeadSeqNum(ThreadID tid)
    {
        return thread[tid].getLoadHeadSeqNum();
    }

    /** Returns the head index of the store queue. */
    int getStoreHead(ThreadID tid)
    { return thread[tid].getStoreHead(); }

    /** Returns the sequence number of the head of the store queue. */
    InstSeqNum getStoreHeadSeqNum(ThreadID tid)
    {
        return thread[tid].getStoreHeadSeqNum();
    }

    /** Returns the number of instructions in all of the queues. */
    int getCount();
    /** Returns the number of instructions in the queues of one thread. */
    int getCount(ThreadID tid)
    { return thread[tid].getCount(); }

    /** Returns the total number of loads in the load queue. */
    int numLoads();
    /** Returns the total number of loads for a single thread. */
    int numLoads(ThreadID tid)
    { return thread[tid].numLoads(); }

    /** Returns the total number of stores in the store queue. */
    int numStores();
    /** Returns the total number of stores for a single thread. */
    int numStores(ThreadID tid)
    { return thread[tid].numStores(); }

    /** Returns the number of free load entries. */
    unsigned numFreeLoadEntries();

    /** Returns the number of free store entries. */
    unsigned numFreeStoreEntries();

    /** Returns the number of free entries for a specific thread. */
    unsigned numFreeEntries(ThreadID tid);

    /** Returns the number of free entries in the LQ for a specific thread. */
    unsigned numFreeLoadEntries(ThreadID tid);

    /** Returns the number of free entries in the SQ for a specific thread. */
    unsigned numFreeStoreEntries(ThreadID tid);

    /** Returns if the LSQ is full (either LQ or SQ is full). */
    bool isFull();
    /**
     * Returns if the LSQ is full for a specific thread (either LQ or SQ is
     * full).
     */
    bool isFull(ThreadID tid);

    /** Returns if the LSQ is empty (both LQ and SQ are empty). */
    bool isEmpty() const;
    /** Returns if all of the LQs are empty. */
    bool lqEmpty() const;
    /** Returns if all of the SQs are empty. */
    bool sqEmpty() const;

    /** Returns if any of the LQs are full. */
    bool lqFull();
    /** Returns if the LQ of a given thread is full. */
    bool lqFull(ThreadID tid);

    /** Returns if any of the SQs are full. */
    bool sqFull();
    /** Returns if the SQ of a given thread is full. */
    bool sqFull(ThreadID tid);

    /**
     * Returns if the LSQ is stalled due to a memory operation that must be
     * replayed.
     */
    bool isStalled();
    /**
     * Returns if the LSQ of a specific thread is stalled due to a memory
     * operation that must be replayed.
     */
    bool isStalled(ThreadID tid);

    /** Returns whether or not there are any stores to write back to memory. */
    bool hasStoresToWB();

    /** Returns whether or not a specific thread has any stores to write back
     * to memory.
     */
    bool hasStoresToWB(ThreadID tid)
    { return thread[tid].hasStoresToWB(); }

    /** Returns the number of stores a specific thread has to write back. */
    int numStoresToWB(ThreadID tid)
    { return thread[tid].numStoresToWB(); }

    /** Returns if the LSQ will write back to memory this cycle. */
    bool willWB();
    /** Returns if the LSQ of a specific thread will write back to memory this
     * cycle.
     */
    bool willWB(ThreadID tid)
    { return thread[tid].willWB(); }

    /** Debugging function to print out all instructions. */
    void dumpInsts() const;
    /** Debugging function to print out instructions from a specific thread. */
    void dumpInsts(ThreadID tid) const
    { thread[tid].dumpInsts(); }

    /** Executes a read operation, using the load specified at the load
     * index.
     */
    Fault read(RequestPtr req, RequestPtr sreqLow, RequestPtr sreqHigh,
               int load_idx);

    /** Executes a store operation, using the store specified at the store
     * index.
     */
    Fault write(RequestPtr req, RequestPtr sreqLow, RequestPtr sreqHigh,
                uint8_t *data, int store_idx);

    /**
     * Retry the previous send that failed.
     */
    void recvReqRetry();

    /**
     * Handles writing back and completing the load or store that has
     * returned from memory.
     *
     * @param pkt Response packet from the memory sub-system
     */
    bool recvTimingResp(PacketPtr pkt);

    void recvTimingSnoopReq(PacketPtr pkt);

    /** The CPU pointer. */
    O3CPU *cpu;

    /** The IEW stage pointer. */
    IEW *iewStage;

  protected:
    /** The LSQ policy for SMT mode. */
    LSQPolicy lsqPolicy;

    /** The LSQ units for individual threads. */
    LSQUnit *thread;

    /** List of Active Threads in System. */
    std::list<ThreadID> *activeThreads;

    /** Total Size of LQ Entries. */
    unsigned LQEntries;
    /** Total Size of SQ Entries. */
    unsigned SQEntries;

    /** Max LQ Size - Used to Enforce Sharing Policies. */
    unsigned maxLQEntries;

    /** Max SQ Size - Used to Enforce Sharing Policies. */
    unsigned maxSQEntries;

    /** Number of Threads. */
    ThreadID numThreads;
};

template <class Impl>
Fault
LSQ<Impl>::read(RequestPtr req, RequestPtr sreqLow, RequestPtr sreqHigh,
                int load_idx)
{
    ThreadID tid = cpu->contextToThread(req->contextId());

    return thread[tid].read(req, sreqLow, sreqHigh, load_idx);
}

template <class Impl>
Fault
LSQ<Impl>::write(RequestPtr req, RequestPtr sreqLow, RequestPtr sreqHigh,
                 uint8_t *data, int store_idx)
{
    ThreadID tid = cpu->contextToThread(req->contextId());

    return thread[tid].write(req, sreqLow, sreqHigh, data, store_idx);
}

#endif // __CPU_O3_LSQ_HH__
