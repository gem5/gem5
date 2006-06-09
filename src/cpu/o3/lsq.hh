/*
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

#include "config/full_system.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/lsq_unit.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"

template <class Impl>
class LSQ {
  public:
    typedef typename Impl::Params Params;
    typedef typename Impl::FullCPU FullCPU;
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
    LSQ(Params *params);

    /** Returns the name of the LSQ. */
    std::string name() const;

    /** Sets the pointer to the list of active threads. */
    void setActiveThreads(std::list<unsigned> *at_ptr);
    /** Sets the CPU pointer. */
    void setCPU(FullCPU *cpu_ptr);
    /** Sets the IEW stage pointer. */
    void setIEW(IEW *iew_ptr);
    /** Switches out the LSQ. */
    void switchOut();
    /** Takes over execution from another CPU's thread. */
    void takeOverFrom();

    /** Number of entries needed for the given amount of threads.*/
    int entryAmount(int num_threads);
    void removeEntries(unsigned tid);
    /** Reset the max entries for each thread. */
    void resetEntries();
    /** Resize the max entries for a thread. */
    void resizeEntries(unsigned size, unsigned tid);

    /** Ticks the LSQ. */
    void tick();
    /** Ticks a specific LSQ Unit. */
    void tick(unsigned tid)
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
    void commitLoads(InstSeqNum &youngest_inst, unsigned tid)
    { thread[tid].commitLoads(youngest_inst); }

    /**
     * Commits stores up until the given sequence number for a specific thread.
     */
    void commitStores(InstSeqNum &youngest_inst, unsigned tid)
    { thread[tid].commitStores(youngest_inst); }

    /**
     * Attempts to write back stores until all cache ports are used or the
     * interface becomes blocked.
     */
    void writebackStores();
    /** Same as above, but only for one thread. */
    void writebackStores(unsigned tid);

    /**
     * Squash instructions from a thread until the specified sequence number.
     */
    void squash(const InstSeqNum &squashed_num, unsigned tid)
    { thread[tid].squash(squashed_num); }

    /** Returns whether or not there was a memory ordering violation. */
    bool violation();
    /**
     * Returns whether or not there was a memory ordering violation for a
     * specific thread.
     */
    bool violation(unsigned tid)
    { return thread[tid].violation(); }

    /** Returns if a load is blocked due to the memory system for a specific
     *  thread.
     */
    bool loadBlocked(unsigned tid)
    { return thread[tid].loadBlocked(); }

    bool isLoadBlockedHandled(unsigned tid)
    { return thread[tid].isLoadBlockedHandled(); }

    void setLoadBlockedHandled(unsigned tid)
    { thread[tid].setLoadBlockedHandled(); }

    /** Gets the instruction that caused the memory ordering violation. */
    DynInstPtr getMemDepViolator(unsigned tid)
    { return thread[tid].getMemDepViolator(); }

    /** Returns the head index of the load queue for a specific thread. */
    int getLoadHead(unsigned tid)
    { return thread[tid].getLoadHead(); }

    /** Returns the sequence number of the head of the load queue. */
    InstSeqNum getLoadHeadSeqNum(unsigned tid)
    {
        return thread[tid].getLoadHeadSeqNum();
    }

    /** Returns the head index of the store queue. */
    int getStoreHead(unsigned tid)
    { return thread[tid].getStoreHead(); }

    /** Returns the sequence number of the head of the store queue. */
    InstSeqNum getStoreHeadSeqNum(unsigned tid)
    {
        return thread[tid].getStoreHeadSeqNum();
    }

    /** Returns the number of instructions in all of the queues. */
    int getCount();
    /** Returns the number of instructions in the queues of one thread. */
    int getCount(unsigned tid)
    { return thread[tid].getCount(); }

    /** Returns the total number of loads in the load queue. */
    int numLoads();
    /** Returns the total number of loads for a single thread. */
    int numLoads(unsigned tid)
    { return thread[tid].numLoads(); }

    /** Returns the total number of stores in the store queue. */
    int numStores();
    /** Returns the total number of stores for a single thread. */
    int numStores(unsigned tid)
    { return thread[tid].numStores(); }

    /** Returns the total number of loads that are ready. */
    int numLoadsReady();
    /** Returns the number of loads that are ready for a single thread. */
    int numLoadsReady(unsigned tid)
    { return thread[tid].numLoadsReady(); }

    /** Returns the number of free entries. */
    unsigned numFreeEntries();
    /** Returns the number of free entries for a specific thread. */
    unsigned numFreeEntries(unsigned tid);

    /** Returns if the LSQ is full (either LQ or SQ is full). */
    bool isFull();
    /**
     * Returns if the LSQ is full for a specific thread (either LQ or SQ is
     * full).
     */
    bool isFull(unsigned tid);

    /** Returns if any of the LQs are full. */
    bool lqFull();
    /** Returns if the LQ of a given thread is full. */
    bool lqFull(unsigned tid);

    /** Returns if any of the SQs are full. */
    bool sqFull();
    /** Returns if the SQ of a given thread is full. */
    bool sqFull(unsigned tid);

    /**
     * Returns if the LSQ is stalled due to a memory operation that must be
     * replayed.
     */
    bool isStalled();
    /**
     * Returns if the LSQ of a specific thread is stalled due to a memory
     * operation that must be replayed.
     */
    bool isStalled(unsigned tid);

    /** Returns whether or not there are any stores to write back to memory. */
    bool hasStoresToWB();

    /** Returns whether or not a specific thread has any stores to write back
     * to memory.
     */
    bool hasStoresToWB(unsigned tid)
    { return thread[tid].hasStoresToWB(); }

    /** Returns the number of stores a specific thread has to write back. */
    int  numStoresToWB(unsigned tid)
    { return thread[tid].numStoresToWB(); }

    /** Returns if the LSQ will write back to memory this cycle. */
    bool willWB();
    /** Returns if the LSQ of a specific thread will write back to memory this
     * cycle.
     */
    bool willWB(unsigned tid)
    { return thread[tid].willWB(); }

    /** Debugging function to print out all instructions. */
    void dumpInsts();
    /** Debugging function to print out instructions from a specific thread. */
    void dumpInsts(unsigned tid)
    { thread[tid].dumpInsts(); }

    /** Executes a read operation, using the load specified at the load index. */
    template <class T>
    Fault read(RequestPtr req, T &data, int load_idx);

    /** Executes a store operation, using the store specified at the store
     *   index.
     */
    template <class T>
    Fault write(RequestPtr req, T &data, int store_idx);

  private:
    /** The LSQ policy for SMT mode. */
    LSQPolicy lsqPolicy;

    /** The LSQ units for individual threads. */
    LSQUnit thread[Impl::MaxThreads];

    /** The CPU pointer. */
    FullCPU *cpu;

    /** The IEW stage pointer. */
    IEW *iewStage;

    /** List of Active Threads in System. */
    std::list<unsigned> *activeThreads;

    /** Total Size of LQ Entries. */
    unsigned LQEntries;
    /** Total Size of SQ Entries. */
    unsigned SQEntries;

    /** Max LQ Size - Used to Enforce Sharing Policies. */
    unsigned maxLQEntries;

    /** Max SQ Size - Used to Enforce Sharing Policies. */
    unsigned maxSQEntries;

    /** Number of Threads. */
    unsigned numThreads;
};

template <class Impl>
template <class T>
Fault
LSQ<Impl>::read(RequestPtr req, T &data, int load_idx)
{
    unsigned tid = req->getThreadNum();

    return thread[tid].read(req, data, load_idx);
}

template <class Impl>
template <class T>
Fault
LSQ<Impl>::write(RequestPtr req, T &data, int store_idx)
{
    unsigned tid = req->getThreadNum();

    return thread[tid].write(req, data, store_idx);
}

#endif // __CPU_O3_LSQ_HH__
