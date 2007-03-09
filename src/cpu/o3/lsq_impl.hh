/*
 * Copyright (c) 2005-2006 The Regents of The University of Michigan
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

#include <algorithm>
#include <list>
#include <string>

#include "cpu/o3/lsq.hh"

template<class Impl>
void
LSQ<Impl>::DcachePort::setPeer(Port *port)
{
    Port::setPeer(port);

#if FULL_SYSTEM
    // Update the ThreadContext's memory ports (Functional/Virtual
    // Ports)
    lsq->updateMemPorts();
#endif
}

template <class Impl>
Tick
LSQ<Impl>::DcachePort::recvAtomic(PacketPtr pkt)
{
    panic("O3CPU model does not work with atomic mode!");
    return curTick;
}

template <class Impl>
void
LSQ<Impl>::DcachePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(LSQ, "LSQ doesn't update things on a recvFunctional.");
}

template <class Impl>
void
LSQ<Impl>::DcachePort::recvStatusChange(Status status)
{
    if (status == RangeChange) {
        if (!snoopRangeSent) {
            snoopRangeSent = true;
            sendStatusChange(Port::RangeChange);
        }
        return;
    }
    panic("O3CPU doesn't expect recvStatusChange callback!");
}

template <class Impl>
bool
LSQ<Impl>::DcachePort::recvTiming(PacketPtr pkt)
{
    if (pkt->isResponse()) {
        lsq->thread[pkt->req->getThreadNum()].completeDataAccess(pkt);
    }
    else {
    //else it is a coherence request, maybe you need to do something
        warn("Recieved a coherence request (Invalidate?), 03CPU doesn't"
             "update LSQ for these\n");
    }
    return true;
}

template <class Impl>
void
LSQ<Impl>::DcachePort::recvRetry()
{
    if (lsq->retryTid == -1)
    {
        //Squashed, so drop it
        return;
    }
    lsq->thread[lsq->retryTid].recvRetry();
    // Speculatively clear the retry Tid.  This will get set again if
    // the LSQUnit was unable to complete its access.
    lsq->retryTid = -1;
}

template <class Impl>
LSQ<Impl>::LSQ(Params *params)
    : dcachePort(this), LQEntries(params->LQEntries),
      SQEntries(params->SQEntries), numThreads(params->numberOfThreads),
      retryTid(-1)
{
    DPRINTF(LSQ, "Creating LSQ object.\n");

    dcachePort.snoopRangeSent = false;

    //**********************************************/
    //************ Handle SMT Parameters ***********/
    //**********************************************/
    std::string policy = params->smtLSQPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Figure out fetch policy
    if (policy == "dynamic") {
        lsqPolicy = Dynamic;

        maxLQEntries = LQEntries;
        maxSQEntries = SQEntries;

        DPRINTF(LSQ, "LSQ sharing policy set to Dynamic\n");

    } else if (policy == "partitioned") {
        lsqPolicy = Partitioned;

        //@todo:make work if part_amt doesnt divide evenly.
        maxLQEntries = LQEntries / numThreads;
        maxSQEntries = SQEntries / numThreads;

        DPRINTF(Fetch, "LSQ sharing policy set to Partitioned: "
                "%i entries per LQ | %i entries per SQ",
                maxLQEntries,maxSQEntries);

    } else if (policy == "threshold") {
        lsqPolicy = Threshold;

        assert(params->smtLSQThreshold > LQEntries);
        assert(params->smtLSQThreshold > SQEntries);

        //Divide up by threshold amount
        //@todo: Should threads check the max and the total
        //amount of the LSQ
        maxLQEntries  = params->smtLSQThreshold;
        maxSQEntries  = params->smtLSQThreshold;

        DPRINTF(LSQ, "LSQ sharing policy set to Threshold: "
                "%i entries per LQ | %i entries per SQ",
                maxLQEntries,maxSQEntries);

    } else {
        assert(0 && "Invalid LSQ Sharing Policy.Options Are:{Dynamic,"
                    "Partitioned, Threshold}");
    }

    //Initialize LSQs
    for (int tid=0; tid < numThreads; tid++) {
        thread[tid].init(params, this, maxLQEntries, maxSQEntries, tid);
        thread[tid].setDcachePort(&dcachePort);
    }
}


template<class Impl>
std::string
LSQ<Impl>::name() const
{
    return iewStage->name() + ".lsq";
}

template<class Impl>
void
LSQ<Impl>::regStats()
{
    //Initialize LSQs
    for (int tid=0; tid < numThreads; tid++) {
        thread[tid].regStats();
    }
}

template<class Impl>
void
LSQ<Impl>::setActiveThreads(std::list<unsigned> *at_ptr)
{
    activeThreads = at_ptr;
    assert(activeThreads != 0);
}

template<class Impl>
void
LSQ<Impl>::setCPU(O3CPU *cpu_ptr)
{
    cpu = cpu_ptr;

    dcachePort.setName(name());

    for (int tid=0; tid < numThreads; tid++) {
        thread[tid].setCPU(cpu_ptr);
    }
}

template<class Impl>
void
LSQ<Impl>::setIEW(IEW *iew_ptr)
{
    iewStage = iew_ptr;

    for (int tid=0; tid < numThreads; tid++) {
        thread[tid].setIEW(iew_ptr);
    }
}

template <class Impl>
void
LSQ<Impl>::switchOut()
{
    for (int tid = 0; tid < numThreads; tid++) {
        thread[tid].switchOut();
    }
}

template <class Impl>
void
LSQ<Impl>::takeOverFrom()
{
    for (int tid = 0; tid < numThreads; tid++) {
        thread[tid].takeOverFrom();
    }
}

template <class Impl>
int
LSQ<Impl>::entryAmount(int num_threads)
{
    if (lsqPolicy == Partitioned) {
        return LQEntries / num_threads;
    } else {
        return 0;
    }
}

template <class Impl>
void
LSQ<Impl>::resetEntries()
{
    if (lsqPolicy != Dynamic || numThreads > 1) {
        int active_threads = activeThreads->size();

        int maxEntries;

        if (lsqPolicy == Partitioned) {
            maxEntries = LQEntries / active_threads;
        } else if (lsqPolicy == Threshold && active_threads == 1) {
            maxEntries = LQEntries;
        } else {
            maxEntries = LQEntries;
        }

        std::list<unsigned>::iterator threads  = activeThreads->begin();
        std::list<unsigned>::iterator end = activeThreads->end();

        while (threads != end) {
            unsigned tid = *threads++;

            resizeEntries(maxEntries, tid);
        }
    }
}

template<class Impl>
void
LSQ<Impl>::removeEntries(unsigned tid)
{
    thread[tid].clearLQ();
    thread[tid].clearSQ();
}

template<class Impl>
void
LSQ<Impl>::resizeEntries(unsigned size,unsigned tid)
{
    thread[tid].resizeLQ(size);
    thread[tid].resizeSQ(size);
}

template<class Impl>
void
LSQ<Impl>::tick()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        thread[tid].tick();
    }
}

template<class Impl>
void
LSQ<Impl>::insertLoad(DynInstPtr &load_inst)
{
    unsigned tid = load_inst->threadNumber;

    thread[tid].insertLoad(load_inst);
}

template<class Impl>
void
LSQ<Impl>::insertStore(DynInstPtr &store_inst)
{
    unsigned tid = store_inst->threadNumber;

    thread[tid].insertStore(store_inst);
}

template<class Impl>
Fault
LSQ<Impl>::executeLoad(DynInstPtr &inst)
{
    unsigned tid = inst->threadNumber;

    return thread[tid].executeLoad(inst);
}

template<class Impl>
Fault
LSQ<Impl>::executeStore(DynInstPtr &inst)
{
    unsigned tid = inst->threadNumber;

    return thread[tid].executeStore(inst);
}

template<class Impl>
void
LSQ<Impl>::writebackStores()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (numStoresToWB(tid) > 0) {
            DPRINTF(Writeback,"[tid:%i] Writing back stores. %i stores "
                "available for Writeback.\n", tid, numStoresToWB(tid));
        }

        thread[tid].writebackStores();
    }
}

template<class Impl>
bool
LSQ<Impl>::violation()
{
    /* Answers: Does Anybody Have a Violation?*/
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (thread[tid].violation())
            return true;
    }

    return false;
}

template<class Impl>
int
LSQ<Impl>::getCount()
{
    unsigned total = 0;

    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        total += getCount(tid);
    }

    return total;
}

template<class Impl>
int
LSQ<Impl>::numLoads()
{
    unsigned total = 0;

    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        total += numLoads(tid);
    }

    return total;
}

template<class Impl>
int
LSQ<Impl>::numStores()
{
    unsigned total = 0;

    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        total += thread[tid].numStores();
    }

    return total;
}

template<class Impl>
int
LSQ<Impl>::numLoadsReady()
{
    unsigned total = 0;

    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        total += thread[tid].numLoadsReady();
    }

    return total;
}

template<class Impl>
unsigned
LSQ<Impl>::numFreeEntries()
{
    unsigned total = 0;

    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        total += thread[tid].numFreeEntries();
    }

    return total;
}

template<class Impl>
unsigned
LSQ<Impl>::numFreeEntries(unsigned tid)
{
    //if (lsqPolicy == Dynamic)
    //return numFreeEntries();
    //else
        return thread[tid].numFreeEntries();
}

template<class Impl>
bool
LSQ<Impl>::isFull()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (!(thread[tid].lqFull() || thread[tid].sqFull()))
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::isFull(unsigned tid)
{
    //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == Dynamic)
        return isFull();
    else
        return thread[tid].lqFull() || thread[tid].sqFull();
}

template<class Impl>
bool
LSQ<Impl>::lqFull()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (!thread[tid].lqFull())
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::lqFull(unsigned tid)
{
    //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == Dynamic)
        return lqFull();
    else
        return thread[tid].lqFull();
}

template<class Impl>
bool
LSQ<Impl>::sqFull()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (!sqFull(tid))
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::sqFull(unsigned tid)
{
     //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == Dynamic)
        return sqFull();
    else
        return thread[tid].sqFull();
}

template<class Impl>
bool
LSQ<Impl>::isStalled()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (!thread[tid].isStalled())
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::isStalled(unsigned tid)
{
    if (lsqPolicy == Dynamic)
        return isStalled();
    else
        return thread[tid].isStalled();
}

template<class Impl>
bool
LSQ<Impl>::hasStoresToWB()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    if (threads == end)
        return false;

    while (threads != end) {
        unsigned tid = *threads++;

        if (!hasStoresToWB(tid))
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::willWB()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        if (!willWB(tid))
            return false;
    }

    return true;
}

template<class Impl>
void
LSQ<Impl>::dumpInsts()
{
    std::list<unsigned>::iterator threads = activeThreads->begin();
    std::list<unsigned>::iterator end = activeThreads->end();

    while (threads != end) {
        unsigned tid = *threads++;

        thread[tid].dumpInsts();
    }
}
