/*
 * Copyright (c) 2011-2012, 2014, 2017-2018 ARM Limited
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
 */

#ifndef __CPU_O3_LSQ_IMPL_HH__
#define __CPU_O3_LSQ_IMPL_HH__

#include <algorithm>
#include <list>
#include <string>

#include "base/logging.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/lsq.hh"
#include "debug/Drain.hh"
#include "debug/Fetch.hh"
#include "debug/LSQ.hh"
#include "debug/Writeback.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template <class Impl>
LSQ<Impl>::LSQ(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params)
    : cpu(cpu_ptr), iewStage(iew_ptr),
      _cacheBlocked(false),
      cacheStorePorts(params->cacheStorePorts), usedStorePorts(0),
      cacheLoadPorts(params->cacheLoadPorts), usedLoadPorts(0),
      lsqPolicy(params->smtLSQPolicy),
      LQEntries(params->LQEntries),
      SQEntries(params->SQEntries),
      maxLQEntries(maxLSQAllocation(lsqPolicy, LQEntries, params->numThreads,
                  params->smtLSQThreshold)),
      maxSQEntries(maxLSQAllocation(lsqPolicy, SQEntries, params->numThreads,
                  params->smtLSQThreshold)),
      dcachePort(this, cpu_ptr),
      numThreads(params->numThreads)
{
    assert(numThreads > 0 && numThreads <= Impl::MaxThreads);

    //**********************************************/
    //************ Handle SMT Parameters ***********/
    //**********************************************/

    /* Run SMT olicy checks. */
        if (lsqPolicy == SMTQueuePolicy::Dynamic) {
        DPRINTF(LSQ, "LSQ sharing policy set to Dynamic\n");
    } else if (lsqPolicy == SMTQueuePolicy::Partitioned) {
        DPRINTF(Fetch, "LSQ sharing policy set to Partitioned: "
                "%i entries per LQ | %i entries per SQ\n",
                maxLQEntries,maxSQEntries);
    } else if (lsqPolicy == SMTQueuePolicy::Threshold) {

        assert(params->smtLSQThreshold > params->LQEntries);
        assert(params->smtLSQThreshold > params->SQEntries);

        DPRINTF(LSQ, "LSQ sharing policy set to Threshold: "
                "%i entries per LQ | %i entries per SQ\n",
                maxLQEntries,maxSQEntries);
    } else {
        panic("Invalid LSQ sharing policy. Options are: Dynamic, "
                    "Partitioned, Threshold");
    }

    thread.reserve(numThreads);
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        thread.emplace_back(maxLQEntries, maxSQEntries);
        thread[tid].init(cpu, iew_ptr, params, this, tid);
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
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        thread[tid].regStats();
    }
}

template<class Impl>
void
LSQ<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
    assert(activeThreads != 0);
}

template <class Impl>
void
LSQ<Impl>::drainSanityCheck() const
{
    assert(isDrained());

    for (ThreadID tid = 0; tid < numThreads; tid++)
        thread[tid].drainSanityCheck();
}

template <class Impl>
bool
LSQ<Impl>::isDrained() const
{
    bool drained(true);

    if (!lqEmpty()) {
        DPRINTF(Drain, "Not drained, LQ not empty.\n");
        drained = false;
    }

    if (!sqEmpty()) {
        DPRINTF(Drain, "Not drained, SQ not empty.\n");
        drained = false;
    }

    return drained;
}

template <class Impl>
void
LSQ<Impl>::takeOverFrom()
{
    usedStorePorts = 0;
    _cacheBlocked = false;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        thread[tid].takeOverFrom();
    }
}

template <class Impl>
void
LSQ<Impl>::tick()
{
    // Re-issue loads which got blocked on the per-cycle load ports limit.
    if (usedLoadPorts == cacheLoadPorts && !_cacheBlocked)
        iewStage->cacheUnblocked();

    usedLoadPorts = 0;
    usedStorePorts = 0;
}

template<class Impl>
bool
LSQ<Impl>::cacheBlocked() const
{
    return _cacheBlocked;
}

template<class Impl>
void
LSQ<Impl>::cacheBlocked(bool v)
{
    _cacheBlocked = v;
}

template<class Impl>
bool
LSQ<Impl>::cachePortAvailable(bool is_load) const
{
    bool ret;
    if (is_load) {
        ret  = usedLoadPorts < cacheLoadPorts;
    } else {
        ret  = usedStorePorts < cacheStorePorts;
    }
    return ret;
}

template<class Impl>
void
LSQ<Impl>::cachePortBusy(bool is_load)
{
    assert(cachePortAvailable(is_load));
    if (is_load) {
        usedLoadPorts++;
    } else {
        usedStorePorts++;
    }
}

template<class Impl>
void
LSQ<Impl>::insertLoad(const DynInstPtr &load_inst)
{
    ThreadID tid = load_inst->threadNumber;

    thread[tid].insertLoad(load_inst);
}

template<class Impl>
void
LSQ<Impl>::insertStore(const DynInstPtr &store_inst)
{
    ThreadID tid = store_inst->threadNumber;

    thread[tid].insertStore(store_inst);
}

template<class Impl>
Fault
LSQ<Impl>::executeLoad(const DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    return thread[tid].executeLoad(inst);
}

template<class Impl>
Fault
LSQ<Impl>::executeStore(const DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    return thread[tid].executeStore(inst);
}

template<class Impl>
void
LSQ<Impl>::writebackStores()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

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
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (thread[tid].violation())
            return true;
    }

    return false;
}

template <class Impl>
void
LSQ<Impl>::recvReqRetry()
{
    iewStage->cacheUnblocked();
    cacheBlocked(false);

    for (ThreadID tid : *activeThreads) {
        thread[tid].recvRetry();
    }
}

template <class Impl>
void
LSQ<Impl>::completeDataAccess(PacketPtr pkt)
{
    auto senderState = dynamic_cast<LSQSenderState*>(pkt->senderState);
    thread[cpu->contextToThread(senderState->contextId())]
        .completeDataAccess(pkt);
}

template <class Impl>
bool
LSQ<Impl>::recvTimingResp(PacketPtr pkt)
{
    if (pkt->isError())
        DPRINTF(LSQ, "Got error packet back for address: %#X\n",
                pkt->getAddr());

    auto senderState = dynamic_cast<LSQSenderState*>(pkt->senderState);
    panic_if(!senderState, "Got packet back with unknown sender state\n");

    thread[cpu->contextToThread(senderState->contextId())].recvTimingResp(pkt);

    if (pkt->isInvalidate()) {
        // This response also contains an invalidate; e.g. this can be the case
        // if cmd is ReadRespWithInvalidate.
        //
        // The calling order between completeDataAccess and checkSnoop matters.
        // By calling checkSnoop after completeDataAccess, we ensure that the
        // fault set by checkSnoop is not lost. Calling writeback (more
        // specifically inst->completeAcc) in completeDataAccess overwrites
        // fault, and in case this instruction requires squashing (as
        // determined by checkSnoop), the ReExec fault set by checkSnoop would
        // be lost otherwise.

        DPRINTF(LSQ, "received invalidation with response for addr:%#x\n",
                pkt->getAddr());

        for (ThreadID tid = 0; tid < numThreads; tid++) {
            thread[tid].checkSnoop(pkt);
        }
    }
    // Update the LSQRequest state (this may delete the request)
    senderState->request()->packetReplied();

    return true;
}

template <class Impl>
void
LSQ<Impl>::recvTimingSnoopReq(PacketPtr pkt)
{
    DPRINTF(LSQ, "received pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    // must be a snoop
    if (pkt->isInvalidate()) {
        DPRINTF(LSQ, "received invalidation for addr:%#x\n",
                pkt->getAddr());
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            thread[tid].checkSnoop(pkt);
        }
    }
}

template<class Impl>
int
LSQ<Impl>::getCount()
{
    unsigned total = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += getCount(tid);
    }

    return total;
}

template<class Impl>
int
LSQ<Impl>::numLoads()
{
    unsigned total = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += numLoads(tid);
    }

    return total;
}

template<class Impl>
int
LSQ<Impl>::numStores()
{
    unsigned total = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += thread[tid].numStores();
    }

    return total;
}

template<class Impl>
unsigned
LSQ<Impl>::numFreeLoadEntries()
{
    unsigned total = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += thread[tid].numFreeLoadEntries();
    }

    return total;
}

template<class Impl>
unsigned
LSQ<Impl>::numFreeStoreEntries()
{
    unsigned total = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += thread[tid].numFreeStoreEntries();
    }

    return total;
}

template<class Impl>
unsigned
LSQ<Impl>::numFreeLoadEntries(ThreadID tid)
{
        return thread[tid].numFreeLoadEntries();
}

template<class Impl>
unsigned
LSQ<Impl>::numFreeStoreEntries(ThreadID tid)
{
        return thread[tid].numFreeStoreEntries();
}

template<class Impl>
bool
LSQ<Impl>::isFull()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!(thread[tid].lqFull() || thread[tid].sqFull()))
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::isFull(ThreadID tid)
{
    //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return isFull();
    else
        return thread[tid].lqFull() || thread[tid].sqFull();
}

template<class Impl>
bool
LSQ<Impl>::isEmpty() const
{
    return lqEmpty() && sqEmpty();
}

template<class Impl>
bool
LSQ<Impl>::lqEmpty() const
{
    list<ThreadID>::const_iterator threads = activeThreads->begin();
    list<ThreadID>::const_iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].lqEmpty())
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::sqEmpty() const
{
    list<ThreadID>::const_iterator threads = activeThreads->begin();
    list<ThreadID>::const_iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].sqEmpty())
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::lqFull()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].lqFull())
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::lqFull(ThreadID tid)
{
    //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return lqFull();
    else
        return thread[tid].lqFull();
}

template<class Impl>
bool
LSQ<Impl>::sqFull()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!sqFull(tid))
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::sqFull(ThreadID tid)
{
     //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return sqFull();
    else
        return thread[tid].sqFull();
}

template<class Impl>
bool
LSQ<Impl>::isStalled()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].isStalled())
            return false;
    }

    return true;
}

template<class Impl>
bool
LSQ<Impl>::isStalled(ThreadID tid)
{
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return isStalled();
    else
        return thread[tid].isStalled();
}

template<class Impl>
bool
LSQ<Impl>::hasStoresToWB()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (hasStoresToWB(tid))
            return true;
    }

    return false;
}

template<class Impl>
bool
LSQ<Impl>::willWB()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (willWB(tid))
            return true;
    }

    return false;
}

template<class Impl>
void
LSQ<Impl>::dumpInsts() const
{
    list<ThreadID>::const_iterator threads = activeThreads->begin();
    list<ThreadID>::const_iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        thread[tid].dumpInsts();
    }
}

template<class Impl>
Fault
LSQ<Impl>::pushRequest(const DynInstPtr& inst, bool isLoad, uint8_t *data,
                       unsigned int size, Addr addr, Request::Flags flags,
                       uint64_t *res, AtomicOpFunctorPtr amo_op,
                       const std::vector<bool>& byte_enable)
{
    // This comming request can be either load, store or atomic.
    // Atomic request has a corresponding pointer to its atomic memory
    // operation
    bool isAtomic M5_VAR_USED = !isLoad && amo_op;

    ThreadID tid = cpu->contextToThread(inst->contextId());
    auto cacheLineSize = cpu->cacheLineSize();
    bool needs_burst = transferNeedsBurst(addr, size, cacheLineSize);
    LSQRequest* req = nullptr;

    // Atomic requests that access data across cache line boundary are
    // currently not allowed since the cache does not guarantee corresponding
    // atomic memory operations to be executed atomically across a cache line.
    // For ISAs such as x86 that supports cross-cache-line atomic instructions,
    // the cache needs to be modified to perform atomic update to both cache
    // lines. For now, such cross-line update is not supported.
    assert(!isAtomic || (isAtomic && !needs_burst));

    if (inst->translationStarted()) {
        req = inst->savedReq;
        assert(req);
    } else {
        if (needs_burst) {
            req = new SplitDataRequest(&thread[tid], inst, isLoad, addr,
                    size, flags, data, res);
        } else {
            req = new SingleDataRequest(&thread[tid], inst, isLoad, addr,
                    size, flags, data, res, std::move(amo_op));
        }
        assert(req);
        if (!byte_enable.empty()) {
            req->_byteEnable = byte_enable;
        }
        inst->setRequest();
        req->taskId(cpu->taskId());

        // There might be fault from a previous execution attempt if this is
        // a strictly ordered load
        inst->getFault() = NoFault;

        req->initiateTranslation();
    }

    /* This is the place were instructions get the effAddr. */
    if (req->isTranslationComplete()) {
        if (req->isMemAccessRequired()) {
            inst->effAddr = req->getVaddr();
            inst->effSize = size;
            inst->effAddrValid(true);

            if (cpu->checker) {
                inst->reqToVerify = std::make_shared<Request>(*req->request());
            }
            Fault fault;
            if (isLoad)
                fault = cpu->read(req, inst->lqIdx);
            else
                fault = cpu->write(req, data, inst->sqIdx);
            // inst->getFault() may have the first-fault of a
            // multi-access split request at this point.
            // Overwrite that only if we got another type of fault
            // (e.g. re-exec).
            if (fault != NoFault)
                inst->getFault() = fault;
        } else if (isLoad) {
            inst->setMemAccPredicate(false);
            // Commit will have to clean up whatever happened.  Set this
            // instruction as executed.
            inst->setExecuted();
        }
    }

    if (inst->traceData)
        inst->traceData->setMem(addr, size, flags);

    return inst->getFault();
}

template<class Impl>
void
LSQ<Impl>::SingleDataRequest::finish(const Fault &fault, const RequestPtr &req,
        ThreadContext* tc, BaseTLB::Mode mode)
{
    _fault.push_back(fault);
    numInTranslationFragments = 0;
    numTranslatedFragments = 1;
    /* If the instruction has been squahsed, let the request know
     * as it may have to self-destruct. */
    if (_inst->isSquashed()) {
        this->squashTranslation();
    } else {
        _inst->strictlyOrdered(req->isStrictlyOrdered());

        flags.set(Flag::TranslationFinished);
        if (fault == NoFault) {
            _inst->physEffAddr = req->getPaddr();
            _inst->memReqFlags = req->getFlags();
            if (req->isCondSwap()) {
                assert(_res);
                req->setExtraData(*_res);
            }
            setState(State::Request);
        } else {
            setState(State::Fault);
        }

        LSQRequest::_inst->fault = fault;
        LSQRequest::_inst->translationCompleted(true);
    }
}

template<class Impl>
void
LSQ<Impl>::SplitDataRequest::finish(const Fault &fault, const RequestPtr &req,
        ThreadContext* tc, BaseTLB::Mode mode)
{
    int i;
    for (i = 0; i < _requests.size() && _requests[i] != req; i++);
    assert(i < _requests.size());
    _fault[i] = fault;

    numInTranslationFragments--;
    numTranslatedFragments++;

    if (fault == NoFault)
        mainReq->setFlags(req->getFlags());

    if (numTranslatedFragments == _requests.size()) {
        if (_inst->isSquashed()) {
            this->squashTranslation();
        } else {
            _inst->strictlyOrdered(mainReq->isStrictlyOrdered());
            flags.set(Flag::TranslationFinished);
            _inst->translationCompleted(true);

            for (i = 0; i < _fault.size() && _fault[i] == NoFault; i++);
            if (i > 0) {
                _inst->physEffAddr = request(0)->getPaddr();
                _inst->memReqFlags = mainReq->getFlags();
                if (mainReq->isCondSwap()) {
                    assert (i == _fault.size());
                    assert(_res);
                    mainReq->setExtraData(*_res);
                }
                if (i == _fault.size()) {
                    _inst->fault = NoFault;
                    setState(State::Request);
                } else {
                  _inst->fault = _fault[i];
                  setState(State::PartialFault);
                }
            } else {
                _inst->fault = _fault[0];
                setState(State::Fault);
            }
        }

    }
}

template<class Impl>
void
LSQ<Impl>::SingleDataRequest::initiateTranslation()
{
    assert(_requests.size() == 0);

    this->addRequest(_addr, _size, _byteEnable);

    if (_requests.size() > 0) {
        _requests.back()->setReqInstSeqNum(_inst->seqNum);
        _requests.back()->taskId(_taskId);
        _inst->translationStarted(true);
        setState(State::Translation);
        flags.set(Flag::TranslationStarted);

        _inst->savedReq = this;
        sendFragmentToTranslation(0);
    } else {
        _inst->setMemAccPredicate(false);
    }
}

template<class Impl>
PacketPtr
LSQ<Impl>::SplitDataRequest::mainPacket()
{
    return _mainPacket;
}

template<class Impl>
RequestPtr
LSQ<Impl>::SplitDataRequest::mainRequest()
{
    return mainReq;
}

template<class Impl>
void
LSQ<Impl>::SplitDataRequest::initiateTranslation()
{
    auto cacheLineSize = _port.cacheLineSize();
    Addr base_addr = _addr;
    Addr next_addr = addrBlockAlign(_addr + cacheLineSize, cacheLineSize);
    Addr final_addr = addrBlockAlign(_addr + _size, cacheLineSize);
    uint32_t size_so_far = 0;

    mainReq = std::make_shared<Request>(base_addr,
                _size, _flags, _inst->masterId(),
                _inst->instAddr(), _inst->contextId());
    if (!_byteEnable.empty()) {
        mainReq->setByteEnable(_byteEnable);
    }

    // Paddr is not used in mainReq. However, we will accumulate the flags
    // from the sub requests into mainReq by calling setFlags() in finish().
    // setFlags() assumes that paddr is set so flip the paddr valid bit here to
    // avoid a potential assert in setFlags() when we call it from  finish().
    mainReq->setPaddr(0);

    /* Get the pre-fix, possibly unaligned. */
    if (_byteEnable.empty()) {
        this->addRequest(base_addr, next_addr - base_addr, _byteEnable);
    } else {
        auto it_start = _byteEnable.begin();
        auto it_end = _byteEnable.begin() + (next_addr - base_addr);
        this->addRequest(base_addr, next_addr - base_addr,
                         std::vector<bool>(it_start, it_end));
    }
    size_so_far = next_addr - base_addr;

    /* We are block aligned now, reading whole blocks. */
    base_addr = next_addr;
    while (base_addr != final_addr) {
        if (_byteEnable.empty()) {
            this->addRequest(base_addr, cacheLineSize, _byteEnable);
        } else {
            auto it_start = _byteEnable.begin() + size_so_far;
            auto it_end = _byteEnable.begin() + size_so_far + cacheLineSize;
            this->addRequest(base_addr, cacheLineSize,
                             std::vector<bool>(it_start, it_end));
        }
        size_so_far += cacheLineSize;
        base_addr += cacheLineSize;
    }

    /* Deal with the tail. */
    if (size_so_far < _size) {
        if (_byteEnable.empty()) {
            this->addRequest(base_addr, _size - size_so_far, _byteEnable);
        } else {
            auto it_start = _byteEnable.begin() + size_so_far;
            auto it_end = _byteEnable.end();
            this->addRequest(base_addr, _size - size_so_far,
                             std::vector<bool>(it_start, it_end));
        }
    }

    if (_requests.size() > 0) {
        /* Setup the requests and send them to translation. */
        for (auto& r: _requests) {
            r->setReqInstSeqNum(_inst->seqNum);
            r->taskId(_taskId);
        }

        _inst->translationStarted(true);
        setState(State::Translation);
        flags.set(Flag::TranslationStarted);
        this->_inst->savedReq = this;
        numInTranslationFragments = 0;
        numTranslatedFragments = 0;
        _fault.resize(_requests.size());

        for (uint32_t i = 0; i < _requests.size(); i++) {
            sendFragmentToTranslation(i);
        }
    } else {
        _inst->setMemAccPredicate(false);
    }
}

template<class Impl>
void
LSQ<Impl>::LSQRequest::sendFragmentToTranslation(int i)
{
    numInTranslationFragments++;
    _port.dTLB()->translateTiming(
            this->request(i),
            this->_inst->thread->getTC(), this,
            this->isLoad() ? BaseTLB::Read : BaseTLB::Write);
}

template<class Impl>
bool
LSQ<Impl>::SingleDataRequest::recvTimingResp(PacketPtr pkt)
{
    assert(_numOutstandingPackets == 1);
    auto state = dynamic_cast<LSQSenderState*>(pkt->senderState);
    flags.set(Flag::Complete);
    state->outstanding--;
    assert(pkt == _packets.front());
    _port.completeDataAccess(pkt);
    return true;
}

template<class Impl>
bool
LSQ<Impl>::SplitDataRequest::recvTimingResp(PacketPtr pkt)
{
    auto state = dynamic_cast<LSQSenderState*>(pkt->senderState);
    uint32_t pktIdx = 0;
    while (pktIdx < _packets.size() && pkt != _packets[pktIdx])
        pktIdx++;
    assert(pktIdx < _packets.size());
    numReceivedPackets++;
    state->outstanding--;
    if (numReceivedPackets == _packets.size()) {
        flags.set(Flag::Complete);
        /* Assemble packets. */
        PacketPtr resp = isLoad()
            ? Packet::createRead(mainReq)
            : Packet::createWrite(mainReq);
        if (isLoad())
            resp->dataStatic(_inst->memData);
        else
            resp->dataStatic(_data);
        resp->senderState = _senderState;
        _port.completeDataAccess(resp);
        delete resp;
    }
    return true;
}

template<class Impl>
void
LSQ<Impl>::SingleDataRequest::buildPackets()
{
    assert(_senderState);
    /* Retries do not create new packets. */
    if (_packets.size() == 0) {
        _packets.push_back(
                isLoad()
                    ?  Packet::createRead(request())
                    :  Packet::createWrite(request()));
        _packets.back()->dataStatic(_inst->memData);
        _packets.back()->senderState = _senderState;
    }
    assert(_packets.size() == 1);
}

template<class Impl>
void
LSQ<Impl>::SplitDataRequest::buildPackets()
{
    /* Extra data?? */
    Addr base_address = _addr;

    if (_packets.size() == 0) {
        /* New stuff */
        if (isLoad()) {
            _mainPacket = Packet::createRead(mainReq);
            _mainPacket->dataStatic(_inst->memData);
        }
        for (int i = 0; i < _requests.size() && _fault[i] == NoFault; i++) {
            RequestPtr r = _requests[i];
            PacketPtr pkt = isLoad() ? Packet::createRead(r)
                                     : Packet::createWrite(r);
            ptrdiff_t offset = r->getVaddr() - base_address;
            if (isLoad()) {
                pkt->dataStatic(_inst->memData + offset);
            } else {
                uint8_t* req_data = new uint8_t[r->getSize()];
                std::memcpy(req_data,
                        _inst->memData + offset,
                        r->getSize());
                pkt->dataDynamic(req_data);
            }
            pkt->senderState = _senderState;
            _packets.push_back(pkt);
        }
    }
    assert(_packets.size() > 0);
}

template<class Impl>
void
LSQ<Impl>::SingleDataRequest::sendPacketToCache()
{
    assert(_numOutstandingPackets == 0);
    if (lsqUnit()->trySendPacket(isLoad(), _packets.at(0)))
        _numOutstandingPackets = 1;
}

template<class Impl>
void
LSQ<Impl>::SplitDataRequest::sendPacketToCache()
{
    /* Try to send the packets. */
    while (numReceivedPackets + _numOutstandingPackets < _packets.size() &&
            lsqUnit()->trySendPacket(isLoad(),
                _packets.at(numReceivedPackets + _numOutstandingPackets))) {
        _numOutstandingPackets++;
    }
}

template<class Impl>
Cycles
LSQ<Impl>::SingleDataRequest::handleLocalAccess(
        ThreadContext *thread, PacketPtr pkt)
{
    return pkt->req->localAccessor(thread, pkt);
}

template<class Impl>
Cycles
LSQ<Impl>::SplitDataRequest::handleLocalAccess(
        ThreadContext *thread, PacketPtr mainPkt)
{
    Cycles delay(0);
    unsigned offset = 0;

    for (auto r: _requests) {
        PacketPtr pkt =
            new Packet(r, isLoad() ? MemCmd::ReadReq : MemCmd::WriteReq);
        pkt->dataStatic(mainPkt->getPtr<uint8_t>() + offset);
        Cycles d = r->localAccessor(thread, pkt);
        if (d > delay)
            delay = d;
        offset += r->getSize();
        delete pkt;
    }
    return delay;
}

template<class Impl>
bool
LSQ<Impl>::SingleDataRequest::isCacheBlockHit(Addr blockAddr, Addr blockMask)
{
    return ( (LSQRequest::_requests[0]->getPaddr() & blockMask) == blockAddr);
}

/**
 * Caches may probe into the load-store queue to enforce memory ordering
 * guarantees. This method supports probes by providing a mechanism to compare
 * snoop messages with requests tracked by the load-store queue.
 *
 * Consistency models must enforce ordering constraints. TSO, for instance,
 * must prevent memory reorderings except stores which are reordered after
 * loads. The reordering restrictions negatively impact performance by
 * cutting down on memory level parallelism. However, the core can regain
 * performance by generating speculative loads. Speculative loads may issue
 * without affecting correctness if precautions are taken to handle invalid
 * memory orders. The load queue must squash under memory model violations.
 * Memory model violations may occur when block ownership is granted to
 * another core or the block cannot be accurately monitored by the load queue.
 */
template<class Impl>
bool
LSQ<Impl>::SplitDataRequest::isCacheBlockHit(Addr blockAddr, Addr blockMask)
{
    bool is_hit = false;
    for (auto &r: _requests) {
       /**
        * The load-store queue handles partial faults which complicates this
        * method. Physical addresses must be compared between requests and
        * snoops. Some requests will not have a valid physical address, since
        * partial faults may have outstanding translations. Therefore, the
        * existence of a valid request address must be checked before
        * comparing block hits. We assume no pipeline squash is needed if a
        * valid request address does not exist.
        */
        if (r->hasPaddr() && (r->getPaddr() & blockMask) == blockAddr) {
            is_hit = true;
            break;
        }
    }
    return is_hit;
}

template <class Impl>
bool
LSQ<Impl>::DcachePort::recvTimingResp(PacketPtr pkt)
{
    return lsq->recvTimingResp(pkt);
}

template <class Impl>
void
LSQ<Impl>::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }
    lsq->recvTimingSnoopReq(pkt);
}

template <class Impl>
void
LSQ<Impl>::DcachePort::recvReqRetry()
{
    lsq->recvReqRetry();
}

#endif//__CPU_O3_LSQ_IMPL_HH__
