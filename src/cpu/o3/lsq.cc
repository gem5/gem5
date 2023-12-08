/*
 * Copyright (c) 2011-2012, 2014, 2017-2019, 2021 ARM Limited
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

#include "cpu/o3/lsq.hh"

#include <algorithm>
#include <list>
#include <string>

#include "base/compiler.hh"
#include "base/logging.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/limits.hh"
#include "debug/Drain.hh"
#include "debug/Fetch.hh"
#include "debug/HtmCpu.hh"
#include "debug/LSQ.hh"
#include "debug/Writeback.hh"
#include "params/BaseO3CPU.hh"

namespace gem5
{

namespace o3
{

LSQ::DcachePort::DcachePort(LSQ *_lsq, CPU *_cpu) :
    RequestPort(_cpu->name() + ".dcache_port"), lsq(_lsq), cpu(_cpu)
{}

LSQ::LSQ(CPU *cpu_ptr, IEW *iew_ptr, const BaseO3CPUParams &params)
    : cpu(cpu_ptr), iewStage(iew_ptr),
      _cacheBlocked(false),
      cacheStorePorts(params.cacheStorePorts), usedStorePorts(0),
      cacheLoadPorts(params.cacheLoadPorts), usedLoadPorts(0),
      waitingForStaleTranslation(false),
      staleTranslationWaitTxnId(0),
      lsqPolicy(params.smtLSQPolicy),
      LQEntries(params.LQEntries),
      SQEntries(params.SQEntries),
      maxLQEntries(maxLSQAllocation(lsqPolicy, LQEntries, params.numThreads,
                  params.smtLSQThreshold)),
      maxSQEntries(maxLSQAllocation(lsqPolicy, SQEntries, params.numThreads,
                  params.smtLSQThreshold)),
      dcachePort(this, cpu_ptr),
      numThreads(params.numThreads)
{
    assert(numThreads > 0 && numThreads <= MaxThreads);

    //**********************************************
    //************ Handle SMT Parameters ***********
    //**********************************************

    /* Run SMT olicy checks. */
        if (lsqPolicy == SMTQueuePolicy::Dynamic) {
        DPRINTF(LSQ, "LSQ sharing policy set to Dynamic\n");
    } else if (lsqPolicy == SMTQueuePolicy::Partitioned) {
        DPRINTF(Fetch, "LSQ sharing policy set to Partitioned: "
                "%i entries per LQ | %i entries per SQ\n",
                maxLQEntries,maxSQEntries);
    } else if (lsqPolicy == SMTQueuePolicy::Threshold) {

        assert(params.smtLSQThreshold > params.LQEntries);
        assert(params.smtLSQThreshold > params.SQEntries);

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


std::string
LSQ::name() const
{
    return iewStage->name() + ".lsq";
}

void
LSQ::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
    assert(activeThreads != 0);
}

void
LSQ::drainSanityCheck() const
{
    assert(isDrained());

    for (ThreadID tid = 0; tid < numThreads; tid++)
        thread[tid].drainSanityCheck();
}

bool
LSQ::isDrained() const
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

void
LSQ::takeOverFrom()
{
    usedStorePorts = 0;
    _cacheBlocked = false;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        thread[tid].takeOverFrom();
    }
}

void
LSQ::tick()
{
    // Re-issue loads which got blocked on the per-cycle load ports limit.
    if (usedLoadPorts == cacheLoadPorts && !_cacheBlocked)
        iewStage->cacheUnblocked();

    usedLoadPorts = 0;
    usedStorePorts = 0;
}

bool
LSQ::cacheBlocked() const
{
    return _cacheBlocked;
}

void
LSQ::cacheBlocked(bool v)
{
    _cacheBlocked = v;
}

bool
LSQ::cachePortAvailable(bool is_load) const
{
    bool ret;
    if (is_load) {
        ret  = usedLoadPorts < cacheLoadPorts;
    } else {
        ret  = usedStorePorts < cacheStorePorts;
    }
    return ret;
}

void
LSQ::cachePortBusy(bool is_load)
{
    assert(cachePortAvailable(is_load));
    if (is_load) {
        usedLoadPorts++;
    } else {
        usedStorePorts++;
    }
}

void
LSQ::insertLoad(const DynInstPtr &load_inst)
{
    ThreadID tid = load_inst->threadNumber;

    thread[tid].insertLoad(load_inst);
}

void
LSQ::insertStore(const DynInstPtr &store_inst)
{
    ThreadID tid = store_inst->threadNumber;

    thread[tid].insertStore(store_inst);
}

Fault
LSQ::executeLoad(const DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    return thread[tid].executeLoad(inst);
}

Fault
LSQ::executeStore(const DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    return thread[tid].executeStore(inst);
}

void
LSQ::commitLoads(InstSeqNum &youngest_inst, ThreadID tid)
{
    thread.at(tid).commitLoads(youngest_inst);
}

void
LSQ::commitStores(InstSeqNum &youngest_inst, ThreadID tid)
{
    thread.at(tid).commitStores(youngest_inst);
}

void
LSQ::writebackStores()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (numStoresToWB(tid) > 0) {
            DPRINTF(Writeback,"[tid:%i] Writing back stores. %i stores "
                "available for Writeback.\n", tid, numStoresToWB(tid));
        }

        thread[tid].writebackStores();
    }
}

void
LSQ::squash(const InstSeqNum &squashed_num, ThreadID tid)
{
    thread.at(tid).squash(squashed_num);
}

bool
LSQ::violation()
{
    /* Answers: Does Anybody Have a Violation?*/
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (thread[tid].violation())
            return true;
    }

    return false;
}

bool LSQ::violation(ThreadID tid) { return thread.at(tid).violation(); }

DynInstPtr
LSQ::getMemDepViolator(ThreadID tid)
{
    return thread.at(tid).getMemDepViolator();
}

int
LSQ::getLoadHead(ThreadID tid)
{
    return thread.at(tid).getLoadHead();
}

InstSeqNum
LSQ::getLoadHeadSeqNum(ThreadID tid)
{
    return thread.at(tid).getLoadHeadSeqNum();
}

int
LSQ::getStoreHead(ThreadID tid)
{
    return thread.at(tid).getStoreHead();
}

InstSeqNum
LSQ::getStoreHeadSeqNum(ThreadID tid)
{
    return thread.at(tid).getStoreHeadSeqNum();
}

int LSQ::getCount(ThreadID tid) { return thread.at(tid).getCount(); }

int LSQ::numLoads(ThreadID tid) { return thread.at(tid).numLoads(); }

int LSQ::numStores(ThreadID tid) { return thread.at(tid).numStores(); }

int
LSQ::numHtmStarts(ThreadID tid) const
{
    if (tid == InvalidThreadID)
        return 0;
    else
        return thread[tid].numHtmStarts();
}
int
LSQ::numHtmStops(ThreadID tid) const
{
    if (tid == InvalidThreadID)
        return 0;
    else
        return thread[tid].numHtmStops();
}

void
LSQ::resetHtmStartsStops(ThreadID tid)
{
    if (tid != InvalidThreadID)
        thread[tid].resetHtmStartsStops();
}

uint64_t
LSQ::getLatestHtmUid(ThreadID tid) const
{
    if (tid == InvalidThreadID)
        return 0;
    else
        return thread[tid].getLatestHtmUid();
}

void
LSQ::setLastRetiredHtmUid(ThreadID tid, uint64_t htmUid)
{
    if (tid != InvalidThreadID)
        thread[tid].setLastRetiredHtmUid(htmUid);
}

void
LSQ::recvReqRetry()
{
    iewStage->cacheUnblocked();
    cacheBlocked(false);

    for (ThreadID tid : *activeThreads) {
        thread[tid].recvRetry();
    }
}

void
LSQ::completeDataAccess(PacketPtr pkt)
{
    LSQRequest *request = dynamic_cast<LSQRequest*>(pkt->senderState);
    thread[cpu->contextToThread(request->contextId())]
        .completeDataAccess(pkt);
}

bool
LSQ::recvTimingResp(PacketPtr pkt)
{
    if (pkt->isError())
        DPRINTF(LSQ, "Got error packet back for address: %#X\n",
                pkt->getAddr());

    LSQRequest *request = dynamic_cast<LSQRequest*>(pkt->senderState);
    panic_if(!request, "Got packet back with unknown sender state\n");

    thread[cpu->contextToThread(request->contextId())].recvTimingResp(pkt);

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
    request->packetReplied();

    if (waitingForStaleTranslation) {
        checkStaleTranslations();
    }

    return true;
}

void
LSQ::recvTimingSnoopReq(PacketPtr pkt)
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
    } else if (pkt->req && pkt->req->isTlbiExtSync()) {
        DPRINTF(LSQ, "received TLBI Ext Sync\n");
        assert(!waitingForStaleTranslation);

        waitingForStaleTranslation = true;
        staleTranslationWaitTxnId = pkt->req->getExtraData();

        for (auto& unit : thread) {
            unit.startStaleTranslationFlush();
        }

        // In case no units have pending ops, just go ahead
        checkStaleTranslations();
    }
}

int
LSQ::getCount()
{
    unsigned total = 0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += getCount(tid);
    }

    return total;
}

int
LSQ::numLoads()
{
    unsigned total = 0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += numLoads(tid);
    }

    return total;
}

int
LSQ::numStores()
{
    unsigned total = 0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += thread[tid].numStores();
    }

    return total;
}

unsigned
LSQ::numFreeLoadEntries()
{
    unsigned total = 0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += thread[tid].numFreeLoadEntries();
    }

    return total;
}

unsigned
LSQ::numFreeStoreEntries()
{
    unsigned total = 0;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        total += thread[tid].numFreeStoreEntries();
    }

    return total;
}

unsigned
LSQ::numFreeLoadEntries(ThreadID tid)
{
        return thread[tid].numFreeLoadEntries();
}

unsigned
LSQ::numFreeStoreEntries(ThreadID tid)
{
        return thread[tid].numFreeStoreEntries();
}

bool
LSQ::isFull()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!(thread[tid].lqFull() || thread[tid].sqFull()))
            return false;
    }

    return true;
}

bool
LSQ::isFull(ThreadID tid)
{
    //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return isFull();
    else
        return thread[tid].lqFull() || thread[tid].sqFull();
}

bool
LSQ::isEmpty() const
{
    return lqEmpty() && sqEmpty();
}

bool
LSQ::lqEmpty() const
{
    std::list<ThreadID>::const_iterator threads = activeThreads->begin();
    std::list<ThreadID>::const_iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].lqEmpty())
            return false;
    }

    return true;
}

bool
LSQ::sqEmpty() const
{
    std::list<ThreadID>::const_iterator threads = activeThreads->begin();
    std::list<ThreadID>::const_iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].sqEmpty())
            return false;
    }

    return true;
}

bool
LSQ::lqFull()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].lqFull())
            return false;
    }

    return true;
}

bool
LSQ::lqFull(ThreadID tid)
{
    //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return lqFull();
    else
        return thread[tid].lqFull();
}

bool
LSQ::sqFull()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!sqFull(tid))
            return false;
    }

    return true;
}

bool
LSQ::sqFull(ThreadID tid)
{
     //@todo: Change to Calculate All Entries for
    //Dynamic Policy
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return sqFull();
    else
        return thread[tid].sqFull();
}

bool
LSQ::isStalled()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!thread[tid].isStalled())
            return false;
    }

    return true;
}

bool
LSQ::isStalled(ThreadID tid)
{
    if (lsqPolicy == SMTQueuePolicy::Dynamic)
        return isStalled();
    else
        return thread[tid].isStalled();
}

bool
LSQ::hasStoresToWB()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (hasStoresToWB(tid))
            return true;
    }

    return false;
}

bool
LSQ::hasStoresToWB(ThreadID tid)
{
    return thread.at(tid).hasStoresToWB();
}

int
LSQ::numStoresToWB(ThreadID tid)
{
    return thread.at(tid).numStoresToWB();
}

bool
LSQ::willWB()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (willWB(tid))
            return true;
    }

    return false;
}

bool
LSQ::willWB(ThreadID tid)
{
    return thread.at(tid).willWB();
}

void
LSQ::dumpInsts() const
{
    std::list<ThreadID>::const_iterator threads = activeThreads->begin();
    std::list<ThreadID>::const_iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        thread[tid].dumpInsts();
    }
}

void
LSQ::dumpInsts(ThreadID tid) const
{
    thread.at(tid).dumpInsts();
}

Fault
LSQ::pushRequest(const DynInstPtr& inst, bool isLoad, uint8_t *data,
        unsigned int size, Addr addr, Request::Flags flags, uint64_t *res,
        AtomicOpFunctorPtr amo_op, const std::vector<bool>& byte_enable)
{
    // This comming request can be either load, store or atomic.
    // Atomic request has a corresponding pointer to its atomic memory
    // operation
    [[maybe_unused]] bool isAtomic = !isLoad && amo_op;

    ThreadID tid = cpu->contextToThread(inst->contextId());
    auto cacheLineSize = cpu->cacheLineSize();
    bool needs_burst = transferNeedsBurst(addr, size, cacheLineSize);
    LSQRequest* request = nullptr;

    // Atomic requests that access data across cache line boundary are
    // currently not allowed since the cache does not guarantee corresponding
    // atomic memory operations to be executed atomically across a cache line.
    // For ISAs such as x86 that supports cross-cache-line atomic instructions,
    // the cache needs to be modified to perform atomic update to both cache
    // lines. For now, such cross-line update is not supported.
    assert(!isAtomic || (isAtomic && !needs_burst));

    const bool htm_cmd = isLoad && (flags & Request::HTM_CMD);
    const bool tlbi_cmd = isLoad && (flags & Request::TLBI_CMD);

    if (inst->translationStarted()) {
        request = inst->savedRequest;
        assert(request);
    } else {
        if (htm_cmd || tlbi_cmd) {
            assert(addr == 0x0lu);
            assert(size == 8);
            request = new UnsquashableDirectRequest(&thread[tid], inst, flags);
        } else if (needs_burst) {
            request = new SplitDataRequest(&thread[tid], inst, isLoad, addr,
                    size, flags, data, res);
        } else {
            request = new SingleDataRequest(&thread[tid], inst, isLoad, addr,
                    size, flags, data, res, std::move(amo_op));
        }
        assert(request);
        request->_byteEnable = byte_enable;
        inst->setRequest();
        request->taskId(cpu->taskId());

        // There might be fault from a previous execution attempt if this is
        // a strictly ordered load
        inst->getFault() = NoFault;

        request->initiateTranslation();
    }

    /* This is the place were instructions get the effAddr. */
    if (request->isTranslationComplete()) {
        if (request->isMemAccessRequired()) {
            inst->effAddr = request->getVaddr();
            inst->effSize = size;
            inst->effAddrValid(true);

            if (cpu->checker) {
                inst->reqToVerify = std::make_shared<Request>(*request->req());
            }
            Fault fault;
            if (isLoad)
                fault = read(request, inst->lqIdx);
            else
                fault = write(request, data, inst->sqIdx);
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

void
LSQ::SingleDataRequest::finish(const Fault &fault, const RequestPtr &request,
        gem5::ThreadContext* tc, BaseMMU::Mode mode)
{
    _fault.push_back(fault);
    numInTranslationFragments = 0;
    numTranslatedFragments = 1;
    /* If the instruction has been squahsed, let the request know
     * as it may have to self-destruct. */
    if (_inst->isSquashed()) {
        squashTranslation();
    } else {
        _inst->strictlyOrdered(request->isStrictlyOrdered());

        flags.set(Flag::TranslationFinished);
        if (fault == NoFault) {
            _inst->physEffAddr = request->getPaddr();
            _inst->memReqFlags = request->getFlags();
            if (request->isCondSwap()) {
                assert(_res);
                request->setExtraData(*_res);
            }
            setState(State::Request);
        } else {
            setState(State::Fault);
        }

        LSQRequest::_inst->fault = fault;
        LSQRequest::_inst->translationCompleted(true);
    }
}

void
LSQ::SplitDataRequest::finish(const Fault &fault, const RequestPtr &req,
        gem5::ThreadContext* tc, BaseMMU::Mode mode)
{
    int i;
    for (i = 0; i < _reqs.size() && _reqs[i] != req; i++);
    assert(i < _reqs.size());
    _fault[i] = fault;

    numInTranslationFragments--;
    numTranslatedFragments++;

    if (fault == NoFault)
        _mainReq->setFlags(req->getFlags());

    if (numTranslatedFragments == _reqs.size()) {
        if (_inst->isSquashed()) {
            squashTranslation();
        } else {
            _inst->strictlyOrdered(_mainReq->isStrictlyOrdered());
            flags.set(Flag::TranslationFinished);
            _inst->translationCompleted(true);

            for (i = 0; i < _fault.size() && _fault[i] == NoFault; i++);
            if (i > 0) {
                _inst->physEffAddr = LSQRequest::req()->getPaddr();
                _inst->memReqFlags = _mainReq->getFlags();
                if (_mainReq->isCondSwap()) {
                    assert (i == _fault.size());
                    assert(_res);
                    _mainReq->setExtraData(*_res);
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

void
LSQ::SingleDataRequest::initiateTranslation()
{
    assert(_reqs.size() == 0);

    addReq(_addr, _size, _byteEnable);

    if (_reqs.size() > 0) {
        _reqs.back()->setReqInstSeqNum(_inst->seqNum);
        _reqs.back()->taskId(_taskId);
        _inst->translationStarted(true);
        setState(State::Translation);
        flags.set(Flag::TranslationStarted);

        _inst->savedRequest = this;
        sendFragmentToTranslation(0);
    } else {
        _inst->setMemAccPredicate(false);
    }
}

PacketPtr
LSQ::SplitDataRequest::mainPacket()
{
    return _mainPacket;
}

RequestPtr
LSQ::SplitDataRequest::mainReq()
{
    return _mainReq;
}

void
LSQ::SplitDataRequest::initiateTranslation()
{
    auto cacheLineSize = _port.cacheLineSize();
    Addr base_addr = _addr;
    Addr next_addr = addrBlockAlign(_addr + cacheLineSize, cacheLineSize);
    Addr final_addr = addrBlockAlign(_addr + _size, cacheLineSize);
    uint32_t size_so_far = 0;

    _mainReq = std::make_shared<Request>(base_addr,
                _size, _flags, _inst->requestorId(),
                _inst->pcState().instAddr(), _inst->contextId());
    _mainReq->setByteEnable(_byteEnable);

    // Paddr is not used in _mainReq. However, we will accumulate the flags
    // from the sub requests into _mainReq by calling setFlags() in finish().
    // setFlags() assumes that paddr is set so flip the paddr valid bit here to
    // avoid a potential assert in setFlags() when we call it from  finish().
    _mainReq->setPaddr(0);

    /* Get the pre-fix, possibly unaligned. */
    auto it_start = _byteEnable.begin();
    auto it_end = _byteEnable.begin() + (next_addr - base_addr);
    addReq(base_addr, next_addr - base_addr,
                     std::vector<bool>(it_start, it_end));
    size_so_far = next_addr - base_addr;

    /* We are block aligned now, reading whole blocks. */
    base_addr = next_addr;
    while (base_addr != final_addr) {
        auto it_start = _byteEnable.begin() + size_so_far;
        auto it_end = _byteEnable.begin() + size_so_far + cacheLineSize;
        addReq(base_addr, cacheLineSize,
                         std::vector<bool>(it_start, it_end));
        size_so_far += cacheLineSize;
        base_addr += cacheLineSize;
    }

    /* Deal with the tail. */
    if (size_so_far < _size) {
        auto it_start = _byteEnable.begin() + size_so_far;
        auto it_end = _byteEnable.end();
        addReq(base_addr, _size - size_so_far,
                         std::vector<bool>(it_start, it_end));
    }

    if (_reqs.size() > 0) {
        /* Setup the requests and send them to translation. */
        for (auto& r: _reqs) {
            r->setReqInstSeqNum(_inst->seqNum);
            r->taskId(_taskId);
        }

        _inst->translationStarted(true);
        setState(State::Translation);
        flags.set(Flag::TranslationStarted);
        _inst->savedRequest = this;
        numInTranslationFragments = 0;
        numTranslatedFragments = 0;
        _fault.resize(_reqs.size());

        for (uint32_t i = 0; i < _reqs.size(); i++) {
            sendFragmentToTranslation(i);
        }
    } else {
        _inst->setMemAccPredicate(false);
    }
}

LSQ::LSQRequest::LSQRequest(
        LSQUnit *port, const DynInstPtr& inst, bool isLoad) :
    _state(State::NotIssued),
    _port(*port), _inst(inst), _data(nullptr),
    _res(nullptr), _addr(0), _size(0), _flags(0),
    _numOutstandingPackets(0), _amo_op(nullptr)
{
    flags.set(Flag::IsLoad, isLoad);
    this->setWBToRegister();
    flags.set(Flag::IsAtomic, _inst->isAtomic());
    install();
}

LSQ::LSQRequest::LSQRequest(
        LSQUnit *port, const DynInstPtr& inst, bool isLoad,
        const Addr& addr, const uint32_t& size, const Request::Flags& flags_,
        PacketDataPtr data, uint64_t* res, AtomicOpFunctorPtr amo_op,
        bool stale_translation)
    : _state(State::NotIssued),
    numTranslatedFragments(0),
    numInTranslationFragments(0),
    _port(*port), _inst(inst), _data(data),
    _res(res), _addr(addr), _size(size),
    _flags(flags_),
    _numOutstandingPackets(0),
    _amo_op(std::move(amo_op)),
    _hasStaleTranslation(stale_translation)
{
    flags.set(Flag::IsLoad, isLoad);
    this->setWBToRegister();
    flags.set(Flag::IsAtomic, _inst->isAtomic());
    install();
}

void
LSQ::LSQRequest::setWBToRegister()
{
    bool need_writeback_to_reg = false;
    if (_inst->isStoreConditional() || _inst->isAtomic()) {
        need_writeback_to_reg = true;
    } else if (_inst->isLoad()) {
        if (_inst->isDataPrefetch() || _inst->isInstPrefetch()) {
            // Prefetch inst.
            if (_port.getCPU()->params().block_on_prefetch_inst) {
                // Only enable this if we block on prefetch inst.
                need_writeback_to_reg = true;
            }
        } else {
            // Normal load inst.
            need_writeback_to_reg = true;
        }
    }
    flags.set(Flag::WriteBackToRegister, need_writeback_to_reg);
    if (_inst->isDataPrefetch() || _inst->isInstPrefetch()) {
        DPRINTF(LSQ, "[sn:%llu] Pf inst NeedWB %d.\n",
            _inst->seqNum, this->needWBToRegister());
    }
}

void
LSQ::LSQRequest::install()
{
    if (isLoad()) {
        _port.loadQueue[_inst->lqIdx].setRequest(this);
    } else {
        // Store, StoreConditional, and Atomic requests are pushed
        // to this storeQueue
        _port.storeQueue[_inst->sqIdx].setRequest(this);
    }
}

bool LSQ::LSQRequest::squashed() const { return _inst->isSquashed(); }

void
LSQ::LSQRequest::addReq(Addr addr, unsigned size,
           const std::vector<bool>& byte_enable)
{
    if (isAnyActiveElement(byte_enable.begin(), byte_enable.end())) {
        auto req = std::make_shared<Request>(
                addr, size, _flags, _inst->requestorId(),
                _inst->pcState().instAddr(), _inst->contextId(),
                std::move(_amo_op));
        req->setByteEnable(byte_enable);

        /* If the request is marked as NO_ACCESS, setup a local access */
        if (_flags.isSet(Request::NO_ACCESS)) {
            req->setLocalAccessor(
                [this, req](gem5::ThreadContext *tc, PacketPtr pkt) -> Cycles
                {
                    if ((req->isHTMStart() || req->isHTMCommit())) {
                        auto& inst = this->instruction();
                        assert(inst->inHtmTransactionalState());
                        pkt->setHtmTransactional(
                            inst->getHtmTransactionUid());
                    }
                    return Cycles(1);
                }
            );
        }

        _reqs.push_back(req);
    }
}

LSQ::LSQRequest::~LSQRequest()
{
    assert(!isAnyOutstandingRequest());
    _inst->savedRequest = nullptr;

    for (auto r: _packets)
        delete r;
};

ContextID
LSQ::LSQRequest::contextId() const
{
    return _inst->contextId();
}

void
LSQ::LSQRequest::sendFragmentToTranslation(int i)
{
    numInTranslationFragments++;
    _port.getMMUPtr()->translateTiming(req(i), _inst->thread->getTC(),
            this, isLoad() ? BaseMMU::Read : BaseMMU::Write);
}

void
LSQ::SingleDataRequest::markAsStaleTranslation()
{
    // If this element has been translated and is currently being requested,
    // then it may be stale
    if ((!flags.isSet(Flag::Complete)) &&
        (!flags.isSet(Flag::Discarded)) &&
        (flags.isSet(Flag::TranslationStarted))) {
        _hasStaleTranslation = true;
    }

    DPRINTF(LSQ, "SingleDataRequest %d 0x%08x isBlocking:%d\n",
        (int)_state, (uint32_t)flags, _hasStaleTranslation);
}

void
LSQ::SplitDataRequest::markAsStaleTranslation()
{
    // If this element has been translated and is currently being requested,
    // then it may be stale
    if ((!flags.isSet(Flag::Complete)) &&
        (!flags.isSet(Flag::Discarded)) &&
        (flags.isSet(Flag::TranslationStarted))) {
        _hasStaleTranslation = true;
    }

    DPRINTF(LSQ, "SplitDataRequest %d 0x%08x isBlocking:%d\n",
        (int)_state, (uint32_t)flags, _hasStaleTranslation);
}

bool
LSQ::SingleDataRequest::recvTimingResp(PacketPtr pkt)
{
    assert(_numOutstandingPackets == 1);
    flags.set(Flag::Complete);
    assert(pkt == _packets.front());
    _port.completeDataAccess(pkt);
    _hasStaleTranslation = false;
    return true;
}

bool
LSQ::SplitDataRequest::recvTimingResp(PacketPtr pkt)
{
    uint32_t pktIdx = 0;
    while (pktIdx < _packets.size() && pkt != _packets[pktIdx])
        pktIdx++;
    assert(pktIdx < _packets.size());
    numReceivedPackets++;
    if (numReceivedPackets == _packets.size()) {
        flags.set(Flag::Complete);
        /* Assemble packets. */
        PacketPtr resp = isLoad()
            ? Packet::createRead(_mainReq)
            : Packet::createWrite(_mainReq);
        if (isLoad())
            resp->dataStatic(_inst->memData);
        else
            resp->dataStatic(_data);
        resp->senderState = this;
        _port.completeDataAccess(resp);
        delete resp;
    }
    _hasStaleTranslation = false;
    return true;
}

void
LSQ::SingleDataRequest::buildPackets()
{
    /* Retries do not create new packets. */
    if (_packets.size() == 0) {
        _packets.push_back(
                isLoad()
                    ?  Packet::createRead(req())
                    :  Packet::createWrite(req()));
        _packets.back()->dataStatic(_inst->memData);
        _packets.back()->senderState = this;

        // hardware transactional memory
        // If request originates in a transaction (not necessarily a HtmCmd),
        // then the packet should be marked as such.
        if (_inst->inHtmTransactionalState()) {
            _packets.back()->setHtmTransactional(
                _inst->getHtmTransactionUid());

            DPRINTF(HtmCpu,
              "HTM %s pc=0x%lx - vaddr=0x%lx - paddr=0x%lx - htmUid=%u\n",
              isLoad() ? "LD" : "ST",
              _inst->pcState().instAddr(),
              _packets.back()->req->hasVaddr() ?
                  _packets.back()->req->getVaddr() : 0lu,
              _packets.back()->getAddr(),
              _inst->getHtmTransactionUid());
        }
    }
    assert(_packets.size() == 1);
}

void
LSQ::SplitDataRequest::buildPackets()
{
    /* Extra data?? */
    Addr base_address = _addr;

    if (_packets.size() == 0) {
        /* New stuff */
        if (isLoad()) {
            _mainPacket = Packet::createRead(_mainReq);
            _mainPacket->dataStatic(_inst->memData);

            // hardware transactional memory
            // If request originates in a transaction,
            // packet should be marked as such
            if (_inst->inHtmTransactionalState()) {
                _mainPacket->setHtmTransactional(
                    _inst->getHtmTransactionUid());
                DPRINTF(HtmCpu,
                  "HTM LD.0 pc=0x%lx-vaddr=0x%lx-paddr=0x%lx-htmUid=%u\n",
                  _inst->pcState().instAddr(),
                  _mainPacket->req->hasVaddr() ?
                      _mainPacket->req->getVaddr() : 0lu,
                  _mainPacket->getAddr(),
                  _inst->getHtmTransactionUid());
            }
        }
        for (int i = 0; i < _reqs.size() && _fault[i] == NoFault; i++) {
            RequestPtr req = _reqs[i];
            PacketPtr pkt = isLoad() ? Packet::createRead(req)
                                     : Packet::createWrite(req);
            ptrdiff_t offset = req->getVaddr() - base_address;
            if (isLoad()) {
                pkt->dataStatic(_inst->memData + offset);
            } else {
                uint8_t* req_data = new uint8_t[req->getSize()];
                std::memcpy(req_data,
                        _inst->memData + offset,
                        req->getSize());
                pkt->dataDynamic(req_data);
            }
            pkt->senderState = this;
            _packets.push_back(pkt);

            // hardware transactional memory
            // If request originates in a transaction,
            // packet should be marked as such
            if (_inst->inHtmTransactionalState()) {
                _packets.back()->setHtmTransactional(
                    _inst->getHtmTransactionUid());
                DPRINTF(HtmCpu,
                  "HTM %s.%d pc=0x%lx-vaddr=0x%lx-paddr=0x%lx-htmUid=%u\n",
                  isLoad() ? "LD" : "ST",
                  i+1,
                  _inst->pcState().instAddr(),
                  _packets.back()->req->hasVaddr() ?
                      _packets.back()->req->getVaddr() : 0lu,
                  _packets.back()->getAddr(),
                  _inst->getHtmTransactionUid());
            }
        }
    }
    assert(_packets.size() > 0);
}

void
LSQ::SingleDataRequest::sendPacketToCache()
{
    assert(_numOutstandingPackets == 0);
    if (lsqUnit()->trySendPacket(isLoad(), _packets.at(0)))
        _numOutstandingPackets = 1;
}

void
LSQ::SplitDataRequest::sendPacketToCache()
{
    /* Try to send the packets. */
    while (numReceivedPackets + _numOutstandingPackets < _packets.size() &&
            lsqUnit()->trySendPacket(isLoad(),
                _packets.at(numReceivedPackets + _numOutstandingPackets))) {
        _numOutstandingPackets++;
    }
}

Cycles
LSQ::SingleDataRequest::handleLocalAccess(
        gem5::ThreadContext *thread, PacketPtr pkt)
{
    return pkt->req->localAccessor(thread, pkt);
}

Cycles
LSQ::SplitDataRequest::handleLocalAccess(
        gem5::ThreadContext *thread, PacketPtr mainPkt)
{
    Cycles delay(0);
    unsigned offset = 0;

    for (auto r: _reqs) {
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

bool
LSQ::SingleDataRequest::isCacheBlockHit(Addr blockAddr, Addr blockMask)
{
    return ( (LSQRequest::_reqs[0]->getPaddr() & blockMask) == blockAddr);
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
bool
LSQ::SplitDataRequest::isCacheBlockHit(Addr blockAddr, Addr blockMask)
{
    bool is_hit = false;
    for (auto &r: _reqs) {
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

bool
LSQ::DcachePort::recvTimingResp(PacketPtr pkt)
{
    return lsq->recvTimingResp(pkt);
}

void
LSQ::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }
    lsq->recvTimingSnoopReq(pkt);
}

void
LSQ::DcachePort::recvReqRetry()
{
    lsq->recvReqRetry();
}

LSQ::UnsquashableDirectRequest::UnsquashableDirectRequest(
    LSQUnit* port,
    const DynInstPtr& inst,
    const Request::Flags& flags_) :
    SingleDataRequest(port, inst, true, 0x0lu, 8, flags_,
        nullptr, nullptr, nullptr)
{
}

void
LSQ::UnsquashableDirectRequest::initiateTranslation()
{
    // Special commands are implemented as loads to avoid significant
    // changes to the cpu and memory interfaces
    // The virtual and physical address uses a dummy value of 0x00
    // Address translation does not really occur thus the code below

    assert(_reqs.size() == 0);

    addReq(_addr, _size, _byteEnable);

    if (_reqs.size() > 0) {
        _reqs.back()->setReqInstSeqNum(_inst->seqNum);
        _reqs.back()->taskId(_taskId);
        _reqs.back()->setPaddr(_addr);
        _reqs.back()->setInstCount(_inst->getCpuPtr()->totalInsts());

        _inst->strictlyOrdered(_reqs.back()->isStrictlyOrdered());
        _inst->fault = NoFault;
        _inst->physEffAddr = _reqs.back()->getPaddr();
        _inst->memReqFlags = _reqs.back()->getFlags();
        _inst->savedRequest = this;

        flags.set(Flag::TranslationStarted);
        flags.set(Flag::TranslationFinished);

        _inst->translationStarted(true);
        _inst->translationCompleted(true);

        setState(State::Request);
    } else {
        panic("unexpected behaviour in initiateTranslation()");
    }
}

void
LSQ::UnsquashableDirectRequest::markAsStaleTranslation()
{
    // HTM/TLBI operations do not translate,
    // so cannot have stale translations
    _hasStaleTranslation = false;
}

void
LSQ::UnsquashableDirectRequest::finish(const Fault &fault,
        const RequestPtr &req, gem5::ThreadContext* tc,
        BaseMMU::Mode mode)
{
    panic("unexpected behaviour - finish()");
}

void
LSQ::checkStaleTranslations()
{
    assert(waitingForStaleTranslation);

    DPRINTF(LSQ, "Checking pending TLBI sync\n");
    // Check if all thread queues are complete
    for (const auto& unit : thread) {
        if (unit.checkStaleTranslations())
            return;
    }
    DPRINTF(LSQ, "No threads have blocking TLBI sync\n");

    // All thread queues have committed their sync operations
    // => send a RubyRequest to the sequencer
    auto req = Request::createMemManagement(
        Request::TLBI_EXT_SYNC_COMP,
        cpu->dataRequestorId());
    req->setExtraData(staleTranslationWaitTxnId);
    PacketPtr pkt = Packet::createRead(req);

    // TODO - reserve some credit for these responses?
    if (!dcachePort.sendTimingReq(pkt)) {
        panic("Couldn't send TLBI_EXT_SYNC_COMP message");
    }

    waitingForStaleTranslation = false;
    staleTranslationWaitTxnId = 0;
}

Fault
LSQ::read(LSQRequest* request, ssize_t load_idx)
{
    assert(request->req()->contextId() == request->contextId());
    ThreadID tid = cpu->contextToThread(request->req()->contextId());

    return thread.at(tid).read(request, load_idx);
}

Fault
LSQ::write(LSQRequest* request, uint8_t *data, ssize_t store_idx)
{
    ThreadID tid = cpu->contextToThread(request->req()->contextId());

    return thread.at(tid).write(request, data, store_idx);
}

} // namespace o3
} // namespace gem5
