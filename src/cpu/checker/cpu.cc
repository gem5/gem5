/*
 * Copyright (c) 2011,2013,2017-2018, 2020 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "cpu/checker/cpu.hh"

#include <list>
#include <string>

#include "arch/generic/tlb.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "cpu/utils.hh"
#include "params/CheckerCPU.hh"
#include "sim/full_system.hh"

namespace gem5
{

void
CheckerCPU::init()
{
    requestorId = systemPtr->getRequestorId(this);
    tc->getIsaPtr()->setThreadContext(tc);
}

CheckerCPU::CheckerCPU(const Params &p)
    : BaseCPU(p, true),
      systemPtr(NULL),
      icachePort(NULL),
      dcachePort(NULL),
      tc(NULL),
      thread(NULL),
      unverifiedReq(nullptr),
      unverifiedMemData(nullptr)
{
    curStaticInst = NULL;
    curMacroStaticInst = NULL;

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    youngestSN = 0;

    changedPC = willChangePC = false;

    exitOnError = p.exitOnError;
    warnOnlyOnLoadError = p.warnOnlyOnLoadError;
    mmu = p.mmu;
    workload = p.workload;

    updateOnError = true;
}

CheckerCPU::~CheckerCPU() {}

void
CheckerCPU::setSystem(System *system)
{
    const Params &p = params();

    systemPtr = system;

    if (FullSystem) {
        thread =
            new SimpleThread(this, 0, systemPtr, mmu, p.isa[0], p.decoder[0]);
    } else {
        thread = new SimpleThread(this, 0, systemPtr,
                                  workload.size() ? workload[0] : NULL, mmu,
                                  p.isa[0], p.decoder[0]);
    }

    tc = thread->getTC();
    threadContexts.push_back(tc);
    // Thread should never be null after this
    assert(thread != NULL);
}

void
CheckerCPU::setIcachePort(RequestPort *icache_port)
{
    icachePort = icache_port;
}

void
CheckerCPU::setDcachePort(RequestPort *dcache_port)
{
    dcachePort = dcache_port;
}

void
CheckerCPU::serialize(std::ostream &os) const
{}

void
CheckerCPU::unserialize(CheckpointIn &cp)
{}

RequestPtr
CheckerCPU::genMemFragmentRequest(Addr frag_addr, int size,
                                  Request::Flags flags,
                                  const std::vector<bool> &byte_enable,
                                  int &frag_size, int &size_left) const
{
    frag_size =
        std::min(cacheLineSize() - addrBlockOffset(frag_addr, cacheLineSize()),
                 (Addr)size_left);
    size_left -= frag_size;

    RequestPtr mem_req;

    // Set up byte-enable mask for the current fragment
    auto it_start = byte_enable.cbegin() + (size - (frag_size + size_left));
    auto it_end = byte_enable.cbegin() + (size - size_left);
    if (isAnyActiveElement(it_start, it_end)) {
        mem_req = std::make_shared<Request>(
            frag_addr, frag_size, flags, requestorId,
            thread->pcState().instAddr(), tc->contextId());
        mem_req->setByteEnable(std::vector<bool>(it_start, it_end));
    }

    return mem_req;
}

Fault
CheckerCPU::readMem(Addr addr, uint8_t *data, unsigned size,
                    Request::Flags flags, const std::vector<bool> &byte_enable)
{
    assert(byte_enable.size() == size);

    Fault fault = NoFault;
    bool checked_flags = false;
    bool flags_match = true;
    Addr pAddr = 0x0;

    Addr frag_addr = addr;
    int frag_size = 0;
    int size_left = size;
    bool predicate;

    // Need to account for multiple accesses like the Atomic and TimingSimple
    while (1) {
        RequestPtr mem_req = genMemFragmentRequest(
            frag_addr, size, flags, byte_enable, frag_size, size_left);

        predicate = (mem_req != nullptr);

        // translate to physical address
        if (predicate) {
            fault = mmu->translateFunctional(mem_req, tc, BaseMMU::Read);
        }

        if (predicate && !checked_flags && fault == NoFault && unverifiedReq) {
            flags_match = checkFlags(unverifiedReq, mem_req->getVaddr(),
                                     mem_req->getPaddr(), mem_req->getFlags());
            pAddr = mem_req->getPaddr();
            checked_flags = true;
        }

        // Now do the access
        if (predicate && fault == NoFault &&
            !mem_req->getFlags().isSet(Request::NO_ACCESS)) {
            PacketPtr pkt = Packet::createRead(mem_req);

            pkt->dataStatic(data);

            if (!(mem_req->isUncacheable() || mem_req->isLocalAccess())) {
                // Access memory to see if we have the same data
                dcachePort->sendFunctional(pkt);
            } else {
                // Assume the data is correct if it's an uncached access
                memcpy(data, unverifiedMemData, frag_size);
            }

            delete pkt;
        }

        if (fault != NoFault) {
            if (mem_req->isPrefetch()) {
                fault = NoFault;
            }
            break;
        }

        // If we don't need to access a second cache line, stop now.
        if (size_left == 0) {
            break;
        }

        // Setup for accessing next cache line
        frag_addr += frag_size;
        data += frag_size;
        unverifiedMemData += frag_size;
    }

    if (!flags_match) {
        warn("%lli: Flags do not match CPU:%#x %#x %#x Checker:%#x %#x %#x\n",
             curTick(), unverifiedReq->getVaddr(), unverifiedReq->getPaddr(),
             unverifiedReq->getFlags(), frag_addr, pAddr, flags);
        handleError();
    }

    return fault;
}

Fault
CheckerCPU::writeMem(uint8_t *data, unsigned size, Addr addr,
                     Request::Flags flags, uint64_t *res,
                     const std::vector<bool> &byte_enable)
{
    assert(byte_enable.size() == size);

    Fault fault = NoFault;
    bool checked_flags = false;
    bool flags_match = true;
    Addr pAddr = 0x0;
    static uint8_t zero_data[64] = {};

    Addr frag_addr = addr;
    int frag_size = 0;
    int size_left = size;
    bool predicate;

    // Need to account for a multiple access like Atomic and Timing CPUs
    while (1) {
        RequestPtr mem_req = genMemFragmentRequest(
            frag_addr, size, flags, byte_enable, frag_size, size_left);

        predicate = (mem_req != nullptr);

        if (predicate) {
            fault = mmu->translateFunctional(mem_req, tc, BaseMMU::Write);
        }

        if (predicate && !checked_flags && fault == NoFault && unverifiedReq) {
            flags_match = checkFlags(unverifiedReq, mem_req->getVaddr(),
                                     mem_req->getPaddr(), mem_req->getFlags());
            pAddr = mem_req->getPaddr();
            checked_flags = true;
        }

        /*
         * We don't actually check memory for the store because there
         * is no guarantee it has left the lsq yet, and therefore we
         * can't verify the memory on stores without lsq snooping
         * enabled.  This is left as future work for the Checker: LSQ snooping
         * and memory validation after stores have committed.
         */
        bool was_prefetch = mem_req->isPrefetch();

        // If we don't need to access a second cache line, stop now.
        if (fault != NoFault || size_left == 0) {
            if (fault != NoFault && was_prefetch) {
                fault = NoFault;
            }
            break;
        }

        frag_addr += frag_size;
    }

    if (!flags_match) {
        warn("%lli: Flags do not match CPU:%#x %#x Checker:%#x %#x %#x\n",
             curTick(), unverifiedReq->getVaddr(), unverifiedReq->getPaddr(),
             unverifiedReq->getFlags(), frag_addr, pAddr, flags);
        handleError();
    }

    // Assume the result was the same as the one passed in.  This checker
    // doesn't check if the SC should succeed or fail, it just checks the
    // value.
    if (unverifiedReq && res && unverifiedReq->extraDataValid())
        *res = unverifiedReq->getExtraData();

    // Entire purpose here is to make sure we are getting the
    // same data to send to the mem system as the CPU did.
    // Cannot check this is actually what went to memory because
    // there stores can be in ld/st queue or coherent operations
    // overwriting values.
    bool extraData = false;
    if (unverifiedReq) {
        extraData = unverifiedReq->extraDataValid() ?
                        unverifiedReq->getExtraData() :
                        true;
    }

    // If the request is to ZERO a cache block, there is no data to check
    // against, but it's all zero. We need something to compare to, so use a
    // const set of zeros.
    if (flags & Request::STORE_NO_DATA) {
        assert(!data);
        assert(sizeof(zero_data) <= size);
        data = zero_data;
    }

    if (unverifiedReq && unverifiedMemData &&
        memcmp(data, unverifiedMemData, size) && extraData) {
        warn("%lli: Store value does not match value sent to memory! "
             "data: %#x inst_data: %#x",
             curTick(), data, unverifiedMemData);
        handleError();
    }

    return fault;
}

/**
 * Checks if the flags set by the Checker and Checkee match.
 */
bool
CheckerCPU::checkFlags(const RequestPtr &unverified_req, Addr vAddr,
                       Addr pAddr, int flags)
{
    Addr unverifiedVAddr = unverified_req->getVaddr();
    Addr unverifiedPAddr = unverified_req->getPaddr();
    int unverifiedFlags = unverified_req->getFlags();

    if (unverifiedVAddr != vAddr || unverifiedPAddr != pAddr ||
        unverifiedFlags != flags) {
        return false;
    }

    return true;
}

void
CheckerCPU::dumpAndExit()
{
    warn("%lli: Checker PC:%s", curTick(), thread->pcState());
    panic("Checker found an error!");
}

} // namespace gem5
