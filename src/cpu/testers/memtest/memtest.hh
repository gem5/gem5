/*
 * Copyright (c) 2015, 2021 Arm Limited
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

#ifndef __CPU_MEMTEST_MEMTEST_HH__
#define __CPU_MEMTEST_MEMTEST_HH__

#include <unordered_map>
#include <unordered_set>

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/MemTest.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "sim/stats.hh"

namespace gem5
{

/**
 * The MemTest class tests a cache coherent memory system by
 * generating false sharing and verifying the read data against a
 * reference updated on the completion of writes. Each tester reads
 * and writes a specific byte in a cache line, as determined by its
 * unique id. Thus, all requests issued by the MemTest instance are a
 * single byte and a specific address is only ever touched by a single
 * tester.
 *
 * In addition to verifying the data, the tester also has timeouts for
 * both requests and responses, thus checking that the memory-system
 * is making progress.
 */
class MemTest : public ClockedObject
{

  public:

    typedef MemTestParams Params;
    MemTest(const Params &p);


    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

  protected:

    void tick();

    EventFunctionWrapper tickEvent;

    void noRequest();

    EventFunctionWrapper noRequestEvent;

    void noResponse();

    EventFunctionWrapper noResponseEvent;

    class CpuPort : public RequestPort
    {
        MemTest &memtest;

      public:

        CpuPort(const std::string &_name, MemTest &_memtest)
            : RequestPort(_name), memtest(_memtest)
        { }

      protected:

        bool recvTimingResp(PacketPtr pkt);

        void recvTimingSnoopReq(PacketPtr pkt) { }

        void recvFunctionalSnoop(PacketPtr pkt) { }

        Tick recvAtomicSnoop(PacketPtr pkt) { return 0; }

        void recvReqRetry();
    };

    CpuPort port;

    PacketPtr retryPkt;

    // Set if reached the maximum number of outstanding requests.
    // Won't tick until a response is received.
    bool waitResponse;

    const unsigned size;

    const Cycles interval;

    const unsigned percentReads;
    const unsigned percentFunctional;
    const unsigned percentUncacheable;
    const unsigned percentAtomic;

    /** Request id for all generated traffic */
    RequestorID requestorId;

    unsigned int id;

    std::unordered_set<Addr> outstandingAddrs;
    std::unordered_map<Addr, uint8_t> atomicPendingData;

    // store the expected value for the addresses we have touched
    std::unordered_map<Addr, uint8_t> referenceData;

    const Addr blockSize;

    const Addr blockAddrMask;

    const unsigned sizeBlocks;

    /**
     * Get the block aligned address.
     *
     * @param addr Address to align
     * @return The block aligned address
     */
    Addr blockAlign(Addr addr) const
    {
        return (addr & ~blockAddrMask);
    }

    const Addr baseAddr1;
    const Addr baseAddr2;
    const Addr uncacheAddr;

    const unsigned progressInterval;  // frequency of progress reports
    const Cycles progressCheck;
    Tick nextProgressMessage;   // access # for next progress report

    uint64_t numReads;
    uint64_t numWrites;
    uint64_t numAtomics;
    const uint64_t maxLoads;

    const bool atomic;

    const bool suppressFuncErrors;
  protected:
    struct MemTestStats : public statistics::Group
    {
        MemTestStats(statistics::Group *parent);
        statistics::Scalar numReads;
        statistics::Scalar numWrites;
        statistics::Scalar numAtomics;
    } stats;

    /**
     * Complete a request by checking the response.
     *
     * @param pkt Response packet
     * @param functional Whether the access was functional or not
     */
    void completeRequest(PacketPtr pkt, bool functional = false);

    bool sendPkt(PacketPtr pkt);

    void recvRetry();

};

} // namespace gem5

#endif // __CPU_MEMTEST_MEMTEST_HH__
