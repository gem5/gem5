/*
* Copyright (c) 2024 The Regents of The University of California
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

#ifndef __CPU_TESTERS_SPATTER_GEN_UTILITY_STRUCTS_HH__
#define __CPU_TESTERS_SPATTER_GEN_UTILITY_STRUCTS_HH__

#include <deque>
#include <queue>

#include "base/random.hh"
#include "base/types.hh"
#include "enums/SpatterKernelType.hh"
#include "mem/packet.hh"

namespace gem5
{

template<typename T>
class TimedQueue
{
  private:
    Tick latency;

    std::queue<T> items;
    std::queue<Tick> insertionTimes;

  public:
    TimedQueue(Tick latency): latency(latency) {}

    void push(T item, Tick insertion_time)
    {
        items.push(item);
        insertionTimes.push(insertion_time);
    }

    void pop()
    {
        items.pop();
        insertionTimes.pop();
    }

    T front() const { return items.front(); }

    bool empty() const { return items.empty(); }

    size_t size() const { return items.size(); }

    bool hasReady(Tick current_time) const
    {
        if (empty()) {
            return false;
        }
        return (current_time - insertionTimes.front()) >= latency;
    }
};



// Represents a single access to a SpatterKernel.
// It supports multiple levels of indirection.
// However, the SpatterKernel class only works with one level of
// indirection (i.e. accessing value[index[i]]).
struct SpatterAccess : public Packet::SenderState
{
    typedef std::tuple<Addr, size_t> AccessPair;
    typedef enums::SpatterKernelType SpatterKernelType;

    RequestorID requestorId;
    SpatterKernelType _kernelType;
    Tick accTripTime;
    std::queue<AccessPair> accessPairs;

    SpatterAccess(
        RequestorID requestor_id,
        SpatterKernelType kernel_type,
        const std::queue<AccessPair>& access_pairs
    ):
        requestorId(requestor_id), _kernelType(kernel_type),
        accTripTime(0), accessPairs(access_pairs)
    {}

    SpatterKernelType type() const { return _kernelType; }

    int tripsLeft() const { return accessPairs.size(); }

    void recordTripTime(Tick trip_time) { accTripTime += trip_time; }

    Tick tripTimeSoFar() const { return accTripTime; }

    AccessPair nextAccessPair()
    {
        assert(tripsLeft() > 0);
        AccessPair access_pair = accessPairs.front();
        accessPairs.pop();
        return access_pair;
    }

    PacketPtr nextPacket()
    {
        Addr addr;
        size_t size;
        std::tie(addr, size) = nextAccessPair();
        MemCmd cmd;
        if (tripsLeft() >= 1){
            cmd = MemCmd::ReadReq;
        } else {
            cmd = _kernelType == \
                SpatterKernelType::gather ? MemCmd::ReadReq : MemCmd::WriteReq;
        }
        return createPacket(addr, size, cmd);
    }

    PacketPtr createPacket(Addr addr, size_t size, MemCmd cmd) const
    {
        RequestPtr req = std::make_shared<Request>(addr, size, 0, requestorId);

        // Dummy PC to have PC-based prefetchers latch on;
        // get entropy into higher bits
        // This piece of code is directly copied from
        // gem5::TrafficGen::
        req->setPC(((Addr) requestorId) << 2);
        PacketPtr pkt = new Packet(req, cmd);
        uint8_t* pkt_data = new uint8_t[req->getSize()];
        // Randomly intialize pkt_data, for testing cache coherence.
        for (int i = 0; i < req->getSize(); i++) {
            pkt_data[i] = random_mt.random<uint8_t>();
        }
        pkt->dataDynamic(pkt_data);
        return pkt;
    }
};

class SpatterKernel
{
  private:
    typedef enums::SpatterKernelType SpatterKernelType;
    typedef SpatterAccess::AccessPair AccessPair;

    RequestorID requestorId;
    uint32_t _id;
    uint32_t delta;
    uint32_t count;

    SpatterKernelType _type;

    size_t indexSize;
    Addr baseIndexAddr;

    size_t valueSize;
    Addr baseValueAddr;

    // needed to iterate over indices multiple times.
    uint32_t index;
    // current iteration over indices
    uint32_t iteration;

    // number of times we have left to roll indices to finish one iteration.
    uint32_t remRolls;
    std::deque<uint32_t> indices;

  public:

    SpatterKernel(
        RequestorID requestor_id,
        uint32_t id, uint32_t delta, uint32_t count,
        SpatterKernelType type,
        size_t index_size, Addr base_index_addr,
        size_t value_size, Addr base_value_addr
    ):
        requestorId(requestor_id),
        _id(id), delta(delta), count(count),
        _type(type),
        indexSize(index_size), baseIndexAddr(base_index_addr),
        valueSize(value_size), baseValueAddr(base_value_addr),
        index(0), iteration(0), remRolls(0)
    {}

    uint32_t id() const { return _id; }

    void setIndices(const std::vector<uint32_t>& pattern)
    {
        indices.assign(pattern.begin(), pattern.end());
        remRolls = indices.size();
    }

    SpatterKernelType type() const { return _type; }

    bool done() const { return iteration == count; }

    SpatterAccess* nextSpatterAccess()
    {
        std::queue<AccessPair> access_pairs;
        Addr index_addr = baseIndexAddr + (index * indexSize);
        access_pairs.emplace(index_addr, indexSize);
        // update index in the index array
        index++;

        uint32_t front = indices.front();
        uint32_t value_index = (delta * iteration) + front;
        Addr value_addr = baseValueAddr + (value_index * valueSize);
        access_pairs.emplace(value_addr, valueSize);
        // roll indices
        indices.pop_front();
        indices.push_back(front);
        remRolls--;
        if (remRolls == 0) {
            remRolls = indices.size();
            iteration++;
        }

        return new SpatterAccess(requestorId, _type, access_pairs);
    }
};

} // namespace gem5

#endif // __CPU_TESTERS_SPATTER_GEN_UTILITY_STRUCTS_HH__
