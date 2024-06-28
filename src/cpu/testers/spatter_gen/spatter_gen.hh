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

#ifndef __CPU_TESTERS_SPATTER_GEN_SPATTER_GEN_HH__
#define __CPU_TESTERS_SPATTER_GEN_SPATTER_GEN_HH__

#include <queue>
#include <unordered_map>
#include <vector>

#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "cpu/testers/spatter_gen/utility_structs.hh"
#include "enums/SpatterKernelType.hh"
#include "enums/SpatterProcessingMode.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "params/SpatterGen.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{


/**
 * @class SpatterGen
 * @brief Spatter Kernel Player
 *
 * This class takes Spatter JSON traces and plays them back in gem5.
 * Each trace includes a list of Spatter kernels, which are played in order.
 * Kernels are either of type scatter or gather.
 * At the time of writing, kernels represent accesses to the memory with
 * one level of indirection.
 * Initially, an access is made to an array which we call index from now on.
 * The index array is streamed through with load accesses.
 * In a high level programming language this access will be similar to below.
 * "for (int i = 0; i < n; i++) { idx = index[i]; }".
 * The value at index[i] is then used to access another array which we will
 * call value from now on.
 * For scatter type kernels, a random value is stored in the location and
 * for gather type kernels, the value is read from the location.
 * In a high level programming language this access will be similar to below.
 * Scatter
 * "for (int i = 0; i < n; i++) { idx = index[i]; value[idx] = rand(); }".
 * Gather
 * "for (int i = 0; i < n; i++) { idx = index[i]; val = value[idx]; }".
 * For more information you can take a look at
 * https://github.com/hpcgarage/spatter/blob/main/README.md
 * While the readme mentions MultiScatter and MultiGather kernels, the
 * trace format is not finalized (at the time of writing).
 */
class SpatterGen: public ClockedObject
{
  private:
    typedef enums::SpatterKernelType SpatterKernelType;
    typedef enums::SpatterProcessingMode SpatterProcessingMode;

    class SpatterGenEvent : public EventFunctionWrapper
    {
      private:
        // TODO: split pending into pendingInput and pendingOutput
        enum class SleepState
        {
            AWAKE,
            ASLEEP
        };

        SleepState _state;

      public:
        SpatterGenEvent(const std::function<void(void)> &callback,
                    const std::string &name):
            EventFunctionWrapper(callback, name), _state(SleepState::AWAKE)
        {}
        // a SpatterGenEvent will only be asleep if it is pending output
        bool pending() const { return _state == SleepState::ASLEEP; }
        void sleep() { _state = SleepState::ASLEEP; }
        void wake() { _state = SleepState::AWAKE; }
    };

    class SpatterGenPort: public RequestPort
    {
      private:
        SpatterGen* owner;
        PacketPtr blockedPacket;

      public:
        SpatterGenPort(SpatterGen* owner, const std::string& name):
            RequestPort(name), owner(owner), blockedPacket(nullptr) {}

        void sendPacket(PacketPtr pkt);
        bool blocked() const { return blockedPacket != nullptr; }

      protected:
        virtual bool recvTimingResp(PacketPtr pkt) override;
        virtual void recvReqRetry() override;
    };

    struct SpatterGenStats: public statistics::Group
    {
        SpatterGen* spatterGen;

        // TODO: When we enable multiple levels of indirection, we should
        // convert this to a vector with one stat for each level of index
        statistics::Scalar numIndexReads;
        // TODO: When we enable multiple levels of indirection, we should
        // convert this to a vector with one stat for each level of index
        statistics::Scalar indexBytesRead;
        statistics::Scalar totalIndexReadLatency;

        statistics::Scalar numValueReads;
        statistics::Scalar numValueWrites;
        statistics::Scalar valueBytesRead;
        statistics::Scalar valueBytesWritten;
        statistics::Scalar totalValueReadLatency;
        statistics::Scalar totalValueWriteLatency;

        // TODO: When we enable multiple levels of indirection, we should
        // convert this to a vector with one stat for each level of index
        statistics::Histogram indexAccessLatency;
        statistics::Histogram valueAccessLatency;
        statistics::Histogram totalIndirectAccessLatency;

        virtual void regStats() override;

        SpatterGenStats(SpatterGen* spatter_gen);
    };

    enum class SpatterGenState
    {
        // waiting for all other cores to get to WAITING state, no accesses
        WAITING,
        // only creating intermediate and ultimate accesses, i.e. wrapping up
        DRAINING,
        // creating all kinds of accesses, initial, intermediate, and ultimate
        RUNNING
    };

    // non param related members
    SpatterGenState state;
    std::queue<SpatterKernel> kernels;
    std::unordered_map<RequestPtr, Tick> requestDepartureTime;

    RequestorID requestorId;
    int numPendingMemRequests;

    SpatterGenStats stats;

    void checkForSimExit();

    bool initAccessOk(int int_regs, int fp_regs, Tick when) const;
    bool interAccessOk(int int_regs, int fp_regs, Tick when) const;
    bool ultAccessOk(int int_regs, int fp_regs, Tick when) const;

    // param related members (not necessarily one-to-one with params)
    SpatterProcessingMode mode;
    SpatterGenPort port;
    // size of the register files,
    // for every memory instruction we need to allocate one register.
    int intRegFileSize;
    int intRegUsed;
    int fpRegFileSize;
    int fpRegUsed;
    // laterncy to generate A request
    int requestGenLatency;
    // number of requests generated per event
    int requestGenRate;
    // tracking smallest tick when at least one "AGU" is available;
    Tick firstGeneratorAvailableTime;
    // tracking the busy state of our so called "AGU"s.
    std::vector<Tick> generatorBusyUntil;
    SpatterGenEvent nextGenEvent;
    void processNextGenEvent();
    // put requests to the cache in the request buffer.
    int requestBufferEntries;
    // store request packet along with their insertion time into this queue.
    TimedQueue<PacketPtr> requestBuffer;
    // if nextGenEvent has to be schedule at tick when then schedule it.
    // this function should only be called when nextGenEvent is not pending.
    void scheduleNextGenEvent(Tick when);

    // bandwidth to issue memory requests to cache,
    // this is supposed to model the number of cache ports
    // we will assume it takes 1 cycle to issue memory requests
    int sendRate;
    Tick firstPortAvailableTime;
    std::vector<Tick> portBusyUntil;
    SpatterGenEvent nextSendEvent;
    void processNextSendEvent();
    // if nextSendEvent has to be schedule at tick when then schedule it.
    // this function should only be called when nextSendEvent is not pending.
    void scheduleNextSendEvent(Tick when);

    // put the memory responses here.
    // no need to limit the size of this buffer.
    // it's a response buffer and it will automatically
    // be limited by requestBufferEntries, intRegFileSize, fpRegFileSize
    TimedQueue<SpatterAccess*> receiveBuffer;

  public:
    PARAMS(SpatterGen);
    SpatterGen(const Params& params);

    Port&
    getPort(const std::string& if_name, PortID idx = InvalidPortID) override;

    virtual void startup() override;

    void recvReqRetry();
    bool recvTimingResp(PacketPtr pkt);
    // PyBindMethod to interface adding a kernel with python JSON frontend.
    void addKernel(
        uint32_t id, uint32_t delta, uint32_t count,
        SpatterKernelType type,
        uint32_t base_index, uint32_t indices_per_stride, uint32_t stride,
        size_t index_size, Addr base_index_addr,
        size_t value_size, Addr base_value_addr,
        const std::vector<uint32_t>& indices
    );

    void proceedPastSyncPoint();
};

} // namespace gem5

#endif // __CPU_TESTERS_SPATTER_GEN_SPATTER_GEN_HH__
