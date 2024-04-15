/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Tester thread issues requests to and receives responses from Ruby memory
 */

#ifndef CPU_TESTERS_PROTOCOL_TESTER_TESTER_THREAD_HH_
#define CPU_TESTERS_PROTOCOL_TESTER_TESTER_THREAD_HH_

#include "cpu/testers/gpu_ruby_test/address_manager.hh"
#include "cpu/testers/gpu_ruby_test/episode.hh"
#include "cpu/testers/gpu_ruby_test/protocol_tester.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "mem/token_port.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class TesterThread : public ClockedObject
{
  public:
    typedef TesterThreadParams Params;
    TesterThread(const Params &p);
    virtual ~TesterThread();

    typedef AddressManager::Location Location;
    typedef AddressManager::Value Value;

    void wakeup();
    void scheduleWakeup();
    void checkDeadlock();
    void scheduleDeadlockCheckEvent();

    void attachTesterThreadToPorts(
        ProtocolTester *_tester, ProtocolTester::SeqPort *_port,
        ProtocolTester::GMTokenPort *_tokenPort = nullptr,
        ProtocolTester::SeqPort *_sqcPort = nullptr,
        ProtocolTester::SeqPort *_scalarPort = nullptr);

    const std::string &
    getName() const
    {
        return threadName;
    }

    // must be implemented by a child class
    virtual void hitCallback(PacketPtr pkt) = 0;

    int
    getTesterThreadId() const
    {
        return threadId;
    }

    int
    getNumLanes() const
    {
        return numLanes;
    }

    // check if the input location would satisfy DRF constraint
    bool checkDRF(Location atomic_loc, Location loc, bool isStore) const;

    void printAllOutstandingReqs(std::stringstream &ss) const;

  protected:
    class TesterThreadEvent : public Event
    {
      private:
        TesterThread *thread;
        std::string desc;

      public:
        TesterThreadEvent(TesterThread *_thread, std::string _description)
            : Event(CPU_Tick_Pri), thread(_thread), desc(_description)
        {}

        void
        setDesc(std::string _description)
        {
            desc = _description;
        }

        void
        process() override
        {
            thread->wakeup();
        }

        const std::string
        name() const override
        {
            return desc;
        }
    };

    TesterThreadEvent threadEvent;

    class DeadlockCheckEvent : public Event
    {
      private:
        TesterThread *thread;

      public:
        DeadlockCheckEvent(TesterThread *_thread)
            : Event(CPU_Tick_Pri), thread(_thread)
        {}

        void
        process() override
        {
            thread->checkDeadlock();
        }

        const std::string
        name() const override
        {
            return "Tester deadlock check";
        }
    };

    DeadlockCheckEvent deadlockCheckEvent;

    struct OutstandingReq
    {
        int lane;
        Location origLoc;
        Value storedValue;
        Cycles issueCycle;

        OutstandingReq(int _lane, Location _loc, Value _val, Cycles _cycle)
            : lane(_lane), origLoc(_loc), storedValue(_val), issueCycle(_cycle)
        {}

        ~OutstandingReq() {}
    };

    // the unique global id of this thread
    int threadId;
    // width of this thread (1 for cpu thread & wf size for gpu wavefront)
    int numLanes;
    // thread name
    std::string threadName;
    // pointer to the main tester
    ProtocolTester *tester;
    // pointer to the address manager
    AddressManager *addrManager;

    ProtocolTester::SeqPort *port; // main data port (GPU-vector data)
    ProtocolTester::GMTokenPort *tokenPort;
    ProtocolTester::SeqPort *scalarPort; // nullptr for CPU
    ProtocolTester::SeqPort *sqcPort;    // nullptr for CPU

    // a list of issued episodes sorted by time
    // the last episode in the list is the current episode
    typedef std::vector<Episode *> EpisodeHistory;
    EpisodeHistory episodeHistory;
    // pointer to the current episode
    Episode *curEpisode;
    // pointer to the current action
    const Episode::Action *curAction;

    // number of outstanding requests that are waiting for their responses
    int pendingLdStCount;
    int pendingFenceCount;
    int pendingAtomicCount;

    // last cycle when there is an event in this thread
    Cycles lastActiveCycle;
    Cycles deadlockThreshold;

    // a per-address list of outstanding requests
    typedef std::vector<OutstandingReq> OutstandingReqList;
    typedef std::unordered_map<Addr, OutstandingReqList> OutstandingReqTable;
    OutstandingReqTable outstandingLoads;
    OutstandingReqTable outstandingStores;
    OutstandingReqTable outstandingAtomics;

    void issueNewEpisode();
    // check if the next action in the current episode satisfies all wait_cnt
    // constraints and is ready to issue
    bool isNextActionReady();
    void issueNextAction();
    int getTokensNeeded();

    // issue Ops to Ruby memory
    // must be implemented by a child class
    virtual void issueLoadOps() = 0;
    virtual void issueStoreOps() = 0;
    virtual void issueAtomicOps() = 0;
    virtual void issueAcquireOp() = 0;
    virtual void issueReleaseOp() = 0;

    // add an outstanding request to its corresponding table
    void addOutstandingReqs(OutstandingReqTable &req_table, Addr addr,
                            int lane, Location loc,
                            Value stored_val = AddressManager::INVALID_VALUE);

    // pop an outstanding request from the input table
    OutstandingReq popOutstandingReq(OutstandingReqTable &req_table,
                                     Addr address);

    // validate all atomic responses
    void validateAtomicResp(Location loc, int lane, Value ret_val);
    // validate all Load responses
    void validateLoadResp(Location loc, int lane, Value ret_val);

    void printOutstandingReqs(const OutstandingReqTable &table,
                              std::stringstream &ss) const;
};

} // namespace gem5

#endif /* CPU_TESTERS_PROTOCOL_TESTER_TESTER_THREAD_HH_ */
