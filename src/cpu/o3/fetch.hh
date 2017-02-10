/*
 * Copyright (c) 2010-2012, 2014 ARM Limited
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
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#ifndef __CPU_O3_FETCH_HH__
#define __CPU_O3_FETCH_HH__

#include "arch/decoder.hh"
#include "arch/utility.hh"
#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/pc_event.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/timebuf.hh"
#include "cpu/translation.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"
#include "sim/probe/probe.hh"

struct DerivO3CPUParams;

/**
 * DefaultFetch class handles both single threaded and SMT fetch. Its
 * width is specified by the parameters; each cycle it tries to fetch
 * that many instructions. It supports using a branch predictor to
 * predict direction and targets.
 * It supports the idling functionality of the CPU by indicating to
 * the CPU when it is active and inactive.
 */
template <class Impl>
class DefaultFetch
{
  public:
    /** Typedefs from Impl. */
    typedef typename Impl::CPUPol CPUPol;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::O3CPU O3CPU;

    /** Typedefs from the CPU policy. */
    typedef typename CPUPol::FetchStruct FetchStruct;
    typedef typename CPUPol::TimeStruct TimeStruct;

    /** Typedefs from ISA. */
    typedef TheISA::MachInst MachInst;

    class FetchTranslation : public BaseTLB::Translation
    {
      protected:
        DefaultFetch<Impl> *fetch;

      public:
        FetchTranslation(DefaultFetch<Impl> *_fetch)
            : fetch(_fetch)
        {}

        void
        markDelayed()
        {}

        void
        finish(const Fault &fault, const RequestPtr &req, ThreadContext *tc,
               BaseTLB::Mode mode)
        {
            assert(mode == BaseTLB::Execute);
            fetch->finishTranslation(fault, req);
            delete this;
        }
    };

  private:
    /* Event to delay delivery of a fetch translation result in case of
     * a fault and the nop to carry the fault cannot be generated
     * immediately */
    class FinishTranslationEvent : public Event
    {
      private:
        DefaultFetch<Impl> *fetch;
        Fault fault;
        RequestPtr req;

      public:
        FinishTranslationEvent(DefaultFetch<Impl> *_fetch)
            : fetch(_fetch), req(nullptr)
        {}

        void setFault(Fault _fault)
        {
            fault = _fault;
        }

        void setReq(const RequestPtr &_req)
        {
            req = _req;
        }

        /** Process the delayed finish translation */
        void process()
        {
            assert(fetch->numInst < fetch->fetchWidth);
            fetch->finishTranslation(fault, req);
        }

        const char *description() const
        {
            return "FullO3CPU FetchFinishTranslation";
        }
      };

  public:
    /** Overall fetch status. Used to determine if the CPU can
     * deschedule itsef due to a lack of activity.
     */
    enum FetchStatus {
        Active,
        Inactive
    };

    /** Individual thread status. */
    enum ThreadStatus {
        Running,
        Idle,
        Squashing,
        Blocked,
        Fetching,
        TrapPending,
        QuiescePending,
        ItlbWait,
        IcacheWaitResponse,
        IcacheWaitRetry,
        IcacheAccessComplete,
        NoGoodAddr
    };

    /** Fetching Policy, Add new policies here.*/
    enum FetchPriority {
        SingleThread,
        RoundRobin,
        Branch,
        IQ,
        LSQ
    };

  private:
    /** Fetch status. */
    FetchStatus _status;

    /** Per-thread status. */
    ThreadStatus fetchStatus[Impl::MaxThreads];

    /** Fetch policy. */
    FetchPriority fetchPolicy;

    /** List that has the threads organized by priority. */
    std::list<ThreadID> priorityList;

    /** Probe points. */
    ProbePointArg<DynInstPtr> *ppFetch;
    /** To probe when a fetch request is successfully sent. */
    ProbePointArg<RequestPtr> *ppFetchRequestSent;

  public:
    /** DefaultFetch constructor. */
    DefaultFetch(O3CPU *_cpu, DerivO3CPUParams *params);

    /** Returns the name of fetch. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Registers probes. */
    void regProbePoints();

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Sets pointer to time buffer used to communicate to the next stage. */
    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    /** Initialize stage. */
    void startupStage();

    /** Handles retrying the fetch access. */
    void recvReqRetry();

    /** Processes cache completion event. */
    void processCacheCompletion(PacketPtr pkt);

    /** Resume after a drain. */
    void drainResume();

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Has the stage drained? */
    bool isDrained() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /**
     * Stall the fetch stage after reaching a safe drain point.
     *
     * The CPU uses this method to stop fetching instructions from a
     * thread that has been drained. The drain stall is different from
     * all other stalls in that it is signaled instantly from the
     * commit stage (without the normal communication delay) when it
     * has reached a safe point to drain from.
     */
    void drainStall(ThreadID tid);

    /** Tells fetch to wake up from a quiesce instruction. */
    void wakeFromQuiesce();

    /** For priority-based fetch policies, need to keep update priorityList */
    void deactivateThread(ThreadID tid);
  private:
    /** Reset this pipeline stage */
    void resetStage();

    /** Changes the status of this stage to active, and indicates this
     * to the CPU.
     */
    inline void switchToActive();

    /** Changes the status of this stage to inactive, and indicates
     * this to the CPU.
     */
    inline void switchToInactive();

    /**
     * Looks up in the branch predictor to see if the next PC should be
     * either next PC+=MachInst or a branch target.
     * @param next_PC Next PC variable passed in by reference.  It is
     * expected to be set to the current PC; it will be updated with what
     * the next PC will be.
     * @param next_NPC Used for ISAs which use delay slots.
     * @return Whether or not a branch was predicted as taken.
     */
    bool lookupAndUpdateNextPC(const DynInstPtr &inst, TheISA::PCState &pc);

    /**
     * Fetches the cache line that contains the fetch PC.  Returns any
     * fault that happened.  Puts the data into the class variable
     * fetchBuffer, which may not hold the entire fetched cache line.
     * @param vaddr The memory address that is being fetched from.
     * @param ret_fault The fault reference that will be set to the result of
     * the icache access.
     * @param tid Thread id.
     * @param pc The actual PC of the current instruction.
     * @return Any fault that occured.
     */
    bool fetchCacheLine(Addr vaddr, ThreadID tid, Addr pc);
    void finishTranslation(const Fault &fault, const RequestPtr &mem_req);


    /** Check if an interrupt is pending and that we need to handle
     */
    bool
    checkInterrupt(Addr pc)
    {
        return (interruptPending && (THE_ISA != ALPHA_ISA || !(pc & 0x3)));
    }

    /** Squashes a specific thread and resets the PC. */
    inline void doSquash(const TheISA::PCState &newPC,
                         const DynInstPtr squashInst, ThreadID tid);

    /** Squashes a specific thread and resets the PC. Also tells the CPU to
     * remove any instructions between fetch and decode that should be sqaushed.
     */
    void squashFromDecode(const TheISA::PCState &newPC,
                          const DynInstPtr squashInst,
                          const InstSeqNum seq_num, ThreadID tid);

    /** Checks if a thread is stalled. */
    bool checkStall(ThreadID tid) const;

    /** Updates overall fetch stage status; to be called at the end of each
     * cycle. */
    FetchStatus updateFetchStatus();

  public:
    /** Squashes a specific thread and resets the PC. Also tells the CPU to
     * remove any instructions that are not in the ROB. The source of this
     * squash should be the commit stage.
     */
    void squash(const TheISA::PCState &newPC, const InstSeqNum seq_num,
                DynInstPtr squashInst, ThreadID tid);

    /** Ticks the fetch stage, processing all inputs signals and fetching
     * as many instructions as possible.
     */
    void tick();

    /** Checks all input signals and updates the status as necessary.
     *  @return: Returns if the status has changed due to input signals.
     */
    bool checkSignalsAndUpdate(ThreadID tid);

    /** Does the actual fetching of instructions and passing them on to the
     * next stage.
     * @param status_change fetch() sets this variable if there was a status
     * change (ie switching to IcacheMissStall).
     */
    void fetch(bool &status_change);

    /** Align a PC to the start of a fetch buffer block. */
    Addr fetchBufferAlignPC(Addr addr)
    {
        return (addr & ~(fetchBufferMask));
    }

    /** The decoder. */
    TheISA::Decoder *decoder[Impl::MaxThreads];

  private:
    DynInstPtr buildInst(ThreadID tid, StaticInstPtr staticInst,
                         StaticInstPtr curMacroop, TheISA::PCState thisPC,
                         TheISA::PCState nextPC, bool trace);

    /** Returns the appropriate thread to fetch, given the fetch policy. */
    ThreadID getFetchingThread(FetchPriority &fetch_priority);

    /** Returns the appropriate thread to fetch using a round robin policy. */
    ThreadID roundRobin();

    /** Returns the appropriate thread to fetch using the IQ count policy. */
    ThreadID iqCount();

    /** Returns the appropriate thread to fetch using the LSQ count policy. */
    ThreadID lsqCount();

    /** Returns the appropriate thread to fetch using the branch count
     * policy. */
    ThreadID branchCount();

    /** Pipeline the next I-cache access to the current one. */
    void pipelineIcacheAccesses(ThreadID tid);

    /** Profile the reasons of fetch stall. */
    void profileStall(ThreadID tid);

  private:
    /** Pointer to the O3CPU. */
    O3CPU *cpu;

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get decode's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromDecode;

    /** Wire to get rename's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromRename;

    /** Wire to get iew's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    //Might be annoying how this name is different than the queue.
    /** Wire used to write any information heading to decode. */
    typename TimeBuffer<FetchStruct>::wire toDecode;

    /** BPredUnit. */
    BPredUnit *branchPred;

    TheISA::PCState pc[Impl::MaxThreads];

    Addr fetchOffset[Impl::MaxThreads];

    StaticInstPtr macroop[Impl::MaxThreads];

    /** Can the fetch stage redirect from an interrupt on this instruction? */
    bool delayedCommit[Impl::MaxThreads];

    /** Memory request used to access cache. */
    RequestPtr memReq[Impl::MaxThreads];

    /** Variable that tracks if fetch has written to the time buffer this
     * cycle. Used to tell CPU if there is activity this cycle.
     */
    bool wroteToTimeBuffer;

    /** Tracks how many instructions has been fetched this cycle. */
    int numInst;

    /** Source of possible stalls. */
    struct Stalls {
        bool decode;
        bool drain;
    };

    /** Tracks which stages are telling fetch to stall. */
    Stalls stalls[Impl::MaxThreads];

    /** Decode to fetch delay. */
    Cycles decodeToFetchDelay;

    /** Rename to fetch delay. */
    Cycles renameToFetchDelay;

    /** IEW to fetch delay. */
    Cycles iewToFetchDelay;

    /** Commit to fetch delay. */
    Cycles commitToFetchDelay;

    /** The width of fetch in instructions. */
    unsigned fetchWidth;

    /** The width of decode in instructions. */
    unsigned decodeWidth;

    /** Is the cache blocked?  If so no threads can access it. */
    bool cacheBlocked;

    /** The packet that is waiting to be retried. */
    PacketPtr retryPkt;

    /** The thread that is waiting on the cache to tell fetch to retry. */
    ThreadID retryTid;

    /** Cache block size. */
    unsigned int cacheBlkSize;

    /** The size of the fetch buffer in bytes. The fetch buffer
     *  itself may be smaller than a cache line.
     */
    unsigned fetchBufferSize;

    /** Mask to align a fetch address to a fetch buffer boundary. */
    Addr fetchBufferMask;

    /** The fetch data that is being fetched and buffered. */
    uint8_t *fetchBuffer[Impl::MaxThreads];

    /** The PC of the first instruction loaded into the fetch buffer. */
    Addr fetchBufferPC[Impl::MaxThreads];

    /** The size of the fetch queue in micro-ops */
    unsigned fetchQueueSize;

    /** Queue of fetched instructions. Per-thread to prevent HoL blocking. */
    std::deque<DynInstPtr> fetchQueue[Impl::MaxThreads];

    /** Whether or not the fetch buffer data is valid. */
    bool fetchBufferValid[Impl::MaxThreads];

    /** Size of instructions. */
    int instSize;

    /** Icache stall statistics. */
    Counter lastIcacheStall[Impl::MaxThreads];

    /** List of Active Threads */
    std::list<ThreadID> *activeThreads;

    /** Number of threads. */
    ThreadID numThreads;

    /** Number of threads that are actively fetching. */
    ThreadID numFetchingThreads;

    /** Thread ID being fetched. */
    ThreadID threadFetched;

    /** Checks if there is an interrupt pending.  If there is, fetch
     * must stop once it is not fetching PAL instructions.
     */
    bool interruptPending;

    /** Set to true if a pipelined I-cache request should be issued. */
    bool issuePipelinedIfetch[Impl::MaxThreads];

    /** Event used to delay fault generation of translation faults */
    FinishTranslationEvent finishTranslationEvent;

    // @todo: Consider making these vectors and tracking on a per thread basis.
    /** Stat for total number of cycles stalled due to an icache miss. */
    Stats::Scalar icacheStallCycles;
    /** Stat for total number of fetched instructions. */
    Stats::Scalar fetchedInsts;
    /** Total number of fetched branches. */
    Stats::Scalar fetchedBranches;
    /** Stat for total number of predicted branches. */
    Stats::Scalar predictedBranches;
    /** Stat for total number of cycles spent fetching. */
    Stats::Scalar fetchCycles;
    /** Stat for total number of cycles spent squashing. */
    Stats::Scalar fetchSquashCycles;
    /** Stat for total number of cycles spent waiting for translation */
    Stats::Scalar fetchTlbCycles;
    /** Stat for total number of cycles spent blocked due to other stages in
     * the pipeline.
     */
    Stats::Scalar fetchIdleCycles;
    /** Total number of cycles spent blocked. */
    Stats::Scalar fetchBlockedCycles;
    /** Total number of cycles spent in any other state. */
    Stats::Scalar fetchMiscStallCycles;
    /** Total number of cycles spent in waiting for drains. */
    Stats::Scalar fetchPendingDrainCycles;
    /** Total number of stall cycles caused by no active threads to run. */
    Stats::Scalar fetchNoActiveThreadStallCycles;
    /** Total number of stall cycles caused by pending traps. */
    Stats::Scalar fetchPendingTrapStallCycles;
    /** Total number of stall cycles caused by pending quiesce instructions. */
    Stats::Scalar fetchPendingQuiesceStallCycles;
    /** Total number of stall cycles caused by I-cache wait retrys. */
    Stats::Scalar fetchIcacheWaitRetryStallCycles;
    /** Stat for total number of fetched cache lines. */
    Stats::Scalar fetchedCacheLines;
    /** Total number of outstanding icache accesses that were dropped
     * due to a squash.
     */
    Stats::Scalar fetchIcacheSquashes;
    /** Total number of outstanding tlb accesses that were dropped
     * due to a squash.
     */
    Stats::Scalar fetchTlbSquashes;
    /** Distribution of number of instructions fetched each cycle. */
    Stats::Distribution fetchNisnDist;
    /** Rate of how often fetch was idle. */
    Stats::Formula idleRate;
    /** Number of branch fetches per cycle. */
    Stats::Formula branchRate;
    /** Number of instruction fetched per cycle. */
    Stats::Formula fetchRate;
};

#endif //__CPU_O3_FETCH_HH__
