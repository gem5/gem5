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
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_FETCH_HH__
#define __CPU_O3_FETCH_HH__

#include "arch/utility.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/pc_event.hh"
#include "mem/packet.hh"
#include "mem/port.hh"
#include "sim/eventq.hh"

class Sampler;

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
    typedef typename Impl::Params Params;

    /** Typedefs from the CPU policy. */
    typedef typename CPUPol::BPredUnit BPredUnit;
    typedef typename CPUPol::FetchStruct FetchStruct;
    typedef typename CPUPol::TimeStruct TimeStruct;

    /** Typedefs from ISA. */
    typedef TheISA::MachInst MachInst;
    typedef TheISA::ExtMachInst ExtMachInst;

    /** IcachePort class for DefaultFetch.  Handles doing the
     * communication with the cache/memory.
     */
    class IcachePort : public Port
    {
      protected:
        /** Pointer to fetch. */
        DefaultFetch<Impl> *fetch;

      public:
        /** Default constructor. */
        IcachePort(DefaultFetch<Impl> *_fetch)
            : Port(_fetch->name() + "-iport"), fetch(_fetch)
        { }

      protected:
        /** Atomic version of receive.  Panics. */
        virtual Tick recvAtomic(PacketPtr pkt);

        /** Functional version of receive.  Panics. */
        virtual void recvFunctional(PacketPtr pkt);

        /** Receives status change.  Other than range changing, panics. */
        virtual void recvStatusChange(Status status);

        /** Returns the address ranges of this device. */
        virtual void getDeviceAddressRanges(AddrRangeList &resp,
                                            AddrRangeList &snoop)
        { resp.clear(); snoop.clear(); }

        /** Timing version of receive.  Handles setting fetch to the
         * proper status to start fetching. */
        virtual bool recvTiming(PacketPtr pkt);

        /** Handles doing a retry of a failed fetch. */
        virtual void recvRetry();
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
        SwitchOut,
        IcacheWaitResponse,
        IcacheWaitRetry,
        IcacheAccessComplete
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
    std::list<unsigned> priorityList;

  public:
    /** DefaultFetch constructor. */
    DefaultFetch(Params *params);

    /** Returns the name of fetch. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets CPU pointer. */
    void setCPU(O3CPU *cpu_ptr);

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<unsigned> *at_ptr);

    /** Sets pointer to time buffer used to communicate to the next stage. */
    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    /** Initialize stage. */
    void initStage();

    /** Processes cache completion event. */
    void processCacheCompletion(PacketPtr pkt);

    /** Begins the switch out of the fetch stage. */
    void switchOut();

    /** Completes the switch out of the fetch stage. */
    void doSwitchOut();

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Checks if the fetch stage is switched out. */
    bool isSwitchedOut() { return switchedOut; }

    /** Tells fetch to wake up from a quiesce instruction. */
    void wakeFromQuiesce();

  private:
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
     * @return Whether or not a branch was predicted as taken.
     */
    bool lookupAndUpdateNextPC(DynInstPtr &inst, Addr &next_PC);

    /**
     * Fetches the cache line that contains fetch_PC.  Returns any
     * fault that happened.  Puts the data into the class variable
     * cacheData.
     * @param fetch_PC The PC address that is being fetched from.
     * @param ret_fault The fault reference that will be set to the result of
     * the icache access.
     * @param tid Thread id.
     * @return Any fault that occured.
     */
    bool fetchCacheLine(Addr fetch_PC, Fault &ret_fault, unsigned tid);

    /** Squashes a specific thread and resets the PC. */
    inline void doSquash(const Addr &new_PC, unsigned tid);

    /** Squashes a specific thread and resets the PC. Also tells the CPU to
     * remove any instructions between fetch and decode that should be sqaushed.
     */
    void squashFromDecode(const Addr &new_PC, const InstSeqNum &seq_num,
                          unsigned tid);

    /** Checks if a thread is stalled. */
    bool checkStall(unsigned tid) const;

    /** Updates overall fetch stage status; to be called at the end of each
     * cycle. */
    FetchStatus updateFetchStatus();

  public:
    /** Squashes a specific thread and resets the PC. Also tells the CPU to
     * remove any instructions that are not in the ROB. The source of this
     * squash should be the commit stage.
     */
    void squash(const Addr &new_PC, unsigned tid);

    /** Ticks the fetch stage, processing all inputs signals and fetching
     * as many instructions as possible.
     */
    void tick();

    /** Checks all input signals and updates the status as necessary.
     *  @return: Returns if the status has changed due to input signals.
     */
    bool checkSignalsAndUpdate(unsigned tid);

    /** Does the actual fetching of instructions and passing them on to the
     * next stage.
     * @param status_change fetch() sets this variable if there was a status
     * change (ie switching to IcacheMissStall).
     */
    void fetch(bool &status_change);

    /** Align a PC to the start of an I-cache block. */
    Addr icacheBlockAlignPC(Addr addr)
    {
        addr = TheISA::realPCToFetchPC(addr);
        return (addr & ~(cacheBlkMask));
    }

  private:
    /** Handles retrying the fetch access. */
    void recvRetry();

    /** Returns the appropriate thread to fetch, given the fetch policy. */
    int getFetchingThread(FetchPriority &fetch_priority);

    /** Returns the appropriate thread to fetch using a round robin policy. */
    int roundRobin();

    /** Returns the appropriate thread to fetch using the IQ count policy. */
    int iqCount();

    /** Returns the appropriate thread to fetch using the LSQ count policy. */
    int lsqCount();

    /** Returns the appropriate thread to fetch using the branch count policy. */
    int branchCount();

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

    /** Internal fetch instruction queue. */
    TimeBuffer<FetchStruct> *fetchQueue;

    //Might be annoying how this name is different than the queue.
    /** Wire used to write any information heading to decode. */
    typename TimeBuffer<FetchStruct>::wire toDecode;

    MemObject *mem;

    /** Icache interface. */
    IcachePort *icachePort;

    /** BPredUnit. */
    BPredUnit branchPred;

    /** Per-thread fetch PC. */
    Addr PC[Impl::MaxThreads];

    /** Per-thread next PC. */
    Addr nextPC[Impl::MaxThreads];

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
        bool rename;
        bool iew;
        bool commit;
    };

    /** Tracks which stages are telling fetch to stall. */
    Stalls stalls[Impl::MaxThreads];

    /** Decode to fetch delay, in ticks. */
    unsigned decodeToFetchDelay;

    /** Rename to fetch delay, in ticks. */
    unsigned renameToFetchDelay;

    /** IEW to fetch delay, in ticks. */
    unsigned iewToFetchDelay;

    /** Commit to fetch delay, in ticks. */
    unsigned commitToFetchDelay;

    /** The width of fetch in instructions. */
    unsigned fetchWidth;

    /** Is the cache blocked?  If so no threads can access it. */
    bool cacheBlocked;

    /** The packet that is waiting to be retried. */
    PacketPtr retryPkt;

    /** The thread that is waiting on the cache to tell fetch to retry. */
    int retryTid;

    /** Cache block size. */
    int cacheBlkSize;

    /** Mask to get a cache block's address. */
    Addr cacheBlkMask;

    /** The cache line being fetched. */
    uint8_t *cacheData[Impl::MaxThreads];

    /** Size of instructions. */
    int instSize;

    /** Icache stall statistics. */
    Counter lastIcacheStall[Impl::MaxThreads];

    /** List of Active Threads */
    std::list<unsigned> *activeThreads;

    /** Number of threads. */
    unsigned numThreads;

    /** Number of threads that are actively fetching. */
    unsigned numFetchingThreads;

    /** Thread ID being fetched. */
    int threadFetched;

    /** Checks if there is an interrupt pending.  If there is, fetch
     * must stop once it is not fetching PAL instructions.
     */
    bool interruptPending;

    /** Records if fetch is switched out. */
    bool switchedOut;

    // @todo: Consider making these vectors and tracking on a per thread basis.
    /** Stat for total number of cycles stalled due to an icache miss. */
    Stats::Scalar<> icacheStallCycles;
    /** Stat for total number of fetched instructions. */
    Stats::Scalar<> fetchedInsts;
    /** Total number of fetched branches. */
    Stats::Scalar<> fetchedBranches;
    /** Stat for total number of predicted branches. */
    Stats::Scalar<> predictedBranches;
    /** Stat for total number of cycles spent fetching. */
    Stats::Scalar<> fetchCycles;
    /** Stat for total number of cycles spent squashing. */
    Stats::Scalar<> fetchSquashCycles;
    /** Stat for total number of cycles spent blocked due to other stages in
     * the pipeline.
     */
    Stats::Scalar<> fetchIdleCycles;
    /** Total number of cycles spent blocked. */
    Stats::Scalar<> fetchBlockedCycles;
    /** Total number of cycles spent in any other state. */
    Stats::Scalar<> fetchMiscStallCycles;
    /** Stat for total number of fetched cache lines. */
    Stats::Scalar<> fetchedCacheLines;
    /** Total number of outstanding icache accesses that were dropped
     * due to a squash.
     */
    Stats::Scalar<> fetchIcacheSquashes;
    /** Distribution of number of instructions fetched each cycle. */
    Stats::Distribution<> fetchNisnDist;
    /** Rate of how often fetch was idle. */
    Stats::Formula idleRate;
    /** Number of branch fetches per cycle. */
    Stats::Formula branchRate;
    /** Number of instruction fetched per cycle. */
    Stats::Formula fetchRate;
};

#endif //__CPU_O3_FETCH_HH__
