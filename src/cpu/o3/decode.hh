/*
 * Copyright (c) 2012 ARM Limited
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
 */

#ifndef __CPU_O3_DECODE_HH__
#define __CPU_O3_DECODE_HH__

#include <queue>

#include "base/statistics.hh"
#include "cpu/timebuf.hh"

struct DerivO3CPUParams;

/**
 * DefaultDecode class handles both single threaded and SMT
 * decode. Its width is specified by the parameters; each cycles it
 * tries to decode that many instructions. Because instructions are
 * actually decoded when the StaticInst is created, this stage does
 * not do much other than check any PC-relative branches.
 */
template<class Impl>
class DefaultDecode
{
  private:
    // Typedefs from the Impl.
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::CPUPol CPUPol;

    // Typedefs from the CPU policy.
    typedef typename CPUPol::FetchStruct FetchStruct;
    typedef typename CPUPol::DecodeStruct DecodeStruct;
    typedef typename CPUPol::TimeStruct TimeStruct;

  public:
    /** Overall decode stage status. Used to determine if the CPU can
     * deschedule itself due to a lack of activity.
     */
    enum DecodeStatus {
        Active,
        Inactive
    };

    /** Individual thread status. */
    enum ThreadStatus {
        Running,
        Idle,
        StartSquash,
        Squashing,
        Blocked,
        Unblocking
    };

  private:
    /** Decode status. */
    DecodeStatus _status;

    /** Per-thread status. */
    ThreadStatus decodeStatus[Impl::MaxThreads];

  public:
    /** DefaultDecode constructor. */
    DefaultDecode(O3CPU *_cpu, DerivO3CPUParams *params);

    void startupStage();

    /** Clear all thread-specific states */
    void clearStates(ThreadID tid);

    void resetStage();

    /** Returns the name of decode. */
    std::string name() const;

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Sets pointer to time buffer used to communicate to the next stage. */
    void setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr);

    /** Sets pointer to time buffer coming from fetch. */
    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Has the stage drained? */
    bool isDrained() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom() { resetStage(); }

    /** Ticks decode, processing all input signals and decoding as many
     * instructions as possible.
     */
    void tick();

    /** Determines what to do based on decode's current status.
     * @param status_change decode() sets this variable if there was a status
     * change (ie switching from from blocking to unblocking).
     * @param tid Thread id to decode instructions from.
     */
    void decode(bool &status_change, ThreadID tid);

    /** Processes instructions from fetch and passes them on to rename.
     * Decoding of instructions actually happens when they are created in
     * fetch, so this function mostly checks if PC-relative branches are
     * correct.
     */
    void decodeInsts(ThreadID tid);

  private:
    /** Inserts a thread's instructions into the skid buffer, to be decoded
     * once decode unblocks.
     */
    void skidInsert(ThreadID tid);

    /** Returns if all of the skid buffers are empty. */
    bool skidsEmpty();

    /** Updates overall decode status based on all of the threads' statuses. */
    void updateStatus();

    /** Separates instructions from fetch into individual lists of instructions
     * sorted by thread.
     */
    void sortInsts();

    /** Reads all stall signals from the backwards communication timebuffer. */
    void readStallSignals(ThreadID tid);

    /** Checks all input signals and updates decode's status appropriately. */
    bool checkSignalsAndUpdate(ThreadID tid);

    /** Checks all stall signals, and returns if any are true. */
    bool checkStall(ThreadID tid) const;

    /** Returns if there any instructions from fetch on this cycle. */
    inline bool fetchInstsValid();

    /** Switches decode to blocking, and signals back that decode has
     * become blocked.
     * @return Returns true if there is a status change.
     */
    bool block(ThreadID tid);

    /** Switches decode to unblocking if the skid buffer is empty, and
     * signals back that decode has unblocked.
     * @return Returns true if there is a status change.
     */
    bool unblock(ThreadID tid);

    /** Squashes if there is a PC-relative branch that was predicted
     * incorrectly. Sends squash information back to fetch.
     */
    void squash(const DynInstPtr &inst, ThreadID tid);

  public:
    /** Squashes due to commit signalling a squash. Changes status to
     * squashing and clears block/unblock signals as needed.
     */
    unsigned squash(ThreadID tid);

  private:
    // Interfaces to objects outside of decode.
    /** CPU interface. */
    O3CPU *cpu;

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get rename's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromRename;

    /** Wire to get iew's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's information from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire to write information heading to previous stages. */
    // Might not be the best name as not only fetch will read it.
    typename TimeBuffer<TimeStruct>::wire toFetch;

    /** Decode instruction queue. */
    TimeBuffer<DecodeStruct> *decodeQueue;

    /** Wire used to write any information heading to rename. */
    typename TimeBuffer<DecodeStruct>::wire toRename;

    /** Fetch instruction queue interface. */
    TimeBuffer<FetchStruct> *fetchQueue;

    /** Wire to get fetch's output from fetch queue. */
    typename TimeBuffer<FetchStruct>::wire fromFetch;

    /** Queue of all instructions coming from fetch this cycle. */
    std::queue<DynInstPtr> insts[Impl::MaxThreads];

    /** Skid buffer between fetch and decode. */
    std::queue<DynInstPtr> skidBuffer[Impl::MaxThreads];

    /** Variable that tracks if decode has written to the time buffer this
     * cycle. Used to tell CPU if there is activity this cycle.
     */
    bool wroteToTimeBuffer;

    /** Source of possible stalls. */
    struct Stalls {
        bool rename;
    };

    /** Tracks which stages are telling decode to stall. */
    Stalls stalls[Impl::MaxThreads];

    /** Rename to decode delay. */
    Cycles renameToDecodeDelay;

    /** IEW to decode delay. */
    Cycles iewToDecodeDelay;

    /** Commit to decode delay. */
    Cycles commitToDecodeDelay;

    /** Fetch to decode delay. */
    Cycles fetchToDecodeDelay;

    /** The width of decode, in instructions. */
    unsigned decodeWidth;

    /** Index of instructions being sent to rename. */
    unsigned toRenameIndex;

    /** number of Active Threads*/
    ThreadID numThreads;

    /** List of active thread ids */
    std::list<ThreadID> *activeThreads;

    /** Maximum size of the skid buffer. */
    unsigned skidBufferMax;

    /** SeqNum of Squashing Branch Delay Instruction (used for MIPS)*/
    Addr bdelayDoneSeqNum[Impl::MaxThreads];

    /** Instruction used for squashing branch (used for MIPS)*/
    DynInstPtr squashInst[Impl::MaxThreads];

    /** Tells when their is a pending delay slot inst. to send
     *  to rename. If there is, then wait squash after the next
     *  instruction (used for MIPS).
     */
    bool squashAfterDelaySlot[Impl::MaxThreads];

    struct DecodeStats : public Stats::Group {
        DecodeStats(O3CPU *cpu);

        /** Stat for total number of idle cycles. */
        Stats::Scalar idleCycles;
        /** Stat for total number of blocked cycles. */
        Stats::Scalar blockedCycles;
        /** Stat for total number of normal running cycles. */
        Stats::Scalar runCycles;
        /** Stat for total number of unblocking cycles. */
        Stats::Scalar unblockCycles;
        /** Stat for total number of squashing cycles. */
        Stats::Scalar squashCycles;
        /** Stat for number of times a branch is resolved at decode. */
        Stats::Scalar branchResolved;
        /** Stat for number of times a branch mispredict is detected. */
        Stats::Scalar branchMispred;
        /** Stat for number of times decode detected a non-control instruction
         * incorrectly predicted as a branch.
         */
        Stats::Scalar controlMispred;
        /** Stat for total number of decoded instructions. */
        Stats::Scalar decodedInsts;
        /** Stat for total number of squashed instructions. */
        Stats::Scalar squashedInsts;
    } stats;
};

#endif // __CPU_O3_DECODE_HH__
