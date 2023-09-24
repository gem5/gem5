/*
 * Copyright (c) 2010-2012, 2014, 2019 ARM Limited
 * All rights reserved.
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

#ifndef __CPU_O3_COMMIT_HH__
#define __CPU_O3_COMMIT_HH__

#include <queue>

#include "base/statistics.hh"
#include "cpu/exetrace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/limits.hh"
#include "cpu/o3/rename_map.hh"
#include "cpu/o3/rob.hh"
#include "cpu/timebuf.hh"
#include "enums/CommitPolicy.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class ThreadState;

/**
 * Commit handles single threaded and SMT commit. Its width is
 * specified by the parameters; each cycle it tries to commit that
 * many instructions. The SMT policy decides which thread it tries to
 * commit instructions from. Non- speculative instructions must reach
 * the head of the ROB before they are ready to execute; once they
 * reach the head, commit will broadcast the instruction's sequence
 * number to the previous stages so that they can issue/ execute the
 * instruction. Only one non-speculative instruction is handled per
 * cycle. Commit is responsible for handling all back-end initiated
 * redirects.  It receives the redirect, and then broadcasts it to all
 * stages, indicating the sequence number they should squash until,
 * and any necessary branch misprediction information as well. It
 * priortizes redirects by instruction's age, only broadcasting a
 * redirect if it corresponds to an instruction that should currently
 * be in the ROB. This is done by tracking the sequence number of the
 * youngest instruction in the ROB, which gets updated to any
 * squashing instruction's sequence number, and only broadcasting a
 * redirect if it corresponds to an older instruction. Commit also
 * supports multiple cycle squashing, to model a ROB that can only
 * remove a certain number of instructions per cycle.
 */
class Commit
{
  public:
    /** Overall commit status. Used to determine if the CPU can deschedule
     * itself due to a lack of activity.
     */
    enum CommitStatus
    {
        Active,
        Inactive
    };

    /** Individual thread status. */
    enum ThreadStatus
    {
        Running,
        Idle,
        ROBSquashing,
        TrapPending,
        FetchTrapPending,
        SquashAfterPending, //< Committing instructions before a squash.
    };

  private:
    /** Overall commit status. */
    CommitStatus _status;
    /** Next commit status, to be set at the end of the cycle. */
    CommitStatus _nextStatus;
    /** Per-thread status. */
    ThreadStatus commitStatus[MaxThreads];
    /** Commit policy used in SMT mode. */
    CommitPolicy commitPolicy;

    /** Probe Points. */
    ProbePointArg<DynInstPtr> *ppCommit;
    ProbePointArg<DynInstPtr> *ppCommitStall;
    /** To probe when an instruction is squashed */
    ProbePointArg<DynInstPtr> *ppSquash;

    /** Mark the thread as processing a trap. */
    void processTrapEvent(ThreadID tid);

  public:
    /** Construct a Commit with the given parameters. */
    Commit(CPU *_cpu, const BaseO3CPUParams &params);

    /** Returns the name of the Commit. */
    std::string name() const;

    /** Registers probes. */
    void regProbePoints();

    /** Sets the list of threads. */
    void setThreads(std::vector<ThreadState *> &threads);

    /** Sets the main time buffer pointer, used for backwards communication. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr);

    /** Sets the pointer to the queue coming from rename. */
    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    /** Sets the pointer to the queue coming from IEW. */
    void setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr);

    /** Sets the pointer to the IEW stage. */
    void setIEWStage(IEW *iew_stage);

    /** The pointer to the IEW stage. Used solely to ensure that
     * various events (traps, interrupts, syscalls) do not occur until
     * all stores have written back.
     */
    IEW *iewStage;

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Sets pointer to the commited state rename map. */
    void setRenameMap(UnifiedRenameMap rm_ptr[MaxThreads]);

    /** Sets pointer to the ROB. */
    void setROB(ROB *rob_ptr);

    /** Initializes stage by sending back the number of free entries. */
    void startupStage();

    /** Clear all thread-specific states */
    void clearStates(ThreadID tid);

    /** Initializes the draining of commit. */
    void drain();

    /** Resumes execution after draining. */
    void drainResume();

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Has the stage drained? */
    bool isDrained() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Deschedules a thread from scheduling */
    void deactivateThread(ThreadID tid);

    /** Is the CPU currently processing a HTM transaction? */
    bool executingHtmTransaction(ThreadID) const;

    /* Reset HTM tracking, e.g. after an abort */
    void resetHtmStartsStops(ThreadID);

    /** Ticks the commit stage, which tries to commit instructions. */
    void tick();

    /** Handles any squashes that are sent from IEW, and adds instructions
     * to the ROB and tries to commit instructions.
     */
    void commit();

    /** Returns the number of free ROB entries for a specific thread. */
    size_t numROBFreeEntries(ThreadID tid);

    /** Generates an event to schedule a squash due to a trap. */
    void generateTrapEvent(ThreadID tid, Fault inst_fault);

    /** Records that commit needs to initiate a squash due to an
     * external state update through the TC.
     */
    void generateTCEvent(ThreadID tid);

  private:
    /** Updates the overall status of commit with the nextStatus, and
     * tell the CPU if commit is active/inactive.
     */
    void updateStatus();

    /** Returns if any of the threads have the number of ROB entries changed
     * on this cycle. Used to determine if the number of free ROB entries needs
     * to be sent back to previous stages.
     */
    bool changedROBEntries();

    /** Squashes all in flight instructions. */
    void squashAll(ThreadID tid);

    /** Handles squashing due to a trap. */
    void squashFromTrap(ThreadID tid);

    /** Handles squashing due to an TC write. */
    void squashFromTC(ThreadID tid);

    /** Handles a squash from a squashAfter() request. */
    void squashFromSquashAfter(ThreadID tid);

    /**
     * Handle squashing from instruction with SquashAfter set.
     *
     * This differs from the other squashes as it squashes following
     * instructions instead of the current instruction and doesn't
     * clean up various status bits about traps/tc writes
     * pending. Since there might have been instructions committed by
     * the commit stage before the squashing instruction was reached
     * and we can't commit and squash in the same cycle, we have to
     * squash in two steps:
     *
     * <ol>
     *   <li>Immediately set the commit status of the thread of
     *       SquashAfterPending. This forces the thread to stop
     *       committing instructions in this cycle. The last
     *       instruction to be committed in this cycle will be the
     *       SquashAfter instruction.
     *   <li>In the next cycle, commit() checks for the
     *       SquashAfterPending state and squashes <i>all</i>
     *       in-flight instructions. Since the SquashAfter instruction
     *       was the last instruction to be committed in the previous
     *       cycle, this causes all subsequent instructions to be
     *       squashed.
     * </ol>
     *
     * @param tid ID of the thread to squash.
     * @param head_inst Instruction that requested the squash.
     */
    void squashAfter(ThreadID tid, const DynInstPtr &head_inst);

    /** Handles processing an interrupt. */
    void handleInterrupt();

    /** Get fetch redirecting so we can handle an interrupt */
    void propagateInterrupt();

    /** Commits as many instructions as possible. */
    void commitInsts();

    /** Tries to commit the head ROB instruction passed in.
     * @param head_inst The instruction to be committed.
     */
    bool commitHead(const DynInstPtr &head_inst, unsigned inst_num);

    /** Gets instructions from rename and inserts them into the ROB. */
    void getInsts();

    /** Marks completed instructions using information sent from IEW. */
    void markCompletedInsts();

    /** Gets the thread to commit, based on the SMT policy. */
    ThreadID getCommittingThread();

    /** Returns the thread ID to use based on a round robin policy. */
    ThreadID roundRobin();

    /** Returns the thread ID to use based on an oldest instruction policy. */
    ThreadID oldestReady();

  public:
    /** Reads the PC of a specific thread. */
    const PCStateBase &pcState(ThreadID tid) { return *pc[tid]; }

    /** Sets the PC of a specific thread. */
    void pcState(const PCStateBase &val, ThreadID tid) { set(pc[tid], val); }

  private:
    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to write information heading to previous stages. */
    TimeBuffer<TimeStruct>::wire toIEW;

    /** Wire to read information from IEW (for ROB). */
    TimeBuffer<TimeStruct>::wire robInfoFromIEW;

    TimeBuffer<FetchStruct> *fetchQueue;

    TimeBuffer<FetchStruct>::wire fromFetch;

    /** IEW instruction queue interface. */
    TimeBuffer<IEWStruct> *iewQueue;

    /** Wire to read information from IEW queue. */
    TimeBuffer<IEWStruct>::wire fromIEW;

    /** Rename instruction queue interface, for ROB. */
    TimeBuffer<RenameStruct> *renameQueue;

    /** Wire to read information from rename queue. */
    TimeBuffer<RenameStruct>::wire fromRename;

  public:
    /** ROB interface. */
    ROB *rob;

  private:
    /** Pointer to O3CPU. */
    CPU *cpu;

    /** Vector of all of the threads. */
    std::vector<ThreadState *> thread;

    /** Records that commit has written to the time buffer this cycle. Used for
     * the CPU to determine if it can deschedule itself if there is no activity.
     */
    bool wroteToTimeBuffer;

    /** Records if the number of ROB entries has changed this cycle. If it has,
     * then the number of free entries must be re-broadcast.
     */
    bool changedROBNumEntries[MaxThreads];

    /** Records if a thread has to squash this cycle due to a trap. */
    bool trapSquash[MaxThreads];

    /** Records if a thread has to squash this cycle due to an XC write. */
    bool tcSquash[MaxThreads];

    /**
     * Instruction passed to squashAfter().
     *
     * The squash after implementation needs to buffer the instruction
     * that caused a squash since this needs to be passed to the fetch
     * stage once squashing starts.
     */
    DynInstPtr squashAfterInst[MaxThreads];

    /** Priority List used for Commit Policy */
    std::list<ThreadID> priority_list;

    /** IEW to Commit delay. */
    const Cycles iewToCommitDelay;

    /** Commit to IEW delay. */
    const Cycles commitToIEWDelay;

    /** Rename to ROB delay. */
    const Cycles renameToROBDelay;

    const Cycles fetchToCommitDelay;

    /** Rename width, in instructions.  Used so ROB knows how many
     *  instructions to get from the rename instruction queue.
     */
    const unsigned renameWidth;

    /** Commit width, in instructions. */
    const unsigned commitWidth;

    /** Number of Active Threads */
    const ThreadID numThreads;

    /** Is a drain pending? Commit is looking for an instruction boundary while
     * there are no pending interrupts
     */
    bool drainPending;

    /** Is a drain imminent? Commit has found an instruction boundary while no
     * interrupts were present or in flight.  This was the last architecturally
     * committed instruction.  Interrupts disabled and pipeline flushed.
     * Waiting for structures to finish draining.
     */
    bool drainImminent;

    /** The latency to handle a trap.  Used when scheduling trap
     * squash event.
     */
    const Cycles trapLatency;

    /** The interrupt fault. */
    Fault interrupt;

    /** The commit PC state of each thread.  Refers to the instruction that
     * is currently being processed/committed.
     */
    std::unique_ptr<PCStateBase> pc[MaxThreads];

    /** The sequence number of the youngest valid instruction in the ROB. */
    InstSeqNum youngestSeqNum[MaxThreads];

    /** The sequence number of the last commited instruction. */
    InstSeqNum lastCommitedSeqNum[MaxThreads];
    InstSeqNum lastCommitedBrSeqNum[MaxThreads];

    /** Records if there is a trap currently in flight. */
    bool trapInFlight[MaxThreads];

    /** Records if there were any stores committed this cycle. */
    bool committedStores[MaxThreads];

    /** Records if commit should check if the ROB is truly empty (see
        commit_impl.hh). */
    bool checkEmptyROB[MaxThreads];

    /** Pointer to the list of active threads. */
    std::list<ThreadID> *activeThreads;

    /** Rename map interface. */
    UnifiedRenameMap *renameMap[MaxThreads];

    /** True if last committed microop can be followed by an interrupt */
    bool canHandleInterrupts;

    /** Have we had an interrupt pending and then seen it de-asserted because
        of a masking change? In this case the variable is set and the next time
        interrupts are enabled and pending the pipeline will squash to avoid
        a possible livelock senario.  */
    bool avoidQuiesceLiveLock;

    /** Updates commit stats based on this instruction. */
    void updateComInstStats(const DynInstPtr &inst);

    // HTM
    int htmStarts[MaxThreads];
    int htmStops[MaxThreads];

    uint64_t mispredBrInstSeq;
    struct CommitStats : public statistics::Group
    {
        CommitStats(CPU *cpu, Commit *commit);
        /** Stat for the total number of squashed instructions discarded by
         * commit.
         */
        statistics::Scalar commitSquashedInsts;
        /** Stat for the total number of times commit has had to stall due
         * to a non-speculative instruction reaching the head of the ROB.
         */
        statistics::Scalar commitNonSpecStalls;
        /** Stat for the total number of branch mispredicts that caused a
         * squash.
         */
        statistics::Scalar branchMispredicts;
        /** Distribution of the number of committed instructions each cycle. */
        statistics::Distribution numCommittedDist;

        /** Stat for the total number of committed atomics. */
        statistics::Vector amos;
        /** Total number of committed memory barriers. */
        statistics::Vector membars;
        /** Total number of function calls */
        statistics::Vector functionCalls;
        /** Committed instructions by instruction type (OpClass) */
        statistics::Vector2d committedInstType;

        /** Number of cycles where the commit bandwidth limit is reached. */
        statistics::Scalar commitEligibleSamples;
    } stats;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_COMMIT_HH__
