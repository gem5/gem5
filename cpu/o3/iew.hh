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
 */

#ifndef __CPU_O3_IEW_HH__
#define __CPU_O3_IEW_HH__

#include <queue>

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "config/full_system.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/scoreboard.hh"
#include "cpu/o3/lsq.hh"

class FUPool;

/**
 * DefaultIEW handles both single threaded and SMT IEW
 * (issue/execute/writeback).  It handles the dispatching of
 * instructions to the LSQ/IQ as part of the issue stage, and has the
 * IQ try to issue instructions each cycle. The execute latency is
 * actually tied into the issue latency to allow the IQ to be able to
 * do back-to-back scheduling without having to speculatively schedule
 * instructions. This happens by having the IQ have access to the
 * functional units, and the IQ gets the execution latencies from the
 * FUs when it issues instructions. Instructions reach the execute
 * stage on the last cycle of their execution, which is when the IQ
 * knows to wake up any dependent instructions, allowing back to back
 * scheduling. The execute portion of IEW separates memory
 * instructions from non-memory instructions, either telling the LSQ
 * to execute the instruction, or executing the instruction directly.
 * The writeback portion of IEW completes the instructions by waking
 * up any dependents, and marking the register ready on the
 * scoreboard.
 */
template<class Impl>
class DefaultIEW
{
  private:
    //Typedefs from Impl
    typedef typename Impl::CPUPol CPUPol;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::Params Params;

    typedef typename CPUPol::IQ IQ;
    typedef typename CPUPol::RenameMap RenameMap;
    typedef typename CPUPol::LSQ LSQ;

    typedef typename CPUPol::TimeStruct TimeStruct;
    typedef typename CPUPol::IEWStruct IEWStruct;
    typedef typename CPUPol::RenameStruct RenameStruct;
    typedef typename CPUPol::IssueStruct IssueStruct;

    friend class Impl::FullCPU;
    friend class CPUPol::IQ;

  public:
    /** Overall IEW stage status. Used to determine if the CPU can
     * deschedule itself due to a lack of activity.
     */
    enum Status {
        Active,
        Inactive
    };

    /** Status for Issue, Execute, and Writeback stages. */
    enum StageStatus {
        Running,
        Blocked,
        Idle,
        StartSquash,
        Squashing,
        Unblocking
    };

  private:
    /** Overall stage status. */
    Status _status;
    /** Dispatch status. */
    StageStatus dispatchStatus[Impl::MaxThreads];
    /** Execute status. */
    StageStatus exeStatus;
    /** Writeback status. */
    StageStatus wbStatus;

  public:
    /** LdWriteback event for a load completion. */
    class LdWritebackEvent : public Event {
      private:
        /** Instruction that is writing back data to the register file. */
        DynInstPtr inst;
        /** Pointer to IEW stage. */
        DefaultIEW<Impl> *iewStage;

      public:
        /** Constructs a load writeback event. */
        LdWritebackEvent(DynInstPtr &_inst, DefaultIEW<Impl> *_iew);

        /** Processes writeback event. */
        virtual void process();
        /** Returns the description of the writeback event. */
        virtual const char *description();
    };

  public:
    /** Constructs a DefaultIEW with the given parameters. */
    DefaultIEW(Params *params);

    /** Returns the name of the DefaultIEW stage. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Initializes stage; sends back the number of free IQ and LSQ entries. */
    void initStage();

    /** Sets CPU pointer for IEW, IQ, and LSQ. */
    void setCPU(FullCPU *cpu_ptr);

    /** Sets main time buffer used for backwards communication. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Sets time buffer for getting instructions coming from rename. */
    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    /** Sets time buffer to pass on instructions to commit. */
    void setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<unsigned> *at_ptr);

    /** Sets pointer to the scoreboard. */
    void setScoreboard(Scoreboard *sb_ptr);

    /** Starts switch out of IEW stage. */
    void switchOut();

    /** Completes switch out of IEW stage. */
    void doSwitchOut();

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Returns if IEW is switched out. */
    bool isSwitchedOut() { return switchedOut; }

    /** Sets page table pointer within LSQ. */
//    void setPageTable(PageTable *pt_ptr);

    /** Squashes instructions in IEW for a specific thread. */
    void squash(unsigned tid);

    /** Wakes all dependents of a completed instruction. */
    void wakeDependents(DynInstPtr &inst);

    /** Tells memory dependence unit that a memory instruction needs to be
     * rescheduled. It will re-execute once replayMemInst() is called.
     */
    void rescheduleMemInst(DynInstPtr &inst);

    /** Re-executes all rescheduled memory instructions. */
    void replayMemInst(DynInstPtr &inst);

    /** Sends an instruction to commit through the time buffer. */
    void instToCommit(DynInstPtr &inst);

    /** Inserts unused instructions of a thread into the skid buffer. */
    void skidInsert(unsigned tid);

    /** Returns the max of the number of entries in all of the skid buffers. */
    int skidCount();

    /** Returns if all of the skid buffers are empty. */
    bool skidsEmpty();

    /** Updates overall IEW status based on all of the stages' statuses. */
    void updateStatus();

    /** Resets entries of the IQ and the LSQ. */
    void resetEntries();

    /** Tells the CPU to wakeup if it has descheduled itself due to no
     * activity. Used mainly by the LdWritebackEvent.
     */
    void wakeCPU();

    /** Reports to the CPU that there is activity this cycle. */
    void activityThisCycle();

    /** Tells CPU that the IEW stage is active and running. */
    inline void activateStage();

    /** Tells CPU that the IEW stage is inactive and idle. */
    inline void deactivateStage();

    /** Returns if the LSQ has any stores to writeback. */
    bool hasStoresToWB() { return ldstQueue.hasStoresToWB(); }

  private:
    /** Sends commit proper information for a squash due to a branch
     * mispredict.
     */
    void squashDueToBranch(DynInstPtr &inst, unsigned thread_id);

    /** Sends commit proper information for a squash due to a memory order
     * violation.
     */
    void squashDueToMemOrder(DynInstPtr &inst, unsigned thread_id);

    /** Sends commit proper information for a squash due to memory becoming
     * blocked (younger issued instructions must be retried).
     */
    void squashDueToMemBlocked(DynInstPtr &inst, unsigned thread_id);

    /** Sets Dispatch to blocked, and signals back to other stages to block. */
    void block(unsigned thread_id);

    /** Unblocks Dispatch if the skid buffer is empty, and signals back to
     * other stages to unblock.
     */
    void unblock(unsigned thread_id);

    /** Determines proper actions to take given Dispatch's status. */
    void dispatch(unsigned tid);

    /** Dispatches instructions to IQ and LSQ. */
    void dispatchInsts(unsigned tid);

    /** Executes instructions. In the case of memory operations, it informs the
     * LSQ to execute the instructions. Also handles any redirects that occur
     * due to the executed instructions.
     */
    void executeInsts();

    /** Writebacks instructions. In our model, the instruction's execute()
     * function atomically reads registers, executes, and writes registers.
     * Thus this writeback only wakes up dependent instructions, and informs
     * the scoreboard of registers becoming ready.
     */
    void writebackInsts();

    /** Returns the number of valid, non-squashed instructions coming from
     * rename to dispatch.
     */
    unsigned validInstsFromRename();

    /** Reads the stall signals. */
    void readStallSignals(unsigned tid);

    /** Checks if any of the stall conditions are currently true. */
    bool checkStall(unsigned tid);

    /** Processes inputs and changes state accordingly. */
    void checkSignalsAndUpdate(unsigned tid);

    /** Sorts instructions coming from rename into lists separated by thread. */
    void sortInsts();

  public:
    /** Ticks IEW stage, causing Dispatch, the IQ, the LSQ, Execute, and
     * Writeback to run for one cycle.
     */
    void tick();

  private:
    /** Updates execution stats based on the instruction. */
    void updateExeInstStats(DynInstPtr &inst);

    /** Pointer to main time buffer used for backwards communication. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to write information heading to previous stages. */
    typename TimeBuffer<TimeStruct>::wire toFetch;

    /** Wire to get commit's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire to write information heading to previous stages. */
    typename TimeBuffer<TimeStruct>::wire toRename;

    /** Rename instruction queue interface. */
    TimeBuffer<RenameStruct> *renameQueue;

    /** Wire to get rename's output from rename queue. */
    typename TimeBuffer<RenameStruct>::wire fromRename;

    /** Issue stage queue. */
    TimeBuffer<IssueStruct> issueToExecQueue;

    /** Wire to read information from the issue stage time queue. */
    typename TimeBuffer<IssueStruct>::wire fromIssue;

    /**
     * IEW stage time buffer.  Holds ROB indices of instructions that
     * can be marked as completed.
     */
    TimeBuffer<IEWStruct> *iewQueue;

    /** Wire to write infromation heading to commit. */
    typename TimeBuffer<IEWStruct>::wire toCommit;

    /** Queue of all instructions coming from rename this cycle. */
    std::queue<DynInstPtr> insts[Impl::MaxThreads];

    /** Skid buffer between rename and IEW. */
    std::queue<DynInstPtr> skidBuffer[Impl::MaxThreads];

    /** Scoreboard pointer. */
    Scoreboard* scoreboard;

  public:
    /** Instruction queue. */
    IQ instQueue;

    /** Load / store queue. */
    LSQ ldstQueue;

    /** Pointer to the functional unit pool. */
    FUPool *fuPool;

  private:
    /** CPU pointer. */
    FullCPU *cpu;

    /** Records if IEW has written to the time buffer this cycle, so that the
     * CPU can deschedule itself if there is no activity.
     */
    bool wroteToTimeBuffer;

    /** Source of possible stalls. */
    struct Stalls {
        bool commit;
    };

    /** Stages that are telling IEW to stall. */
    Stalls stalls[Impl::MaxThreads];

    /** Debug function to print instructions that are issued this cycle. */
    void printAvailableInsts();

  public:
    /** Records if the LSQ needs to be updated on the next cycle, so that
     * IEW knows if there will be activity on the next cycle.
     */
    bool updateLSQNextCycle;

  private:
    /** Records if there is a fetch redirect on this cycle for each thread. */
    bool fetchRedirect[Impl::MaxThreads];

    /** Used to track if all instructions have been dispatched this cycle.
     * If they have not, then blocking must have occurred, and the instructions
     * would already be added to the skid buffer.
     * @todo: Fix this hack.
     */
    bool dispatchedAllInsts;

    /** Records if the queues have been changed (inserted or issued insts),
     * so that IEW knows to broadcast the updated amount of free entries.
     */
    bool updatedQueues;

    /** Commit to IEW delay, in ticks. */
    unsigned commitToIEWDelay;

    /** Rename to IEW delay, in ticks. */
    unsigned renameToIEWDelay;

    /**
     * Issue to execute delay, in ticks.  What this actually represents is
     * the amount of time it takes for an instruction to wake up, be
     * scheduled, and sent to a FU for execution.
     */
    unsigned issueToExecuteDelay;

    /** Width of issue's read path, in instructions.  The read path is both
     *  the skid buffer and the rename instruction queue.
     *  Note to self: is this really different than issueWidth?
     */
    unsigned issueReadWidth;

    /** Width of issue, in instructions. */
    unsigned issueWidth;

    /** Width of execute, in instructions.  Might make more sense to break
     *  down into FP vs int.
     */
    unsigned executeWidth;

    /** Index into queue of instructions being written back. */
    unsigned wbNumInst;

    /** Cycle number within the queue of instructions being written back.
     * Used in case there are too many instructions writing back at the current
     * cycle and writesbacks need to be scheduled for the future. See comments
     * in instToCommit().
     */
    unsigned wbCycle;

    /** Number of active threads. */
    unsigned numThreads;

    /** Pointer to list of active threads. */
    std::list<unsigned> *activeThreads;

    /** Maximum size of the skid buffer. */
    unsigned skidBufferMax;

    /** Is this stage switched out. */
    bool switchedOut;

    /** Stat for total number of idle cycles. */
    Stats::Scalar<> iewIdleCycles;
    /** Stat for total number of squashing cycles. */
    Stats::Scalar<> iewSquashCycles;
    /** Stat for total number of blocking cycles. */
    Stats::Scalar<> iewBlockCycles;
    /** Stat for total number of unblocking cycles. */
    Stats::Scalar<> iewUnblockCycles;
    /** Stat for total number of instructions dispatched. */
    Stats::Scalar<> iewDispatchedInsts;
    /** Stat for total number of squashed instructions dispatch skips. */
    Stats::Scalar<> iewDispSquashedInsts;
    /** Stat for total number of dispatched load instructions. */
    Stats::Scalar<> iewDispLoadInsts;
    /** Stat for total number of dispatched store instructions. */
    Stats::Scalar<> iewDispStoreInsts;
    /** Stat for total number of dispatched non speculative instructions. */
    Stats::Scalar<> iewDispNonSpecInsts;
    /** Stat for number of times the IQ becomes full. */
    Stats::Scalar<> iewIQFullEvents;
    /** Stat for number of times the LSQ becomes full. */
    Stats::Scalar<> iewLSQFullEvents;
    /** Stat for total number of executed instructions. */
    Stats::Scalar<> iewExecutedInsts;
    /** Stat for total number of executed load instructions. */
    Stats::Vector<> iewExecLoadInsts;
    /** Stat for total number of executed store instructions. */
//    Stats::Scalar<> iewExecStoreInsts;
    /** Stat for total number of squashed instructions skipped at execute. */
    Stats::Scalar<> iewExecSquashedInsts;
    /** Stat for total number of memory ordering violation events. */
    Stats::Scalar<> memOrderViolationEvents;
    /** Stat for total number of incorrect predicted taken branches. */
    Stats::Scalar<> predictedTakenIncorrect;
    /** Stat for total number of incorrect predicted not taken branches. */
    Stats::Scalar<> predictedNotTakenIncorrect;
    /** Stat for total number of mispredicted branches detected at execute. */
    Stats::Formula branchMispredicts;

    /** Number of executed software prefetches. */
    Stats::Vector<> exeSwp;
    /** Number of executed nops. */
    Stats::Vector<> exeNop;
    /** Number of executed meomory references. */
    Stats::Vector<> exeRefs;
    /** Number of executed branches. */
    Stats::Vector<> exeBranches;

//    Stats::Vector<> issued_ops;
/*
    Stats::Vector<> stat_fu_busy;
    Stats::Vector2d<> stat_fuBusy;
    Stats::Vector<> dist_unissued;
    Stats::Vector2d<> stat_issued_inst_type;
*/
    /** Number of instructions issued per cycle. */
    Stats::Formula issueRate;
    /** Number of executed store instructions. */
    Stats::Formula iewExecStoreInsts;
//    Stats::Formula issue_op_rate;
//    Stats::Formula fu_busy_rate;
    /** Number of instructions sent to commit. */
    Stats::Vector<> iewInstsToCommit;
    /** Number of instructions that writeback. */
    Stats::Vector<> writebackCount;
    /** Number of instructions that wake consumers. */
    Stats::Vector<> producerInst;
    /** Number of instructions that wake up from producers. */
    Stats::Vector<> consumerInst;
    /** Number of instructions that were delayed in writing back due
     * to resource contention.
     */
    Stats::Vector<> wbPenalized;

    /** Number of instructions per cycle written back. */
    Stats::Formula wbRate;
    /** Average number of woken instructions per writeback. */
    Stats::Formula wbFanout;
    /** Number of instructions per cycle delayed in writing back . */
    Stats::Formula wbPenalizedRate;
};

#endif // __CPU_O3_IEW_HH__
