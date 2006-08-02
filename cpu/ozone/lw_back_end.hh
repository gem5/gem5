/*
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

#ifndef __CPU_OZONE_LW_BACK_END_HH__
#define __CPU_OZONE_LW_BACK_END_HH__

#include <list>
#include <queue>
#include <set>
#include <string>

#include "arch/faults.hh"
#include "base/timebuf.hh"
#include "cpu/inst_seq.hh"
#include "cpu/ozone/rename_table.hh"
#include "cpu/ozone/thread_state.hh"
#include "mem/functional/functional.hh"
#include "mem/mem_interface.hh"
#include "mem/mem_req.hh"
#include "sim/eventq.hh"

template <class>
class Checker;
class ExecContext;

template <class Impl>
class OzoneThreadState;

template <class Impl>
class LWBackEnd
{
  public:
    typedef OzoneThreadState<Impl> Thread;

    typedef typename Impl::Params Params;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::FrontEnd FrontEnd;
    typedef typename Impl::FullCPU::CommStruct CommStruct;

    struct SizeStruct {
        int size;
    };

    typedef SizeStruct DispatchToIssue;
    typedef SizeStruct IssueToExec;
    typedef SizeStruct ExecToCommit;
    typedef SizeStruct Writeback;

    TimeBuffer<DispatchToIssue> d2i;
    typename TimeBuffer<DispatchToIssue>::wire instsToDispatch;
    TimeBuffer<IssueToExec> i2e;
    typename TimeBuffer<IssueToExec>::wire instsToExecute;
    TimeBuffer<ExecToCommit> e2c;
    TimeBuffer<Writeback> numInstsToWB;

    TimeBuffer<CommStruct> *comm;
    typename TimeBuffer<CommStruct>::wire toIEW;
    typename TimeBuffer<CommStruct>::wire fromCommit;

    class TrapEvent : public Event {
      private:
        LWBackEnd<Impl> *be;

      public:
        TrapEvent(LWBackEnd<Impl> *_be);

        void process();
        const char *description();
    };

    /** LdWriteback event for a load completion. */
    class LdWritebackEvent : public Event {
      private:
        /** Instruction that is writing back data to the register file. */
        DynInstPtr inst;
        /** Pointer to IEW stage. */
        LWBackEnd *be;

        bool dcacheMiss;

      public:
        /** Constructs a load writeback event. */
        LdWritebackEvent(DynInstPtr &_inst, LWBackEnd *be);

        /** Processes writeback event. */
        virtual void process();
        /** Returns the description of the writeback event. */
        virtual const char *description();

        void setDcacheMiss() { dcacheMiss = true; be->addDcacheMiss(inst); }
    };

    LWBackEnd(Params *params);

    std::string name() const;

    void regStats();

    void setCPU(FullCPU *cpu_ptr);

    void setFrontEnd(FrontEnd *front_end_ptr)
    { frontEnd = front_end_ptr; }

    void setXC(ExecContext *xc_ptr)
    { xc = xc_ptr; }

    void setThreadState(Thread *thread_ptr)
    { thread = thread_ptr; }

    void setCommBuffer(TimeBuffer<CommStruct> *_comm);

    void tick();
    void squash();
    void generateXCEvent() { xcSquash = true; }
    void squashFromXC();
    void squashFromTrap();
    void checkInterrupts();
    bool trapSquash;
    bool xcSquash;

    template <class T>
    Fault read(MemReqPtr &req, T &data, int load_idx);

    template <class T>
    Fault write(MemReqPtr &req, T &data, int store_idx);

    Addr readCommitPC() { return commitPC; }

    Addr commitPC;

    Tick lastCommitCycle;

    bool robEmpty() { return instList.empty(); }

    bool isFull() { return numInsts >= numROBEntries; }
    bool isBlocked() { return status == Blocked || dispatchStatus == Blocked; }

    void fetchFault(Fault &fault);

    int wakeDependents(DynInstPtr &inst, bool memory_deps = false);

    /** Tells memory dependence unit that a memory instruction needs to be
     * rescheduled. It will re-execute once replayMemInst() is called.
     */
    void rescheduleMemInst(DynInstPtr &inst);

    /** Re-executes all rescheduled memory instructions. */
    void replayMemInst(DynInstPtr &inst);

    /** Completes memory instruction. */
    void completeMemInst(DynInstPtr &inst) { }

    void addDcacheMiss(DynInstPtr &inst)
    {
        waitingMemOps.insert(inst->seqNum);
        numWaitingMemOps++;
        DPRINTF(BE, "Adding a Dcache miss mem op [sn:%lli], total %i\n",
                inst->seqNum, numWaitingMemOps);
    }

    void removeDcacheMiss(DynInstPtr &inst)
    {
        assert(waitingMemOps.find(inst->seqNum) != waitingMemOps.end());
        waitingMemOps.erase(inst->seqNum);
        numWaitingMemOps--;
        DPRINTF(BE, "Removing a Dcache miss mem op [sn:%lli], total %i\n",
                inst->seqNum, numWaitingMemOps);
    }

    void addWaitingMemOp(DynInstPtr &inst)
    {
        waitingMemOps.insert(inst->seqNum);
        numWaitingMemOps++;
        DPRINTF(BE, "Adding a waiting mem op [sn:%lli], total %i\n",
                inst->seqNum, numWaitingMemOps);
    }

    void removeWaitingMemOp(DynInstPtr &inst)
    {
        assert(waitingMemOps.find(inst->seqNum) != waitingMemOps.end());
        waitingMemOps.erase(inst->seqNum);
        numWaitingMemOps--;
        DPRINTF(BE, "Removing a waiting mem op [sn:%lli], total %i\n",
                inst->seqNum, numWaitingMemOps);
    }

    void instToCommit(DynInstPtr &inst);

    void switchOut();
    void doSwitchOut();
    void takeOverFrom(ExecContext *old_xc = NULL);

    bool isSwitchedOut() { return switchedOut; }

  private:
    void generateTrapEvent(Tick latency = 0);
    void handleFault(Fault &fault, Tick latency = 0);
    void updateStructures();
    void dispatchInsts();
    void dispatchStall();
    void checkDispatchStatus();
    void executeInsts();
    void commitInsts();
    void addToLSQ(DynInstPtr &inst);
    void writebackInsts();
    bool commitInst(int inst_num);
    void squash(const InstSeqNum &sn);
    void squashDueToBranch(DynInstPtr &inst);
    void squashDueToMemViolation(DynInstPtr &inst);
    void squashDueToMemBlocked(DynInstPtr &inst);
    void updateExeInstStats(DynInstPtr &inst);
    void updateComInstStats(DynInstPtr &inst);

  public:
    FullCPU *cpu;

    FrontEnd *frontEnd;

    ExecContext *xc;

    Thread *thread;

    enum Status {
        Running,
        Idle,
        DcacheMissStall,
        DcacheMissComplete,
        Blocked,
        TrapPending
    };

    Status status;

    Status dispatchStatus;

    Status commitStatus;

    Counter funcExeInst;

  private:
    typedef typename Impl::LdstQueue LdstQueue;

    LdstQueue LSQ;
  public:
    RenameTable<Impl> commitRenameTable;

    RenameTable<Impl> renameTable;
  private:
    class DCacheCompletionEvent : public Event
    {
      private:
        LWBackEnd *be;

      public:
        DCacheCompletionEvent(LWBackEnd *_be);

        virtual void process();
        virtual const char *description();
    };

    friend class DCacheCompletionEvent;

    DCacheCompletionEvent cacheCompletionEvent;

    MemInterface *dcacheInterface;

    MemReqPtr memReq;

    // General back end width. Used if the more specific isn't given.
    int width;

    // Dispatch width.
    int dispatchWidth;
    int numDispatchEntries;
    int dispatchSize;

    int waitingInsts;

    int issueWidth;

    // Writeback width
    int wbWidth;

    // Commit width
    int commitWidth;

    /** Index into queue of instructions being written back. */
    unsigned wbNumInst;

    /** Cycle number within the queue of instructions being written
     * back.  Used in case there are too many instructions writing
     * back at the current cycle and writesbacks need to be scheduled
     * for the future. See comments in instToCommit().
     */
    unsigned wbCycle;

    int numROBEntries;
    int numInsts;

    std::set<InstSeqNum> waitingMemOps;
    typedef std::set<InstSeqNum>::iterator MemIt;
    int numWaitingMemOps;
    unsigned maxOutstandingMemOps;

    bool squashPending;
    InstSeqNum squashSeqNum;
    Addr squashNextPC;

    Fault faultFromFetch;
    bool fetchHasFault;

    bool switchedOut;
    bool switchPending;

    DynInstPtr memBarrier;

  private:
    struct pqCompare {
        bool operator() (const DynInstPtr &lhs, const DynInstPtr &rhs) const
        {
            return lhs->seqNum > rhs->seqNum;
        }
    };

    typedef typename std::priority_queue<DynInstPtr, std::vector<DynInstPtr>, pqCompare> ReadyInstQueue;
    ReadyInstQueue exeList;

    typedef typename std::list<DynInstPtr>::iterator InstListIt;

    std::list<DynInstPtr> instList;
    std::list<DynInstPtr> waitingList;
    std::list<DynInstPtr> replayList;
    std::list<DynInstPtr> writeback;

    int latency;

    int squashLatency;

    bool exactFullStall;

    // number of cycles stalled for D-cache misses
/*    Stats::Scalar<> dcacheStallCycles;
      Counter lastDcacheStall;
*/
    Stats::Vector<> robCapEvents;
    Stats::Vector<> robCapInstCount;
    Stats::Vector<> iqCapEvents;
    Stats::Vector<> iqCapInstCount;
    // total number of instructions executed
    Stats::Vector<> exeInst;
    Stats::Vector<> exeSwp;
    Stats::Vector<> exeNop;
    Stats::Vector<> exeRefs;
    Stats::Vector<> exeLoads;
    Stats::Vector<> exeBranches;

    Stats::Vector<> issuedOps;

    // total number of loads forwaded from LSQ stores
    Stats::Vector<> lsqForwLoads;

    // total number of loads ignored due to invalid addresses
    Stats::Vector<> invAddrLoads;

    // total number of software prefetches ignored due to invalid addresses
    Stats::Vector<> invAddrSwpfs;
    // ready loads blocked due to memory disambiguation
    Stats::Vector<> lsqBlockedLoads;

    Stats::Scalar<> lsqInversion;

    Stats::Vector<> nIssuedDist;
    Stats::VectorDistribution<> issueDelayDist;

    Stats::VectorDistribution<> queueResDist;
/*
    Stats::Vector<> stat_fu_busy;
    Stats::Vector2d<> stat_fuBusy;
    Stats::Vector<> dist_unissued;
    Stats::Vector2d<> stat_issued_inst_type;

    Stats::Formula misspec_cnt;
    Stats::Formula misspec_ipc;
    Stats::Formula issue_rate;
    Stats::Formula issue_stores;
    Stats::Formula issue_op_rate;
    Stats::Formula fu_busy_rate;
    Stats::Formula commit_stores;
    Stats::Formula commit_ipc;
    Stats::Formula commit_ipb;
    Stats::Formula lsq_inv_rate;
*/
    Stats::Vector<> writebackCount;
    Stats::Vector<> producerInst;
    Stats::Vector<> consumerInst;
    Stats::Vector<> wbPenalized;

    Stats::Formula wbRate;
    Stats::Formula wbFanout;
    Stats::Formula wbPenalizedRate;

    // total number of instructions committed
    Stats::Vector<> statComInst;
    Stats::Vector<> statComSwp;
    Stats::Vector<> statComRefs;
    Stats::Vector<> statComLoads;
    Stats::Vector<> statComMembars;
    Stats::Vector<> statComBranches;

    Stats::Distribution<> nCommittedDist;

    Stats::Scalar<> commitEligibleSamples;
    Stats::Vector<> commitEligible;

    Stats::Vector<> squashedInsts;
    Stats::Vector<> ROBSquashedInsts;

    Stats::Scalar<> ROBFcount;
    Stats::Formula ROBFullRate;

    Stats::Vector<>  ROBCount;	 // cumulative ROB occupancy
    Stats::Formula ROBOccRate;
    Stats::VectorDistribution<> ROBOccDist;
  public:
    void dumpInsts();

    Checker<DynInstPtr> *checker;
};

template <class Impl>
template <class T>
Fault
LWBackEnd<Impl>::read(MemReqPtr &req, T &data, int load_idx)
{
    return LSQ.read(req, data, load_idx);
}

template <class Impl>
template <class T>
Fault
LWBackEnd<Impl>::write(MemReqPtr &req, T &data, int store_idx)
{
    return LSQ.write(req, data, store_idx);
}

#endif // __CPU_OZONE_LW_BACK_END_HH__
