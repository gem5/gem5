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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_OZONE_BACK_END_HH__
#define __CPU_OZONE_BACK_END_HH__

#include <list>
#include <queue>
#include <string>

#include "cpu/ozone/rename_table.hh"
#include "cpu/ozone/thread_state.hh"
#include "cpu/inst_seq.hh"
#include "cpu/timebuf.hh"
#include "mem/request.hh"
#include "sim/eventq.hh"
#include "sim/faults.hh"

class ThreadContext;

template <class Impl>
class OzoneThreadState;

template <class Impl>
class BackEnd
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

    class InstQueue {
        enum queue {
            NonSpec,
            IQ,
            ToBeScheduled,
            ReadyList,
            ReplayList
        };
        struct pqCompare {
            bool operator() (const DynInstPtr &lhs, const DynInstPtr &rhs) const
            {
                return lhs->seqNum > rhs->seqNum;
            }
        };
      public:
        InstQueue(Params *params);

        std::string name() const;

        void regStats();

        void setIssueExecQueue(TimeBuffer<IssueToExec> *i2e_queue);

        void setBE(BackEnd *_be) { be = _be; }

        void insert(DynInstPtr &inst);

        void scheduleReadyInsts();

        void scheduleNonSpec(const InstSeqNum &sn);

        DynInstPtr getReadyInst();

        void commit(const InstSeqNum &sn) {}

        void squash(const InstSeqNum &sn);

        int wakeDependents(DynInstPtr &inst);

        /** Tells memory dependence unit that a memory instruction needs to be
         * rescheduled. It will re-execute once replayMemInst() is called.
         */
        void rescheduleMemInst(DynInstPtr &inst);

        /** Re-executes all rescheduled memory instructions. */
        void replayMemInst(DynInstPtr &inst);

        /** Completes memory instruction. */
        void completeMemInst(DynInstPtr &inst);

        void violation(DynInstPtr &inst, DynInstPtr &violation) { }

        bool isFull() { return numInsts >= size; }

        void dumpInsts();

      private:
        bool find(queue q, typename std::list<DynInstPtr>::iterator it);
        BackEnd *be;
        TimeBuffer<IssueToExec> *i2e;
        typename TimeBuffer<IssueToExec>::wire numIssued;
        typedef typename std::list<DynInstPtr> InstList;
        typedef typename std::list<DynInstPtr>::iterator InstListIt;
        typedef typename std::priority_queue<DynInstPtr, std::vector<DynInstPtr>, pqCompare> ReadyInstQueue;
        // Not sure I need the IQ list; it just needs to be a count.
        InstList iq;
        InstList toBeScheduled;
        InstList readyList;
        InstList nonSpec;
        InstList replayList;
        ReadyInstQueue readyQueue;
      public:
        int size;
        int numInsts;
        int width;

        Stats::VectorDistribution occ_dist;

        Stats::Vector inst_count;
        Stats::Vector peak_inst_count;
        Stats::Scalar empty_count;
        Stats::Scalar current_count;
        Stats::Scalar fullCount;

        Stats::Formula occ_rate;
        Stats::Formula avg_residency;
        Stats::Formula empty_rate;
        Stats::Formula full_rate;
    };

    /** LdWriteback event for a load completion. */
    class LdWritebackEvent : public Event {
      private:
        /** Instruction that is writing back data to the register file. */
        DynInstPtr inst;
        /** Pointer to IEW stage. */
        BackEnd *be;

      public:
        /** Constructs a load writeback event. */
        LdWritebackEvent(DynInstPtr &_inst, BackEnd *be);

        /** Processes writeback event. */
        virtual void process();
        /** Returns the description of the writeback event. */
        virtual const char *description() const;
    };

    BackEnd(Params *params);

    std::string name() const;

    void regStats();

    void setCPU(FullCPU *cpu_ptr)
    { cpu = cpu_ptr; }

    void setFrontEnd(FrontEnd *front_end_ptr)
    { frontEnd = front_end_ptr; }

    void setTC(ThreadContext *tc_ptr)
    { tc = tc_ptr; }

    void setThreadState(Thread *thread_ptr)
    { thread = thread_ptr; }

    void setCommBuffer(TimeBuffer<CommStruct> *_comm);

    void tick();
    void squash();
    void squashFromTC();
    bool tcSquash;

    template <class T>
    Fault read(RequestPtr req, T &data, int load_idx);

    template <class T>
    Fault write(RequestPtr req, T &data, int store_idx);

    Addr readCommitPC() { return commitPC; }

    Addr commitPC;

    bool robEmpty() { return instList.empty(); }

    bool isFull() { return numInsts >= numROBEntries; }
    bool isBlocked() { return status == Blocked || dispatchStatus == Blocked; }

    /** Tells memory dependence unit that a memory instruction needs to be
     * rescheduled. It will re-execute once replayMemInst() is called.
     */
    void rescheduleMemInst(DynInstPtr &inst)
    { IQ.rescheduleMemInst(inst); }

    /** Re-executes all rescheduled memory instructions. */
    void replayMemInst(DynInstPtr &inst)
    { IQ.replayMemInst(inst); }

    /** Completes memory instruction. */
    void completeMemInst(DynInstPtr &inst)
    { IQ.completeMemInst(inst); }

    void fetchFault(Fault &fault);

  private:
    void updateStructures();
    void dispatchInsts();
    void dispatchStall();
    void checkDispatchStatus();
    void scheduleReadyInsts();
    void executeInsts();
    void commitInsts();
    void addToIQ(DynInstPtr &inst);
    void addToLSQ(DynInstPtr &inst);
    void instToCommit(DynInstPtr &inst);
    void writebackInsts();
    bool commitInst(int inst_num);
    void squash(const InstSeqNum &sn);
    void squashDueToBranch(DynInstPtr &inst);
    void squashDueToMemBlocked(DynInstPtr &inst);
    void updateExeInstStats(DynInstPtr &inst);
    void updateComInstStats(DynInstPtr &inst);

  public:
    FullCPU *cpu;

    FrontEnd *frontEnd;

    ThreadContext *tc;

    Thread *thread;

    enum Status {
        Running,
        Idle,
        DcacheMissStall,
        DcacheMissComplete,
        Blocked
    };

    Status status;

    Status dispatchStatus;

    Counter funcExeInst;

  private:
//    typedef typename Impl::InstQueue InstQueue;

    InstQueue IQ;

    typedef typename Impl::LdstQueue LdstQueue;

    LdstQueue LSQ;
  public:
    RenameTable<Impl> commitRenameTable;

    RenameTable<Impl> renameTable;
  private:
    class DCacheCompletionEvent : public Event
    {
      private:
        BackEnd *be;

      public:
        DCacheCompletionEvent(BackEnd *_be);

        virtual void process();
        virtual const char *description() const;
    };

    friend class DCacheCompletionEvent;

    DCacheCompletionEvent cacheCompletionEvent;

    MemInterface *dcacheInterface;

    Request *memReq;

    // General back end width. Used if the more specific isn't given.
    int width;

    // Dispatch width.
    int dispatchWidth;
    int numDispatchEntries;
    int dispatchSize;

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

    bool squashPending;
    InstSeqNum squashSeqNum;
    Addr squashNextPC;

    Fault faultFromFetch;

  private:
    typedef typename std::list<DynInstPtr>::iterator InstListIt;

    std::list<DynInstPtr> instList;
    std::list<DynInstPtr> dispatch;
    std::list<DynInstPtr> writeback;

    int latency;

    int squashLatency;

    bool exactFullStall;

    bool fetchRedirect[Impl::MaxThreads];

    // number of cycles stalled for D-cache misses
/*    Stats::Scalar dcacheStallCycles;
      Counter lastDcacheStall;
*/
    Stats::Vector rob_cap_events;
    Stats::Vector rob_cap_inst_count;
    Stats::Vector iq_cap_events;
    Stats::Vector iq_cap_inst_count;
    // total number of instructions executed
    Stats::Vector exe_inst;
    Stats::Vector exe_swp;
    Stats::Vector exe_nop;
    Stats::Vector exe_refs;
    Stats::Vector exe_loads;
    Stats::Vector exe_branches;

    Stats::Vector issued_ops;

    // total number of loads forwaded from LSQ stores
    Stats::Vector lsq_forw_loads;

    // total number of loads ignored due to invalid addresses
    Stats::Vector inv_addr_loads;

    // total number of software prefetches ignored due to invalid addresses
    Stats::Vector inv_addr_swpfs;
    // ready loads blocked due to memory disambiguation
    Stats::Vector lsq_blocked_loads;

    Stats::Scalar lsqInversion;

    Stats::Vector n_issued_dist;
    Stats::VectorDistribution issue_delay_dist;

    Stats::VectorDistribution queue_res_dist;
/*
    Stats::Vector stat_fu_busy;
    Stats::Vector2d stat_fuBusy;
    Stats::Vector dist_unissued;
    Stats::Vector2d stat_issued_inst_type;

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
    Stats::Vector writeback_count;
    Stats::Vector producer_inst;
    Stats::Vector consumer_inst;
    Stats::Vector wb_penalized;

    Stats::Formula wb_rate;
    Stats::Formula wb_fanout;
    Stats::Formula wb_penalized_rate;

    // total number of instructions committed
    Stats::Vector stat_com_inst;
    Stats::Vector stat_com_swp;
    Stats::Vector stat_com_refs;
    Stats::Vector stat_com_loads;
    Stats::Vector stat_com_membars;
    Stats::Vector stat_com_branches;

    Stats::Distribution n_committed_dist;

    Stats::Scalar commit_eligible_samples;
    Stats::Vector commit_eligible;

    Stats::Scalar ROB_fcount;
    Stats::Formula ROB_full_rate;

    Stats::Vector  ROB_count;  // cumulative ROB occupancy
    Stats::Formula ROB_occ_rate;
    Stats::VectorDistribution ROB_occ_dist;
  public:
    void dumpInsts();
};

template <class Impl>
template <class T>
Fault
BackEnd<Impl>::read(RequestPtr req, T &data, int load_idx)
{
/*    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    Fault fault = cpu->translateDataReadReq(memReq);

    // if we have a cache, do cache access too
    if (fault == NoFault && dcacheInterface) {
        memReq->cmd = Read;
        memReq->completionEvent = NULL;
        memReq->time = curTick();
        memReq->flags &= ~INST_READ;
        MemAccessResult result = dcacheInterface->access(memReq);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT && dcacheInterface->doEvents()) {
            // Fix this hack for keeping funcExeInst correct with loads that
            // are executed twice.
            --funcExeInst;

            memReq->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick();
//          unscheduleTickEvent();
//          status = DcacheMissStall;
            DPRINTF(OzoneCPU, "Dcache miss stall!\n");
        } else {
            // do functional access
            fault = thread->mem->read(memReq, data);

        }
    }
*/
    return LSQ.read(req, data, load_idx);
}

template <class Impl>
template <class T>
Fault
BackEnd<Impl>::write(RequestPtr req, T &data, int store_idx)
{
/*
    memReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    Fault fault = cpu->translateDataWriteReq(memReq);

    if (fault == NoFault && dcacheInterface) {
        memReq->cmd = Write;
        memcpy(memReq->data,(uint8_t *)&data,memReq->size);
        memReq->completionEvent = NULL;
        memReq->time = curTick();
        memReq->flags &= ~INST_READ;
        MemAccessResult result = dcacheInterface->access(memReq);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT && dcacheInterface->doEvents()) {
            memReq->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick();
//          unscheduleTickEvent();
//          status = DcacheMissStall;
            DPRINTF(OzoneCPU, "Dcache miss stall!\n");
        }
    }

    if (res && (fault == NoFault))
        *res = memReq->result;
        */
    return LSQ.write(req, data, store_idx);
}

#endif // __CPU_OZONE_BACK_END_HH__
