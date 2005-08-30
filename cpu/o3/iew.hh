/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

//Todo: Update with statuses.
//Need to handle delaying writes to the writeback bus if it's full at the
//given time.

#ifndef __CPU_O3_CPU_SIMPLE_IEW_HH__
#define __CPU_O3_CPU_SIMPLE_IEW_HH__

#include <queue>

#include "config/full_system.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/o3/comm.hh"

template<class Impl>
class SimpleIEW
{
  private:
    //Typedefs from Impl
    typedef typename Impl::ISA ISA;
    typedef typename Impl::CPUPol CPUPol;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::Params Params;

    typedef typename CPUPol::IQ IQ;
    typedef typename CPUPol::RenameMap RenameMap;
    typedef typename CPUPol::LDSTQ LDSTQ;

    typedef typename CPUPol::TimeStruct TimeStruct;
    typedef typename CPUPol::IEWStruct IEWStruct;
    typedef typename CPUPol::RenameStruct RenameStruct;
    typedef typename CPUPol::IssueStruct IssueStruct;

    friend class Impl::FullCPU;
  public:
    enum Status {
        Running,
        Blocked,
        Idle,
        Squashing,
        Unblocking
    };

  private:
    Status _status;
    Status _issueStatus;
    Status _exeStatus;
    Status _wbStatus;

  public:
    class WritebackEvent : public Event {
      private:
        DynInstPtr inst;
        SimpleIEW<Impl> *iewStage;

      public:
        WritebackEvent(DynInstPtr &_inst, SimpleIEW<Impl> *_iew);

        virtual void process();
        virtual const char *description();
    };

  public:
    SimpleIEW(Params &params);

    void regStats();

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    void setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr);

    void setRenameMap(RenameMap *rm_ptr);

    void squash();

    void squashDueToBranch(DynInstPtr &inst);

    void squashDueToMem(DynInstPtr &inst);

    void block();

    inline void unblock();

    void wakeDependents(DynInstPtr &inst);

    void instToCommit(DynInstPtr &inst);

  private:
    void dispatchInsts();

    void executeInsts();

  public:
    void tick();

    void iew();

    //Interfaces to objects inside and outside of IEW.
    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

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

    //Will need internal queue to hold onto instructions coming from
    //the rename stage in case of a stall.
    /** Skid buffer between rename and IEW. */
    std::queue<RenameStruct> skidBuffer;

  protected:
    /** Instruction queue. */
    IQ instQueue;

    LDSTQ ldstQueue;

#if !FULL_SYSTEM
  public:
    void lsqWriteback();
#endif

  private:
    /** Pointer to rename map.  Might not want this stage to directly
     *  access this though...
     */
    RenameMap *renameMap;

    /** CPU interface. */
    FullCPU *cpu;

  private:
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

    /** Number of cycles stage has been squashing.  Used so that the stage
     *  knows when it can start unblocking, which is when the previous stage
     *  has received the stall signal and clears up its outputs.
     */
    unsigned cyclesSquashing;

    Stats::Scalar<> iewIdleCycles;
    Stats::Scalar<> iewSquashCycles;
    Stats::Scalar<> iewBlockCycles;
    Stats::Scalar<> iewUnblockCycles;
//    Stats::Scalar<> iewWBInsts;
    Stats::Scalar<> iewDispatchedInsts;
    Stats::Scalar<> iewDispSquashedInsts;
    Stats::Scalar<> iewDispLoadInsts;
    Stats::Scalar<> iewDispStoreInsts;
    Stats::Scalar<> iewDispNonSpecInsts;
    Stats::Scalar<> iewIQFullEvents;
    Stats::Scalar<> iewExecutedInsts;
    Stats::Scalar<> iewExecLoadInsts;
    Stats::Scalar<> iewExecStoreInsts;
    Stats::Scalar<> iewExecSquashedInsts;
    Stats::Scalar<> memOrderViolationEvents;
    Stats::Scalar<> predictedTakenIncorrect;
};

#endif // __CPU_O3_CPU_IEW_HH__
