//Todo: Update with statuses.
//Need to handle delaying writes to the writeback bus if it's full at the
//given time.  Load store queue.

#ifndef __CPU_BETA_CPU_SIMPLE_IEW_HH__
#define __CPU_BETA_CPU_SIMPLE_IEW_HH__

#include <queue>

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/beta_cpu/comm.hh"

//Can IEW even stall?  Space should be available/allocated already...maybe
//if there's not enough write ports on the ROB or waiting for CDB
//arbitration.
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

#ifndef FULL_SYSTEM
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

#endif // __CPU_BETA_CPU_IEW_HH__
