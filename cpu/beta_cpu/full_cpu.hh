//Todo: Add in a lot of the functions that are ISA specific.  Also define
//the functions that currently exist within the base cpu class.  Define
//everything for the simobject stuff so it can be serialized and
//instantiated, add in debugging statements everywhere.  Have CPU schedule
//itself properly.  Constructor.  Derived alpha class.  Threads!
// Avoid running stages and advancing queues if idle/stalled.

#ifndef __SIMPLE_FULL_CPU_HH__
#define __SIMPLE_FULL_CPU_HH__

#include <iostream>
#include <list>

#include "cpu/beta_cpu/comm.hh"

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/base_cpu.hh"
#include "cpu/exec_context.hh"
#include "cpu/beta_cpu/cpu_policy.hh"
#include "sim/process.hh"

using namespace std;

class FunctionalMemory;
class Process;

class BaseFullCPU : public BaseCPU
{
    //Stuff that's pretty ISA independent will go here.
  public:
    class Params
    {
      public:
#ifdef FULL_SYSTEM
        std::string name;
        int numberOfThreads;
        Counter maxInstsAnyThread;
        Counter maxInstsAllThreads;
        Counter maxLoadsAnyThread;
        Counter maxLoadsAllThreads;
        System *_system;
        Tick freq;
#else
        std::string name;
        int numberOfThreads;
        Counter maxInstsAnyThread;
        Counter maxInstsAllThreads;
        Counter maxLoadsAnyThread;
        Counter maxLoadsAllThreads;
#endif // FULL_SYSTEM
    };

#ifdef FULL_SYSTEM
    BaseFullCPU(Params &params);
#else
    BaseFullCPU(Params &params);
#endif // FULL_SYSTEM
};

template <class Impl>
class FullBetaCPU : public BaseFullCPU
{
  public:
    //Put typedefs from the Impl here.
    typedef typename Impl::CPUPol CPUPolicy;
    typedef typename Impl::Params Params;
    typedef typename Impl::DynInstPtr DynInstPtr;

  public:
    enum Status {
        Running,
        Idle,
        Halted,
        Blocked // ?
    };

    Status _status;

  private:
    class TickEvent : public Event
    {
      private:
        FullBetaCPU<Impl> *cpu;

      public:
        TickEvent(FullBetaCPU<Impl> *c);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + delay);
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + delay);
    }

    /// Unschedule tick event, regardless of its current state.
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  public:
    void tick();

    FullBetaCPU(Params &params);
    ~FullBetaCPU();

    void init();

    void fullCPURegStats();

    void activateContext(int thread_num, int delay);
    void suspendContext(int thread_num);
    void deallocateContext(int thread_num);
    void haltContext(int thread_num);

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

    /** Get the current instruction sequence number, and increment it. */
    InstSeqNum getAndIncrementInstSeq();

#ifdef FULL_SYSTEM
    /** Check if this address is a valid instruction address. */
    bool validInstAddr(Addr addr) { return true; }

    /** Check if this address is a valid data address. */
    bool validDataAddr(Addr addr) { return true; }

    /** Get instruction asid. */
    int getInstAsid() { return ITB_ASN_ASN(regs.ipr[ISA::IPR_ITB_ASN]); }

    /** Get data asid. */
    int getDataAsid() { return DTB_ASN_ASN(regs.ipr[ISA::IPR_DTB_ASN]); }
#else
    bool validInstAddr(Addr addr)
    { return process->validInstAddr(addr); }

    bool validDataAddr(Addr addr)
    { return process->validDataAddr(addr); }

    int getInstAsid() { return asid; }
    int getDataAsid() { return asid; }

#endif

    //
    // New accessors for new decoder.
    //
    uint64_t readIntReg(int reg_idx);

    float readFloatRegSingle(int reg_idx);

    double readFloatRegDouble(int reg_idx);

    uint64_t readFloatRegInt(int reg_idx);

    void setIntReg(int reg_idx, uint64_t val);

    void setFloatRegSingle(int reg_idx, float val);

    void setFloatRegDouble(int reg_idx, double val);

    void setFloatRegInt(int reg_idx, uint64_t val);

    uint64_t readPC();

    void setNextPC(uint64_t val);

    void setPC(Addr new_PC);

    /** Function to add instruction onto the head of the list of the
     *  instructions.  Used when new instructions are fetched.
     */
    void addInst(DynInstPtr &inst);

    /** Function to tell the CPU that an instruction has completed. */
    void instDone();

    /** Remove all instructions in back of the given instruction, but leave
     *  that instruction in the list.  This is useful in a squash, when there
     *  are instructions in this list that don't exist in structures such as
     *  the ROB.  The instruction doesn't have to be the last instruction in
     *  the list, but will be once this function completes.
     *  @todo: Remove only up until that inst?  Squashed inst is most likely
     *  valid.
     */
    void removeBackInst(DynInstPtr &inst);

    /** Remove an instruction from the front of the list.  It is expected
     *  that there are no instructions in front of it (that is, none are older
     *  than the instruction being removed).  Used when retiring instructions.
     *  @todo: Remove the argument to this function, and just have it remove
     *  last instruction once it's verified that commit has the same ordering
     *  as the instruction list.
     */
    void removeFrontInst(DynInstPtr &inst);

    /** Remove all instructions that are not currently in the ROB. */
    void removeInstsNotInROB();

    /** Remove all instructions younger than the given sequence number. */
    void removeInstsUntil(const InstSeqNum &seq_num);

    /** Remove all instructions from the list. */
    void removeAllInsts();

    void dumpInsts();

    /** Basically a wrapper function so that instructions executed at
     *  commit can tell the instruction queue that they have completed.
     *  Eventually this hack should be removed.
     */
    void wakeDependents(DynInstPtr &inst);

  public:
    /** List of all the instructions in flight. */
    list<DynInstPtr> instList;

    //not sure these should be private.
  protected:
    /** The fetch stage. */
    typename CPUPolicy::Fetch fetch;

    /** The fetch stage's status. */
    typename CPUPolicy::Fetch::Status fetchStatus;

    /** The decode stage. */
    typename CPUPolicy::Decode decode;

    /** The decode stage's status. */
    typename CPUPolicy::Decode::Status decodeStatus;

    /** The dispatch stage. */
    typename CPUPolicy::Rename rename;

    /** The dispatch stage's status. */
    typename CPUPolicy::Rename::Status renameStatus;

    /** The issue/execute/writeback stages. */
    typename CPUPolicy::IEW iew;

    /** The issue/execute/writeback stage's status. */
    typename CPUPolicy::IEW::Status iewStatus;

    /** The commit stage. */
    typename CPUPolicy::Commit commit;

    /** The fetch stage's status. */
    typename CPUPolicy::Commit::Status commitStatus;

    //Might want to just pass these objects in to the constructors of the
    //appropriate stage.  regFile is in iew, freeList in dispatch, renameMap
    //in dispatch, and the rob in commit.
    /** The register file. */
    typename CPUPolicy::RegFile regFile;

    /** The free list. */
    typename CPUPolicy::FreeList freeList;

    /** The rename map. */
    typename CPUPolicy::RenameMap renameMap;

    /** The re-order buffer. */
    typename CPUPolicy::ROB rob;

  public:
    /** Typedefs from the Impl to get the structs that each of the
     *  time buffers should use.
     */
    typedef typename CPUPolicy::TimeStruct TimeStruct;

    typedef typename CPUPolicy::FetchStruct FetchStruct;

    typedef typename CPUPolicy::DecodeStruct DecodeStruct;

    typedef typename CPUPolicy::RenameStruct RenameStruct;

    typedef typename CPUPolicy::IEWStruct IEWStruct;

    /** The main time buffer to do backwards communication. */
    TimeBuffer<TimeStruct> timeBuffer;

    /** The fetch stage's instruction queue. */
    TimeBuffer<FetchStruct> fetchQueue;

    /** The decode stage's instruction queue. */
    TimeBuffer<DecodeStruct> decodeQueue;

    /** The rename stage's instruction queue. */
    TimeBuffer<RenameStruct> renameQueue;

    /** The IEW stage's instruction queue. */
    TimeBuffer<IEWStruct> iewQueue;

  public:
    /** The temporary exec context to support older accessors. */
    ExecContext *xc;

    /** Temporary function to get pointer to exec context. */
    ExecContext *xcBase() { return xc; }

    InstSeqNum globalSeqNum;

#ifdef FULL_SYSTEM
    System *system;

    MemoryController *memCtrl;
    PhysicalMemory *physmem;

    AlphaITB *itb;
    AlphaDTB *dtb;

//    SWContext *swCtx;
#else
    Process *process;

    // Address space ID.  Note that this is used for TIMING cache
    // simulation only; all functional memory accesses should use
    // one of the FunctionalMemory pointers above.
    short asid;
#endif

    FunctionalMemory *mem;

    MemInterface *icacheInterface;
    MemInterface *dcacheInterface;

    bool deferRegistration;

    Counter numInsts;

    Counter funcExeInst;
};

#endif
