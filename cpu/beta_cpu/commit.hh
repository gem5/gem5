// Todo: Maybe have a special method for handling interrupts/traps.
//
// Traps:  Have IEW send a signal to commit saying that there's a trap to
// be handled.  Have commit send the PC back to the fetch stage, along
// with the current commit PC.  Fetch will directly access the IPR and save
// off all the proper stuff.  Commit can send out a squash, or something
// close to it.
// Do the same for hwrei().  However, requires that commit be specifically
// built to support that kind of stuff.  Probably not horrible to have
// commit support having the CPU tell it to squash the other stages and
// restart at a given address.  The IPR register does become an issue.
// Probably not a big deal if the IPR stuff isn't cycle accurate.  Can just
// have the original function handle writing to the IPR register.

#ifndef __CPU_BETA_CPU_SIMPLE_COMMIT_HH__
#define __CPU_BETA_CPU_SIMPLE_COMMIT_HH__

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "mem/memory_interface.hh"

template<class Impl>
class SimpleCommit
{
  public:
    // Typedefs from the Impl.
    typedef typename Impl::ISA ISA;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::Params Params;
    typedef typename Impl::CPUPol CPUPol;

    typedef typename CPUPol::ROB ROB;

    typedef typename CPUPol::TimeStruct TimeStruct;
    typedef typename CPUPol::IEWStruct IEWStruct;
    typedef typename CPUPol::RenameStruct RenameStruct;

  public:
    // I don't believe commit can block, so it will only have two
    // statuses for now.
    // Actually if there's a cache access that needs to block (ie
    // uncachable load or just a mem access in commit) then the stage
    // may have to wait.
    enum Status {
        Running,
        Idle,
        ROBSquashing,
        DcacheMissStall,
        DcacheMissComplete
    };

  private:
    Status _status;

  public:
    SimpleCommit(Params &params);

    void regStats();

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    void setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr);

    void setROB(ROB *rob_ptr);

    void tick();

    void commit();

    uint64_t readCommitPC();

    void setSquashing() { _status = ROBSquashing; }

  private:

    void commitInsts();

    bool commitHead(DynInstPtr &head_inst, unsigned inst_num);

    void getInsts();

    void markCompletedInsts();

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to write information heading to previous stages. */
    typename TimeBuffer<TimeStruct>::wire toIEW;

    /** Wire to read information from IEW (for ROB). */
    typename TimeBuffer<TimeStruct>::wire robInfoFromIEW;

    /** IEW instruction queue interface. */
    TimeBuffer<IEWStruct> *iewQueue;

    /** Wire to read information from IEW queue. */
    typename TimeBuffer<IEWStruct>::wire fromIEW;

    /** Rename instruction queue interface, for ROB. */
    TimeBuffer<RenameStruct> *renameQueue;

    /** Wire to read information from rename queue. */
    typename TimeBuffer<RenameStruct>::wire fromRename;

    /** ROB interface. */
    ROB *rob;

    /** Pointer to FullCPU. */
    FullCPU *cpu;

    /** Pointer to the rename map.  DO NOT USE if possible. */
//    typename Impl::CPUPol::RenameMap *renameMap;

    //Store buffer interface?  Will need to move committed stores to the
    //store buffer

    /** Memory interface.  Used for d-cache accesses. */
    MemInterface *dcacheInterface;

  private:
    /** IEW to Commit delay, in ticks. */
    unsigned iewToCommitDelay;

    /** Rename to ROB delay, in ticks. */
    unsigned renameToROBDelay;

    /** Rename width, in instructions.  Used so ROB knows how many
     *  instructions to get from the rename instruction queue.
     */
    unsigned renameWidth;

    /** IEW width, in instructions.  Used so ROB knows how many
     *  instructions to get from the IEW instruction queue.
     */
    unsigned iewWidth;

    /** Commit width, in instructions. */
    unsigned commitWidth;

    Stats::Scalar<> commitCommittedInsts;
    Stats::Scalar<> commitSquashedInsts;
    Stats::Scalar<> commitSquashEvents;
    Stats::Scalar<> commitNonSpecStalls;
    Stats::Scalar<> commitCommittedBranches;
    Stats::Scalar<> commitCommittedLoads;
    Stats::Scalar<> commitCommittedMemRefs;
    Stats::Scalar<> branchMispredicts;

    Stats::Distribution<> n_committed_dist;
};

#endif // __CPU_BETA_CPU_SIMPLE_COMMIT_HH__
