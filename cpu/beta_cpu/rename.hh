// Todo:
// Figure out rename map for reg vs fp (probably just have one rename map).
// In simple case, there is no renaming, so have this stage do basically
// nothing.
// Fix up trap and barrier handling.  Fix up squashing too, as it's too
// dependent upon the iew stage continually telling it to squash.
// Have commit send back information whenever a branch has committed.  This
// way the history buffer can be cleared beyond the point where the branch
// was.

#ifndef __SIMPLE_RENAME_HH__
#define __SIMPLE_RENAME_HH__

//Will want to include: time buffer, structs, free list, rename map
#include <list>

#include "base/timebuf.hh"
#include "cpu/beta_cpu/comm.hh"
#include "cpu/beta_cpu/rename_map.hh"
#include "cpu/beta_cpu/free_list.hh"

using namespace std;

// Will need rename maps for both the int reg file and fp reg file.
// Or change rename map class to handle both. (RegFile handles both.)
template<class Impl>
class SimpleRename
{
  public:
    // Typedefs from the Impl.
    typedef typename Impl::ISA ISA;
    typedef typename Impl::CPUPol CPUPol;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::Params Params;

    typedef typename Impl::FetchStruct FetchStruct;
    typedef typename Impl::DecodeStruct DecodeStruct;
    typedef typename Impl::RenameStruct RenameStruct;
    typedef typename Impl::TimeStruct TimeStruct;

    // Typedefs from the CPUPol
    typedef typename CPUPol::FreeList FreeList;
    typedef typename CPUPol::RenameMap RenameMap;

    // Typedefs from the ISA.
    typedef typename ISA::Addr Addr;

  public:
    // Rename will block if ROB becomes full or issue queue becomes full,
    // or there are no free registers to rename to.
    // Only case where rename squashes is if IEW squashes.
    enum Status {
        Running,
        Idle,
        Squashing,
        Blocked,
        Unblocking,
        BarrierStall
    };

  private:
    Status _status;

  public:
    SimpleRename(Params &params);

    void setCPU(FullCPU *cpu_ptr);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    void setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr);

    void setRenameMap(RenameMap *rm_ptr);

    void setFreeList(FreeList *fl_ptr);

    void dumpHistory();

    void tick();

    void rename();

    void squash();

  private:
    void block();

    inline void unblock();

    void doSquash();

    void removeFromHistory(InstSeqNum inst_seq_num);

    /** Holds the previous information for each rename.
     *  Note that often times the inst may have been deleted, so only access
     *  the pointer for the address and do not dereference it.
     */
    struct RenameHistory {
        RenameHistory(InstSeqNum _instSeqNum, RegIndex _archReg,
                      PhysRegIndex _newPhysReg, PhysRegIndex _prevPhysReg)
            : instSeqNum(_instSeqNum), archReg(_archReg),
              newPhysReg(_newPhysReg), prevPhysReg(_prevPhysReg),
              placeHolder(false)
        {
        }

        /** Constructor used specifically for cases where a place holder
         *  rename history entry is being made.
         */
        RenameHistory(InstSeqNum _instSeqNum)
            : instSeqNum(_instSeqNum), archReg(0), newPhysReg(0),
              prevPhysReg(0), placeHolder(true)
        {
        }

        InstSeqNum instSeqNum;
        RegIndex archReg;
        PhysRegIndex newPhysReg;
        PhysRegIndex prevPhysReg;
        bool placeHolder;
    };

    list<RenameHistory> historyBuffer;

    /** CPU interface. */
    FullCPU *cpu;

    // Interfaces to objects outside of rename.
    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get IEW's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire to write infromation heading to previous stages. */
    // Might not be the best name as not only decode will read it.
    typename TimeBuffer<TimeStruct>::wire toDecode;

    /** Rename instruction queue. */
    TimeBuffer<RenameStruct> *renameQueue;

    /** Wire to write any information heading to IEW. */
    typename TimeBuffer<RenameStruct>::wire toIEW;

    /** Decode instruction queue interface. */
    TimeBuffer<DecodeStruct> *decodeQueue;

    /** Wire to get decode's output from decode queue. */
    typename TimeBuffer<DecodeStruct>::wire fromDecode;

    /** Skid buffer between rename and decode. */
    queue<DecodeStruct> skidBuffer;

    /** Rename map interface. */
    SimpleRenameMap *renameMap;

    /** Free list interface. */
    FreeList *freeList;

    /** Delay between iew and rename, in ticks. */
    int iewToRenameDelay;

    /** Delay between decode and rename, in ticks. */
    int decodeToRenameDelay;

    /** Delay between commit and rename, in ticks. */
    unsigned commitToRenameDelay;

    /** Rename width, in instructions. */
    unsigned renameWidth;

    /** Commit width, in instructions.  Used so rename knows how many
     *  instructions might have freed registers in the previous cycle.
     */
    unsigned commitWidth;
};

#endif // __SIMPLE_RENAME_HH__
