#ifndef __INST_QUEUE_HH__
#define __INST_QUEUE_HH__

#include <list>
#include <map>
#include <queue>
#include <stdint.h>
#include <vector>

#include "base/timebuf.hh"
#include "cpu/inst_seq.hh"

//Perhaps have a better separation between the data structure underlying
//and the actual algorithm.
//somewhat nasty to try to have a nice ordering.
// Consider moving to STL list or slist for the LL stuff.

/**
 * A standard instruction queue class.  It holds instructions in an
 * array, holds the ordering of the instructions within a linked list,
 * and tracks producer/consumer dependencies within a separate linked
 * list.  Similar to the rename map and the free list, it expects that
 * floating point registers have their indices start after the integer
 * registers (ie with 96 int and 96 fp registers, regs 0-95 are integer
 * and 96-191 are fp).  This remains true even for both logical and
 * physical register indices.
 */
template <class Impl>
class InstructionQueue
{
  public:
    //Typedefs from the Impl.
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::Params Params;

    typedef typename Impl::CPUPol::MemDepUnit MemDepUnit;
    typedef typename Impl::CPUPol::IssueStruct IssueStruct;
    typedef typename Impl::CPUPol::TimeStruct TimeStruct;

    // Typedef of iterator through the list of instructions.  Might be
    // better to untie this from the FullCPU or pass its information to
    // the stages.
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    /**
     * Struct for comparing entries to be added to the priority queue.  This
     * gives reverse ordering to the instructions in terms of sequence
     * numbers: the instructions with smaller sequence numbers (and hence
     * are older) will be at the top of the priority queue.
     */
    struct pqCompare
    {
        bool operator() (const DynInstPtr &lhs, const DynInstPtr &rhs) const
        {
            return lhs->seqNum > rhs->seqNum;
        }
    };

    /**
     * Struct for comparing entries to be added to the set.  This gives
     * standard ordering in terms of sequence numbers.
     */
    struct setCompare
    {
        bool operator() (const DynInstPtr &lhs, const DynInstPtr &rhs) const
        {
            return lhs->seqNum < rhs->seqNum;
        }
    };

    typedef std::priority_queue<DynInstPtr, vector<DynInstPtr>, pqCompare>
    ReadyInstQueue;

    InstructionQueue(Params &params);

    void setCPU(FullCPU *cpu);

    void setIssueToExecuteQueue(TimeBuffer<IssueStruct> *i2eQueue);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    unsigned numFreeEntries();

    bool isFull();

    void insert(DynInstPtr &new_inst);

    void insertNonSpec(DynInstPtr &new_inst);

    void advanceTail(DynInstPtr &inst);

    void scheduleReadyInsts();

    void scheduleNonSpec(const InstSeqNum &inst);

    void wakeDependents(DynInstPtr &completed_inst);

    void violation(DynInstPtr &store, DynInstPtr &faulting_load);

    void squash();

    void doSquash();

    void stopSquash();

    /** Debugging function to dump all the list sizes, as well as print
     *  out the list of nonspeculative instructions.  Should not be used
     *  in any other capacity, but it has no harmful sideaffects.
     */
    void dumpLists();

  private:
    /** Debugging function to count how many entries are in the IQ.  It does
     *  a linear walk through the instructions, so do not call this function
     *  during normal execution.
     */
    int countInsts();

  private:
    /** Pointer to the CPU. */
    FullCPU *cpu;

    /** The memory dependence unit, which tracks/predicts memory dependences
     *  between instructions.
     */
    MemDepUnit memDepUnit;

    /** The queue to the execute stage.  Issued instructions will be written
     *  into it.
     */
    TimeBuffer<IssueStruct> *issueToExecuteQueue;

    /** The backwards time buffer. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to read information from timebuffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    enum InstList {
        Int,
        Float,
        Branch,
        Memory,
        Misc,
        Squashed,
        None
    };

    /** List of ready int instructions.  Used to keep track of the order in
     *  which instructions should issue.
     */
    ReadyInstQueue readyIntInsts;

    /** List of ready floating point instructions. */
    ReadyInstQueue readyFloatInsts;

    /** List of ready branch instructions. */
    ReadyInstQueue readyBranchInsts;

    /** List of ready memory instructions. */
    ReadyInstQueue readyMemInsts;

    /** List of ready miscellaneous instructions. */
    ReadyInstQueue readyMiscInsts;

    /** List of squashed instructions (which are still valid and in IQ).
     *  Implemented using a priority queue; the entries must contain both
     *  the IQ index and sequence number of each instruction so that
     *  ordering based on sequence numbers can be used.
     */
    ReadyInstQueue squashedInsts;

    /** List of non-speculative instructions that will be scheduled
     *  once the IQ gets a signal from commit.  While it's redundant to
     *  have the key be a part of the value (the sequence number is stored
     *  inside of DynInst), when these instructions are woken up only
     *  the sequence number will be available.  Thus it is necessary to be
     *  able to search by the sequence number alone.
     */
    std::map<InstSeqNum, DynInstPtr> nonSpecInsts;

    typedef typename std::map<InstSeqNum, DynInstPtr>::iterator non_spec_it_t;

    /** Number of free IQ entries left. */
    unsigned freeEntries;

    /** The number of entries in the instruction queue. */
    unsigned numEntries;

    /** The number of integer instructions that can be issued in one
     *  cycle.
     */
    unsigned intWidth;

    /** The number of floating point instructions that can be issued
     *  in one cycle.
     */
    unsigned floatWidth;

    /** The number of branches that can be issued in one cycle. */
    unsigned branchWidth;

    /** The number of memory instructions that can be issued in one cycle. */
    unsigned memoryWidth;

    /** The total number of instructions that can be issued in one cycle. */
    unsigned totalWidth;

    //The number of physical registers in the CPU.
    unsigned numPhysRegs;

    /** The number of physical integer registers in the CPU. */
    unsigned numPhysIntRegs;

    /** The number of floating point registers in the CPU. */
    unsigned numPhysFloatRegs;

    /** Delay between commit stage and the IQ.
     *  @todo: Make there be a distinction between the delays within IEW.
     */
    unsigned commitToIEWDelay;

    //////////////////////////////////
    // Variables needed for squashing
    //////////////////////////////////

    /** The sequence number of the squashed instruction. */
    InstSeqNum squashedSeqNum;

    /** Iterator that points to the oldest instruction in the IQ. */
//    ListIt head;

    /** Iterator that points to the youngest instruction in the IQ. */
    ListIt tail;

    /** Iterator that points to the last instruction that has been squashed.
     *  This will not be valid unless the IQ is in the process of squashing.
     */
    ListIt squashIt;

    ///////////////////////////////////
    // Dependency graph stuff
    ///////////////////////////////////

    class DependencyEntry
    {
      public:
        DynInstPtr inst;
        //Might want to include data about what arch. register the
        //dependence is waiting on.
        DependencyEntry *next;

        //This function, and perhaps this whole class, stand out a little
        //bit as they don't fit a classification well.  I want access
        //to the underlying structure of the linked list, yet at
        //the same time it feels like this should be something abstracted
        //away.  So for now it will sit here, within the IQ, until
        //a better implementation is decided upon.
        // This function probably shouldn't be within the entry...
        void insert(DynInstPtr &new_inst);

        void remove(DynInstPtr &inst_to_remove);
    };

    /** Array of linked lists.  Each linked list is a list of all the
     *  instructions that depend upon a given register.  The actual
     *  register's index is used to index into the graph; ie all
     *  instructions in flight that are dependent upon r34 will be
     *  in the linked list of dependGraph[34].
     */
    DependencyEntry *dependGraph;

    /** A cache of the recently woken registers.  It is 1 if the register
     *  has been woken up recently, and 0 if the register has been added
     *  to the dependency graph and has not yet received its value.  It
     *  is basically a secondary scoreboard, and should pretty much mirror
     *  the scoreboard that exists in the rename map.
     */
    vector<bool> regScoreboard;

    bool addToDependents(DynInstPtr &new_inst);
    void insertDependency(DynInstPtr &new_inst);
    void createDependency(DynInstPtr &new_inst);
    void dumpDependGraph();

    void addIfReady(DynInstPtr &inst);
};

#endif //__INST_QUEUE_HH__
