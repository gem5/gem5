#ifndef __INST_QUEUE_HH__
#define __INST_QUEUE_HH__

#include <list>
#include <queue>
#include <stdint.h>

#include "base/timebuf.hh"

using namespace std;

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
template<class Impl>
class InstructionQueue
{
  public:
    //Typedefs from the Impl.
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::Params Params;

    typedef typename Impl::IssueStruct IssueStruct;
    typedef typename Impl::TimeStruct TimeStruct;

    // Typedef of iterator through the list of instructions.  Might be
    // better to untie this from the FullCPU or pass its information to
    // the stages.
    typedef typename list<DynInst *>::iterator ListIt;

    /**
     * Class for priority queue entries.  Mainly made so that the < operator
     * is defined.
     */
    struct ReadyEntry {
        DynInst *inst;

        ReadyEntry(DynInst *_inst)
            : inst(_inst)
        { }

        /** Compare(lhs,rhs) checks if rhs is "bigger" than lhs.  If so, rhs
         *  goes higher on the priority queue.  The oldest instruction should
         *  be on the top of the instruction queue, so in this case "bigger"
         *  has the reverse meaning; the instruction with the lowest
         *  sequence number is on the top.
         */
        bool operator <(const ReadyEntry &rhs) const
        {
            if (this->inst->seqNum > rhs.inst->seqNum)
                return true;
            return false;
        }
    };

    InstructionQueue(Params &params);

    void setCPU(FullCPU *cpu);

    void setIssueToExecuteQueue(TimeBuffer<IssueStruct> *i2eQueue);

    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    unsigned numFreeEntries();

    bool isFull();

    void insert(DynInst *new_inst);

    void advanceTail(DynInst *inst);

    void scheduleReadyInsts();

    void wakeDependents(DynInst *completed_inst);

    void doSquash();

    void squash();

    void stopSquash();

  private:
    /** Debugging function to count how many entries are in the IQ.  It does
     *  a linear walk through the instructions, so do not call this function
     *  during normal execution.
     */
    int countInsts();

  private:
    /** Pointer to the CPU. */
    FullCPU *cpu;

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
        Squashed,
        None
    };

    /** List of ready int instructions.  Used to keep track of the order in
     *  which */
    priority_queue<ReadyEntry> readyIntInsts;

    /** List of ready floating point instructions. */
    priority_queue<ReadyEntry> readyFloatInsts;

    /** List of ready branch instructions. */
    priority_queue<ReadyEntry> readyBranchInsts;

    /** List of squashed instructions (which are still valid and in IQ).
     *  Implemented using a priority queue; the entries must contain both
     *  the IQ index and sequence number of each instruction so that
     *  ordering based on sequence numbers can be used.
     */
    priority_queue<ReadyEntry> squashedInsts;

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
    ListIt head;

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
        DynInst *inst;
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
        void insert(DynInst *new_inst);

        void remove(DynInst *inst_to_remove);
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

    bool addToDependents(DynInst *new_inst);
    void insertDependency(DynInst *new_inst);
    void createDependency(DynInst *new_inst);

    void addIfReady(DynInst *inst);
};

#endif //__INST_QUEUE_HH__
