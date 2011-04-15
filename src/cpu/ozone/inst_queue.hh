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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_OZONE_INST_QUEUE_HH__
#define __CPU_OZONE_INST_QUEUE_HH__

#include <list>
#include <map>
#include <queue>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/timebuf.hh"

class FUPool;
class MemInterface;

/**
 * A standard instruction queue class.  It holds ready instructions, in
 * order, in seperate priority queues to facilitate the scheduling of
 * instructions.  The IQ uses a separate linked list to track dependencies.
 * Similar to the rename map and the free list, it expects that
 * floating point registers have their indices start after the integer
 * registers (ie with 96 int and 96 fp registers, regs 0-95 are integer
 * and 96-191 are fp).  This remains true even for both logical and
 * physical register indices. The IQ depends on the memory dependence unit to
 * track when memory operations are ready in terms of ordering; register
 * dependencies are tracked normally. Right now the IQ also handles the
 * execution timing; this is mainly to allow back-to-back scheduling without
 * requiring IEW to be able to peek into the IQ. At the end of the execution
 * latency, the instruction is put into the queue to execute, where it will
 * have the execute() function called on it.
 * @todo: Make IQ able to handle multiple FU pools.
 */
template <class Impl>
class InstQueue
{
  public:
    //Typedefs from the Impl.
    typedef typename Impl::FullCPU FullCPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::Params Params;
    typedef typename Impl::IssueStruct IssueStruct;
/*
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::MemDepUnit MemDepUnit;
    typedef typename Impl::CPUPol::IssueStruct IssueStruct;
    typedef typename Impl::CPUPol::TimeStruct TimeStruct;
*/
    // Typedef of iterator through the list of instructions.
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    friend class Impl::FullCPU;
#if 0
    /** FU completion event class. */
    class FUCompletion : public Event {
      private:
        /** Executing instruction. */
        DynInstPtr inst;

        /** Index of the FU used for executing. */
        int fuIdx;

        /** Pointer back to the instruction queue. */
        InstQueue<Impl> *iqPtr;

      public:
        /** Construct a FU completion event. */
        FUCompletion(DynInstPtr &_inst, int fu_idx,
                     InstQueue<Impl> *iq_ptr);

        virtual void process();
        virtual const char *description() const;
    };
#endif
    /** Constructs an IQ. */
    InstQueue(Params *params);

    /** Destructs the IQ. */
    ~InstQueue();

    /** Returns the name of the IQ. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets CPU pointer. */
    void setCPU(FullCPU *_cpu) { cpu = _cpu; }
#if 0
    /** Sets active threads list. */
    void setActiveThreads(list<unsigned> *at_ptr);

    /** Sets the IEW pointer. */
    void setIEW(IEW *iew_ptr) { iewStage = iew_ptr; }
#endif
    /** Sets the timer buffer between issue and execute. */
    void setIssueToExecuteQueue(TimeBuffer<IssueStruct> *i2eQueue);
#if 0
    /** Sets the global time buffer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Number of entries needed for given amount of threads. */
    int entryAmount(ThreadID num_threads);

    /** Resets max entries for all threads. */
    void resetEntries();
#endif
    /** Returns total number of free entries. */
    unsigned numFreeEntries();

    /** Returns number of free entries for a thread. */
    unsigned numFreeEntries(ThreadID tid);

    /** Returns whether or not the IQ is full. */
    bool isFull();

    /** Returns whether or not the IQ is full for a specific thread. */
    bool isFull(ThreadID tid);

    /** Returns if there are any ready instructions in the IQ. */
    bool hasReadyInsts();

    /** Inserts a new instruction into the IQ. */
    void insert(DynInstPtr &new_inst);

    /** Inserts a new, non-speculative instruction into the IQ. */
    void insertNonSpec(DynInstPtr &new_inst);
#if 0
    /**
     * Advances the tail of the IQ, used if an instruction is not added to the
     * IQ for scheduling.
     * @todo: Rename this function.
     */
    void advanceTail(DynInstPtr &inst);

    /** Process FU completion event. */
    void processFUCompletion(DynInstPtr &inst, int fu_idx);
#endif
    /**
     * Schedules ready instructions, adding the ready ones (oldest first) to
     * the queue to execute.
     */
    void scheduleReadyInsts();

    /** Schedules a single specific non-speculative instruction. */
    void scheduleNonSpec(const InstSeqNum &inst);

    /**
     * Commits all instructions up to and including the given sequence number,
     * for a specific thread.
     */
    void commit(const InstSeqNum &inst, ThreadID tid = 0);

    /** Wakes all dependents of a completed instruction. */
    void wakeDependents(DynInstPtr &completed_inst);

    /** Adds a ready memory instruction to the ready list. */
    void addReadyMemInst(DynInstPtr &ready_inst);
#if 0
    /**
     * Reschedules a memory instruction. It will be ready to issue once
     * replayMemInst() is called.
     */
    void rescheduleMemInst(DynInstPtr &resched_inst);

    /** Replays a memory instruction. It must be rescheduled first. */
    void replayMemInst(DynInstPtr &replay_inst);
#endif
    /** Completes a memory operation. */
    void completeMemInst(DynInstPtr &completed_inst);
#if 0
    /** Indicates an ordering violation between a store and a load. */
    void violation(DynInstPtr &store, DynInstPtr &faulting_load);
#endif
    /**
     * Squashes instructions for a thread. Squashing information is obtained
     * from the time buffer.
     */
    void squash(ThreadID tid); // Probably want the ISN

    /** Returns the number of used entries for a thread. */
    unsigned getCount(ThreadID tid) { return count[tid]; };

    /** Updates the number of free entries. */
    void updateFreeEntries(int num) { freeEntries += num; }

    /** Debug function to print all instructions. */
    void printInsts();

  private:
    /** Does the actual squashing. */
    void doSquash(ThreadID tid);

    /////////////////////////
    // Various pointers
    /////////////////////////

    /** Pointer to the CPU. */
    FullCPU *cpu;

    /** Cache interface. */
    MemInterface *dcacheInterface;
#if 0
    /** Pointer to IEW stage. */
    IEW *iewStage;

    /** The memory dependence unit, which tracks/predicts memory dependences
     *  between instructions.
     */
    MemDepUnit memDepUnit[Impl::MaxThreads];
#endif
    /** The queue to the execute stage.  Issued instructions will be written
     *  into it.
     */
    TimeBuffer<IssueStruct> *issueToExecuteQueue;
#if 0
    /** The backwards time buffer. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to read information from timebuffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Function unit pool. */
    FUPool *fuPool;
#endif
    //////////////////////////////////////
    // Instruction lists, ready queues, and ordering
    //////////////////////////////////////

    /** List of all the instructions in the IQ (some of which may be issued). */
    std::list<DynInstPtr> instList[Impl::MaxThreads];

    /**
     * Struct for comparing entries to be added to the priority queue.  This
     * gives reverse ordering to the instructions in terms of sequence
     * numbers: the instructions with smaller sequence numbers (and hence
     * are older) will be at the top of the priority queue.
     */
    struct pqCompare {
        bool operator() (const DynInstPtr &lhs, const DynInstPtr &rhs) const
        {
            return lhs->seqNum > rhs->seqNum;
        }
    };

    /**
     * Struct for an IQ entry. It includes the instruction and an iterator
     * to the instruction's spot in the IQ.
     */
    struct IQEntry {
        DynInstPtr inst;
        ListIt iqIt;
    };

    typedef std::priority_queue<DynInstPtr, std::vector<DynInstPtr>, pqCompare>
    ReadyInstQueue;

    typedef std::map<DynInstPtr, pqCompare> ReadyInstMap;
    typedef typename std::map<DynInstPtr, pqCompare>::iterator ReadyMapIt;

    /** List of ready instructions.
     */
    ReadyInstQueue readyInsts;

    /** List of non-speculative instructions that will be scheduled
     *  once the IQ gets a signal from commit.  While it's redundant to
     *  have the key be a part of the value (the sequence number is stored
     *  inside of DynInst), when these instructions are woken up only
     *  the sequence number will be available.  Thus it is most efficient to be
     *  able to search by the sequence number alone.
     */
    std::map<InstSeqNum, DynInstPtr> nonSpecInsts;

    typedef typename std::map<InstSeqNum, DynInstPtr>::iterator NonSpecMapIt;
#if 0
    /** Entry for the list age ordering by op class. */
    struct ListOrderEntry {
        OpClass queueType;
        InstSeqNum oldestInst;
    };

    /** List that contains the age order of the oldest instruction of each
     *  ready queue.  Used to select the oldest instruction available
     *  among op classes.
     */
    std::list<ListOrderEntry> listOrder;

    typedef typename std::list<ListOrderEntry>::iterator ListOrderIt;

    /** Tracks if each ready queue is on the age order list. */
    bool queueOnList[Num_OpClasses];

    /** Iterators of each ready queue.  Points to their spot in the age order
     *  list.
     */
    ListOrderIt readyIt[Num_OpClasses];

    /** Add an op class to the age order list. */
    void addToOrderList(OpClass op_class);

    /**
     * Called when the oldest instruction has been removed from a ready queue;
     * this places that ready queue into the proper spot in the age order list.
     */
    void moveToYoungerInst(ListOrderIt age_order_it);
#endif
    //////////////////////////////////////
    // Various parameters
    //////////////////////////////////////
#if 0
    /** IQ Resource Sharing Policy */
    enum IQPolicy {
        Dynamic,
        Partitioned,
        Threshold
    };

    /** IQ sharing policy for SMT. */
    IQPolicy iqPolicy;
#endif
    /** Number of Total Threads*/
    unsigned numThreads;
#if 0
    /** Pointer to list of active threads. */
    list<unsigned> *activeThreads;
#endif
    /** Per Thread IQ count */
    unsigned count[Impl::MaxThreads];

    /** Max IQ Entries Per Thread */
    unsigned maxEntries[Impl::MaxThreads];

    /** Number of free IQ entries left. */
    unsigned freeEntries;

    /** The number of entries in the instruction queue. */
    unsigned numEntries;

    /** The total number of instructions that can be issued in one cycle. */
    unsigned totalWidth;
#if 0
    /** The number of physical registers in the CPU. */
    unsigned numPhysRegs;

    /** The number of physical integer registers in the CPU. */
    unsigned numPhysIntRegs;

    /** The number of floating point registers in the CPU. */
    unsigned numPhysFloatRegs;
#endif
    /** Delay between commit stage and the IQ.
     *  @todo: Make there be a distinction between the delays within IEW.
     */
    unsigned commitToIEWDelay;

    //////////////////////////////////
    // Variables needed for squashing
    //////////////////////////////////

    /** The sequence number of the squashed instruction. */
    InstSeqNum squashedSeqNum[Impl::MaxThreads];

    /** Iterator that points to the last instruction that has been squashed.
     *  This will not be valid unless the IQ is in the process of squashing.
     */
    ListIt squashIt[Impl::MaxThreads];
#if 0
    ///////////////////////////////////
    // Dependency graph stuff
    ///////////////////////////////////

    class DependencyEntry
    {
      public:
        DependencyEntry()
            : inst(NULL), next(NULL)
        { }

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

        // Debug variable, remove when done testing.
        static unsigned mem_alloc_counter;
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

    /** Adds an instruction to the dependency graph, as a producer. */
    bool addToDependents(DynInstPtr &new_inst);

    /** Adds an instruction to the dependency graph, as a consumer. */
    void createDependency(DynInstPtr &new_inst);
#endif
    /** Moves an instruction to the ready queue if it is ready. */
    void addIfReady(DynInstPtr &inst);

    /** Debugging function to count how many entries are in the IQ.  It does
     *  a linear walk through the instructions, so do not call this function
     *  during normal execution.
     */
    int countInsts();
#if 0
    /** Debugging function to dump out the dependency graph.
     */
    void dumpDependGraph();
#endif
    /** Debugging function to dump all the list sizes, as well as print
     *  out the list of nonspeculative instructions.  Should not be used
     *  in any other capacity, but it has no harmful sideaffects.
     */
    void dumpLists();

    /** Debugging function to dump out all instructions that are in the
     *  IQ.
     */
    void dumpInsts();

    /** Stat for number of instructions added. */
    Stats::Scalar iqInstsAdded;
    /** Stat for number of non-speculative instructions added. */
    Stats::Scalar iqNonSpecInstsAdded;
//    Stats::Scalar iqIntInstsAdded;
    /** Stat for number of integer instructions issued. */
    Stats::Scalar iqIntInstsIssued;
//    Stats::Scalar iqFloatInstsAdded;
    /** Stat for number of floating point instructions issued. */
    Stats::Scalar iqFloatInstsIssued;
//    Stats::Scalar iqBranchInstsAdded;
    /** Stat for number of branch instructions issued. */
    Stats::Scalar iqBranchInstsIssued;
//    Stats::Scalar iqMemInstsAdded;
    /** Stat for number of memory instructions issued. */
    Stats::Scalar iqMemInstsIssued;
//    Stats::Scalar iqMiscInstsAdded;
    /** Stat for number of miscellaneous instructions issued. */
    Stats::Scalar iqMiscInstsIssued;
    /** Stat for number of squashed instructions that were ready to issue. */
    Stats::Scalar iqSquashedInstsIssued;
    /** Stat for number of squashed instructions examined when squashing. */
    Stats::Scalar iqSquashedInstsExamined;
    /** Stat for number of squashed instruction operands examined when
     * squashing.
     */
    Stats::Scalar iqSquashedOperandsExamined;
    /** Stat for number of non-speculative instructions removed due to a squash.
     */
    Stats::Scalar iqSquashedNonSpecRemoved;

};

#endif //__CPU_OZONE_INST_QUEUE_HH__
