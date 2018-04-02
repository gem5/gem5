/*
 * Copyright (c) 2012, 2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#ifndef __CPU_O3_RENAME_HH__
#define __CPU_O3_RENAME_HH__

#include <list>
#include <utility>

#include "base/statistics.hh"
#include "config/the_isa.hh"
#include "cpu/timebuf.hh"
#include "sim/probe/probe.hh"

struct DerivO3CPUParams;

/**
 * DefaultRename handles both single threaded and SMT rename. Its
 * width is specified by the parameters; each cycle it tries to rename
 * that many instructions. It holds onto the rename history of all
 * instructions with destination registers, storing the
 * arch. register, the new physical register, and the old physical
 * register, to allow for undoing of mappings if squashing happens, or
 * freeing up registers upon commit. Rename handles blocking if the
 * ROB, IQ, or LSQ is going to be full. Rename also handles barriers,
 * and does so by stalling on the instruction until the ROB is empty
 * and there are no instructions in flight to the ROB.
 */
template<class Impl>
class DefaultRename
{
  public:
    // Typedefs from the Impl.
    typedef typename Impl::CPUPol CPUPol;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::O3CPU O3CPU;

    // Typedefs from the CPUPol
    typedef typename CPUPol::DecodeStruct DecodeStruct;
    typedef typename CPUPol::RenameStruct RenameStruct;
    typedef typename CPUPol::TimeStruct TimeStruct;
    typedef typename CPUPol::FreeList FreeList;
    typedef typename CPUPol::RenameMap RenameMap;
    // These are used only for initialization.
    typedef typename CPUPol::IEW IEW;
    typedef typename CPUPol::Commit Commit;

    // A deque is used to queue the instructions. Barrier insts must
    // be added to the front of the queue, which is the only reason for
    // using a deque instead of a queue. (Most other stages use a
    // queue)
    typedef std::deque<DynInstPtr> InstQueue;

  public:
    /** Overall rename status. Used to determine if the CPU can
     * deschedule itself due to a lack of activity.
     */
    enum RenameStatus {
        Active,
        Inactive
    };

    /** Individual thread status. */
    enum ThreadStatus {
        Running,
        Idle,
        StartSquash,
        Squashing,
        Blocked,
        Unblocking,
        SerializeStall
    };

  private:
    /** Rename status. */
    RenameStatus _status;

    /** Per-thread status. */
    ThreadStatus renameStatus[Impl::MaxThreads];

    /** Probe points. */
    typedef typename std::pair<InstSeqNum, PhysRegIdPtr> SeqNumRegPair;
    /** To probe when register renaming for an instruction is complete */
    ProbePointArg<DynInstPtr> *ppRename;
    /**
     * To probe when an instruction is squashed and the register mapping
     * for it needs to be undone
     */
    ProbePointArg<SeqNumRegPair> *ppSquashInRename;

  public:
    /** DefaultRename constructor. */
    DefaultRename(O3CPU *_cpu, DerivO3CPUParams *params);

    /** Returns the name of rename. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Registers probes. */
    void regProbePoints();

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Sets pointer to time buffer used to communicate to the next stage. */
    void setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr);

    /** Sets pointer to time buffer coming from decode. */
    void setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr);

    /** Sets pointer to IEW stage. Used only for initialization. */
    void setIEWStage(IEW *iew_stage)
    { iew_ptr = iew_stage; }

    /** Sets pointer to commit stage. Used only for initialization. */
    void setCommitStage(Commit *commit_stage)
    { commit_ptr = commit_stage; }

  private:
    /** Pointer to IEW stage. Used only for initialization. */
    IEW *iew_ptr;

    /** Pointer to commit stage. Used only for initialization. */
    Commit *commit_ptr;

  public:
    /** Initializes variables for the stage. */
    void startupStage();

    /** Clear all thread-specific states */
    void clearStates(ThreadID tid);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Sets pointer to rename maps (per-thread structures). */
    void setRenameMap(RenameMap rm_ptr[Impl::MaxThreads]);

    /** Sets pointer to the free list. */
    void setFreeList(FreeList *fl_ptr);

    /** Sets pointer to the scoreboard. */
    void setScoreboard(Scoreboard *_scoreboard);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Has the stage drained? */
    bool isDrained() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Squashes all instructions in a thread. */
    void squash(const InstSeqNum &squash_seq_num, ThreadID tid);

    /** Ticks rename, which processes all input signals and attempts to rename
     * as many instructions as possible.
     */
    void tick();

    /** Debugging function used to dump history buffer of renamings. */
    void dumpHistory();

  private:
    /** Reset this pipeline stage */
    void resetStage();

    /** Determines what to do based on rename's current status.
     * @param status_change rename() sets this variable if there was a status
     * change (ie switching from blocking to unblocking).
     * @param tid Thread id to rename instructions from.
     */
    void rename(bool &status_change, ThreadID tid);

    /** Renames instructions for the given thread. Also handles serializing
     * instructions.
     */
    void renameInsts(ThreadID tid);

    /** Inserts unused instructions from a given thread into the skid buffer,
     * to be renamed once rename unblocks.
     */
    void skidInsert(ThreadID tid);

    /** Separates instructions from decode into individual lists of instructions
     * sorted by thread.
     */
    void sortInsts();

    /** Returns if all of the skid buffers are empty. */
    bool skidsEmpty();

    /** Updates overall rename status based on all of the threads' statuses. */
    void updateStatus();

    /** Switches rename to blocking, and signals back that rename has become
     * blocked.
     * @return Returns true if there is a status change.
     */
    bool block(ThreadID tid);

    /** Switches rename to unblocking if the skid buffer is empty, and signals
     * back that rename has unblocked.
     * @return Returns true if there is a status change.
     */
    bool unblock(ThreadID tid);

    /** Executes actual squash, removing squashed instructions. */
    void doSquash(const InstSeqNum &squash_seq_num, ThreadID tid);

    /** Removes a committed instruction's rename history. */
    void removeFromHistory(InstSeqNum inst_seq_num, ThreadID tid);

    /** Renames the source registers of an instruction. */
    inline void renameSrcRegs(const DynInstPtr &inst, ThreadID tid);

    /** Renames the destination registers of an instruction. */
    inline void renameDestRegs(const DynInstPtr &inst, ThreadID tid);

    /** Calculates the number of free ROB entries for a specific thread. */
    inline int calcFreeROBEntries(ThreadID tid);

    /** Calculates the number of free IQ entries for a specific thread. */
    inline int calcFreeIQEntries(ThreadID tid);

    /** Calculates the number of free LQ entries for a specific thread. */
    inline int calcFreeLQEntries(ThreadID tid);

    /** Calculates the number of free SQ entries for a specific thread. */
    inline int calcFreeSQEntries(ThreadID tid);

    /** Returns the number of valid instructions coming from decode. */
    unsigned validInsts();

    /** Reads signals telling rename to block/unblock. */
    void readStallSignals(ThreadID tid);

    /** Checks if any stages are telling rename to block. */
    bool checkStall(ThreadID tid);

    /** Gets the number of free entries for a specific thread. */
    void readFreeEntries(ThreadID tid);

    /** Checks the signals and updates the status. */
    bool checkSignalsAndUpdate(ThreadID tid);

    /** Either serializes on the next instruction available in the InstQueue,
     * or records that it must serialize on the next instruction to enter
     * rename.
     * @param inst_list The list of younger, unprocessed instructions for the
     * thread that has the serializeAfter instruction.
     * @param tid The thread id.
     */
    void serializeAfter(InstQueue &inst_list, ThreadID tid);

    /** Holds the information for each destination register rename. It holds
     * the instruction's sequence number, the arch register, the old physical
     * register for that arch. register, and the new physical register.
     */
    struct RenameHistory {
        RenameHistory(InstSeqNum _instSeqNum, const RegId& _archReg,
                      PhysRegIdPtr _newPhysReg,
                      PhysRegIdPtr _prevPhysReg)
            : instSeqNum(_instSeqNum), archReg(_archReg),
              newPhysReg(_newPhysReg), prevPhysReg(_prevPhysReg)
        {
        }

        /** The sequence number of the instruction that renamed. */
        InstSeqNum instSeqNum;
        /** The architectural register index that was renamed. */
        RegId archReg;
        /** The new physical register that the arch. register is renamed to. */
        PhysRegIdPtr newPhysReg;
        /** The old physical register that the arch. register was renamed to.
         */
        PhysRegIdPtr prevPhysReg;
    };

    /** A per-thread list of all destination register renames, used to either
     * undo rename mappings or free old physical registers.
     */
    std::list<RenameHistory> historyBuffer[Impl::MaxThreads];

    /** Pointer to CPU. */
    O3CPU *cpu;

    /** Pointer to main time buffer used for backwards communication. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get IEW's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromIEW;

    /** Wire to get commit's output from backwards time buffer. */
    typename TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire to write infromation heading to previous stages. */
    typename TimeBuffer<TimeStruct>::wire toDecode;

    /** Rename instruction queue. */
    TimeBuffer<RenameStruct> *renameQueue;

    /** Wire to write any information heading to IEW. */
    typename TimeBuffer<RenameStruct>::wire toIEW;

    /** Decode instruction queue interface. */
    TimeBuffer<DecodeStruct> *decodeQueue;

    /** Wire to get decode's output from decode queue. */
    typename TimeBuffer<DecodeStruct>::wire fromDecode;

    /** Queue of all instructions coming from decode this cycle. */
    InstQueue insts[Impl::MaxThreads];

    /** Skid buffer between rename and decode. */
    InstQueue skidBuffer[Impl::MaxThreads];

    /** Rename map interface. */
    RenameMap *renameMap[Impl::MaxThreads];

    /** Free list interface. */
    FreeList *freeList;

    /** Pointer to the list of active threads. */
    std::list<ThreadID> *activeThreads;

    /** Pointer to the scoreboard. */
    Scoreboard *scoreboard;

    /** Count of instructions in progress that have been sent off to the IQ
     * and ROB, but are not yet included in their occupancy counts.
     */
    int instsInProgress[Impl::MaxThreads];

    /** Count of Load instructions in progress that have been sent off to the IQ
     * and ROB, but are not yet included in their occupancy counts.
     */
    int loadsInProgress[Impl::MaxThreads];

    /** Count of Store instructions in progress that have been sent off to the IQ
     * and ROB, but are not yet included in their occupancy counts.
     */
    int storesInProgress[Impl::MaxThreads];

    /** Variable that tracks if decode has written to the time buffer this
     * cycle. Used to tell CPU if there is activity this cycle.
     */
    bool wroteToTimeBuffer;

    /** Structures whose free entries impact the amount of instructions that
     * can be renamed.
     */
    struct FreeEntries {
        unsigned iqEntries;
        unsigned robEntries;
        unsigned lqEntries;
        unsigned sqEntries;
    };

    /** Per-thread tracking of the number of free entries of back-end
     * structures.
     */
    FreeEntries freeEntries[Impl::MaxThreads];

    /** Records if the ROB is empty. In SMT mode the ROB may be dynamically
     * partitioned between threads, so the ROB must tell rename when it is
     * empty.
     */
    bool emptyROB[Impl::MaxThreads];

    /** Source of possible stalls. */
    struct Stalls {
        bool iew;
        bool commit;
    };

    /** Tracks which stages are telling decode to stall. */
    Stalls stalls[Impl::MaxThreads];

    /** The serialize instruction that rename has stalled on. */
    DynInstPtr serializeInst[Impl::MaxThreads];

    /** Records if rename needs to serialize on the next instruction for any
     * thread.
     */
    bool serializeOnNextInst[Impl::MaxThreads];

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

    /** The index of the instruction in the time buffer to IEW that rename is
     * currently using.
     */
    unsigned toIEWIndex;

    /** Whether or not rename needs to block this cycle. */
    bool blockThisCycle;

    /** Whether or not rename needs to resume a serialize instruction
     * after squashing. */
    bool resumeSerialize;

    /** Whether or not rename needs to resume clearing out the skidbuffer
     * after squashing. */
    bool resumeUnblocking;

    /** The number of threads active in rename. */
    ThreadID numThreads;

    /** The maximum skid buffer size. */
    unsigned skidBufferMax;

    /** Enum to record the source of a structure full stall.  Can come from
     * either ROB, IQ, LSQ, and it is priortized in that order.
     */
    enum FullSource {
        ROB,
        IQ,
        LQ,
        SQ,
        NONE
    };

    /** Function used to increment the stat that corresponds to the source of
     * the stall.
     */
    inline void incrFullStat(const FullSource &source);

    /** Stat for total number of cycles spent squashing. */
    Stats::Scalar renameSquashCycles;
    /** Stat for total number of cycles spent idle. */
    Stats::Scalar renameIdleCycles;
    /** Stat for total number of cycles spent blocking. */
    Stats::Scalar renameBlockCycles;
    /** Stat for total number of cycles spent stalling for a serializing inst. */
    Stats::Scalar renameSerializeStallCycles;
    /** Stat for total number of cycles spent running normally. */
    Stats::Scalar renameRunCycles;
    /** Stat for total number of cycles spent unblocking. */
    Stats::Scalar renameUnblockCycles;
    /** Stat for total number of renamed instructions. */
    Stats::Scalar renameRenamedInsts;
    /** Stat for total number of squashed instructions that rename discards. */
    Stats::Scalar renameSquashedInsts;
    /** Stat for total number of times that the ROB starts a stall in rename. */
    Stats::Scalar renameROBFullEvents;
    /** Stat for total number of times that the IQ starts a stall in rename. */
    Stats::Scalar renameIQFullEvents;
    /** Stat for total number of times that the LQ starts a stall in rename. */
    Stats::Scalar renameLQFullEvents;
    /** Stat for total number of times that the SQ starts a stall in rename. */
    Stats::Scalar renameSQFullEvents;
    /** Stat for total number of times that rename runs out of free registers
     * to use to rename. */
    Stats::Scalar renameFullRegistersEvents;
    /** Stat for total number of renamed destination registers. */
    Stats::Scalar renameRenamedOperands;
    /** Stat for total number of source register rename lookups. */
    Stats::Scalar renameRenameLookups;
    Stats::Scalar intRenameLookups;
    Stats::Scalar fpRenameLookups;
    Stats::Scalar vecRenameLookups;
    Stats::Scalar vecPredRenameLookups;
    /** Stat for total number of committed renaming mappings. */
    Stats::Scalar renameCommittedMaps;
    /** Stat for total number of mappings that were undone due to a squash. */
    Stats::Scalar renameUndoneMaps;
    /** Number of serialize instructions handled. */
    Stats::Scalar renamedSerializing;
    /** Number of instructions marked as temporarily serializing. */
    Stats::Scalar renamedTempSerializing;
    /** Number of instructions inserted into skid buffers. */
    Stats::Scalar renameSkidInsts;
};

#endif // __CPU_O3_RENAME_HH__
