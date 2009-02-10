/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Korey Sewell
 *
 */

#ifndef __CPU_INORDER_PIPELINE_STAGE_HH__
#define __CPU_INORDER_PIPELINE_STAGE_HH__

#include <queue>
#include <vector>

#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/comm.hh"
#include "params/InOrderCPU.hh"
#include "cpu/inorder/pipeline_traits.hh"

class InOrderCPU;

class PipelineStage
{
  protected:
    typedef ThePipeline::Params Params;
    typedef ThePipeline::DynInstPtr DynInstPtr;

  public:
    /** Overall stage status. Used to determine if the CPU can
     * deschedule itself due to a lack of activity.
     */
    enum StageStatus {
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
        MemWaitResponse,
        MemWaitRetry,
        MemAccessComplete
    };

  protected:
    /** The Number of This Pipeline Stage */
    unsigned stageNum;

    /** The width of stage, in instructions. */
    unsigned stageWidth;

    /** Number of Threads*/
    unsigned numThreads;

    /** Stage status. */
    StageStatus _status;

    /** Per-thread status. */
    ThreadStatus stageStatus[ThePipeline::MaxThreads];

  public:
    PipelineStage(Params *params, unsigned stage_num);

    /** MUST use init() function if this constructor is used. */
    PipelineStage() { }

    virtual ~PipelineStage() { }

    /** PipelineStage initialization. */
    void init(Params *params, unsigned stage_num);

    /** Returns the name of stage. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets CPU pointer. */
    virtual void setCPU(InOrderCPU *cpu_ptr);

    virtual void scheduleStageStart(int delay, int tid) { }

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Sets pointer to time buffer coming from fetch. */
    void setPrevStageQueue(TimeBuffer<InterStageStruct> *prev_stage_ptr);

    /** Sets pointer to time buffer used to communicate to the next stage. */
    void setNextStageQueue(TimeBuffer<InterStageStruct> *next_stage_ptr);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<unsigned> *at_ptr);

    bool nextStageQueueValid(int stage_num);

    bool isBlocked(unsigned tid);

    /** Changes the status of this stage to active, and indicates this
     * to the CPU.
     */
    //inline void switchToActive();

    /** Changes the status of this stage to inactive, and indicates
     * this to the CPU.
     */
    //inline void switchToInactive();

    /** Switches out the stage stage. */
    void switchOut();

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Ticks stage, processing all input signals and executing as many
     *  instructions as possible.
     */
    virtual void tick();

    /** Is out of order processing valid? */
    bool outOfOrderValid();

    /** Set a resource stall in the pipeline-stage */
    void setResStall(ResReqPtr res_req, unsigned tid);

    /** Unset a resource stall in the pipeline-stage */
    void unsetResStall(ResReqPtr res_req, unsigned tid);

    /** Remove all stall signals for a particular thread; */
    virtual void removeStalls(unsigned tid);

    /** Is there room in the stage buffer? */
    int stageBufferAvail();

  protected:
    /** Evaluate Stage Conditions and then process stage */
    virtual void processStage(bool &status_change);

    /** Determines what to do based on stage's current status.
     * @param status_change stage() sets this variable if there was a status
     * change (ie switching from from blocking to unblocking).
     * @param tid Thread id to stage instructions from.
     */
    virtual void processThread(bool &status_change, unsigned tid);

    /** Processes instructions from fetch and passes them on to rename.
     * Decoding of instructions actually happens when they are created in
     * fetch, so this function mostly checks if PC-relative branches are
     * correct.
     */
    virtual void processInsts(unsigned tid);

    /** Process all resources on an instruction's resource schedule */
    virtual bool processInstSchedule(DynInstPtr inst);

    /** Is there room in the next stage buffer for this instruction? */
    virtual bool canSendInstToNextStage();

    /** Send an instruction to the next stage buffer */
    virtual bool sendInstToNextStage(DynInstPtr inst);

    /** Inserts a thread's instructions into the skid buffer, to be staged
     * once stage unblocks.
     */
    virtual void skidInsert(unsigned tid);

    /** Returns if all of the skid buffers are empty. */
    bool skidsEmpty();

    /** Updates overall stage status based on all of the threads' statuses. */
    virtual void updateStatus();

    /** Separates instructions from fetch into individual lists of instructions
     * sorted by thread.
     */
    void sortInsts();

    /** Reads all stall signals from the backwards communication timebuffer. */
    virtual void readStallSignals(unsigned tid);

    /** Checks all input signals and updates stage's status appropriately. */
    virtual bool checkSignalsAndUpdate(unsigned tid);

    /** Checks all stall signals, and returns if any are true. */
    virtual bool checkStall(unsigned tid) const;

    /** Returns if there any instructions from the previous stage
     * on this cycle.
     */
    inline bool prevStageInstsValid();

    /** Switches stage to blocking, and signals back that stage has
     * become blocked.
     * @return Returns true if there is a status change.
     */
    virtual bool block(unsigned tid);

    void blockDueToBuffer(unsigned tid);

    /** Switches stage to unblocking if the skid buffer is empty, and
     * signals back that stage has unblocked.
     * @return Returns true if there is a status change.
     */
    virtual bool unblock(unsigned tid);


  public:
    /** Squashes if there is a PC-relative branch that was predicted
     * incorrectly. Sends squash information back to fetch.
     */
    virtual void squashDueToBranch(DynInstPtr &inst, unsigned tid);

    /** Squash instructions from stage buffer  */
    virtual void squashPrevStageInsts(InstSeqNum squash_seq_num, unsigned tid);

    /** Squashes due to commit signalling a squash. Changes status to
     *  squashing and clears block/unblock signals as needed.
     */
    virtual void squash(InstSeqNum squash_num, unsigned tid);

    void dumpInsts();

  protected:
    /** CPU interface. */
    InOrderCPU *cpu;

    Trace::InOrderTrace *tracer;

    /** List of active thread ids */
    std::list<unsigned> *activeThreads;

    /** Queue of all instructions coming from previous stage on this cycle. */
    std::queue<DynInstPtr> insts[ThePipeline::MaxThreads];

    /** Queue of instructions that are finished processing and ready to go next stage.
     *  This is used to prevent from processing an instrution more than once on any
     *  stage. NOTE: It is up to the PROGRAMMER must manage this as a queue
     */
    std::list<DynInstPtr> instsToNextStage;

    /** Skid buffer between previous stage and this one. */
    std::queue<DynInstPtr> skidBuffer[ThePipeline::MaxThreads];

    /** Instruction used to signify that there is no *real* instruction in buffer slot */
    DynInstPtr dummyBufferInst;

    /** SeqNum of Squashing Branch Delay Instruction (used for MIPS) */
    Addr bdelayDoneSeqNum[ThePipeline::MaxThreads];

    /** Instruction used for squashing branch (used for MIPS) */
    DynInstPtr squashInst[ThePipeline::MaxThreads];

    /** Tells when their is a pending delay slot inst. to send
     *  to rename. If there is, then wait squash after the next
     *  instruction (used for MIPS).
     */
    bool squashAfterDelaySlot[ThePipeline::MaxThreads];

    /** Maximum size of the inter-stage buffer connecting the previous stage to
     *  this stage (which we call a skid buffer) */
    unsigned stageBufferMax;

    /** Variable that tracks if stage has written to the time buffer this
     * cycle. Used to tell CPU if there is activity this cycle.
     */
    bool wroteToTimeBuffer;

    /** Index of instructions being sent to the next stage. */
    unsigned toNextStageIndex;

    /** The last stage that this particular stage should look for stalls */
    int lastStallingStage[ThePipeline::MaxThreads];

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

  public:
    /** Wire to get rename's output from backwards time buffer. */
    TimeBuffer<TimeStruct>::wire fromNextStages;

    /** Wire to get iew's information from backwards time buffer. */
    TimeBuffer<TimeStruct>::wire toPrevStages;

    /** Instruction queue linking previous stage */
    TimeBuffer<InterStageStruct> *prevStageQueue;

    /** Wire to get the previous stage's. */
    TimeBuffer<InterStageStruct>::wire prevStage;

    /** Instruction queue linking next stage */
    TimeBuffer<InterStageStruct> *nextStageQueue;

    /** Wire to write to the next stage */
    TimeBuffer<InterStageStruct>::wire nextStage;

    /** Is Previous Stage Valid? */
    bool prevStageValid;

    /** Is Next Stage Valid? */
    bool nextStageValid;

    /** Source of possible stalls. */
    struct Stalls {
        bool stage[ThePipeline::NumStages];
        std::vector<ResReqPtr> resources;
    };

    /** Tracks which stages are telling decode to stall. */
    Stalls stalls[ThePipeline::MaxThreads];

    //@TODO: Use Stats for the pipeline stages
    /** Stat for total number of idle cycles. */
    //Stats::Scalar<> stageIdleCycles;
    /** Stat for total number of blocked cycles. */
    //Stats::Scalar<> stageBlockedCycles;
    /** Stat for total number of normal running cycles. */
    //Stats::Scalar<> stageRunCycles;
    /** Stat for total number of unblocking cycles. */
    //Stats::Scalar<> stageUnblockCycles;
    /** Stat for total number of squashing cycles. */
    //Stats::Scalar<> stageSquashCycles;
    /** Stat for total number of staged instructions. */
    //Stats::Scalar<> stageProcessedInsts;
    /** Stat for total number of squashed instructions. */
    //Stats::Scalar<> stageSquashedInsts;
};

#endif
