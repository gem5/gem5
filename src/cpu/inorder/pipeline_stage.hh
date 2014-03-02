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
#include "cpu/inorder/comm.hh"
#include "cpu/inorder/inorder_dyn_inst.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/timebuf.hh"
#include "params/InOrderCPU.hh"

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
    ThreadID numThreads;

    /** Stage status. */
    StageStatus _status;

    /** Per-thread status. */
    ThreadStatus stageStatus[ThePipeline::MaxThreads];

  public:
    PipelineStage(Params *params, unsigned stage_num);

    virtual ~PipelineStage();

    /** PipelineStage initialization. */
    void init(Params *params);

    /** Returns the name of stage. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets CPU pointer. */
    void setCPU(InOrderCPU *cpu_ptr);

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Sets pointer to time buffer coming from fetch. */
    void setPrevStageQueue(TimeBuffer<InterStageStruct> *prev_stage_ptr);

    /** Sets pointer to time buffer used to communicate to the next stage. */
    void setNextStageQueue(TimeBuffer<InterStageStruct> *next_stage_ptr);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    bool nextStageQueueValid(int stage_num);

    bool isBlocked(ThreadID tid);

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
    virtual void takeOverFrom();

    /** Ticks stage, processing all input signals and executing as many
     *  instructions as possible.
     */
    void tick();

    /** Set a resource stall in the pipeline-stage */
    void setResStall(ResReqPtr res_req, ThreadID tid);

    /** Unset a resource stall in the pipeline-stage */
    void unsetResStall(ResReqPtr res_req, ThreadID tid);

    /** Remove all stall signals for a particular thread; */
    void removeStalls(ThreadID tid);

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
    void processThread(bool &status_change, ThreadID tid);

    /** Processes instructions from fetch and passes them on to rename.
     * Decoding of instructions actually happens when they are created in
     * fetch, so this function mostly checks if PC-relative branches are
     * correct.
     */
    virtual void processInsts(ThreadID tid);

    /** Process all resources on an instruction's resource schedule */
    bool processInstSchedule(DynInstPtr inst, int &reqs_processed);

    /** Is there room in the next stage buffer for this instruction? */
    bool canSendInstToStage(unsigned stage_num);

    /** Send an instruction to the next stage buffer */
    bool sendInstToNextStage(DynInstPtr inst);

    /** Total size of all skid buffers */
    int skidSize();

    /** Returns if all of the skid buffers are empty. */
    bool skidsEmpty();

    /** Updates overall stage status based on all of the threads' statuses. */
    void updateStatus();

    /** Separates instructions from fetch into individual lists of instructions
     * sorted by thread.
     */
    void sortInsts();

    /** Reads all stall signals from the backwards communication timebuffer. */
    void readStallSignals(ThreadID tid);

    /** Checks all input signals and updates stage's status appropriately. */
    bool checkSignalsAndUpdate(ThreadID tid);

    /** Checks all stall signals, and returns if any are true. */
    bool checkStall(ThreadID tid) const;

    /** Returns if there any instructions from the previous stage
     * on this cycle.
     */
    inline bool prevStageInstsValid();

    /** Switches stage to blocking, and signals back that stage has
     * become blocked.
     * @return Returns true if there is a status change.
     */
    bool block(ThreadID tid);

    void blockDueToBuffer(ThreadID tid);

    /** Switches stage to unblocking if the skid buffer is empty, and
     * signals back that stage has unblocked.
     * @return Returns true if there is a status change.
     */
    bool unblock(ThreadID tid);


  public:
    void activateThread(ThreadID tid);
    
    /** Setup Squashing Information to be passed back thru the pipeline */
    void setupSquash(DynInstPtr inst, ThreadID tid);

    virtual void squashDueToMemStall(InstSeqNum seq_num, ThreadID tid);

    /** Perform squash of instructions above seq_num */
    virtual void squash(InstSeqNum squash_num, ThreadID tid);

    /** Squash instructions from stage buffer  */
    void squashPrevStageInsts(InstSeqNum squash_seq_num, ThreadID tid);

    void dumpInsts();

  protected:
    /** CPU interface. */
    InOrderCPU *cpu;

    Trace::InOrderTrace *tracer;

    /** List of active thread ids */
    std::list<ThreadID> *activeThreads;

    /** Buffer of instructions switched out to mem-stall. 
     *  Only used when using SwitchOnCacheMiss threading model
     *  Used as 1-to-1 mapping between ThreadID and Entry. 
     */
    std::vector<DynInstPtr> switchedOutBuffer;
    std::vector<bool> switchedOutValid;

    /** Instructions that we've processed this tick
     *  NOTE: "Processed" means completed at least 1 instruction request 
     */
    unsigned instsProcessed;    

    /** Skid buffer between previous stage and this one. */
    std::list<DynInstPtr> skidBuffer[ThePipeline::MaxThreads];

    /** Instruction used to signify that there is no *real* instruction in
     *  buffer slot */
    DynInstPtr dummyBufferInst;

    /** SeqNum of Squashing Branch Delay Instruction (used for MIPS) */
    Addr bdelayDoneSeqNum[ThePipeline::MaxThreads];

    /** Tells when their is a pending delay slot inst. to send
     *  to rename. If there is, then wait squash after the next
     *  instruction (used for MIPS).
     */
    bool squashAfterDelaySlot[ThePipeline::MaxThreads];

    /** Instruction used for squashing branch (used for MIPS) */
    DynInstPtr squashInst[ThePipeline::MaxThreads];

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

    bool idle;
    
    /** Source of possible stalls. */
    struct Stalls {
        bool stage[ThePipeline::NumStages];
        std::vector<ResReqPtr> resources;
    };

    /** Tracks stage/resource stalls */
    Stalls stalls[ThePipeline::MaxThreads];

    /** Number of cycles 0 instruction(s) are processed. */
    Stats::Scalar idleCycles;

    /** Number of cycles 1+ instructions are processed. */
    Stats::Scalar runCycles;

    /** Percentage of cycles 1+ instructions are processed. */
    Stats::Formula utilization;


};

#endif
