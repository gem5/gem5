/*
 * Copyright (c) 2022-2023 The University of Edinburgh
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
 */


#ifndef __CPU_O3_BAC_HH__
#define __CPU_O3_BAC_HH__

#include <list>

#include "base/statistics.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/limits.hh"
#include "cpu/pred/bpred_unit.hh"
#include "cpu/pred/branch_type.hh"
#include "cpu/timebuf.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace o3
{

class CPU;
class FTQ;
class FetchTarget;
typedef std::shared_ptr<FetchTarget> FetchTargetPtr;


/********************************************************************
 * BAC: Branch and Address Calculation
 *
 * The BAC stage handles branch prediction and next PC address
 * calculation and mainly consists of two parts:
 *
 * (1) The branch prediction part that manages the branch predictor.
 *     It checks the backward communication signals from the pipleline
 *     to update the branch predictor accordingly.
 *     This part can be decoupled from the fetch stage and used to
 *     feed the FTQ with fetch targets which will be consumed by the
 *     fetch stage.
 *     When decoupling is enabled this part is active and acts as
 *     a separate pipeline stage. Otherwise the stage remains idle and
 *     everything is in sync with the fetch stage. The branch prediction
 *     itself is then triggered from the PC calculation part.
 *
 * (2) The PC calculation part that is tightly coupled to the
 *     fetch stage. Fetch calls the updatePC() method after each
 *     predecoded instruction to advance the PC to the next instruction.
 *
 * Note that this organization means that the BAC stage has spans over
 * two pipeline stages and the second part is in the same stage as
 * the fetch stage which is not the nicest organization. However,
 * it allows to keep the fetch stage simple and most of the decoupling
 * in the BAC stage.
*/
class BAC
{
  typedef branch_prediction::BranchType BranchType;

  public:
    /** Overall decoupled BPU stage status. Used to determine if the CPU can
     * deschedule itself due to a lack of activity.
     */
    enum BACStatus
    {
        Active,
        Inactive
    };

    /** Individual thread status. */
    enum ThreadStatus
    {
        Idle,
        Running,
        Squashing,
        Blocked,
        FTQFull,
        FTQLocked
    };

  private:
    /** Decode status. */
    BACStatus _status;

    /** Per-thread status. */
    ThreadStatus bacStatus[MaxThreads];

  public:
    /** BAC constructor. */
    BAC(CPU *_cpu, const BaseO3CPUParams &params);


    // Interfaces to CPU ----------------------------------
    // These functions are to manage the BAC stage
    // and the interface with the remaining CPU.

    /** Returns the name of the stage. */
    std::string name() const;

    /** Registers probes and listeners. */
    void regProbePoints() {}

    /** Sets the main backwards communication time buffer pointer. */
    void setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr);

    /** Sets pointer to list of active threads. */
    void setActiveThreads(std::list<ThreadID> *at_ptr);

    /** Initialize stage. */
    void setFetchTargetQueue(FTQ * _ptr);

    /** Initialize stage. */
    void startupStage();

    /** Clear all thread-specific states*/
    void clearStates(ThreadID tid);

    /** Resume after a drain. */
    void drainResume();

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Has the stage drained? */
    bool isDrained() const;

    /**
     * Stall the fetch stage after reaching a safe drain point.
     *
     * The CPU uses this method to stop fetching instructions from a
     * thread that has been drained. The drain stall is different from
     * all other stalls in that it is signaled instantly from the
     * commit stage (without the normal communication delay) when it
     * has reached a safe point to drain from.
     */
    void drainStall(ThreadID tid);

    /** Takes over from another CPU's thread. */
    void takeOverFrom() { resetStage(); }

    void deactivateThread(ThreadID tid) {};

    /** Processing all input signals and create the next fetch target. */
    void tick();

  private:
    /** Reset this pipeline stage */
    void resetStage();

    /** Changes the status of this stage to active, and indicates this
     * to the CPU.
     */
    void switchToActive();

    /** Changes the status of this stage to inactive, and indicates
     * this to the CPU.
     */
    void switchToInactive();

    /** Checks if a thread is stalled. */
    bool checkStall(ThreadID tid) const;

    /** Updates overall BAC stage status; to be called at the end of each
     * cycle. */
    void updateBACStatus();

    /** Checks all input signals and updates the status as necessary.
     *  @return: Returns if the status has changed due to input signals.
     */
    bool checkSignalsAndUpdate(ThreadID tid);

    /** Check the backward signals that update the BPU. */
    bool checkAndUpdateBPUSignals(ThreadID tid);


  private:

    /* ----------------------------------------------------------------
     * Decoupled Frontend Functionality
     *
     * In a decoupled frontend the Branch predictor unit (BPU)
     * is not directly queried by Fetch once it pre-decodes
     * a branch. Instead BPU and Fetch is separated and
     * connected via a queue fetch target queue (FTQ). The BPU generates
     * fetch targets, similar to basic blocks (BB) and inserts them in
     * the queue. Fetch will consume the addresses and read them from the
     * I-cache.
     * The advantages are that it (1) cuts the critical path and (2)
     * allow a precise, BPU guided prefetching of the fetch targets.
     *
     * For determining next PC addresses the BPU relies solely on the BTB.
     *
     * The follwing functions are only used in the decoupled scenario.
     */

    /**
     * Create a new fetch target.
     * @param start_pc The current PC. Will be the start address of the
     * fetch target.
    */
    FetchTargetPtr newFetchTarget(ThreadID tid, const PCStateBase &start_pc);

    /**
     * The prediction function for the BAC stage. In the decoupled scenario
     * the branch history is not added to the BPUs very own predictor history
     * because at the moment a prediction is made the sequence number in
     * not known.
     * @param inst The branch instruction.
     * @param ft The fetch target that is currently processed.
     * @param PC The predicted PC is passed back through this parameter.
     * @return Returns if the branch is taken or not.
     */
    bool predict(ThreadID tid, const StaticInstPtr &inst,
                 const FetchTargetPtr &ft, PCStateBase &pc);


    /**
     * Main function that feeds the FTQ with new fetch targets.
     * By leveraging the BTB up to N consecutive addresses are searched
     * to detect a branch instruction. For every BTB hit the direction
     * predictor is asked to make a prediction.
     * In every cycle one fetch target is created. A fetch target ends
     * once the first branch instruction is detected or the maximum
     * search bandwidth for a cycle is reached.
     **/
    void generateFetchTargets(ThreadID tid, bool &status_change);



    /* ----------------------------------------------------------------
     * Next PC address calculation
     *
     * To update the PC the fetch stage will call the updatePC() method
     * with the currently pre-decoded instruction.
     * The BAC stage will then check if the instruction is a branch and
     * either preform the branch prediction (non-decoupled scenario) or
     * read the branch prediction from the currentently processed fetch
     * target (decoupled scenario).
     */
  public:
    /**
     * Calculate the next PC address depending on the instruction type
     * and the branch prediction.
     * @param inst The currently processed dynamic instruction.
     * @param fetch_pc The current fetch PC passed in by reference. It will
     * be updated with what the next PC will be.
     * @param ft The currently processed fetch target. Can be nullptr for
     * the non-decoupled scenario.
     * @return Whether or not a branch was predicted as taken.
     */
    bool updatePC(const DynInstPtr &inst, PCStateBase &fetch_pc,
                  FetchTargetPtr &ft);


  private:

    /** Pre-decode update -----------------------------------------
     * After predecoding instruction in the fetch stage all instructions
     * are known together and a sequence number is assigned to them.
     * The fetch stage will call this function for every branch instruction
     * to allow the BAC stage to update the branch predictor history.
     *
     * There can be the following two cases:
     * - The branch was detected by the BAC stage and a prediction was made.
     *   In that case the branch history is moved from the FTQ to the BPU.
     *
     * - The branch was not detected by the BAC stage. In that case a "dummy"
     *   branch history is created and inserted into the BPU. For this dummy
     *   prediction it is assumed that the branch is not taken.
     *   If it turns unconditional or is taken decode or commit will squash
     *   the branch.
     *
     * This function performs the following steps:
     *  - For every branch where a prediction was made in the first place
     * It moves the branch history from the FTQ to the BPU.
     *
     * Together with inserting an instruction into the instruction queue
     *  instruction matches the predicted
     * instruction type. If so update the information with the new.
     * In case the types dont match something is wrong and we need
     * to squash. (should not be the case.)
     * @param seq_num The branches sequence that we want to update.
     * @param inst The new pre-decoded branch instruction.
     * @param tid The thread id.
     * @return Returns if the update was successful.
     */
    bool updatePreDecode(ThreadID tid, const InstSeqNum seqNum,
                         const StaticInstPtr &inst, PCStateBase &pc,
                         const FetchTargetPtr &ft);

  private:

    /** Squashes BAC for a specific thread and resets the PC. */
    void squash(const PCStateBase &new_pc, ThreadID tid);

    /**
     * Squashes the BPU histories in the FTQ.
     * by iterating from tail to head and reverts the predictions made.
     **/
    void squashBpuHistories(ThreadID tid);

    /** Update the stats per cycle */
    void profileCycle(ThreadID tid);

  private:
    /** Pointer to the main CPU. */
    CPU* cpu;

    /** BPredUnit. */
    branch_prediction::BPredUnit* bpu;

    /** Fetch target Queue. */
    FTQ* ftq;

    /** Time buffer interface. */
    TimeBuffer<TimeStruct> *timeBuffer;

    /** Wire to get fetches's information from backwards time buffer. */
    TimeBuffer<TimeStruct>::wire fromFetch;

    /** Wire to get decode's information from backwards time buffer. */
    TimeBuffer<TimeStruct>::wire fromDecode;

    /** Wire to get commit's information from backwards time buffer. */
    TimeBuffer<TimeStruct>::wire fromCommit;

    /** Wire used to write any information heading to fetch. */
    TimeBuffer<FetchStruct>::wire toFetch;

    /** The decoupled PC which runs ahead of fetch */
    std::unique_ptr<PCStateBase> bacPC[MaxThreads];


    /** Variable that tracks if BAC has written to the time buffer this
     * cycle. Used to tell CPU if there is activity this cycle.
     */
    bool wroteToTimeBuffer;

    /** Source of possible stalls. */
    struct Stalls
    {
        bool fetch;
        bool drain;
        bool bpu;
    };

    /** Tracks which stages are telling the ftq to stall. */
    Stalls stalls[MaxThreads];

    /** Enables the decoupled front-end */
    const bool decoupledFrontEnd;

    /** Fetch to BAC delay. */
    const Cycles fetchToBacDelay;

    /** Decode to fetch delay. (Same delay for BAC as for fetch) */
    const Cycles decodeToFetchDelay;

    /** Commit to fetch delay. (Same delay for BAC as for fetch) */
    const Cycles commitToFetchDelay;

    /** BAC to fetch delay. */
    const Cycles bacToFetchDelay;

    /** The maximum width of a fetch target. This also determines the
     * maximum addresses searched in one cycle. (FT width / minInstSize) */
    const unsigned fetchTargetWidth;

    /** The minimum size an instruction can have in the current
     * architecture. It determines the search granularity of the decoupled
     * front-end. I.e. for x86 this must be 1. For fixed size ISA's it
     * should be equal to the instruction size to speedup simulation time.*/
    const unsigned minInstSize;

    /** List of Active FTQ Threads */
    std::list<ThreadID> *activeThreads;

    /** Number of threads. */
    const ThreadID numThreads;



  protected:
    struct BACStats : public statistics::Group
    {
      BACStats(CPU *cpu, BAC *bac);

      /** Stat for total number of idle cycles. */
      statistics::Scalar idleCycles;
      /** Stat for total number of normal running cycles. */
      statistics::Scalar runCycles;
      /** Stat for total number of squashing cycles. */
      statistics::Scalar squashCycles;
      /** Stat for total number of cycles the FTQ was full. */
      statistics::Scalar ftqFullCycles;

      /** Stat for total number fetch targets created. */
      statistics::Scalar fetchTargets;
      /** Total number of branches detected. */
      statistics::Scalar branches;
      /** Total number of branches predicted taken. */
      statistics::Scalar predTakenBranches;
      /** Total number of fetched branches. */
      statistics::Scalar branchesNotLastuOp;

      /** Stat for total number of misspredicted instructions. */
      statistics::Scalar branchMisspredict;
      statistics::Scalar noBranchMisspredict;

      /** Stat for total number of misspredicted instructions. */
      statistics::Scalar squashBranchDecode;
      statistics::Scalar squashBranchCommit;

      /** Number of post updates */
      statistics::Vector preDecUpdate;
      /** Number of branches undetected by the BPU */
      statistics::Vector noHistType;

      /** Stat for the two corner cases */
      statistics::Scalar typeMissmatch;
      statistics::Scalar multiBranchInst;

      /** Distribution of number of bytes per fetch target. */
      statistics::Distribution ftSizeDist;

    } stats;
    /** @} */
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_BAC_HH__
