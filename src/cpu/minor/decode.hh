/*
 * Copyright (c) 2013-2014 ARM Limited
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

/**
 * @file
 *
 *  Decode collects macro-ops from Fetch2 and splits them into micro-ops
 *  passed to Execute.
 */

#ifndef __CPU_MINOR_DECODE_HH__
#define __CPU_MINOR_DECODE_HH__

#include <vector>

#include "base/named.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/dyn_inst.hh"
#include "cpu/minor/pipe_data.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/BaseMinorCPU.hh"
#include "cpu/minor/scoreboard.hh"
#include "cpu/minor/func_unit.hh"

namespace gem5
{

  GEM5_DEPRECATED_NAMESPACE(Minor, minor);
  namespace minor
  {

    /* Decode takes instructions from Fetch2 and decomposes them into micro-ops
     * to feed to Execute.  It generates a new sequence number for each
     * instruction: execSeqNum.
     */
    class Decode : public Named
    {
    protected:
      /** Pointer back to the containing CPU */
      MinorCPU &cpu;

      /** Input port carrying macro instructions from Fetch1 */
      Latch<ForwardLineData>::Output inp;

      /** Input port carrying branches from Execute.  This is a snoop of the
       *  data provided to F1. */
      Latch<BranchData>::Output branchInp;

      /** Output port carrying predictions back to Fetch1 */
      Latch<BranchData>::Input predictionOut;

      /** Output port carrying micro-op decomposed instructions to Execute */
      Latch<ForwardInstData>::Input out;

      /** Interface to reserve space in the next stage */
      std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;

      /** Width of output of this stage/input of next in instructions */
      unsigned int outputWidth;
      unsigned int branchDelaySlot = 1;
      unsigned int delayBranchTid = 0;
      gem5::InstSeqNum delayStreamSeqNum = InstId::firstStreamSeqNum;
      int branchDelay = -1;
      /** If true, more than one input word can be processed each cycle if
       *  there is room in the output to contain its processed data */
      bool processMoreThanOneInput;

      /** Branch predictor passed from Python configuration */
      branch_prediction::BPredUnit &branchPredictor;

      /** True when there are still microinstructions to extract from a
       *  macroinstruction and to be packed into output */
      bool macroInstPending = false;

      /** Pointer to the macroinstruction that needs further
       * decomposition */
      MinorDynInstPtr macroInstPendingPtr = NULL;

      /** True when there is a microinstruction waiting for a scoreboard
       *  entry to become available */
      std::vector<bool> instWaitingDependencies;

      /** Pointer to the inst that is waiting for a scoreboard
       *  entry to become available */
      std::vector<MinorDynInstPtr> instWaitingDependenciesPtr;

      /** Scoreboard reference from execute stage */
      std::vector<Scoreboard> &scoreboard;

      /** Functional units reference from execute stage */
      std::vector<FUPipeline *> &funcUnits;

    public:
      /* Public for Pipeline to be able to pass it to Fetch1 */
      std::vector<InputBuffer<ForwardLineData>> inputBuffer;

    protected:
      /** Data members after this line are cycle-to-cycle state */

      struct DecodeThreadInfo
      {
        DecodeThreadInfo() {}

        DecodeThreadInfo(const DecodeThreadInfo &other) : inputIndex(other.inputIndex),
                                                          havePC(other.havePC),
                                                          lastStreamSeqNum(other.lastStreamSeqNum),
                                                          expectedStreamSeqNum(other.expectedStreamSeqNum),
                                                          predictionSeqNum(other.predictionSeqNum),
                                                          inMacroop(other.inMacroop),
                                                          execSeqNum(other.execSeqNum),
                                                          blocked(other.blocked)
        {
          set(pc, other.pc);
          set(microopPC, other.microopPC);
        }

        /** Index into the inputBuffer's head marking the start of unhandled
         *  instructions */
        unsigned int inputIndex = 0;

        /** Remembered program counter value.  Between contiguous lines, this
         *  is just updated with advancePC.  For lines following changes of
         *  stream, a new PC must be loaded and havePC be set.
         *  havePC is needed to accomodate instructions which span across
         *  lines meaning that Decode and the decoder need to remember a PC
         *  value and a partially-offered instruction from the previous line */
        std::unique_ptr<PCStateBase> pc;

        /** PC is currently valid.  Initially false, gets set to true when a
         *  change-of-stream line is received and false again when lines are
         *  discarded for any reason */
        bool havePC = false;

        /** Stream sequence number of the last seen line used to identify
         *  changes of instruction stream */
        InstSeqNum lastStreamSeqNum = InstId::firstStreamSeqNum;

        /** Fetch2 is the source of fetch sequence numbers.  These represent the
         *  sequence that instructions were extracted from fetched lines. */
        InstSeqNum fetchSeqNum = InstId::firstFetchSeqNum;

        /** Stream sequence number remembered from last time the
         *  predictionSeqNum changed.  Lines should only be discarded when their
         *  predictionSeqNums disagree with Fetch2::predictionSeqNum *and* they
         *  are from the same stream that bore that prediction number */
        InstSeqNum expectedStreamSeqNum = InstId::firstStreamSeqNum;

        /** Fetch2 is the source of prediction sequence numbers.  These
         *  represent predicted changes of control flow sources from branch
         *  prediction in Fetch2. */
        InstSeqNum predictionSeqNum = InstId::firstPredictionSeqNum;

        /** True when we're in the process of decomposing a micro-op and
         *  microopPC will be valid.  This is only the case when there isn't
         *  sufficient space in Executes input buffer to take the whole of a
         *  decomposed instruction and some of that instructions micro-ops must
         *  be generated in a later cycle */
        bool inMacroop = false;
        std::unique_ptr<PCStateBase> microopPC;

        /** Source of execSeqNums to number instructions. */
        InstSeqNum execSeqNum = InstId::firstExecSeqNum;

        /** Blocked indication for report */
        bool blocked = false;
      };

      std::vector<DecodeThreadInfo> decodeInfo;
      ThreadID threadPriority;

      struct DecodeStats : public statistics::Group
      {
        DecodeStats(MinorCPU *cpu);
        /** Stats */
        statistics::Scalar intInstructions;
        statistics::Scalar fpInstructions;
        statistics::Scalar vecInstructions;
        statistics::Scalar loadInstructions;
        statistics::Scalar storeInstructions;
        statistics::Scalar amoInstructions;
      } stats;

    protected:
      /** Get a piece of data to work on, or 0 if there is no data. */
      const ForwardLineData *getInput(ThreadID tid);

      /** Pop an element off the input buffer, if there are any */
      void popInput(ThreadID tid);

      /** Dump the whole contents of the input buffer.  Useful after a
       *  prediction changes control flow */
      void dumpAllInput(ThreadID tid);

      /** Update local branch prediction structures from feedback from
       *  Execute. */
      void updateBranchPrediction(const BranchData &branch);

      /** Predicts branches for the given instruction.  Updates the
       *  instruction's predicted... fields and also the branch which
       *  carries the prediction to Fetch1 */
      void predictBranch(MinorDynInstPtr inst, BranchData &branch, bool &early_branch);

      /** Use the current threading policy to determine the next thread to
       *  decode from. */
      ThreadID getScheduledThread();

      /** Check if a specific instruction can be ran from the
       * next clock cycle. If so, update the scoreboard and return true */
      bool checkScoreboardAndUpdate(MinorDynInstPtr output_inst, ThreadID tid);

      /** Find the functional unit that can execute the given instruction.
       * Returns -1 if no functional unit can execute the instruction. */
      int findFunctionUnit(MinorDynInstPtr output_inst,
                           std::vector<FUPipeline *> &funcUnits);

    public:
      Decode(const std::string &name,
             MinorCPU &cpu_,
             const BaseMinorCPUParams &params,
             Latch<ForwardLineData>::Output inp_,
             Latch<BranchData>::Output branchInp_,
             Latch<BranchData>::Input predictionOut_,
             Latch<ForwardInstData>::Input out_,
             std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer,
             std::vector<Scoreboard> &scoreboard_,
             std::vector<FUPipeline *> &funcUnits_);

    protected:
      /** Push input coming from input wire into input buffer */
      void pushIntoInpBuffer();
      void earlyBranch(MinorDynInstPtr inst, Fault &f, BranchData &branch);
      /** Check what branches were taken by execute and dumps all
       * lines that are now old */
      void dumpIfBranchesExecuted(const BranchData &branch);

      /** Pops all lines that have a Prediction Sequence Number mismatch */
      void popLinesIfPredictionMismatch(ThreadID tid);

      /** Mark each thread as blocked if it cannot reserve any space in
       * next stage, then calls popLinesIfPredictionMismatch(tid)  */
      void updateAllThreadsStatus();

      /** Assign a PC to the line if it's not to discard */
      void givePcIfValidInstruction(ThreadID tid, bool discard_line,
                                    const ForwardLineData *line_in,
                                    InstDecoder *decoder);

      /** Creates a dynamic instruction that is a fault */
      MinorDynInstPtr packFault(const ForwardLineData *line_in, ThreadID tid);

      /** Creates a dynamic instruction from static decoded instruction */
      MinorDynInstPtr packInst(StaticInstPtr decoded_inst, InstId id,
                               ThreadID tid);

      /** Collect some statistics about the decoded static instruction */
      void collectStats(StaticInstPtr decoded_inst);

      /** Set inputindex to the next in order to process next input */
      void advanceInput(ThreadID tid);

      /** Set the pc to fetch the next micro instruction */
      void setUpPcForMicroop(ThreadID tid, MinorDynInstPtr inst);

      /** Extract a micro instruction from a macroop */
      StaticInstPtr extractMicroInst(ThreadID tid, StaticInstPtr static_inst);

      /** Create dynamic instruction from static decoded  micro instruction */
      /*MinorDynInstPtr packInst(StaticInstPtr static_micro_inst,
                  InstId id, ThreadID tid);*/

      /** Only allows last microop to contain a predicted next address */
      void allowPredictionOnLastMicroop(StaticInstPtr static_micro_inst,
                                        MinorDynInstPtr output_inst, MinorDynInstPtr macro_inst);

      /** Perform a macroop decomposition into microop isntructions */
      MinorDynInstPtr decomposition(ThreadID tid, MinorDynInstPtr inst,
                                    unsigned int output_index);

      /** Assign the ExecSeqNum to the instruction */
      void assignExecSeqNum(ThreadID tid, MinorDynInstPtr output_inst);

      /** Pack instruction into output wire */
      void packIntoOutput(MinorDynInstPtr output_inst,
                          ForwardInstData &insts_out, unsigned int *output_index);

      /** Macroop tracing */
      void macroopTraceInst(MinorDynInstPtr dyn_inst);

      /** Sets line_in and PC and handles input buffer depending on what
       * type of instruction it's being processed */
      void finishLineProcessing(ThreadID tid, const ForwardLineData **line_in,
                                bool prediction, bool discard_line);

      /** If the stage can process more than one input, it gets another line
       * from the input buffer */
      const ForwardLineData *maybeMoreInput(ThreadID tid,
                                            const ForwardLineData *insts_in);

      /** If some instruction has been produced it reserves space for it
       * in the next stage */
      void reserveSpaceInNextStage(ForwardInstData &insts_out, ThreadID tid);

      /** Mark the stage as active */
      void markStageActivity();

      /** Push input buffer tail to make the buffer shift */
      void pushTailInpBuffer();
      bool decode_is_stalling = false;
      bool was_stalling = false;
      bool has_branched = false;
      bool waiting_fetch = true;

    public:
      /** Pass on input/buffer data to the output if you can */
      void evaluate();

      void minorTrace() const;

      /** Is this stage drained?  For Decoed, draining is initiated by
       *  Execute halting Fetch1 causing Fetch2 to naturally drain
       *  into Decode and on to Execute which is responsible for
       *  actually killing instructions */
      bool isDrained();
      bool decode_was_stalling() const
      {
        return was_stalling;
      }

      bool decode_has_branched() const
      {
        return has_branched;
      }

      bool decode_is_waiting_fetch_after_branch() const
      {
        return waiting_fetch;
      }

      gem5::Addr last_inst_pc = gem5::MaxAddr;
      gem5::Addr last_branch_pc = gem5::MaxAddr;
    };

  } // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_DECODE_HH__ */
