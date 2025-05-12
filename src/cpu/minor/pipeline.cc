/*
 * Copyright (c) 2013-2014, 2020 ARM Limited
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

#include "cpu/minor/pipeline.hh"

#include <algorithm>

#include "cpu/minor/decode.hh"
#include "cpu/minor/execute.hh"
#include "cpu/minor/fetch1.hh"
#include "cpu/minor/fetch2.hh"
#include "debug/Drain.hh"
#include "debug/MinorCPU.hh"
#include "debug/MinorTrace.hh"
#include "debug/Quiesce.hh"

namespace gem5
{

    GEM5_DEPRECATED_NAMESPACE(Minor, minor);
    namespace minor
    {

        Pipeline::Pipeline(MinorCPU &cpu_, const BaseMinorCPUParams &params) : Ticked(cpu_, &(cpu_.BaseCPU::baseStats.numCycles)),
                                                                               cpu(cpu_),
                                                                               allow_idling(params.enableIdling),
                                                                               /* f1ToF2(cpu.name() + ".f1ToF2", "lines",
                                                                                   params.fetch1ToFetch2ForwardDelay),
                                                                               f2ToF1(cpu.name() + ".f2ToF1", "prediction",
                                                                                   params.fetch1ToFetch2BackwardDelay, true),
                                                                               f2ToD(cpu.name() + ".f2ToD", "insts",
                                                                                   params.fetch2ToDecodeForwardDelay), */
                                                                               f1ToD(cpu.name() + ".f1ToD", "lines",
                                                                                     params.fetch1ToFetch2ForwardDelay),
                                                                               dToF1(cpu.name() + ".DToF1", "prediction",
                                                                                     params.fetch1ToFetch2BackwardDelay, true),
                                                                               dToE(cpu.name() + ".dToE", "insts",
                                                                                    params.decodeToExecuteForwardDelay),
                                                                               eToF1(cpu.name() + ".eToF1", "branch",
                                                                                     params.executeBranchDelay),
                                                                               eToW(cpu.name() + ".eToW", "insts", params.decodeToExecuteForwardDelay),
                                                                               eToW_branch(cpu.name() + ".eToW_branch", "branch", params.decodeToExecuteForwardDelay),
                                                                               eToM(cpu.name() + ".eToM", "insts", params.decodeToExecuteForwardDelay),
                                                                               eToM_branch(cpu.name() + ".eToM_branch", "branch", params.decodeToExecuteForwardDelay),
                                                                               mToW(cpu.name() + ".mToW", "insts", params.decodeToExecuteForwardDelay),
                                                                               mToF1_branch(cpu.name() + ".mToF1_branch", "branch", params.decodeToExecuteForwardDelay),
                                                                               writeback(cpu.name() + ".writeback", cpu, params,
                                                                                         mToW.output(), eToF1.input(), eToW_branch.output()), // TODO change branch latch
                                                                               memory(cpu.name() + ".memory", cpu, params, eToM.output(), mToW.input(), eToF1.input(), eToM_branch.output(), writeback.inputBuffer),
                                                                               execute(cpu.name() + ".execute", cpu, params,
                                                                                       dToE.output(), eToF1.input(), eToW_branch.input(), memory.inputBuffer, eToM.input(), eToM_branch.input()),
                                                                               lsq(cpu.name() + ".lsq", cpu.name() + ".dcache_port",
                                                                                   cpu_, *this,
                                                                                   params.executeMaxAccessesInMemory,
                                                                                   params.executeMemoryWidth,
                                                                                   params.executeLSQRequestsQueueSize,
                                                                                   params.executeLSQTransfersQueueSize,
                                                                                   params.executeLSQStoreBufferSize,
                                                                                   params.executeLSQMaxStoreBufferStoresPerCycle),
                                                                               /*decode(cpu.name() + ".decode", cpu, params,
                                                                                   f2ToD.output(), dToE.input(), execute.inputBuffer),*/
                                                                               decode(cpu.name() + ".decode", cpu, params,
                                                                                      f1ToD.output(), eToF1.output(), dToF1.input(), dToE.input(),
                                                                                      execute.inputBuffer, execute.scoreboard, execute.funcUnits),
                                                                               /*fetch2(cpu.name() + ".fetch2", cpu, params,
                                                                                   f1ToF2.output(), eToF1.output(), f2ToF1.input(), f2ToD.input(),
                                                                                   decode.inputBuffer),*/
                                                                               /*fetch1(cpu.name() + ".fetch1", cpu, params,
                                                                                   eToF1.output(), f1ToF2.input(), f2ToF1.output(), fetch2.inputBuffer),*/
                                                                               fetch1(cpu.name() + ".fetch1", cpu, params,
                                                                                      eToF1.output(), f1ToD.input(), dToF1.output(), mToF1_branch.output(), decode.inputBuffer),
                                                                               activityRecorder(cpu.name() + ".activity", Num_StageId,
                                                                                                /* The max depth of inter-stage FIFOs */
                                                                                                std::max(params.fetch1ToFetch2ForwardDelay,
                                                                                                         std::max(params.fetch2ToDecodeForwardDelay,
                                                                                                                  std::max(params.decodeToExecuteForwardDelay,
                                                                                                                           params.executeBranchDelay)))),
                                                                               needToSignalDrained(false)
        {
            if (params.fetch1ToFetch2ForwardDelay < 1)
            {
                fatal("%s: fetch1ToFetch2ForwardDelay must be >= 1 (%d)\n",
                      cpu.name(), params.fetch1ToFetch2ForwardDelay);
            }

            if (params.fetch2ToDecodeForwardDelay < 1)
            {
                fatal("%s: fetch2ToDecodeForwardDelay must be >= 1 (%d)\n",
                      cpu.name(), params.fetch2ToDecodeForwardDelay);
            }

            if (params.decodeToExecuteForwardDelay < 1)
            {
                fatal("%s: decodeToExecuteForwardDelay must be >= 1 (%d)\n",
                      cpu.name(), params.decodeToExecuteForwardDelay);
            }
            if (params.executeBranchDelay < 1)
            {
                fatal("%s: executeBranchDelay must be >= 1\n",
                      cpu.name(), params.executeBranchDelay);
            }
        }

        void
        Pipeline::minorTrace() const
        {
            fetch1.minorTrace();
            // f1ToF2.minorTrace();
            // f2ToF1.minorTrace();
            // fetch2.minorTrace();
            // f2ToD.minorTrace();
            f1ToD.minorTrace();
            dToF1.minorTrace();
            decode.minorTrace();
            dToE.minorTrace();
            execute.minorTrace();
            eToF1.minorTrace();
            eToM.minorTrace();
            eToW_branch.minorTrace();
            eToM_branch.minorTrace();
            memory.minorTrace();
            mToF1_branch.minorTrace();
            mToW.minorTrace();
            writeback.minorTrace();
            activityRecorder.minorTrace();
        }

        bool Pipeline::decodeWasStalling() const
        {
            return decode.decode_was_stalling(); //|| !activityRecorder.getStageActive(DecodeStageId);
        }

        gem5::Addr Pipeline::getDecodeLastInstPC() const
        {
            return decode.last_inst_pc;
        }

        bool Pipeline::decodeHasBranched() const
        {
            return decode.decode_has_branched();
        }

        bool Pipeline::decodeIsWaitingFetchAfterBranch() const
        {
            return decode.decode_is_waiting_fetch_after_branch();
        }

            void
            Pipeline::evaluate()
        {
            /** We tick the CPU to update the BaseCPU cycle counters */
            cpu.tick();
            /* Note that it's important to evaluate the stages in order to allow
             *  'immediate', 0-time-offset TimeBuffer activity to be visible from
             *  later stages to earlier ones in the same cycle */
            writeback.evaluate();
            memory.evaluate();
            execute.evaluate();
            decode.evaluate();
            dToF1.evaluate();
            // fetch2.evaluate();
            fetch1.evaluate();

            if (debug::MinorTrace)
                minorTrace();

            /* Update the time buffers after the stages */
            // f1ToF2.evaluate();
            // f2ToF1.evaluate();
            // f2ToD.evaluate();
            f1ToD.evaluate();
            dToE.evaluate();
            eToF1.evaluate();
            eToM.evaluate();
            eToM_branch.evaluate();
            mToW.evaluate();
            mToF1_branch.evaluate();
            eToW_branch.evaluate();

            /* The activity recorder must be be called after all the stages and
             *  before the idler (which acts on the advice of the activity recorder */
            activityRecorder.evaluate();

            if (allow_idling)
            {
                /* Become idle if we can but are not draining */
                if (!activityRecorder.active() && !needToSignalDrained)
                {
                    DPRINTF(Quiesce, "Suspending as the processor is idle\n");
                    stop();
                }

                /* Deactivate all stages.  Note that the stages *could*
                 *  activate and deactivate themselves but that's fraught
                 *  with additional difficulty.
                 *  As organised herre */
                activityRecorder.deactivateStage(Pipeline::CPUStageId);
                activityRecorder.deactivateStage(Pipeline::Fetch1StageId);
                activityRecorder.deactivateStage(Pipeline::Fetch2StageId);
                activityRecorder.deactivateStage(Pipeline::DecodeStageId);
                activityRecorder.deactivateStage(Pipeline::ExecuteStageId);
                activityRecorder.deactivateStage(Pipeline::MemoryStageId);
                activityRecorder.deactivateStage(Pipeline::WritebackStageId);
            }

            if (needToSignalDrained) /* Must be draining */
            {
                DPRINTF(Drain, "Still draining\n");
                if (isDrained())
                {
                    DPRINTF(Drain, "Signalling end of draining\n");
                    cpu.signalDrainDone();
                    needToSignalDrained = false;
                    stop();
                }
            }
        }

        std::vector<Scoreboard> &
        Pipeline::getScoreboard()
        {
            return execute.getScoreboard();
        }

        MinorCPU::MinorCPUPort &
        Pipeline::getInstPort()
        {
            return fetch1.getIcachePort();
        }

        MinorCPU::MinorCPUPort &
        Pipeline::getDataPort()
        {
            return memory.getDcachePort();
        }

        void
        Pipeline::wakeupFetch(ThreadID tid)
        {
            fetch1.wakeupFetch(tid);
        }

        bool
        Pipeline::drain()
        {
            DPRINTF(MinorCPU, "Draining pipeline by halting inst fetches. "
                              " Execution should drain naturally\n");

            writeback.drain();
            memory.drain();
            execute.drain();

            /* Make sure that needToSignalDrained isn't accidentally set if we
             *  are 'pre-drained' */
            bool drained = isDrained();
            needToSignalDrained = !drained;

            return drained;
        }

        void
        Pipeline::drainResume()
        {
            DPRINTF(Drain, "Drain resume\n");

            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                fetch1.wakeupFetch(tid);
            }

            execute.drainResume();
            memory.drainResume();
            writeback.drainResume();
        }

        bool
        Pipeline::isDrained()
        {
            bool fetch1_drained = fetch1.isDrained();
            // bool fetch2_drained = fetch2.isDrained();
            bool decode_drained = decode.isDrained();
            bool execute_drained = execute.isDrained();
            bool memory_drained = memory.isDrained();
            bool writeback_drained = writeback.isDrained();

            // bool f1_to_f2_drained = f1ToF2.empty();
            // bool f2_to_f1_drained = f2ToF1.empty();
            // bool f2_to_d_drained = f2ToD.empty();
            bool f1_to_d_drained = f1ToD.empty();
            bool d_to_f1_drained = dToF1.empty();
            bool d_to_e_drained = dToE.empty();
            bool e_to_m_drained = eToM.empty();
            bool m_to_w_drained = mToW.empty();
            /* bool ret = fetch1_drained && fetch2_drained &&
                decode_drained && execute_drained &&
                f1_to_f2_drained && f2_to_f1_drained &&
                f2_to_d_drained && d_to_e_drained; */
            bool ret = fetch1_drained &&
                       decode_drained && execute_drained && writeback_drained &&
                       f1_to_d_drained && d_to_f1_drained && d_to_e_drained && e_to_m_drained &&
                       m_to_w_drained;

            DPRINTF(MinorCPU, "Pipeline undrained stages state:%s%s%s%s%s%s%s%s\n",
                    (fetch1_drained ? "" : " Fetch1"),
                    //(fetch2_drained ? "" : " Fetch2"),
                    (decode_drained ? "" : " Decode"),
                    (execute_drained ? "" : " Execute"),
                    (memory_drained ? "" : " Memory"),
                    (writeback_drained ? "" : "Writeback"),
                    //(f1_to_f2_drained ? "" : " F1->F2"),
                    //(f2_to_f1_drained ? "" : " F2->F1"),
                    //(f2_to_d_drained ? "" : " F2->D"),
                    (f1_to_d_drained ? "" : " F1->D"),
                    (d_to_f1_drained ? "" : " D->F1"),
                    (d_to_e_drained ? "" : " D->E"),
                    (e_to_m_drained ? "" : " E->M"),
                    (m_to_w_drained ? "" : " M->W"));

            return ret;
        }

    } // namespace minor
} // namespace gem5
