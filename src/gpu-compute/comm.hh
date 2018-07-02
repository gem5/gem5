/*
 * Copyright (c) 2018 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Anthony Gutierrez
 */

#ifndef __GPU_COMPUTE_COMM_HH__
#define __GPU_COMPUTE_COMM_HH__

#include <array>
#include <vector>

#include "gpu-compute/exec_stage.hh"
#include "gpu-compute/misc.hh"

struct ComputeUnitParams;
class Wavefront;

class PipeStageIFace
{
  public:
    /**
     * Reset the pipe stage interface. This is called to remove
     * any stale state from the pipe stage that is leftover from
     * the prior cycle. This is needed when stages do not actually
     * consume the information passed via the stage interfaces.
     */
    virtual void reset() = 0;
};

/**
 * Communication interface between ScoreboardCheck and Schedule stages.
 */
class ScoreboardCheckToSchedule : public PipeStageIFace
{
  public:
    ScoreboardCheckToSchedule() = delete;
    ScoreboardCheckToSchedule(const ComputeUnitParams *p);
    void reset() override;
    /**
     * Mark the WF as ready for execution on a particular functional
     * unit.
     */
    void markWFReady(Wavefront *wf, int func_unit_id);
    /**
     * Returns the number of ready lists (i.e., the number of functional
     * units). Each functional unit has its own list of ready WFs to
     * consider for arbitration.
     */
    int numReadyLists() const;
    /**
     * TODO: These methods expose this class' implementation too much by
     *       returning references to its internal data structures directly.
     *       These are to support legacy functionality in the CU pipeline.
     *       They should be removed eventually for an API that hides such
     *       implementation details.
     */
    std::vector<Wavefront*>& readyWFs(int func_unit_id);

    // TODO: Leftover from old CU code, needs to go away.
    void updateReadyList(int func_unit_id);

  private:
    std::vector<std::vector<Wavefront*>> _readyWFs;
};

/**
 * Communication interface between Schedule and Execute stages.
 */
class ScheduleToExecute : public PipeStageIFace
{
  public:
    ScheduleToExecute() = delete;
    ScheduleToExecute(const ComputeUnitParams *p);
    void reset() override;
    GPUDynInstPtr& readyInst(int func_unit_id);
    /**
     * Once the scheduler has chosen a winning WF for execution, and
     * after the WF's oldest instruction's operands have been read,
     * this method is used to mark the instruction as ready to execute.
     * This puts it on the dispatch list to be consumed by the execute
     * stage.
     */
    void dispatchTransition(const GPUDynInstPtr &gpu_dyn_inst,
                            int func_unit_id, DISPATCH_STATUS disp_status);
    void dispatchTransition(int func_unit_id, DISPATCH_STATUS disp_status);
    DISPATCH_STATUS dispatchStatus(int func_unit_id) const;

  private:
    std::vector<GPUDynInstPtr> _readyInsts;
    std::vector<DISPATCH_STATUS> _dispatchStatus;
};

#endif // __GPU_COMPUTE_COMM_HH__
