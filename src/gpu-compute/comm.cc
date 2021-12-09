/*
 * Copyright (c) 2018 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#include "gpu-compute/comm.hh"

#include <cassert>

#include "gpu-compute/wavefront.hh"
#include "params/ComputeUnit.hh"

namespace gem5
{

/**
 * Scoreboard/Schedule stage interface.
 */
ScoreboardCheckToSchedule::ScoreboardCheckToSchedule(const ComputeUnitParams
                                                     &p)
{
    int num_func_units = p.num_SIMDs + p.num_scalar_cores
        + p.num_global_mem_pipes + p.num_shared_mem_pipes
        + p.num_scalar_mem_pipes;
    _readyWFs.resize(num_func_units);

    for (auto &func_unit_wf_list : _readyWFs) {
        func_unit_wf_list.reserve(p.n_wf);
    }
}

void
ScoreboardCheckToSchedule::reset()
{
    for (auto &func_unit_wf_list : _readyWFs) {
        func_unit_wf_list.resize(0);
    }
}

void
ScoreboardCheckToSchedule::markWFReady(Wavefront *wf, int func_unit_id)
{
    _readyWFs[func_unit_id].push_back(wf);
}

int
ScoreboardCheckToSchedule::numReadyLists() const
{
    return _readyWFs.size();
}

std::vector<Wavefront*>&
ScoreboardCheckToSchedule::readyWFs(int func_unit_id)
{
    return _readyWFs[func_unit_id];
}

/**
 * Delete all wavefronts that have been marked as ready at scoreboard stage
 * but are found to have empty instruction buffers at schedule stage.
 */
void
ScoreboardCheckToSchedule::updateReadyList(int func_unit_id)
{
    std::vector<Wavefront*> &func_unit_wf_list = _readyWFs[func_unit_id];

    for (auto it = func_unit_wf_list.begin(); it != func_unit_wf_list.end();) {
        if ((*it)->instructionBuffer.empty()) {
            it = func_unit_wf_list.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * Schedule/Execute stage interface.
 */
ScheduleToExecute::ScheduleToExecute(const ComputeUnitParams &p)
{
    int num_func_units = p.num_SIMDs + p.num_scalar_cores
        + p.num_global_mem_pipes + p.num_shared_mem_pipes
        + p.num_scalar_mem_pipes;
    _readyInsts.resize(num_func_units, nullptr);
    _dispatchStatus.resize(num_func_units, EMPTY);
}

void
ScheduleToExecute::reset()
{
    for (auto &func_unit_ready_inst : _readyInsts) {
        func_unit_ready_inst = nullptr;
    }

    for (auto &func_unit_status : _dispatchStatus) {
        func_unit_status = EMPTY;
    }
}

GPUDynInstPtr&
ScheduleToExecute::readyInst(int func_unit_id)
{
    return _readyInsts[func_unit_id];
}

void
ScheduleToExecute::dispatchTransition(const GPUDynInstPtr &gpu_dyn_inst,
                                      int func_unit_id,
                                      DISPATCH_STATUS disp_status)
{
    _readyInsts[func_unit_id] = gpu_dyn_inst;
    _dispatchStatus[func_unit_id] = disp_status;
}

void
ScheduleToExecute::dispatchTransition(int func_unit_id,
                                      DISPATCH_STATUS disp_status)
{
    _readyInsts[func_unit_id] = nullptr;
    _dispatchStatus[func_unit_id] = disp_status;
}

DISPATCH_STATUS
ScheduleToExecute::dispatchStatus(int func_unit_id) const
{
    return _dispatchStatus[func_unit_id];
}

} // namespace gem5
