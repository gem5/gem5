/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: Sooraj Puthoor
 */

#include "gpu-compute/schedule_stage.hh"

#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

ScheduleStage::ScheduleStage(const ComputeUnitParams *p)
    : numSIMDs(p->num_SIMDs),
      numMemUnits(p->num_global_mem_pipes + p->num_shared_mem_pipes)
{
    for (int j = 0; j < numSIMDs + numMemUnits; ++j) {
        Scheduler newScheduler(p);
        scheduler.push_back(newScheduler);
    }
}

ScheduleStage::~ScheduleStage()
{
    scheduler.clear();
    waveStatusList.clear();
}

void
ScheduleStage::init(ComputeUnit *cu)
{
    computeUnit = cu;
    _name = computeUnit->name() + ".ScheduleStage";

    for (int j = 0; j < numSIMDs + numMemUnits; ++j) {
        scheduler[j].bindList(&computeUnit->readyList[j]);
    }

    for (int j = 0; j < numSIMDs; ++j) {
        waveStatusList.push_back(&computeUnit->waveStatusList[j]);
    }

    dispatchList = &computeUnit->dispatchList;
}

void
ScheduleStage::arbitrate()
{
    // iterate over all Memory pipelines
    for (int j = numSIMDs; j < numSIMDs + numMemUnits; ++j) {
        if (dispatchList->at(j).first) {
            Wavefront *waveToMemPipe = dispatchList->at(j).first;
            // iterate over all execution pipelines
            for (int i = 0; i < numSIMDs + numMemUnits; ++i) {
                if ((i != j) && (dispatchList->at(i).first)) {
                    Wavefront *waveToExePipe = dispatchList->at(i).first;
                    // if the two selected wavefronts are mapped to the same
                    // SIMD unit then they share the VRF
                    if (waveToMemPipe->simdId == waveToExePipe->simdId) {
                        int simdId = waveToMemPipe->simdId;
                        // Read VRF port arbitration:
                        // If there are read VRF port conflicts between the
                        // a memory and another instruction we drop the other
                        // instruction. We don't need to check for write VRF
                        // port conflicts because the memory instruction either
                        // does not need to write to the VRF (store) or will
                        // write to the VRF when the data comes back (load) in
                        // which case the arbiter of the memory pipes will
                        // resolve any conflicts
                        if (computeUnit->vrf[simdId]->
                            isReadConflict(waveToMemPipe->wfSlotId,
                            waveToExePipe->wfSlotId)) {
                            // FIXME: The "second" member variable is never
                            // used in the model. I am setting it to READY
                            // simply to follow the protocol of setting it
                            // when the WF has an instruction ready to issue
                            waveStatusList[simdId]->at(waveToExePipe->wfSlotId)
                                                    .second = READY;

                            dispatchList->at(i).first = nullptr;
                            dispatchList->at(i).second = EMPTY;
                            break;
                        }
                    }
                }
            }
        }
    }
}

void
ScheduleStage::exec()
{
    for (int j = 0; j < numSIMDs + numMemUnits; ++j) {
         uint32_t readyListSize = computeUnit->readyList[j].size();

         // If no wave is ready to be scheduled on the execution resource
         // then skip scheduling for this execution resource
         if (!readyListSize) {
             continue;
         }

         Wavefront *waveToBeDispatched = scheduler[j].chooseWave();
         dispatchList->at(j).first = waveToBeDispatched;
         waveToBeDispatched->updateResources();
         dispatchList->at(j).second = FILLED;

         waveStatusList[waveToBeDispatched->simdId]->at(
                 waveToBeDispatched->wfSlotId).second = BLOCKED;

         assert(computeUnit->readyList[j].size() == readyListSize - 1);
    }
    // arbitrate over all shared resources among instructions being issued
    // simultaneously
    arbitrate();
}

void
ScheduleStage::regStats()
{
}
