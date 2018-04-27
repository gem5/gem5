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
 * Authors: John Kalamatianos,
 *          Sooraj Puthoor,
 *          Mark Wyse
 */

#include "gpu-compute/scoreboard_check_stage.hh"

#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "params/ComputeUnit.hh"

ScoreboardCheckStage::ScoreboardCheckStage(const ComputeUnitParams *p)
    : numSIMDs(p->num_SIMDs),
      numMemUnits(p->num_global_mem_pipes + p->num_shared_mem_pipes),
      numShrMemPipes(p->num_shared_mem_pipes),
      vectorAluInstAvail(nullptr),
      lastGlbMemSimd(-1),
      lastShrMemSimd(-1), glbMemInstAvail(nullptr),
      shrMemInstAvail(nullptr)
{
}

ScoreboardCheckStage::~ScoreboardCheckStage()
{
    readyList.clear();
    waveStatusList.clear();
    shrMemInstAvail = nullptr;
    glbMemInstAvail = nullptr;
}

void
ScoreboardCheckStage::init(ComputeUnit *cu)
{
    computeUnit = cu;
    _name = computeUnit->name() + ".ScoreboardCheckStage";

    for (int unitId = 0; unitId < numSIMDs + numMemUnits; ++unitId) {
        readyList.push_back(&computeUnit->readyList[unitId]);
    }

    for (int unitId = 0; unitId < numSIMDs; ++unitId) {
        waveStatusList.push_back(&computeUnit->waveStatusList[unitId]);
    }

    vectorAluInstAvail = &computeUnit->vectorAluInstAvail;
    glbMemInstAvail= &computeUnit->glbMemInstAvail;
    shrMemInstAvail= &computeUnit->shrMemInstAvail;
}

void
ScoreboardCheckStage::initStatistics()
{
    lastGlbMemSimd = -1;
    lastShrMemSimd = -1;
    *glbMemInstAvail = 0;
    *shrMemInstAvail = 0;

    for (int unitId = 0; unitId < numSIMDs; ++unitId)
        vectorAluInstAvail->at(unitId) = false;
}

void
ScoreboardCheckStage::collectStatistics(Wavefront *curWave, int unitId)
{
    if (curWave->instructionBuffer.empty())
        return;

    // track which vector SIMD unit has at least one WV with a vector
    // ALU as the oldest instruction in its Instruction buffer
    vectorAluInstAvail->at(unitId) = vectorAluInstAvail->at(unitId) ||
                                     curWave->isOldestInstALU();

    // track how many vector SIMD units have at least one WV with a
    // vector Global memory instruction as the oldest instruction
    // in its Instruction buffer
    if ((curWave->isOldestInstGMem() || curWave->isOldestInstPrivMem() ||
         curWave->isOldestInstFlatMem()) && lastGlbMemSimd != unitId &&
        *glbMemInstAvail <= 1) {
        (*glbMemInstAvail)++;
        lastGlbMemSimd = unitId;
    }

    // track how many vector SIMD units have at least one WV with a
    // vector shared memory (LDS) instruction as the oldest instruction
    // in its Instruction buffer
    // TODO: parametrize the limit of the LDS units
    if (curWave->isOldestInstLMem() && (*shrMemInstAvail <= numShrMemPipes) &&
        lastShrMemSimd != unitId) {
        (*shrMemInstAvail)++;
        lastShrMemSimd = unitId;
    }
}

void
ScoreboardCheckStage::exec()
{
    initStatistics();

    // reset the ready list for all execution units; it will be
    // constructed every cycle since resource availability may change
    for (int unitId = 0; unitId < numSIMDs + numMemUnits; ++unitId) {
        readyList[unitId]->clear();
    }

    // iterate over the Wavefronts of all SIMD units
    for (int unitId = 0; unitId < numSIMDs; ++unitId) {
        for (int wvId = 0; wvId < computeUnit->shader->n_wf; ++wvId) {
            // reset the ready status of each wavefront
            waveStatusList[unitId]->at(wvId).second = BLOCKED;
            Wavefront *curWave = waveStatusList[unitId]->at(wvId).first;
            collectStatistics(curWave, unitId);

            if (curWave->ready(Wavefront::I_ALU)) {
                readyList[unitId]->push_back(curWave);
                waveStatusList[unitId]->at(wvId).second = READY;
            } else if (curWave->ready(Wavefront::I_GLOBAL)) {
                if (computeUnit->cedeSIMD(unitId, wvId)) {
                    continue;
                }

                readyList[computeUnit->GlbMemUnitId()]->push_back(curWave);
                waveStatusList[unitId]->at(wvId).second = READY;
            } else if (curWave->ready(Wavefront::I_SHARED)) {
                readyList[computeUnit->ShrMemUnitId()]->push_back(curWave);
                waveStatusList[unitId]->at(wvId).second = READY;
            } else if (curWave->ready(Wavefront::I_FLAT)) {
                readyList[computeUnit->GlbMemUnitId()]->push_back(curWave);
                waveStatusList[unitId]->at(wvId).second = READY;
            } else if (curWave->ready(Wavefront::I_PRIVATE)) {
                readyList[computeUnit->GlbMemUnitId()]->push_back(curWave);
                waveStatusList[unitId]->at(wvId).second = READY;
            }
        }
    }
}

void
ScoreboardCheckStage::regStats()
{
}
