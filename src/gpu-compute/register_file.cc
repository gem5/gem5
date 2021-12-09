/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
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

#include "gpu-compute/register_file.hh"

#include <sstream>
#include <string>

#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/GPURF.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "params/RegisterFile.hh"

namespace gem5
{

RegisterFile::RegisterFile(const RegisterFileParams &p)
    : SimObject(p), simdId(p.simd_id), _numRegs(p.num_regs), stats(this)
{
    fatal_if((_numRegs % 2) != 0, "VRF size is illegal\n");
    fatal_if(simdId < 0, "Illegal SIMD id for VRF");

    busy.clear();
    busy.resize(_numRegs, 0);
}

RegisterFile::~RegisterFile()
{
}

void
RegisterFile::setParent(ComputeUnit *_computeUnit)
{
    computeUnit = _computeUnit;
}

std::string
RegisterFile::dump() const
{
    std::stringstream ss;
    ss << "Busy: ";
    for (int i = 0; i < busy.size(); i++) {
        ss << (int)busy[i];
    }
    ss << "\n";
    return ss.str();
}

// Scoreboard functions

bool
RegisterFile::operandsReady(Wavefront *w, GPUDynInstPtr ii) const
{
    return true;
}

bool
RegisterFile::regBusy(int idx) const
{
    return busy.at(idx);
}

void
RegisterFile::markReg(int regIdx, bool value)
{
    DPRINTF(GPURF, "SIMD[%d] markReg(): physReg[%d] = %d\n",
            simdId, regIdx, (int)value);
    busy.at(regIdx) = value;
}

void
RegisterFile::enqRegFreeEvent(uint32_t regIdx, uint64_t delay)
{
    DPRINTF(GPURF, "SIMD[%d] enqRegFreeEvent physReg[%d] at %llu\n",
            simdId, regIdx, curTick() + delay);
    schedule(new MarkRegFreeScbEvent(this, regIdx),
             curTick() + delay);
}

void
RegisterFile::enqRegBusyEvent(uint32_t regIdx, uint64_t delay)
{
    DPRINTF(GPURF, "SIMD[%d] enqRegBusyEvent physReg[%d] at %llu\n",
            simdId, regIdx, curTick() + delay);
    schedule(new MarkRegBusyScbEvent(this, regIdx),
             curTick() + delay);
}

// Schedule functions
bool
RegisterFile::canScheduleReadOperands(Wavefront *w, GPUDynInstPtr ii)
{
    return true;
}

void
RegisterFile::scheduleReadOperands(Wavefront *w, GPUDynInstPtr ii)
{
}

bool
RegisterFile::canScheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii)
{
    return true;
}

void
RegisterFile::scheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii)
{
}

bool
RegisterFile::canScheduleWriteOperandsFromLoad(Wavefront *w, GPUDynInstPtr ii)
{
    return true;
}

void
RegisterFile::scheduleWriteOperandsFromLoad(Wavefront *w, GPUDynInstPtr ii)
{
}

bool
RegisterFile::operandReadComplete(Wavefront *w, GPUDynInstPtr ii)
{
    return true;
}

// Exec functions
void
RegisterFile::exec()
{
}

void
RegisterFile::waveExecuteInst(Wavefront *w, GPUDynInstPtr ii)
{
}

// Events

// Mark a register as free in the scoreboard/busy vector
void
RegisterFile::MarkRegFreeScbEvent::process()
{
    rf->markReg(regIdx, false);
}

// Mark a register as busy in the scoreboard/busy vector
void
RegisterFile::MarkRegBusyScbEvent::process()
{
    rf->markReg(regIdx, true);
}

void
RegisterFile::dispatchInstruction(GPUDynInstPtr ii)
{
}

RegisterFile::RegisterFileStats::RegisterFileStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(registerReads,
              "Total number of DWORDs read from register file"),
      ADD_STAT(registerWrites,
              "Total number of DWORDS written to register file"),
      ADD_STAT(sramReads,
              "Total number of register file bank SRAM activations for reads"),
      ADD_STAT(sramWrites,
              "Total number of register file bank SRAM activations for writes")
{
}

} // namespace gem5
