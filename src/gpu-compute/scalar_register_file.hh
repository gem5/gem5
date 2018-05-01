/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
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
 *          Mark Wyse
 */

#ifndef __GPU_COMPUTE_SCALAR_REGISTER_FILE_HH__
#define __GPU_COMPUTE_SCALAR_REGISTER_FILE_HH__

#include "arch/gpu_isa.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/GPUSRF.hh"
#include "gpu-compute/register_file.hh"
#include "gpu-compute/wavefront.hh"

struct ScalarRegisterFileParams;

// Scalar Register File
class ScalarRegisterFile : public RegisterFile
{
  public:
    using ScalarRegU32 = TheGpuISA::ScalarRegU32;

    ScalarRegisterFile(const ScalarRegisterFileParams *p);
    ~ScalarRegisterFile() { }

    virtual bool operandsReady(Wavefront *w, GPUDynInstPtr ii) const override;
    virtual void scheduleWriteOperands(Wavefront *w,
                                       GPUDynInstPtr ii) override;
    virtual void scheduleWriteOperandsFromLoad(Wavefront *w,
                                               GPUDynInstPtr ii) override;
    virtual void waveExecuteInst(Wavefront *w, GPUDynInstPtr ii) override;

    void
    setParent(ComputeUnit *_computeUnit) override
    {
        RegisterFile::setParent(_computeUnit);
    }

    // Read a register that is writeable (e.g., a DST operand)
    ScalarRegU32&
    readWriteable(int regIdx)
    {
        return regFile[regIdx];
    }

    // Read a register that is not writeable (e.g., src operand)
    ScalarRegU32
    read(int regIdx) const
    {
        return regFile[regIdx];
    }

    // Write a register
    void
    write(int regIdx, ScalarRegU32 value)
    {
        regFile[regIdx] = value;
    }

    void
    printReg(Wavefront *wf, int regIdx) const
    {
        DPRINTF(GPUSRF, "WF[%d][%d]: Id%d s[%d] = %#x\n", wf->simdId,
            wf->wfSlotId, wf->wfDynId, regIdx, regFile[regIdx]);
    }

  private:
    std::vector<ScalarRegU32> regFile;
};

#endif // __GPU_COMPUTE_SCALAR_REGISTER_FILE_HH__
