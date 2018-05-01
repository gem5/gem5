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
 */

#ifndef __VECTOR_REGISTER_FILE_HH__
#define __VECTOR_REGISTER_FILE_HH__

#include "arch/gpu_isa.hh"
#include "config/the_gpu_isa.hh"
#include "debug/GPUVRF.hh"
#include "gpu-compute/register_file.hh"
#include "gpu-compute/wavefront.hh"

struct VectorRegisterFileParams;

// Vector Register File
class VectorRegisterFile : public RegisterFile
{
  public:
    using VecRegContainer = TheGpuISA::VecRegContainerU32;

    VectorRegisterFile(const VectorRegisterFileParams *p);
    ~VectorRegisterFile() { }

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
    VecRegContainer&
    readWriteable(int regIdx)
    {
        return regFile[regIdx];
    }

    // Read a register that is not writeable (e.g., src operand)
    const VecRegContainer&
    read(int regIdx) const
    {
        return regFile[regIdx];
    }

    // Write a register
    void
    write(int regIdx, const VecRegContainer &value)
    {
        regFile[regIdx] = value;
    }

    void
    printReg(Wavefront *wf, int regIdx) const
    {
#ifndef NDEBUG
        const auto &vec_reg_cont = regFile[regIdx];
        auto vgpr = vec_reg_cont.as<TheGpuISA::VecElemU32>();

        for (int lane = 0; lane < TheGpuISA::NumVecElemPerVecReg; ++lane) {
            if (wf->execMask(lane)) {
                DPRINTF(GPUVRF, "WF[%d][%d]: WV[%d] v[%d][%d] = %#x\n",
                    wf->simdId, wf->wfSlotId, wf->wfDynId, regIdx, lane,
                    vgpr[lane]);
            }
        }
#endif
    }

  private:
    std::vector<VecRegContainer> regFile;
};

#endif // __VECTOR_REGISTER_FILE_HH__
