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

#include <list>

#include "base/statistics.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/GPUVRF.hh"
#include "gpu-compute/vector_register_state.hh"
#include "sim/sim_object.hh"

class ComputeUnit;
class Shader;
class SimplePoolManager;
class Wavefront;

struct VectorRegisterFileParams;

enum class VrfAccessType : uint8_t
{
    READ = 0x01,
    WRITE = 0x02,
    RD_WR = READ | WRITE
};

// Vector Register File
class VectorRegisterFile : public SimObject
{
  public:
    VectorRegisterFile(const VectorRegisterFileParams *p);

    void setParent(ComputeUnit *_computeUnit);

    // Read a register
    template<typename T>
    T
    read(int regIdx, int threadId=0)
    {
        T p0 = vgprState->read<T>(regIdx, threadId);
        DPRINTF(GPUVRF, "reading vreg[%d][%d] = %u\n", regIdx, threadId, (uint64_t)p0);

        return p0;
    }

    // Write a register
    template<typename T>
    void
    write(int regIdx, T value, int threadId=0)
    {
        DPRINTF(GPUVRF, "writing vreg[%d][%d] = %u\n", regIdx, threadId, (uint64_t)value);
        vgprState->write<T>(regIdx, value, threadId);
    }

    uint8_t regBusy(int idx, uint32_t operandSize) const;
    uint8_t regNxtBusy(int idx, uint32_t operandSize) const;

    int numRegs() const { return numRegsPerSimd; }

    void markReg(int regIdx, uint32_t operandSize, uint8_t value);
    void preMarkReg(int regIdx, uint32_t operandSize, uint8_t value);

    virtual void exec(GPUDynInstPtr ii, Wavefront *w);

    virtual int exec(uint64_t dynamic_id, Wavefront *w,
                     std::vector<uint32_t> &regVec, uint32_t operandSize,
                     uint64_t timestamp);

    bool operandsReady(Wavefront *w, GPUDynInstPtr ii) const;
    virtual void updateEvents() { }
    virtual void updateResources(Wavefront *w, GPUDynInstPtr ii);

    virtual bool
    isReadConflict(int memWfId, int exeWfId) const
    {
        return false;
    }

    virtual bool
    isWriteConflict(int memWfId, int exeWfId) const
    {
        return false;
    }

    virtual bool vrfOperandAccessReady(uint64_t dynamic_id, Wavefront *w,
                                       GPUDynInstPtr ii,
                                       VrfAccessType accessType);

    virtual bool vrfOperandAccessReady(Wavefront *w, GPUDynInstPtr ii,
                                       VrfAccessType accessType);

    SimplePoolManager *manager;

  protected:
    ComputeUnit* computeUnit;
    int simdId;

    // flag indicating if a register is busy
    std::vector<uint8_t> busy;
    // flag indicating if a register will be busy (by instructions
    // in the SIMD pipeline)
    std::vector<uint8_t> nxtBusy;

    // numer of registers (bank size) per simd unit (bank)
    int numRegsPerSimd;

    // vector register state
    VecRegisterState *vgprState;
};

#endif // __VECTOR_REGISTER_FILE_HH__
