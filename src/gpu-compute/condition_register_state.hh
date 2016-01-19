/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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
 * Author: John Kalamatianos
 */

#ifndef __CONDITION_REGISTER_STATE_HH__
#define __CONDITION_REGISTER_STATE_HH__

#include <string>
#include <vector>

#include "gpu-compute/misc.hh"

class ComputeUnit;
class GPUStaticInst;
class Shader;
class Wavefront;

// Condition Register State (used only when executing HSAIL)
class ConditionRegisterState
{
  public:
    ConditionRegisterState();
    void init(uint32_t _size);
    const std::string name() const { return _name; }
    void setParent(ComputeUnit *_computeUnit);
    void regStats() { }

    template<typename T>
    T
    read(int regIdx, int threadId)
    {
        bool tmp = c_reg[regIdx][threadId];
        T *p0 = (T*)(&tmp);

        return *p0;
    }

    template<typename T>
    void
    write(int regIdx, int threadId, T value)
    {
        c_reg[regIdx][threadId] = (bool)(value & 0x01);
    }

    void
    markReg(int regIdx, uint8_t value)
    {
        busy.at(regIdx) = value;
    }

    uint8_t
    regBusy(int idx)
    {
        uint8_t status = busy.at(idx);
        return status;
    }

    int numRegs() { return c_reg.size(); }
    void exec(GPUStaticInst *ii, Wavefront *w);

  private:
    ComputeUnit* computeUnit;
    std::string _name;
    // Condition Register state
    std::vector<VectorMask> c_reg;
    // flag indicating if a register is busy
    std::vector<uint8_t> busy;
};

#endif
