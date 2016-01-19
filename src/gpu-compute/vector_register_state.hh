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

#ifndef __VECTOR_REGISTER_STATE_HH__
#define __VECTOR_REGISTER_STATE_HH__

#include <array>
#include <cassert>
#include <string>
#include <vector>

#include "gpu-compute/misc.hh"

class ComputeUnit;

// Vector Register State per SIMD unit (contents of the vector
// registers in the VRF of the SIMD)
class VecRegisterState
{
  public:
    VecRegisterState();
    void init(uint32_t _size);

    const std::string& name() const { return _name; }
    void setParent(ComputeUnit *_computeUnit);
    void regStats() { }

    // Access methods
    template<typename T>
    T
    read(int regIdx, int threadId=0) {
        T *p0;
        assert(sizeof(T) == 4 || sizeof(T) == 8);
        if (sizeof(T) == 4) {
            p0 = (T*)(&s_reg[regIdx][threadId]);
        } else {
            p0 = (T*)(&d_reg[regIdx][threadId]);
        }

        return *p0;
    }

    template<typename T>
    void
    write(unsigned int regIdx, T value, int threadId=0) {
        T *p0;
        assert(sizeof(T) == 4 || sizeof(T) == 8);
        if (sizeof(T) == 4) {
            p0 = (T*)(&s_reg[regIdx][threadId]);
        } else {
            p0 = (T*)(&d_reg[regIdx][threadId]);
        }

        *p0 = value;
    }

    // (Single Precision) Vector Register File size.
    int regSize() { return s_reg.size(); }

  private:
    ComputeUnit *computeUnit;
    std::string _name;
    // 32-bit Single Precision Vector Register State
    std::vector<std::array<uint32_t, VSZ>> s_reg;
    // 64-bit Double Precision Vector Register State
    std::vector<std::array<uint64_t, VSZ>> d_reg;
};

#endif // __VECTOR_REGISTER_STATE_HH__
