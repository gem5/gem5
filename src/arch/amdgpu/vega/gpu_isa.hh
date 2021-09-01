/*
 * Copyright (c) 2016-2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_VEGA_GPU_ISA_HH__
#define __ARCH_VEGA_GPU_ISA_HH__

#include <array>
#include <type_traits>

#include "arch/amdgpu/vega/gpu_registers.hh"
#include "arch/amdgpu/vega/tlb.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "gpu-compute/misc.hh"

namespace gem5
{

class Wavefront;

namespace VegaISA
{
    class GPUISA
    {
      public:
        GPUISA(Wavefront &wf);

        template<typename T> T
        readConstVal(int opIdx) const
        {
            panic_if(!std::is_integral_v<T>,
                    "Constant values must be an integer.");
            T val(0);

            if (isPosConstVal(opIdx)) {
                val = (T)readPosConstReg(opIdx);
            }

            if (isNegConstVal(opIdx)) {
                val = (T)readNegConstReg(opIdx);
            }

            return val;
        }

        ScalarRegU32 readMiscReg(int opIdx) const;
        void writeMiscReg(int opIdx, ScalarRegU32 operandVal);
        bool hasScalarUnit() const { return true; }
        void advancePC(GPUDynInstPtr gpuDynInst);

      private:
        ScalarRegU32 readPosConstReg(int opIdx) const
        {
            return posConstRegs[opIdx - REG_INT_CONST_POS_MIN];
        }

        ScalarRegI32 readNegConstReg(int opIdx) const
        {
            return negConstRegs[opIdx - REG_INT_CONST_NEG_MIN];
        }

        static const std::array<const ScalarRegU32, NumPosConstRegs>
            posConstRegs;
        static const std::array<const ScalarRegI32, NumNegConstRegs>
            negConstRegs;

        // parent wavefront
        Wavefront &wavefront;

        // shader status bits
        StatusReg statusReg;
        // memory descriptor reg
        ScalarRegU32 m0;
    };
} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_GPU_ISA_HH__
