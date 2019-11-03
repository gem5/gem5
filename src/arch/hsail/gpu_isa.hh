/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
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
 * Authors: Anthony Gutierrez
 */

#ifndef __ARCH_HSAIL_GPU_ISA_HH__
#define __ARCH_HSAIL_GPU_ISA_HH__

#include <cstdint>

#include "arch/hsail/gpu_types.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "gpu-compute/misc.hh"

namespace HsailISA
{
    class GPUISA
    {
      public:
        GPUISA()
        {
        }

        void
        writeMiscReg(int opIdx, RegVal operandVal)
        {
            fatal("HSAIL does not implement misc registers yet\n");
        }

        RegVal
        readMiscReg(int opIdx) const
        {
            fatal("HSAIL does not implement misc registers yet\n");
        }

        bool hasScalarUnit() const { return false; }

        uint32_t
        advancePC(uint32_t old_pc, GPUDynInstPtr gpuDynInst)
        {
            return old_pc + sizeof(RawMachInst);
        }
    };
}

#endif // __ARCH_HSAIL_GPU_ISA_HH__
