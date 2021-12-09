/*
 * Copyright (c) 2015-2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_VEGA_INSTS_GPU_STATIC_INST_HH__
#define __ARCH_VEGA_INSTS_GPU_STATIC_INST_HH__

#include "arch/amdgpu/vega/gpu_registers.hh"
#include "arch/amdgpu/vega/operand.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

namespace VegaISA
{
    class VEGAGPUStaticInst : public GPUStaticInst
    {
      public:
        VEGAGPUStaticInst(const std::string &opcode);
        ~VEGAGPUStaticInst();

        void generateDisassembly() override { disassembly = _opcode; }

        bool
        isFlatScratchRegister(int opIdx) override
        {
            return isFlatScratchReg(opIdx);
        }

        bool
        isExecMaskRegister(int opIdx) override
        {
            return isExecMask(opIdx);
        }

        void initOperandInfo() override { return; }
        int getOperandSize(int opIdx) override { return 0; }

        /**
          * Return the number of tokens needed by the coalescer. In VEGA there
          * is generally one packet per memory request per lane generated. In
          * HSAIL, the number of dest operands is used for loads and src
          * operands for stores. This method should be overriden on a per-inst
          * basis when this value differs.
          */
        int coalescerTokenCount() const override { return 1; }
        ScalarRegU32 srcLiteral() const override { return _srcLiteral; }

      protected:
        void panicUnimplemented() const;

        /**
         * if the instruction has a src literal - an immediate
         * value that is part of the instruction stream - we
         * store that here
         */
        ScalarRegU32 _srcLiteral;
    }; // class VEGAGPUStaticInst

} // namespace VegaISA
} // namespace gem5

#endif //__ARCH_VEGA_INSTS_GPU_STATIC_INST_HH__
