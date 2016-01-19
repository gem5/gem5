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
 * Author: Anthony Gutierrez
 */

#include "arch/hsail/insts/branch.hh"

#include "gpu-compute/hsail_code.hh"

namespace HsailISA
{
    GPUStaticInst*
    decodeBrn(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        // Detect direct vs indirect branch by seeing whether we have a
        // register operand.
        unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
        const Brig::BrigOperand *reg = obj->getOperand(op_offs);

        if (reg->kind == Brig::BRIG_KIND_OPERAND_REGISTER) {
            return new BrnIndirectInst(ib, obj);
        } else {
            return new BrnDirectInst(ib, obj);
        }
    }

    GPUStaticInst*
    decodeCbr(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        // Detect direct vs indirect branch by seeing whether we have a
        // second register operand (after the condition).
        unsigned op_offs = obj->getOperandPtr(ib->operands, 1);
        const Brig::BrigOperand *reg = obj->getOperand(op_offs);

        if (reg->kind == Brig::BRIG_KIND_OPERAND_REGISTER) {
            return new CbrIndirectInst(ib, obj);
        } else {
            return new CbrDirectInst(ib, obj);
        }
    }

    GPUStaticInst*
    decodeBr(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        // Detect direct vs indirect branch by seeing whether we have a
        // second register operand (after the condition).
        unsigned op_offs = obj->getOperandPtr(ib->operands, 1);
        const Brig::BrigOperand *reg = obj->getOperand(op_offs);

        if (reg->kind == Brig::BRIG_KIND_OPERAND_REGISTER) {
            return new BrIndirectInst(ib, obj);
        } else {
            return new BrDirectInst(ib, obj);
        }
    }
} // namespace HsailISA
