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
 * Authors: Anthony Gutierrez
 */

#include "arch/gcn3/registers.hh"

namespace Gcn3ISA
{
    std::string
    opSelectorToRegSym(int idx, int numRegs)
    {
        std::string reg_sym;

        // we have an SGPR
        if (idx <= REG_SGPR_MAX) {
            if (numRegs > 1)
                reg_sym = "s[" + std::to_string(idx) + ":" +
                    std::to_string(idx + numRegs - 1) + "]";
            else
                reg_sym = "s" + std::to_string(idx);
            return reg_sym;
        } else if (idx >= REG_VGPR_MIN && idx <= REG_VGPR_MAX) {
            if (numRegs > 1)
                reg_sym = "v[" + std::to_string(idx - REG_VGPR_MIN) + ":" +
                    std::to_string(idx - REG_VGPR_MIN + numRegs - 1) + "]";
            else
                reg_sym = "v" + std::to_string(idx - REG_VGPR_MIN);
            return reg_sym;
        } else if (idx >= REG_INT_CONST_POS_MIN &&
                   idx <= REG_INT_CONST_POS_MAX) {
            reg_sym = std::to_string(idx - REG_INT_CONST_POS_MIN + 1);
            return reg_sym;
        } else if (idx >= REG_INT_CONST_NEG_MIN &&
                   idx <= REG_INT_CONST_NEG_MAX) {
            int inline_val = -1 - (idx - REG_INT_CONST_NEG_MIN);
            reg_sym = std::to_string(inline_val);
            return reg_sym;
        }

        switch (idx) {
          case REG_FLAT_SCRATCH_LO:
            reg_sym = "flat_scratch_lo";
            break;
          case REG_FLAT_SCRATCH_HI:
            reg_sym = "flat_scratch_hi";
            break;
          case REG_VCC_LO:
            reg_sym = "vcc";
            break;
          case REG_M0:
            reg_sym = "m0";
            break;
          case REG_EXEC_LO:
            reg_sym = "exec";
            break;
          case REG_ZERO:
            reg_sym = "0";
            break;
          case REG_POS_HALF:
            reg_sym = "0.5";
            break;
          case REG_NEG_HALF:
            reg_sym = "-0.5";
            break;
          case REG_POS_ONE:
            reg_sym = "1";
            break;
          case REG_NEG_ONE:
            reg_sym = "-1";
            break;
          case REG_POS_TWO:
            reg_sym = "2";
            break;
          case REG_NEG_TWO:
            reg_sym = "-2";
            break;
          case REG_POS_FOUR:
            reg_sym = "4";
            break;
          case REG_NEG_FOUR:
            reg_sym = "-4";
            break;
          default:
            fatal("GCN3 ISA instruction has unknown register index %u\n", idx);
            break;
        }

        return reg_sym;
    }

    int
    opSelectorToRegIdx(int idx, int numScalarRegs)
    {
        int regIdx = -1;

        if (idx <= REG_SGPR_MAX) {
            regIdx = idx;
        } else if (idx >= REG_VGPR_MIN && idx <= REG_VGPR_MAX) {
            regIdx = idx - REG_VGPR_MIN;
        } else if (idx == REG_VCC_LO) {
            /**
             * the VCC register occupies the two highest numbered
             * SRF entries. VCC is typically indexed by specifying
             * VCC_LO (simply called VCC) in the instruction encoding
             * and reading it as a 64b value so we only return the
             * index to the lower half of the VCC register.
             *
             * VCC_LO = s[NUM_SGPRS - 2]
             * VCC_HI = s[NUM_SGPRS - 1]
             *
             */
            regIdx = numScalarRegs - 2;
        } else if (idx == REG_FLAT_SCRATCH_LO) {
            /**
             * the FLAT_SCRATCH register occupies the two SRF entries
             * just below VCC. FLAT_SCRATCH is typically indexed by
             * specifying FLAT_SCRATCH_LO (simply called FLAT_SCRATCH)
             * in the instruction encoding and reading it as a 64b value
             * so we only return the index to the lower half of the
             * FLAT_SCRATCH register.
             *
             * FLAT_SCRATCH_LO = s[NUM_SGPRS - 4]
             * FLAT_SCRATCH_HI = s[NUM_SGPRS - 3]
             *
             */
            regIdx = numScalarRegs - 4;
        } else if (idx == REG_FLAT_SCRATCH_HI) {
            regIdx = numScalarRegs - 3;
        }

        return regIdx;
    }

    bool
    isLiteral(int opIdx)
    {
        return opIdx == REG_SRC_LITERAL;
    }

    bool
    isExecMask(int opIdx)
    {
        return opIdx == REG_EXEC_LO || opIdx == REG_EXEC_HI;
    }

    bool
    isVccReg(int opIdx)
    {
        return opIdx == REG_VCC_LO || opIdx == REG_VCC_HI;
    }

    bool
    isFlatScratchReg(int opIdx)
    {
        return opIdx == REG_FLAT_SCRATCH_LO || opIdx == REG_FLAT_SCRATCH_HI;
    }

    bool
    isScalarReg(int opIdx)
    {
        // FLAT_SCRATCH and VCC are stored in an SGPR pair
        if (opIdx <= REG_SGPR_MAX || opIdx == REG_FLAT_SCRATCH_LO ||
            opIdx == REG_FLAT_SCRATCH_HI || opIdx == REG_VCC_LO ||
            opIdx == REG_VCC_HI) {
            return true;
        }

        return false;
    }

    bool
    isVectorReg(int opIdx)
    {
        if (opIdx >= REG_VGPR_MIN && opIdx <= REG_VGPR_MAX)
            return true;

        return false;
    }

} // namespace Gcn3ISA
