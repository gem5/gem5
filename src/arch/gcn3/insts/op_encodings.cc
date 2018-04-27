/*
 * Copyright (c) 2016-2017 Advanced Micro Devices, Inc.
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

#include "arch/gcn3/insts/op_encodings.hh"

#include <iomanip>

namespace Gcn3ISA
{
    // --- Inst_SOP2 base class methods ---

    Inst_SOP2::Inst_SOP2(InFmt_SOP2 *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Scalar);

        // copy first instruction DWORD
        instData = iFmt[0];
        if (hasSecondDword(iFmt)) {
            // copy second instruction DWORD into union
            extData = ((MachInst)iFmt)[1];
            _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
            varSize = 4 + 4;
        } else {
            varSize = 4;
        } // if
    } // Inst_SOP2

    int
    Inst_SOP2::instSize() const
    {
        return varSize;
    } // instSize

    bool
    Inst_SOP2::hasSecondDword(InFmt_SOP2 *iFmt)
    {
        if (iFmt->SSRC0 == REG_SRC_LITERAL)
            return true;

        if (iFmt->SSRC1 == REG_SRC_LITERAL)
            return true;

        return false;
    }

    void
    Inst_SOP2::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        dis_stream << opSelectorToRegSym(instData.SDST) << ", ";

        if (instData.SSRC0 == REG_SRC_LITERAL) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << _srcLiteral << ", ";
        } else {
            dis_stream << opSelectorToRegSym(instData.SSRC0) << ", ";
        }

        if (instData.SSRC1 == REG_SRC_LITERAL) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << _srcLiteral;
        } else {
            dis_stream << opSelectorToRegSym(instData.SSRC1);
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_SOP2::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return isScalarReg(instData.SSRC0);
          case 1:
            return isScalarReg(instData.SSRC1);
          case 2:
            return isScalarReg(instData.SDST);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_SOP2::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOP2 instructions cannot access VGPRs
        return false;
    }

    int
    Inst_SOP2::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return opSelectorToRegIdx(instData.SSRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            return opSelectorToRegIdx(instData.SSRC1,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 2:
            return opSelectorToRegIdx(instData.SDST,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_SOPK base class methods ---

    Inst_SOPK::Inst_SOPK(InFmt_SOPK *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Scalar);

        // copy first instruction DWORD
        instData = iFmt[0];
    } // Inst_SOPK

    Inst_SOPK::~Inst_SOPK()
    {
    } // ~Inst_SOPK

    int
    Inst_SOPK::instSize() const
    {
        return 4;
    } // instSize

    void
    Inst_SOPK::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        dis_stream << opSelectorToRegSym(instData.SDST) << ", ";

            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(4)
                       << instData.SIMM16;

        disassembly = dis_stream.str();
    }

    bool
    Inst_SOPK::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
              return false;
          case 1:
              return isScalarReg(instData.SDST);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_SOPK::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOPK instruction cannot access VGPRs
        return false;
    }

    int
    Inst_SOPK::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return  -1;
          case 1:
            return opSelectorToRegIdx(instData.SDST,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_SOP1 base class methods ---

    Inst_SOP1::Inst_SOP1(InFmt_SOP1 *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Scalar);

        // copy first instruction DWORD
        instData = iFmt[0];
        if (hasSecondDword(iFmt)) {
            // copy second instruction DWORD into union
            extData = ((MachInst)iFmt)[1];
            _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
            varSize = 4 + 4;
        } else {
            varSize = 4;
        } // if
    } // Inst_SOP1

    Inst_SOP1::~Inst_SOP1()
    {
    } // ~Inst_SOP1

    int
    Inst_SOP1::instSize() const
    {
        return varSize;
    } // instSize

    bool
    Inst_SOP1::hasSecondDword(InFmt_SOP1 *iFmt)
    {
        if (iFmt->SSRC0 == REG_SRC_LITERAL)
            return true;

        return false;
    }

    void
    Inst_SOP1::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        dis_stream << opSelectorToRegSym(instData.SDST) << ", ";

        if (instData.SSRC0 == REG_SRC_LITERAL) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << extData.imm_u32;
        } else {
            dis_stream << opSelectorToRegSym(instData.SSRC0);
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_SOP1::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
              return isScalarReg(instData.SSRC0);
          case 1:
              return isScalarReg(instData.SDST);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_SOP1::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOP1 instruction cannot access VGPRs
        return false;
    }

    int
    Inst_SOP1::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return opSelectorToRegIdx(instData.SSRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            return opSelectorToRegIdx(instData.SDST,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_SOPC base class methods ---

    Inst_SOPC::Inst_SOPC(InFmt_SOPC *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Scalar);

        // copy first instruction DWORD
        instData = iFmt[0];
        if (hasSecondDword(iFmt)) {
            // copy second instruction DWORD into union
            extData = ((MachInst)iFmt)[1];
            _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
            varSize = 4 + 4;
        } else {
            varSize = 4;
        } // if
    } // Inst_SOPC

    Inst_SOPC::~Inst_SOPC()
    {
    } // ~Inst_SOPC

    int
    Inst_SOPC::instSize() const
    {
        return varSize;
    } // instSize

    bool
    Inst_SOPC::hasSecondDword(InFmt_SOPC *iFmt)
    {
        if (iFmt->SSRC0 == REG_SRC_LITERAL)
            return true;

        if (iFmt->SSRC1 == REG_SRC_LITERAL)
            return true;

        return false;
    }

    void
    Inst_SOPC::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";

        if (instData.SSRC0 == REG_SRC_LITERAL) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << extData.imm_u32;
        } else {
            dis_stream << opSelectorToRegSym(instData.SSRC0) << ", ";
        }

        if (instData.SSRC1 == REG_SRC_LITERAL) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << extData.imm_u32;
        } else {
            dis_stream << opSelectorToRegSym(instData.SSRC1);
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_SOPC::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
              // SSRC0 is always a scalar reg or a constant
              return isScalarReg(instData.SSRC0);
          case 1:
              // SSRC1 is always a scalar reg or a constant
              return isScalarReg(instData.SSRC1);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_SOPC::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOPC instructions cannot access VGPRs
        return false;
    }

    int
    Inst_SOPC::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return opSelectorToRegIdx(instData.SSRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            return opSelectorToRegIdx(instData.SSRC1,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_SOPP base class methods ---

    Inst_SOPP::Inst_SOPP(InFmt_SOPP *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Scalar);

        // copy first instruction DWORD
        instData = iFmt[0];
    } // Inst_SOPP

    Inst_SOPP::~Inst_SOPP()
    {
    } // ~Inst_SOPP

    int
    Inst_SOPP::instSize() const
    {
        return 4;
    } // instSize

    void
    Inst_SOPP::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode;

        switch (instData.OP) {
          case 8:
            {
                dis_stream << " ";
                int dest = 4 * instData.SIMM16 + 4;
                dis_stream << "label_" << std::hex << dest;
            }
             break;
          case 12:
            {
                dis_stream << " ";

                int vm_cnt = 0;
                int exp_cnt = 0;
                int lgkm_cnt = 0;

                vm_cnt = bits<uint16_t>(instData.SIMM16, 3, 0);
                exp_cnt = bits<uint16_t>(instData.SIMM16, 6, 4);
                lgkm_cnt = bits<uint16_t>(instData.SIMM16, 11, 8);

                // if the counts are not maxed out, then we
                // print out the count value
                if (vm_cnt != 0xf) {
                    dis_stream << "vmcnt(" << vm_cnt << ")";
                }

                if (lgkm_cnt != 0xf) {
                    if (vm_cnt != 0xf)
                        dis_stream << " & ";

                    dis_stream << "lgkmcnt(" << lgkm_cnt << ")";
                }

                if (exp_cnt != 0x7) {
                    if (vm_cnt != 0xf || lgkm_cnt != 0xf)
                        dis_stream << " & ";

                    dis_stream << "expcnt(" << exp_cnt << ")";
                }
            }
            break;
          default:
            break;
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_SOPP::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOPP instructions have a maximum of 1 operand,
        // and it's always an immediate value
        return false;
    }

    bool
    Inst_SOPP::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOPP instructions have a maximum of 1 operand,
        // and it's always an immediate value
        return false;
    }

    int
    Inst_SOPP::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SOPP instructions have a maximum of 1 operand,
        // and it's always an immediate value
        return -1;
    }

    // --- Inst_SMEM base class methods ---

    Inst_SMEM::Inst_SMEM(InFmt_SMEM *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Scalar);
        setFlag(GlobalSegment);

        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_SMEM_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);

        if (instData.GLC)
            setFlag(GloballyCoherent);
    } // Inst_SMEM

    Inst_SMEM::~Inst_SMEM()
    {
    } // ~Inst_SMEM

    int
    Inst_SMEM::instSize() const
    {
        return 8;
    } // instSize

    void
    Inst_SMEM::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        if (numDstRegOperands()) {
            if (getOperandSize(getNumOperands() - 1) > 4) {
                dis_stream << "s[" << instData.SDATA << ":"
                    << instData.SDATA + getOperandSize(getNumOperands() - 1) /
                    4 - 1 << "], ";
            } else {
                dis_stream << "s" << instData.SDATA << ", ";
            }
        }

        // SBASE has an implied LSB of 0, so we need
        // to shift by one to get the actual value
        dis_stream << "s[" << (instData.SBASE << 1) << ":"
               << ((instData.SBASE << 1) + 1) << "], ";

        if (instData.IMM) {
            // IMM == 1 implies OFFSET should be
            // used as the offset
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(2)
                       << extData.OFFSET;
        } else {
            // IMM == 0 implies OFFSET should be
            // used to specify SGRP in which the
            // offset is held
            dis_stream << "s" << extData.OFFSET;
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_SMEM::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SBASE is always a scalar
            return true;
          case 1:
            if (instData.IMM) {
                return false;
            } else {
                return isScalarReg(extData.OFFSET);
            }
          case 2:
            return isScalarReg(instData.SDATA);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_SMEM::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // SMEM instructions cannot access VGPRs
        return false;
    }

    int
    Inst_SMEM::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SBASE has an implied LSB of 0, so we need
            // to shift by one to get the actual value
            return opSelectorToRegIdx(instData.SBASE << 1,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            if (instData.IMM) {
              // operand is an immediate value, not a register
              return -1;
            } else {
              return extData.OFFSET;
            }
          case 2:
            return opSelectorToRegIdx(instData.SDATA,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_VOP2 base class methods ---

    Inst_VOP2::Inst_VOP2(InFmt_VOP2 *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        if (hasSecondDword(iFmt)) {
            // copy second instruction DWORD into union
            extData = ((MachInst)iFmt)[1];
            _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
            varSize = 4 + 4;
            if (iFmt->SRC0 == REG_SRC_DPP) {
                setFlag(IsDPP);
            } else if (iFmt->SRC0 == REG_SRC_SWDA) {
                setFlag(IsSDWA);
            }
        } else {
            varSize = 4;
        } // if
    } // Inst_VOP2

    Inst_VOP2::~Inst_VOP2()
    {
    } // ~Inst_VOP2

    int
    Inst_VOP2::instSize() const
    {
        return varSize;
    } // instSize

    bool
    Inst_VOP2::hasSecondDword(InFmt_VOP2 *iFmt)
    {
        /*
          There are a few cases where VOP2 instructions have a second dword:

            1.  SRC0 is a literal
            2.  SRC0 is being used to add a data parallel primitive (DPP)
            operation to the instruction.
            3.  SRC0 is being used for sub d-word addressing (SDWA) of the
            operands in the instruction.
            4.  VOP2 instructions also have four special opcodes:',
            V_MADMK_{F16, F32} (0x24, 0x17), and V_MADAK_{F16, F32}',
            (0x25, 0x18), that are always 64b. the only way to',
            detect these special cases is to explicitly check,',
            the opcodes',
        */
        if (iFmt->SRC0 == REG_SRC_LITERAL || (iFmt->SRC0 == REG_SRC_DPP) ||
            (iFmt->SRC0 == REG_SRC_SWDA) || iFmt->OP == 0x17 ||
            iFmt->OP == 0x18 || iFmt->OP == 0x24 || iFmt->OP == 0x25)
            return true;

        return false;
    }

    void
    Inst_VOP2::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        dis_stream << "v" << instData.VDST << ", ";

        if (writesVCC())
            dis_stream << "vcc, ";

        if ((instData.SRC0 == REG_SRC_LITERAL) ||
            (instData.SRC0 == REG_SRC_DPP) ||
            (instData.SRC0 == REG_SRC_SWDA)) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << _srcLiteral << ", ";
        } else {
            dis_stream << opSelectorToRegSym(instData.SRC0) << ", ";
        }

        // VOP2 instructions have four special opcodes:',
        // V_MADMK_{F16, F32} (0x24, 0x17), and V_MADAK_{F16, F32}',
        // (0x25, 0x18), that are always 64b. the only way to',
        // detect these special cases is to explicitly check,',
        // the opcodes',
        if (instData.OP == 0x17 || instData.OP == 0x18 || instData.OP == 0x24
            || instData.OP == 0x25) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << extData.imm_u32 << ", ";
        }

        dis_stream << "v" << instData.VSRC1;

        if (readsVCC())
            dis_stream << ", vcc";

        disassembly = dis_stream.str();
    }

    bool
    Inst_VOP2::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0 may be a scalar or vector register, an
            // inline constant, or a special HW register
            return isScalarReg(instData.SRC0);
          case 1:
            // instData.VSRC1 is never a scalar reg
            return false;
          case 2:
            if (readsVCC()) {
                return true;
            } else {
                // instData.VDST is never a scalar reg
                return false;
            }
          case 3:
            // if a VOP2 instruction has more than 3 ops
            // it must read from or write to VCC, and
            // VCC is always in an SGPR
            assert(readsVCC() || writesVCC());
            if (readsVCC()) {
                return false;
            } else {
                return true;
            }
          case 4:
            // if a VOP2 instruction has more than 4 ops
            // it must read from and write to VCC, and
            // VCC is always in an SGPR
            assert(writesVCC() && readsVCC());
            return true;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_VOP2::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0 may be a scalar or vector register, an
            // inline constant, or a special HW register
            return isVectorReg(instData.SRC0);
          case 1:
            // instData.VSRC1 is always a vector reg
            return true;
          case 2:
            if (readsVCC()) {
                return false;
            } else {
                // instData.VDST is always a vector reg
                return true;
            }
          case 3:
            // if a VOP2 instruction has more than 3 ops
            // it must read from or write to VCC, and
            // VCC is always in an SGPR
            assert(writesVCC() || readsVCC());
            if (readsVCC()) {
                return true;
            } else {
                return false;
            }
          case 4:
            // if a VOP2 instruction has more than 4 ops
            // it must read from and write to VCC, and
            // VCC is always in an SGPR
            assert(writesVCC() && readsVCC());
            return false;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    int
    Inst_VOP2::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return opSelectorToRegIdx(instData.SRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            return instData.VSRC1;
          case 2:
            if (readsVCC()) {
                return opSelectorToRegIdx(REG_VCC_LO,
                        gpuDynInst->wavefront()->reservedScalarRegs);
            } else {
                return instData.VDST;
            }
          case 3:
            assert(writesVCC() || readsVCC());
            if (readsVCC()) {
                return instData.VDST;
            } else {
                return opSelectorToRegIdx(REG_VCC_LO,
                        gpuDynInst->wavefront()->reservedScalarRegs);
            }
          case 4:
            assert(writesVCC() && readsVCC());
            return opSelectorToRegIdx(REG_VCC_LO,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_VOP1 base class methods ---

    Inst_VOP1::Inst_VOP1(InFmt_VOP1 *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        if (hasSecondDword(iFmt)) {
            // copy second instruction DWORD into union
            extData = ((MachInst)iFmt)[1];
            _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
            varSize = 4 + 4;
            if (iFmt->SRC0 == REG_SRC_DPP) {
                setFlag(IsDPP);
            } else if (iFmt->SRC0 == REG_SRC_SWDA) {
                setFlag(IsSDWA);
            }
        } else {
            varSize = 4;
        } // if
    } // Inst_VOP1

    Inst_VOP1::~Inst_VOP1()
    {
    } // ~Inst_VOP1

    int
    Inst_VOP1::instSize() const
    {
        return varSize;
    } // instSize

    bool
    Inst_VOP1::hasSecondDword(InFmt_VOP1 *iFmt)
    {
        /*
          There are several cases where VOP1 instructions have a second dword:

            1.  SRC0 is a literal.
            2.  SRC0 is being used to add a data parallel primitive (DPP)
            operation to the instruction.
            3.  SRC0 is being used for sub d-word addressing (SDWA) of the
            operands in the instruction.
        */
        if ((iFmt->SRC0 == REG_SRC_LITERAL) || (iFmt->SRC0 == REG_SRC_DPP) ||
            (iFmt->SRC0 == REG_SRC_SWDA))
            return true;

        return false;
    }

    void
    Inst_VOP1::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        dis_stream << "v" << instData.VDST << ", ";

        if ((instData.SRC0 == REG_SRC_LITERAL) ||
            (instData.SRC0 == REG_SRC_DPP) ||
            (instData.SRC0 == REG_SRC_SWDA)) {
            dis_stream << "0x" << std::hex << std::setfill('0') << std::setw(8)
                       << _srcLiteral;
        } else {
            dis_stream << opSelectorToRegSym(instData.SRC0);
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_VOP1::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
              return isScalarReg(instData.SRC0);
          case 1:
              // VDST is never a scalar reg
              return false;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_VOP1::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
              return isVectorReg(instData.SRC0);
          case 1:
              // VDST is always a vector reg
              return true;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    int
    Inst_VOP1::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return opSelectorToRegIdx(instData.SRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            return instData.VDST;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_VOPC base class methods ---

    Inst_VOPC::Inst_VOPC(InFmt_VOPC *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        if (hasSecondDword(iFmt)) {
            // copy second instruction DWORD into union
            extData = ((MachInst)iFmt)[1];
            _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
            varSize = 4 + 4;
            if (iFmt->SRC0 == REG_SRC_DPP) {
                setFlag(IsDPP);
            } else if (iFmt->SRC0 == REG_SRC_SWDA) {
                setFlag(IsSDWA);
            }
        } else {
            varSize = 4;
        } // if
    } // Inst_VOPC

    Inst_VOPC::~Inst_VOPC()
    {
    } // ~Inst_VOPC

    int
    Inst_VOPC::instSize() const
    {
        return varSize;
    } // instSize

    bool
    Inst_VOPC::hasSecondDword(InFmt_VOPC *iFmt)
    {
        /*
          There are several cases where VOPC instructions have a second dword:

            1.  SRC0 is a literal.
            2.  SRC0 is being used to add a data parallel primitive (DPP)
            operation to the instruction.
            3.  SRC0 is being used for sub d-word addressing (SDWA) of the
            operands in the instruction.
        */
        if ((iFmt->SRC0 == REG_SRC_LITERAL) || (iFmt->SRC0 == REG_SRC_DPP) ||
            (iFmt->SRC0 == REG_SRC_SWDA))
            return true;

        return false;
    }

    void
    Inst_VOPC::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " vcc, ";

        dis_stream << opSelectorToRegSym(instData.SRC0) << ", ";
        dis_stream << "v" << instData.VSRC1;

        disassembly = dis_stream.str();
    }

    bool
    Inst_VOPC::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return isScalarReg(instData.SRC0);
          case 1:
            // VSRC1 is never a scalar register
            return false;
          case 2:
            // VCC is always a scalar register
            return true;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_VOPC::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return isVectorReg(instData.SRC0);
          case 1:
            // VSRC1 is never a scalar register
            return true;
          case 2:
            // VCC is always a scalar register
            return false;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    int
    Inst_VOPC::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return opSelectorToRegIdx(instData.SRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            return instData.VSRC1;
          case 2:
            // VCC
            return opSelectorToRegIdx(REG_VCC_LO,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_VINTRP base class methods ---

    Inst_VINTRP::Inst_VINTRP(InFmt_VINTRP *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
    } // Inst_VINTRP

    Inst_VINTRP::~Inst_VINTRP()
    {
    } // ~Inst_VINTRP

    int
    Inst_VINTRP::instSize() const
    {
        return 4;
    } // instSize

    // --- Inst_VOP3 base class methods ---

    Inst_VOP3::Inst_VOP3(InFmt_VOP3 *iFmt, const std::string &opcode,
                         bool sgpr_dst)
        : GCN3GPUStaticInst(opcode), sgprDst(sgpr_dst)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_VOP3_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
    } // Inst_VOP3

    Inst_VOP3::~Inst_VOP3()
    {
    } // ~Inst_VOP3

    int
    Inst_VOP3::instSize() const
    {
        return 8;
    } // instSize

    void
    Inst_VOP3::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        int num_regs = 0;

        if (getOperandSize(getNumOperands() - 1) > 4) {
            num_regs = getOperandSize(getNumOperands() - 1) / 4;
            if (sgprDst)
                dis_stream << "s[";
            else
                dis_stream << "v[";
            dis_stream << instData.VDST << ":" << instData.VDST +
                          num_regs - 1 << "], ";
        } else {
            if (sgprDst)
                dis_stream << "s";
            else
                dis_stream << "v";
            dis_stream << instData.VDST << ", ";
        }

        num_regs = getOperandSize(0) / 4;

        if (extData.NEG & 0x1) {
            dis_stream << "-" << opSelectorToRegSym(extData.SRC0, num_regs);
        } else {
            dis_stream << opSelectorToRegSym(extData.SRC0, num_regs);
        }

        if (numSrcRegOperands() > 1) {
            num_regs = getOperandSize(1) / 4;

            if (extData.NEG & 0x2) {
                dis_stream << ", -"
                    << opSelectorToRegSym(extData.SRC1, num_regs);
            } else {
                dis_stream << ", "
                    << opSelectorToRegSym(extData.SRC1, num_regs);
            }
        }

        if (numSrcRegOperands() > 2) {
            num_regs = getOperandSize(2) / 4;

            if (extData.NEG & 0x4) {
                dis_stream << ", -"
                    << opSelectorToRegSym(extData.SRC2, num_regs);
            } else {
                dis_stream << ", "
                    << opSelectorToRegSym(extData.SRC2, num_regs);
            }
        }

        disassembly = dis_stream.str();
    }

    bool
    Inst_VOP3::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0 may be a scalar or vector register, an
            // inline constant, or a special HW register
            return isScalarReg(extData.SRC0);
          case 1:
            if (numSrcRegOperands() > 1) {
                // if we have more than 1 source operand then
                // op index 1 corresponds to SRC1. SRC1 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isScalarReg(extData.SRC1);
            } else {
                // if we only have 1 source operand, opIdx 1
                // will be VDST, and VDST is only a scalar
                // for v_cmp instructions
                if (sgprDst)
                    return true;
                return false;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                // if we have more than 2 source operand then
                // op index 2 corresponds to SRC2. SRC2 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isScalarReg(extData.SRC2);
            } else if (numSrcRegOperands() == 2) {
                // if we only have 2 source operands, opIdx 2
                // will be VDST, and VDST is only a scalar
                // for v_cmp instructions
                if (sgprDst)
                    return true;
                return false;
            } else {
                // if this idx doesn't correspond to SRCX or
                // VDST then it must be a VCC read or write,
                // and VCC is always stored in an SGPR pair
                assert(writesVCC() || readsVCC());
                return true;
            }
          case 3:
            if (numSrcRegOperands() == 3) {
                // if we have 3 source operands, opIdx 3
                // will be VDST, and VDST is only a scalar
                // for v_cmp instructions
                if (sgprDst)
                    return true;
                return false;
            } else {
                // if this idx doesn't correspond to VDST
                // then it must be a VCC read or write, and
                // and VCC is always stored in an SGPR pair
                assert(writesVCC() || readsVCC());
                return true;
            }
          case 4:
            // if a VOP3 instruction has more than 4 ops
            // it must read from and write to VCC, and
            // VCC is always in an SGPR
            assert(writesVCC() || readsVCC());
            return true;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_VOP3::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0 may be a scalar or vector register, an
            // inline constant, or a special HW register
            return isVectorReg(extData.SRC0);
          case 1:
            if (numSrcRegOperands() > 1) {
                // if we have more than 1 source operand then
                // op index 1 corresponds to SRC1. SRC1 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isVectorReg(extData.SRC1);
            } else {
                // if we only have 1 source operands, opIdx 1
                // will be VDST, and VDST is a scalar for v_cmp
                // instructions
                if (sgprDst)
                    return false;
                return true;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                // if we have more than 2 source operand then
                // op index 2 corresponds to SRC2. SRC2 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isVectorReg(extData.SRC2);
            } else if (numSrcRegOperands() == 2) {
                // if we only have 2 source operands, opIdx 2
                // will be VDST, and VDST is a scalar for v_cmp
                // instructions
                if (sgprDst)
                    return false;
                return true;
            } else {
                // if this idx doesn't correspond to SRCX or
                // VDST then it must be a VCC read or write,
                // and VCC is never stored in a VGPR
                assert(writesVCC() || readsVCC());
                return false;
            }
          case 3:
            if (numSrcRegOperands() == 3) {
                // if we have 3 source operands, opIdx 3
                // will be VDST, and VDST is a scalar for v_cmp
                // instructions
                if (sgprDst)
                    return false;
                return true;
            } else {
                // if this idx doesn't correspond to VDST
                // then it must be a VCC read or write, and
                // and VCC is never stored in a VGPR
                assert(writesVCC() || readsVCC());
                return false;
            }
          case 4:
            // if a VOP3 instruction has more than 4 ops
            // it must read from and write to VCC, and
            // VCC is never stored in a VGPR
            assert(writesVCC() || readsVCC());
            return false;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    int
    Inst_VOP3::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0
            return opSelectorToRegIdx(extData.SRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            if (numSrcRegOperands() > 1) {
                // if we have more than 1 source operand then
                // op index 1 corresponds to SRC1
                return opSelectorToRegIdx(extData.SRC1,
                    gpuDynInst->wavefront()->reservedScalarRegs);
            } else {
                // if we only have 1 source operand, opIdx 1
                // will be VDST
                if (sgprDst) {
                    return opSelectorToRegIdx(instData.VDST,
                            gpuDynInst->wavefront()->reservedScalarRegs);
                }
                return instData.VDST;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                // if we have more than 2 source operand then
                // op index 2 corresponds to SRC2. SRC2 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return opSelectorToRegIdx(extData.SRC2,
                    gpuDynInst->wavefront()->reservedScalarRegs);
            } else if (numSrcRegOperands() == 2) {
                // if we only have 2 source operands, opIdx 2
                // will be VDST, and VDST is always a vector
                // reg
                if (sgprDst) {
                    return opSelectorToRegIdx(instData.VDST,
                            gpuDynInst->wavefront()->reservedScalarRegs);
                }
                return instData.VDST;
            } else {
                // if this idx doesn't correspond to SRCX or
                // VDST then it must be a VCC read or write,
                // and VCC is never stored in a VGPR
                assert(writesVCC() || readsVCC());
                return opSelectorToRegIdx(REG_VCC_LO,
                        gpuDynInst->wavefront()->reservedScalarRegs);
            }
          case 3:
            if (numSrcRegOperands() == 3) {
                // if we have 3 source operands then op
                // idx 3 will correspond to VDST
                if (sgprDst) {
                    return opSelectorToRegIdx(instData.VDST,
                            gpuDynInst->wavefront()->reservedScalarRegs);
                }
                return instData.VDST;
            } else {
                // if this idx doesn't correspond to VDST
                // then it must be a VCC read or write
                assert(writesVCC() || readsVCC());
                return opSelectorToRegIdx(REG_VCC_LO,
                        gpuDynInst->wavefront()->reservedScalarRegs);
            }
          case 4:
            // if a VOP3 instruction has more than 4 ops
            // it must read from and write to VCC
            assert(writesVCC() || readsVCC());
            return opSelectorToRegIdx(REG_VCC_LO,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_VOP3_SDST_ENC base class methods ---

    Inst_VOP3_SDST_ENC::Inst_VOP3_SDST_ENC(InFmt_VOP3_SDST_ENC *iFmt,
                                           const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_VOP3_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
    } // Inst_VOP3_SDST_ENC

    Inst_VOP3_SDST_ENC::~Inst_VOP3_SDST_ENC()
    {
    } // ~Inst_VOP3_SDST_ENC

    int
    Inst_VOP3_SDST_ENC::instSize() const
    {
        return 8;
    } // instSize

    void
    Inst_VOP3_SDST_ENC::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";

        dis_stream << "v" << instData.VDST << ", ";

        if (numDstRegOperands() == 2) {
            if (getOperandSize(getNumOperands() - 1) > 4) {
                int num_regs = getOperandSize(getNumOperands() - 1) / 4;
                dis_stream << opSelectorToRegSym(instData.SDST, num_regs)
                    << ", ";
            } else {
                dis_stream << opSelectorToRegSym(instData.SDST) << ", ";
            }
        }

        if (extData.NEG & 0x1) {
            dis_stream << "-" << opSelectorToRegSym(extData.SRC0) << ", ";
        } else {
            dis_stream << opSelectorToRegSym(extData.SRC0) << ", ";
        }

        if (extData.NEG & 0x2) {
            dis_stream << "-" << opSelectorToRegSym(extData.SRC1);
        } else {
            dis_stream << opSelectorToRegSym(extData.SRC1);
        }

        if (numSrcRegOperands() == 3) {
            if (extData.NEG & 0x4) {
                dis_stream << ", -" << opSelectorToRegSym(extData.SRC2);
            } else {
                dis_stream << ", " << opSelectorToRegSym(extData.SRC2);
            }
        }

        if (readsVCC())
            dis_stream << ", vcc";

        disassembly = dis_stream.str();
    }

    bool
    Inst_VOP3_SDST_ENC::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0 may be a scalar or vector register, an
            // inline constant, or a special HW register
            return isScalarReg(extData.SRC0);
          case 1:
            if (numSrcRegOperands() > 1) {
                // if we have more than 1 source operand then
                // op index 1 corresponds to SRC1. SRC1 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isScalarReg(extData.SRC1);
            } else {
                // if we only have 1 source operand, opIdx 1
                // will be VDST, and VDST is never a scalar
                // reg
                if (instData.VDST == REG_VCC_LO)
                    return true;
                return false;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                // if we have more than 2 source operand then
                // op index 2 corresponds to SRC2. SRC2 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isScalarReg(extData.SRC2);
            } else if (numSrcRegOperands() == 2) {
                // if we only have 2 source operands, opIdx 2
                // will be VDST, and VDST is never a scalar
                // reg
                if (instData.VDST == REG_VCC_LO)
                    return true;
                return false;
            } else {
                // if this idx doesn't correspond to SRCX or
                // VDST then it must be a VCC read or write,
                // and VCC is always stored in an SGPR pair
                assert(writesVCC() || readsVCC());
                return true;
            }
          case 3:
            if (numSrcRegOperands() == 3) {
                // if we have 3 source operands then op
                // idx 3 will correspond to VDST, and VDST
                // is never a scalar reg
                if (instData.VDST == REG_VCC_LO)
                    return true;
                return false;
            } else {
                // if this idx doesn't correspond to VDST
                // then it must be a VCC read or write, and
                // and VCC is always stored in an SGPR pair
                assert(writesVCC() || readsVCC());
                return true;
            }
          case 4:
            // if a VOP3 instruction has more than 4 ops
            // it must read from and write to VCC, and
            // VCC is always in an SGPR
            assert(writesVCC() || readsVCC());
            return true;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_VOP3_SDST_ENC::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0 may be a scalar or vector register, an
            // inline constant, or a special HW register
            return isVectorReg(extData.SRC0);
          case 1:
            if (numSrcRegOperands() > 1) {
                // if we have more than 1 source operand then
                // op index 1 corresponds to SRC1. SRC1 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isVectorReg(extData.SRC1);
            } else {
                // if we only have 1 source operand, opIdx 1
                // will be VDST, and VDST is always a vector
                // reg
                if (instData.VDST == REG_VCC_LO)
                    return false;
                return true;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                // if we have more than 2 source operand then
                // op index 2 corresponds to SRC2. SRC2 may be
                // a scalar or vector register, an inline
                // constant, or a special HW register
                return isVectorReg(extData.SRC2);
            } else if (numSrcRegOperands() == 2) {
                // if we only have 2 source operands, opIdx 2
                // will be VDST, and VDST is always a vector
                // reg
                if (instData.VDST == REG_VCC_LO)
                    return false;
                return true;
            } else {
                // if this idx doesn't correspond to SRCX or
                // VDST then it must be a VCC read or write,
                // and VCC is never stored in a VGPR
                assert(writesVCC() || readsVCC());
                return false;
            }
          case 3:
            if (numSrcRegOperands() == 3) {
                // if we have 3 source operands then op
                // idx 3 will correspond to VDST, and VDST
                // is always a vector reg
                if (instData.VDST == REG_VCC_LO)
                    return false;
                return true;
            } else {
                // if this idx doesn't correspond to VDST
                // then it must be a VCC read or write, and
                // and VCC is never stored in a VGPR
                assert(writesVCC() || readsVCC());
                return false;
            }
          case 4:
            // if a VOP3 instruction has more than 4 ops
            // it must read from and write to VCC, and
            // VCC is never stored in a VGPR
            assert(writesVCC() || readsVCC());
            return false;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    int
    Inst_VOP3_SDST_ENC::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            // SRC0
            return opSelectorToRegIdx(extData.SRC0,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          case 1:
            if (numSrcRegOperands() > 1) {
                // if we have more than 1 source operand then
                // op index 1 corresponds to SRC1
                return opSelectorToRegIdx(extData.SRC1,
                    gpuDynInst->wavefront()->reservedScalarRegs);
            } else {
                // if we only have 1 source operand, opIdx 1
                // will be VDST
                return instData.VDST;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                // if we have more than 2 source operand then
                // op index 2 corresponds to SRC2
                return opSelectorToRegIdx(extData.SRC2,
                    gpuDynInst->wavefront()->reservedScalarRegs);
            } else if (numSrcRegOperands() == 2) {
                // if we only have 2 source operands, opIdx 2
                // will be VDST
                return instData.VDST;
            } else {
                // if this idx doesn't correspond to SRCX or
                // VDST then it must be a VCC read or write
                assert(writesVCC() || readsVCC());
                return opSelectorToRegIdx(instData.SDST,
                        gpuDynInst->wavefront()->reservedScalarRegs);
            }
          case 3:
            if (numSrcRegOperands() == 3) {
                // if we have 3 source operands then op
                // idx 3 will correspond to VDST
                return instData.VDST;
            } else {
                // if this idx doesn't correspond to VDST
                // then it must be a VCC read or write
                assert(writesVCC() || readsVCC());
                return opSelectorToRegIdx(instData.SDST,
                        gpuDynInst->wavefront()->reservedScalarRegs);
            }
          case 4:
            // if a VOP3 instruction has more than 4 ops
            // it must read from and write to VCC
            assert(writesVCC() || readsVCC());
            return opSelectorToRegIdx(instData.SDST,
                    gpuDynInst->wavefront()->reservedScalarRegs);
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_DS base class methods ---

    Inst_DS::Inst_DS(InFmt_DS *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(GroupSegment);

        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_DS_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
    } // Inst_DS

    Inst_DS::~Inst_DS()
    {
    } // ~Inst_DS

    int
    Inst_DS::instSize() const
    {
        return 8;
    } // instSize

    void
    Inst_DS::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";

        if (numDstRegOperands())
            dis_stream << "v" << extData.VDST << ", ";

        dis_stream << "v" << extData.ADDR;

        if (numSrcRegOperands() > 1)
            dis_stream << ", v" << extData.DATA0;

        if (numSrcRegOperands() > 2)
            dis_stream << ", v" << extData.DATA1;

        uint16_t offset = 0;

        if (instData.OFFSET1) {
            offset += instData.OFFSET1;
            offset <<= 8;
        }

        if (instData.OFFSET0)
            offset += instData.OFFSET0;

        if (offset)
            dis_stream << " offset:" << offset;

        disassembly = dis_stream.str();
    }

    bool
    Inst_DS::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // DS instructions cannot access SGPRs
        return false;
    }

    bool
    Inst_DS::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // DS instructions only access VGPRs
        return true;
    }

    int
    Inst_DS::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return extData.ADDR;
          case 1:
            if (numSrcRegOperands() > 1) {
                return extData.DATA0;
            } else if (numDstRegOperands()) {
                return extData.VDST;
            }
          case 2:
            if (numSrcRegOperands() > 2) {
                return extData.DATA1;
            } else if (numDstRegOperands()) {
                return extData.VDST;
            }
          case 3:
            assert(numDstRegOperands());
            return extData.VDST;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_MUBUF base class methods ---

    Inst_MUBUF::Inst_MUBUF(InFmt_MUBUF *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_MUBUF_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);

        if (instData.GLC)
            setFlag(GloballyCoherent);

        if (instData.SLC)
            setFlag(SystemCoherent);
    } // Inst_MUBUF

    Inst_MUBUF::~Inst_MUBUF()
    {
    } // ~Inst_MUBUF

    int
    Inst_MUBUF::instSize() const
    {
        return 8;
    } // instSize

    void
    Inst_MUBUF::generateDisassembly()
    {
        // SRSRC is always in units of 4 SGPRs
        int srsrc_val = extData.SRSRC * 4;
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";
        dis_stream << "v" << extData.VDATA << ", v" << extData.VADDR << ", ";
        dis_stream << "s[" << srsrc_val << ":"
                   << srsrc_val + 3 << "], ";
        dis_stream << "s" << extData.SOFFSET;

        if (instData.OFFSET)
            dis_stream << ", offset:" << instData.OFFSET;

        disassembly = dis_stream.str();
    }

    bool
    Inst_MUBUF::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return false;
          case 1:
            return true;
          case 2:
            return true;
          case 3:
            return false;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    bool
    Inst_MUBUF::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return true;
          case 1:
            return false;
          case 2:
            return false;
          case 3:
            return true;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return false;
        }
    }

    int
    Inst_MUBUF::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
              return extData.VADDR;
          case 1:
              // SRSRC is always in units of 4 SGPRs
              return extData.SRSRC * 4;
          case 2:
              return extData.SOFFSET;
          case 3:
            return extData.VDATA;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }

    // --- Inst_MTBUF base class methods ---

    Inst_MTBUF::Inst_MTBUF(InFmt_MTBUF *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_MTBUF_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);

        if (instData.GLC)
            setFlag(GloballyCoherent);

        if (extData.SLC)
            setFlag(SystemCoherent);

    } // Inst_MTBUF

    Inst_MTBUF::~Inst_MTBUF()
    {
    } // ~Inst_MTBUF

    int
    Inst_MTBUF::instSize() const
    {
        return 8;
    } // instSize

    // --- Inst_MIMG base class methods ---

    Inst_MIMG::Inst_MIMG(InFmt_MIMG *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_MIMG_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);

        if (instData.GLC)
            setFlag(GloballyCoherent);

        if (instData.SLC)
            setFlag(SystemCoherent);
    } // Inst_MIMG

    Inst_MIMG::~Inst_MIMG()
    {
    } // ~Inst_MIMG

    int
    Inst_MIMG::instSize() const
    {
        return 8;
    } // instSize

    // --- Inst_EXP base class methods ---

    Inst_EXP::Inst_EXP(InFmt_EXP *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_EXP_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);
    } // Inst_EXP

    Inst_EXP::~Inst_EXP()
    {
    } // ~Inst_EXP

    int
    Inst_EXP::instSize() const
    {
        return 8;
    } // instSize

    // --- Inst_FLAT base class methods ---

    Inst_FLAT::Inst_FLAT(InFmt_FLAT *iFmt, const std::string &opcode)
        : GCN3GPUStaticInst(opcode)
    {
        setFlag(Flat);
        // copy first instruction DWORD
        instData = iFmt[0];
        // copy second instruction DWORD
        extData = ((InFmt_FLAT_1 *)iFmt)[1];
        _srcLiteral = *reinterpret_cast<uint32_t*>(&iFmt[1]);

        if (instData.GLC)
            setFlag(GloballyCoherent);

        if (instData.SLC)
            setFlag(SystemCoherent);
    } // Inst_FLAT

    Inst_FLAT::~Inst_FLAT()
    {
    } // ~Inst_FLAT

    int
    Inst_FLAT::instSize() const
    {
        return 8;
    } // instSize

    void
    Inst_FLAT::generateDisassembly()
    {
        std::stringstream dis_stream;
        dis_stream << _opcode << " ";

        if (isLoad())
            dis_stream << "v" << extData.VDST << ", ";

        dis_stream << "v[" << extData.ADDR << ":" << extData.ADDR + 1 << "]";

        if (isStore())
            dis_stream << ", v" << extData.DATA;

        disassembly = dis_stream.str();
    }

    bool
    Inst_FLAT::isScalarRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // if a FLAT instruction has more than two
        // operands it must be an atomic
        if (opIdx == 2)
            assert(isAtomic());

        // FLAT instructions cannot access SGPRs
        return false;
    }

    bool
    Inst_FLAT::isVectorRegister(int opIdx)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        // if a FLAT instruction has more than two
        // operands it must be an atomic
        if (opIdx == 2)
            assert(isAtomic());

        // FLAT instructions only access VGPRs
        return true;
    }

    int
    Inst_FLAT::getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst)
    {
        assert(opIdx >= 0);
        assert(opIdx < getNumOperands());

        switch (opIdx) {
          case 0:
            return extData.ADDR;
          case 1:
            if (isStore()) {
                return extData.DATA;
            } else if (isLoad()) {
                return extData.VDST;
            } else if (isAtomic()) {
                // For flat_atomic instructions,
                // the DATA VGPR gives the source
                return extData.DATA;
            } else {
                fatal("Unsupported flat instr type\n");
            }
          case 2:
             // if a FLAT instruction has more than two
             // operands it must be an atomic
            assert(isAtomic());
            return extData.VDST;
          default:
            fatal("Operand at idx %i does not exist\n", opIdx);
            return -1;
        }
    }
} // namespace Gcn3ISA
