/*
 * Copyright (c) 2022 PLCT Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch/riscv/insts/vector.hh"

#include <sstream>
#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/utility.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * This function translates the 3-bit value of vlmul bits to the corresponding
 * lmul value as specified in RVV 1.0 spec p11-12 chapter 3.4.2.
 *
 * I.e.,
 * vlmul = -3 -> LMUL = 1/8
 * vlmul = -2 -> LMUL = 1/4
 * vlmul = -1 -> LMUL = 1/2
 * vlmul = 0 -> LMUL = 1
 * vlmul = 1 -> LMUL = 2
 * vlmul = 2 -> LMUL = 4
 * vlmul = 3 -> LMUL = 8
 *
**/
float
getVflmul(uint32_t vlmul_encoding) {
  int vlmul = sext<3>(vlmul_encoding & 7);
  float vflmul = vlmul >= 0 ? 1 << vlmul : 1.0 / (1 << -vlmul);
  return vflmul;
}

uint32_t
getVlmax(VTYPE vtype, uint32_t vlen) {
  uint32_t sew = getSew(vtype.vsew);
  // vlmax is defined in RVV 1.0 spec p12 chapter 3.4.2.
  uint32_t vlmax = (vlen/sew) * getVflmul(vtype.vlmul);
  return vlmax;
}

std::string
VConfOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (bit31 && bit30 == 0) {
        ss << registerName(srcRegIdx(0)) << ", " << registerName(srcRegIdx(1));
    } else if (bit31 && bit30) {
        ss << uimm << ", " << generateZimmDisassembly();
    } else {
        ss << registerName(srcRegIdx(0)) << ", " << generateZimmDisassembly();
    }
    return ss.str();
}

std::string
VConfOp::generateZimmDisassembly() const
{
    std::stringstream s;

    // VSETIVLI uses ZIMM10 and VSETVLI uses ZIMM11
    uint64_t zimm = (bit31 && bit30) ? zimm10 : zimm11;

    bool frac_lmul = bits(zimm, 2);
    int sew = 1 << (bits(zimm, 5, 3) + 3);
    int lmul = bits(zimm, 1, 0);
    auto vta = bits(zimm, 6) == 1 ? "ta" : "tu";
    auto vma = bits(zimm, 7) == 1 ? "ma" : "mu";
    s << "e" << sew;
    if (frac_lmul) {
        std::string lmul_str = "";
        switch(lmul){
        case 3:
            lmul_str = "f2";
            break;
        case 2:
            lmul_str = "f4";
            break;
        case 1:
            lmul_str = "f8";
            break;
        default:
            panic("Unsupport fractional LMUL");
        }
        s << ", m" << lmul_str;
    } else {
        s << ", m" << (1 << lmul);
    }
    s << ", " << vta << ", " << vma;
    return s.str();
}

} // namespace RiscvISA
} // namespace gem5
