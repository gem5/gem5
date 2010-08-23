/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2007-2008 The Florida State University
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
 *
 * Authors: Stephen Hines
 */

#include "arch/arm/insts/macromem.hh"
#include "arch/arm/decoder.hh"

using namespace ArmISAInst;

namespace ArmISA
{

MacroMemOp::MacroMemOp(const char *mnem, ExtMachInst machInst,
                       OpClass __opClass, IntRegIndex rn,
                       bool index, bool up, bool user, bool writeback,
                       bool load, uint32_t reglist) :
    PredMacroOp(mnem, machInst, __opClass)
{
    uint32_t regs = reglist;
    uint32_t ones = number_of_ones(reglist);
    // Remember that writeback adds a uop
    numMicroops = ones + (writeback ? 1 : 0) + 1;
    microOps = new StaticInstPtr[numMicroops];
    uint32_t addr = 0;

    if (!up)
        addr = (ones << 2) - 4;

    if (!index)
        addr += 4;

    StaticInstPtr *uop = microOps;
    StaticInstPtr wbUop;
    if (writeback) {
        if (up) {
            wbUop = new MicroAddiUop(machInst, rn, rn, ones * 4);
        } else {
            wbUop = new MicroSubiUop(machInst, rn, rn, ones * 4);
        }
    }

    // Add 0 to Rn and stick it in ureg0.
    // This is equivalent to a move.
    *uop = new MicroAddiUop(machInst, INTREG_UREG0, rn, 0);

    // Write back at the start for loads. This covers the ldm exception return
    // case where the base needs to be written in the old mode. Stores may need
    // the original value of the base, but they don't change mode and can
    // write back at the end like before.
    if (load && writeback) {
        *++uop = wbUop;
    }

    unsigned reg = 0;
    bool force_user = user & !bits(reglist, 15);
    bool exception_ret = user & bits(reglist, 15);

    for (int i = 0; i < ones; i++) {
        // Find the next register.
        while (!bits(regs, reg))
            reg++;
        replaceBits(regs, reg, 0);

        unsigned regIdx = reg;
        if (force_user) {
            regIdx = intRegInMode(MODE_USER, regIdx);
        }

        if (load) {
            if (reg == INTREG_PC && exception_ret) {
                // This must be the exception return form of ldm.
                *++uop = new MicroLdrRetUop(machInst, regIdx,
                                           INTREG_UREG0, up, addr);
            } else {
                *++uop = new MicroLdrUop(machInst, regIdx,
                                        INTREG_UREG0, up, addr);
            }
        } else {
            *++uop = new MicroStrUop(machInst, regIdx, INTREG_UREG0, up, addr);
        }

        if (up)
            addr += 4;
        else
            addr -= 4;
    }

    if (!load && writeback) {
        *++uop = wbUop;
    }

    (*uop)->setLastMicroop();

    for (StaticInstPtr *curUop = microOps;
            !(*curUop)->isLastMicroop(); curUop++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(curUop->get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
}

MacroVFPMemOp::MacroVFPMemOp(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, IntRegIndex rn,
                             RegIndex vd, bool single, bool up,
                             bool writeback, bool load, uint32_t offset) :
    PredMacroOp(mnem, machInst, __opClass)
{
    int i = 0;

    // The lowest order bit selects fldmx (set) or fldmd (clear). These seem
    // to be functionally identical except that fldmx is deprecated. For now
    // we'll assume they're otherwise interchangable.
    int count = (single ? offset : (offset / 2));
    if (count == 0 || count > NumFloatArchRegs)
        warn_once("Bad offset field for VFP load/store multiple.\n");
    if (count == 0) {
        // Force there to be at least one microop so the macroop makes sense.
        writeback = true;
    }
    if (count > NumFloatArchRegs)
        count = NumFloatArchRegs;

    numMicroops = count * (single ? 1 : 2) + (writeback ? 1 : 0);
    microOps = new StaticInstPtr[numMicroops];

    int64_t addr = 0;

    if (!up)
        addr = 4 * offset;

    bool tempUp = up;
    for (int j = 0; j < count; j++) {
        if (load) {
            microOps[i++] = new MicroLdrFpUop(machInst, vd++, rn,
                                              tempUp, addr);
            if (!single)
                microOps[i++] = new MicroLdrFpUop(machInst, vd++, rn, tempUp,
                                                  addr + (up ? 4 : -4));
        } else {
            microOps[i++] = new MicroStrFpUop(machInst, vd++, rn,
                                              tempUp, addr);
            if (!single)
                microOps[i++] = new MicroStrFpUop(machInst, vd++, rn, tempUp,
                                                  addr + (up ? 4 : -4));
        }
        if (!tempUp) {
            addr -= (single ? 4 : 8);
            // The microops don't handle negative displacement, so turn if we
            // hit zero, flip polarity and start adding.
            if (addr <= 0) {
                tempUp = true;
                addr = -addr;
            }
        } else {
            addr += (single ? 4 : 8);
        }
    }

    if (writeback) {
        if (up) {
            microOps[i++] =
                new MicroAddiUop(machInst, rn, rn, 4 * offset);
        } else {
            microOps[i++] =
                new MicroSubiUop(machInst, rn, rn, 4 * offset);
        }
    }

    assert(numMicroops == i);
    microOps[numMicroops - 1]->setLastMicroop();

    for (StaticInstPtr *curUop = microOps;
            !(*curUop)->isLastMicroop(); curUop++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(curUop->get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
}

std::string
MicroIntOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, ura);
    ss << ", ";
    printReg(ss, urb);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    return ss.str();
}

std::string
MicroMemOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, ura);
    ss << ", [";
    printReg(ss, urb);
    ss << ", ";
    ccprintf(ss, "#%d", imm);
    ss << "]";
    return ss.str();
}

}
