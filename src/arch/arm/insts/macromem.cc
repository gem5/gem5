/*
 * Copyright (c) 2010-2013 ARM Limited
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

#include <sstream>

#include "arch/arm/insts/macromem.hh"

#include "arch/arm/generated/decoder.hh"
#include "arch/arm/insts/neon64_mem.hh"

using namespace std;
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
    // Remember that writeback adds a uop or two and the temp register adds one
    numMicroops = ones + (writeback ? (load ? 2 : 1) : 0) + 1;

    // It's technically legal to do a lot of nothing
    if (!ones)
        numMicroops = 1;

    microOps = new StaticInstPtr[numMicroops];
    uint32_t addr = 0;

    if (!up)
        addr = (ones << 2) - 4;

    if (!index)
        addr += 4;

    StaticInstPtr *uop = microOps;

    // Add 0 to Rn and stick it in ureg0.
    // This is equivalent to a move.
    *uop = new MicroAddiUop(machInst, INTREG_UREG0, rn, 0);

    unsigned reg = 0;
    unsigned regIdx = 0;
    bool force_user = user & !bits(reglist, 15);
    bool exception_ret = user & bits(reglist, 15);

    for (int i = 0; i < ones; i++) {
        // Find the next register.
        while (!bits(regs, reg))
            reg++;
        replaceBits(regs, reg, 0);

        regIdx = reg;
        if (force_user) {
            regIdx = intRegInMode(MODE_USER, regIdx);
        }

        if (load) {
            if (writeback && i == ones - 1) {
                // If it's a writeback and this is the last register
                // do the load into a temporary register which we'll move
                // into the final one later
                *++uop = new MicroLdrUop(machInst, INTREG_UREG1, INTREG_UREG0,
                        up, addr);
            } else {
                // Otherwise just do it normally
                if (reg == INTREG_PC && exception_ret) {
                    // This must be the exception return form of ldm.
                    *++uop = new MicroLdrRetUop(machInst, regIdx,
                                               INTREG_UREG0, up, addr);
                    if (!(condCode == COND_AL || condCode == COND_UC))
                        (*uop)->setFlag(StaticInst::IsCondControl);
                    else
                        (*uop)->setFlag(StaticInst::IsUncondControl);
                } else {
                    *++uop = new MicroLdrUop(machInst, regIdx,
                                            INTREG_UREG0, up, addr);
                    if (reg == INTREG_PC) {
                        (*uop)->setFlag(StaticInst::IsControl);
                        if (!(condCode == COND_AL || condCode == COND_UC))
                            (*uop)->setFlag(StaticInst::IsCondControl);
                        else
                            (*uop)->setFlag(StaticInst::IsUncondControl);
                        (*uop)->setFlag(StaticInst::IsIndirectControl);
                    }
                }
            }
        } else {
            *++uop = new MicroStrUop(machInst, regIdx, INTREG_UREG0, up, addr);
        }

        if (up)
            addr += 4;
        else
            addr -= 4;
    }

    if (writeback && ones) {
        // put the register update after we're done all loading
        if (up)
            *++uop = new MicroAddiUop(machInst, rn, rn, ones * 4);
        else
            *++uop = new MicroSubiUop(machInst, rn, rn, ones * 4);

        // If this was a load move the last temporary value into place
        // this way we can't take an exception after we update the base
        // register.
        if (load && reg == INTREG_PC && exception_ret) {
            *++uop = new MicroUopRegMovRet(machInst, 0, INTREG_UREG1);
            if (!(condCode == COND_AL || condCode == COND_UC))
                (*uop)->setFlag(StaticInst::IsCondControl);
            else
                (*uop)->setFlag(StaticInst::IsUncondControl);
        } else if (load) {
            *++uop = new MicroUopRegMov(machInst, regIdx, INTREG_UREG1);
            if (reg == INTREG_PC) {
                (*uop)->setFlag(StaticInst::IsControl);
                (*uop)->setFlag(StaticInst::IsCondControl);
                (*uop)->setFlag(StaticInst::IsIndirectControl);
                // This is created as a RAS POP
                if (rn == INTREG_SP)
                    (*uop)->setFlag(StaticInst::IsReturn);

            }
        }
    }

    (*uop)->setLastMicroop();

    for (StaticInstPtr *curUop = microOps;
            !(*curUop)->isLastMicroop(); curUop++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(curUop->get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
}

PairMemOp::PairMemOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                     uint32_t size, bool fp, bool load, bool noAlloc,
                     bool signExt, bool exclusive, bool acrel,
                     int64_t imm, AddrMode mode,
                     IntRegIndex rn, IntRegIndex rt, IntRegIndex rt2) :
    PredMacroOp(mnem, machInst, __opClass)
{
    bool writeback = (mode != AddrMd_Offset);
    numMicroops = 1 + (size / 4) + (writeback ? 1 : 0);
    microOps = new StaticInstPtr[numMicroops];

    StaticInstPtr *uop = microOps;

    bool post = (mode == AddrMd_PostIndex);

    rn = makeSP(rn);

    *uop = new MicroAddXiSpAlignUop(machInst, INTREG_UREG0, rn, post ? 0 : imm);

    if (fp) {
        if (size == 16) {
            if (load) {
                *++uop = new MicroLdrQBFpXImmUop(machInst, rt,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                *++uop = new MicroLdrQTFpXImmUop(machInst, rt,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                *++uop = new MicroLdrQBFpXImmUop(machInst, rt2,
                        INTREG_UREG0, 16, noAlloc, exclusive, acrel);
                *++uop = new MicroLdrQTFpXImmUop(machInst, rt2,
                        INTREG_UREG0, 16, noAlloc, exclusive, acrel);
            } else {
                *++uop = new MicroStrQBFpXImmUop(machInst, rt,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                *++uop = new MicroStrQTFpXImmUop(machInst, rt,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                *++uop = new MicroStrQBFpXImmUop(machInst, rt2,
                        INTREG_UREG0, 16, noAlloc, exclusive, acrel);
                *++uop = new MicroStrQTFpXImmUop(machInst, rt2,
                        INTREG_UREG0, 16, noAlloc, exclusive, acrel);
            }
        } else if (size == 8) {
            if (load) {
                *++uop = new MicroLdrFpXImmUop(machInst, rt,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                *++uop = new MicroLdrFpXImmUop(machInst, rt2,
                        INTREG_UREG0, 8, noAlloc, exclusive, acrel);
            } else {
                *++uop = new MicroStrFpXImmUop(machInst, rt,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                *++uop = new MicroStrFpXImmUop(machInst, rt2,
                        INTREG_UREG0, 8, noAlloc, exclusive, acrel);
            }
        } else if (size == 4) {
            if (load) {
                *++uop = new MicroLdrDFpXImmUop(machInst, rt, rt2,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
            } else {
                *++uop = new MicroStrDFpXImmUop(machInst, rt, rt2,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
            }
        }
    } else {
        if (size == 8) {
            if (load) {
                *++uop = new MicroLdrXImmUop(machInst, rt, INTREG_UREG0,
                        0, noAlloc, exclusive, acrel);
                *++uop = new MicroLdrXImmUop(machInst, rt2, INTREG_UREG0,
                        size, noAlloc, exclusive, acrel);
            } else {
                *++uop = new MicroStrXImmUop(machInst, rt, INTREG_UREG0,
                        0, noAlloc, exclusive, acrel);
                *++uop = new MicroStrXImmUop(machInst, rt2, INTREG_UREG0,
                        size, noAlloc, exclusive, acrel);
            }
        } else if (size == 4) {
            if (load) {
                if (signExt) {
                    *++uop = new MicroLdrDSXImmUop(machInst, rt, rt2,
                            INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                } else {
                    *++uop = new MicroLdrDUXImmUop(machInst, rt, rt2,
                            INTREG_UREG0, 0, noAlloc, exclusive, acrel);
                }
            } else {
                *++uop = new MicroStrDXImmUop(machInst, rt, rt2,
                        INTREG_UREG0, 0, noAlloc, exclusive, acrel);
            }
        }
    }

    if (writeback) {
        *++uop = new MicroAddXiUop(machInst, rn, INTREG_UREG0,
                                   post ? imm : 0);
    }

    (*uop)->setLastMicroop();

    for (StaticInstPtr *curUop = microOps;
            !(*curUop)->isLastMicroop(); curUop++) {
        (*curUop)->setDelayedCommit();
    }
}

BigFpMemImmOp::BigFpMemImmOp(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, bool load, IntRegIndex dest,
                             IntRegIndex base, int64_t imm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    numMicroops = 2;
    microOps = new StaticInstPtr[numMicroops];

    if (load) {
        microOps[0] = new MicroLdrQBFpXImmUop(machInst, dest, base, imm);
        microOps[1] = new MicroLdrQTFpXImmUop(machInst, dest, base, imm);
    } else {
        microOps[0] = new MicroStrQBFpXImmUop(machInst, dest, base, imm);
        microOps[1] = new MicroStrQTFpXImmUop(machInst, dest, base, imm);
    }
    microOps[0]->setDelayedCommit();
    microOps[1]->setLastMicroop();
}

BigFpMemPostOp::BigFpMemPostOp(const char *mnem, ExtMachInst machInst,
                               OpClass __opClass, bool load, IntRegIndex dest,
                               IntRegIndex base, int64_t imm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    numMicroops = 3;
    microOps = new StaticInstPtr[numMicroops];

    if (load) {
        microOps[0] = new MicroLdrQBFpXImmUop(machInst, dest, base, 0);
        microOps[1] = new MicroLdrQTFpXImmUop(machInst, dest, base, 0);
    } else {
        microOps[0] = new MicroStrQBFpXImmUop(machInst, dest, base, 0);
        microOps[1] = new MicroStrQTFpXImmUop(machInst, dest, base, 0);
    }
    microOps[2] = new MicroAddXiUop(machInst, base, base, imm);

    microOps[0]->setDelayedCommit();
    microOps[1]->setDelayedCommit();
    microOps[2]->setLastMicroop();
}

BigFpMemPreOp::BigFpMemPreOp(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, bool load, IntRegIndex dest,
                             IntRegIndex base, int64_t imm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    numMicroops = 3;
    microOps = new StaticInstPtr[numMicroops];

    if (load) {
        microOps[0] = new MicroLdrQBFpXImmUop(machInst, dest, base, imm);
        microOps[1] = new MicroLdrQTFpXImmUop(machInst, dest, base, imm);
    } else {
        microOps[0] = new MicroStrQBFpXImmUop(machInst, dest, base, imm);
        microOps[1] = new MicroStrQTFpXImmUop(machInst, dest, base, imm);
    }
    microOps[2] = new MicroAddXiUop(machInst, base, base, imm);

    microOps[0]->setDelayedCommit();
    microOps[1]->setDelayedCommit();
    microOps[2]->setLastMicroop();
}

BigFpMemRegOp::BigFpMemRegOp(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, bool load, IntRegIndex dest,
                             IntRegIndex base, IntRegIndex offset,
                             ArmExtendType type, int64_t imm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    numMicroops = 2;
    microOps = new StaticInstPtr[numMicroops];

    if (load) {
        microOps[0] = new MicroLdrQBFpXRegUop(machInst, dest, base,
                                              offset, type, imm);
        microOps[1] = new MicroLdrQTFpXRegUop(machInst, dest, base,
                                              offset, type, imm);
    } else {
        microOps[0] = new MicroStrQBFpXRegUop(machInst, dest, base,
                                              offset, type, imm);
        microOps[1] = new MicroStrQTFpXRegUop(machInst, dest, base,
                                              offset, type, imm);
    }

    microOps[0]->setDelayedCommit();
    microOps[1]->setLastMicroop();
}

BigFpMemLitOp::BigFpMemLitOp(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, IntRegIndex dest,
                             int64_t imm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    numMicroops = 2;
    microOps = new StaticInstPtr[numMicroops];

    microOps[0] = new MicroLdrQBFpXLitUop(machInst, dest, imm);
    microOps[1] = new MicroLdrQTFpXLitUop(machInst, dest, imm);

    microOps[0]->setDelayedCommit();
    microOps[1]->setLastMicroop();
}

VldMultOp::VldMultOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                     unsigned elems, RegIndex rn, RegIndex vd, unsigned regs,
                     unsigned inc, uint32_t size, uint32_t align, RegIndex rm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    assert(regs > 0 && regs <= 4);
    assert(regs % elems == 0);

    numMicroops = (regs > 2) ? 2 : 1;
    bool wb = (rm != 15);
    bool deinterleave = (elems > 1);

    if (wb) numMicroops++;
    if (deinterleave) numMicroops += (regs / elems);
    microOps = new StaticInstPtr[numMicroops];

    RegIndex rMid = deinterleave ? NumFloatV7ArchRegs : vd * 2;

    uint32_t noAlign = TLB::MustBeOne;

    unsigned uopIdx = 0;
    switch (regs) {
      case 4:
        microOps[uopIdx++] = newNeonMemInst<MicroLdrNeon16Uop>(
                size, machInst, rMid, rn, 0, align);
        microOps[uopIdx++] = newNeonMemInst<MicroLdrNeon16Uop>(
                size, machInst, rMid + 4, rn, 16, noAlign);
        break;
      case 3:
        microOps[uopIdx++] = newNeonMemInst<MicroLdrNeon16Uop>(
                size, machInst, rMid, rn, 0, align);
        microOps[uopIdx++] = newNeonMemInst<MicroLdrNeon8Uop>(
                size, machInst, rMid + 4, rn, 16, noAlign);
        break;
      case 2:
        microOps[uopIdx++] = newNeonMemInst<MicroLdrNeon16Uop>(
                size, machInst, rMid, rn, 0, align);
        break;
      case 1:
        microOps[uopIdx++] = newNeonMemInst<MicroLdrNeon8Uop>(
                size, machInst, rMid, rn, 0, align);
        break;
      default:
        // Unknown number of registers
        microOps[uopIdx++] = new Unknown(machInst);
    }
    if (wb) {
        if (rm != 15 && rm != 13) {
            microOps[uopIdx++] =
                new MicroAddUop(machInst, rn, rn, rm, 0, ArmISA::LSL);
        } else {
            microOps[uopIdx++] =
                new MicroAddiUop(machInst, rn, rn, regs * 8);
        }
    }
    if (deinterleave) {
        switch (elems) {
          case 4:
            assert(regs == 4);
            microOps[uopIdx++] = newNeonMixInst<MicroDeintNeon8Uop>(
                    size, machInst, vd * 2, rMid, inc * 2);
            break;
          case 3:
            assert(regs == 3);
            microOps[uopIdx++] = newNeonMixInst<MicroDeintNeon6Uop>(
                    size, machInst, vd * 2, rMid, inc * 2);
            break;
          case 2:
            assert(regs == 4 || regs == 2);
            if (regs == 4) {
                microOps[uopIdx++] = newNeonMixInst<MicroDeintNeon4Uop>(
                        size, machInst, vd * 2, rMid, inc * 2);
                microOps[uopIdx++] = newNeonMixInst<MicroDeintNeon4Uop>(
                        size, machInst, vd * 2 + 2, rMid + 4, inc * 2);
            } else {
                microOps[uopIdx++] = newNeonMixInst<MicroDeintNeon4Uop>(
                        size, machInst, vd * 2, rMid, inc * 2);
            }
            break;
          default:
            // Bad number of elements to deinterleave
            microOps[uopIdx++] = new Unknown(machInst);
        }
    }
    assert(uopIdx == numMicroops);

    for (unsigned i = 0; i < numMicroops - 1; i++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(microOps[i].get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VldSingleOp::VldSingleOp(const char *mnem, ExtMachInst machInst,
                         OpClass __opClass, bool all, unsigned elems,
                         RegIndex rn, RegIndex vd, unsigned regs,
                         unsigned inc, uint32_t size, uint32_t align,
                         RegIndex rm, unsigned lane) :
    PredMacroOp(mnem, machInst, __opClass)
{
    assert(regs > 0 && regs <= 4);
    assert(regs % elems == 0);

    unsigned eBytes = (1 << size);
    unsigned loadSize = eBytes * elems;
    unsigned loadRegs M5_VAR_USED = (loadSize + sizeof(FloatRegBits) - 1) /
                        sizeof(FloatRegBits);

    assert(loadRegs > 0 && loadRegs <= 4);

    numMicroops = 1;
    bool wb = (rm != 15);

    if (wb) numMicroops++;
    numMicroops += (regs / elems);
    microOps = new StaticInstPtr[numMicroops];

    RegIndex ufp0 = NumFloatV7ArchRegs;

    unsigned uopIdx = 0;
    switch (loadSize) {
      case 1:
        microOps[uopIdx++] = new MicroLdrNeon1Uop<uint8_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 2:
        if (eBytes == 2) {
            microOps[uopIdx++] = new MicroLdrNeon2Uop<uint16_t>(
                    machInst, ufp0, rn, 0, align);
        } else {
            microOps[uopIdx++] = new MicroLdrNeon2Uop<uint8_t>(
                    machInst, ufp0, rn, 0, align);
        }
        break;
      case 3:
        microOps[uopIdx++] = new MicroLdrNeon3Uop<uint8_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 4:
        switch (eBytes) {
          case 1:
            microOps[uopIdx++] = new MicroLdrNeon4Uop<uint8_t>(
                    machInst, ufp0, rn, 0, align);
            break;
          case 2:
            microOps[uopIdx++] = new MicroLdrNeon4Uop<uint16_t>(
                    machInst, ufp0, rn, 0, align);
            break;
          case 4:
            microOps[uopIdx++] = new MicroLdrNeon4Uop<uint32_t>(
                    machInst, ufp0, rn, 0, align);
            break;
        }
        break;
      case 6:
        microOps[uopIdx++] = new MicroLdrNeon6Uop<uint16_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 8:
        switch (eBytes) {
          case 2:
            microOps[uopIdx++] = new MicroLdrNeon8Uop<uint16_t>(
                    machInst, ufp0, rn, 0, align);
            break;
          case 4:
            microOps[uopIdx++] = new MicroLdrNeon8Uop<uint32_t>(
                    machInst, ufp0, rn, 0, align);
            break;
        }
        break;
      case 12:
        microOps[uopIdx++] = new MicroLdrNeon12Uop<uint32_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 16:
        microOps[uopIdx++] = new MicroLdrNeon16Uop<uint32_t>(
                machInst, ufp0, rn, 0, align);
        break;
      default:
        // Unrecognized load size
        microOps[uopIdx++] = new Unknown(machInst);
    }
    if (wb) {
        if (rm != 15 && rm != 13) {
            microOps[uopIdx++] =
                new MicroAddUop(machInst, rn, rn, rm, 0, ArmISA::LSL);
        } else {
            microOps[uopIdx++] =
                new MicroAddiUop(machInst, rn, rn, loadSize);
        }
    }
    switch (elems) {
      case 4:
        assert(regs == 4);
        switch (size) {
          case 0:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to8Uop<uint8_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to8Uop<uint8_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          case 1:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to8Uop<uint16_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to8Uop<uint16_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          case 2:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon4to8Uop<uint32_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon4to8Uop<uint32_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          default:
            // Bad size
            microOps[uopIdx++] = new Unknown(machInst);
            break;
        }
        break;
      case 3:
        assert(regs == 3);
        switch (size) {
          case 0:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to6Uop<uint8_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to6Uop<uint8_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          case 1:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to6Uop<uint16_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to6Uop<uint16_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          case 2:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon4to6Uop<uint32_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon4to6Uop<uint32_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          default:
            // Bad size
            microOps[uopIdx++] = new Unknown(machInst);
            break;
        }
        break;
      case 2:
        assert(regs == 2);
        assert(loadRegs <= 2);
        switch (size) {
          case 0:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to4Uop<uint8_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to4Uop<uint8_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          case 1:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to4Uop<uint16_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to4Uop<uint16_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          case 2:
            if (all) {
                microOps[uopIdx++] = new MicroUnpackAllNeon2to4Uop<uint32_t>(
                        machInst, vd * 2, ufp0, inc * 2);
            } else {
                microOps[uopIdx++] = new MicroUnpackNeon2to4Uop<uint32_t>(
                        machInst, vd * 2, ufp0, inc * 2, lane);
            }
            break;
          default:
            // Bad size
            microOps[uopIdx++] = new Unknown(machInst);
            break;
        }
        break;
      case 1:
        assert(regs == 1 || (all && regs == 2));
        assert(loadRegs <= 2);
        for (unsigned offset = 0; offset < regs; offset++) {
            switch (size) {
              case 0:
                if (all) {
                    microOps[uopIdx++] =
                        new MicroUnpackAllNeon2to2Uop<uint8_t>(
                            machInst, (vd + offset) * 2, ufp0, inc * 2);
                } else {
                    microOps[uopIdx++] =
                        new MicroUnpackNeon2to2Uop<uint8_t>(
                            machInst, (vd + offset) * 2, ufp0, inc * 2, lane);
                }
                break;
              case 1:
                if (all) {
                    microOps[uopIdx++] =
                        new MicroUnpackAllNeon2to2Uop<uint16_t>(
                            machInst, (vd + offset) * 2, ufp0, inc * 2);
                } else {
                    microOps[uopIdx++] =
                        new MicroUnpackNeon2to2Uop<uint16_t>(
                            machInst, (vd + offset) * 2, ufp0, inc * 2, lane);
                }
                break;
              case 2:
                if (all) {
                    microOps[uopIdx++] =
                        new MicroUnpackAllNeon2to2Uop<uint32_t>(
                            machInst, (vd + offset) * 2, ufp0, inc * 2);
                } else {
                    microOps[uopIdx++] =
                        new MicroUnpackNeon2to2Uop<uint32_t>(
                            machInst, (vd + offset) * 2, ufp0, inc * 2, lane);
                }
                break;
              default:
                // Bad size
                microOps[uopIdx++] = new Unknown(machInst);
                break;
            }
        }
        break;
      default:
        // Bad number of elements to unpack
        microOps[uopIdx++] = new Unknown(machInst);
    }
    assert(uopIdx == numMicroops);

    for (unsigned i = 0; i < numMicroops - 1; i++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(microOps[i].get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VstMultOp::VstMultOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                     unsigned elems, RegIndex rn, RegIndex vd, unsigned regs,
                     unsigned inc, uint32_t size, uint32_t align, RegIndex rm) :
    PredMacroOp(mnem, machInst, __opClass)
{
    assert(regs > 0 && regs <= 4);
    assert(regs % elems == 0);

    numMicroops = (regs > 2) ? 2 : 1;
    bool wb = (rm != 15);
    bool interleave = (elems > 1);

    if (wb) numMicroops++;
    if (interleave) numMicroops += (regs / elems);
    microOps = new StaticInstPtr[numMicroops];

    uint32_t noAlign = TLB::MustBeOne;

    RegIndex rMid = interleave ? NumFloatV7ArchRegs : vd * 2;

    unsigned uopIdx = 0;
    if (interleave) {
        switch (elems) {
          case 4:
            assert(regs == 4);
            microOps[uopIdx++] = newNeonMixInst<MicroInterNeon8Uop>(
                    size, machInst, rMid, vd * 2, inc * 2);
            break;
          case 3:
            assert(regs == 3);
            microOps[uopIdx++] = newNeonMixInst<MicroInterNeon6Uop>(
                    size, machInst, rMid, vd * 2, inc * 2);
            break;
          case 2:
            assert(regs == 4 || regs == 2);
            if (regs == 4) {
                microOps[uopIdx++] = newNeonMixInst<MicroInterNeon4Uop>(
                        size, machInst, rMid, vd * 2, inc * 2);
                microOps[uopIdx++] = newNeonMixInst<MicroInterNeon4Uop>(
                        size, machInst, rMid + 4, vd * 2 + 2, inc * 2);
            } else {
                microOps[uopIdx++] = newNeonMixInst<MicroInterNeon4Uop>(
                        size, machInst, rMid, vd * 2, inc * 2);
            }
            break;
          default:
            // Bad number of elements to interleave
            microOps[uopIdx++] = new Unknown(machInst);
        }
    }
    switch (regs) {
      case 4:
        microOps[uopIdx++] = newNeonMemInst<MicroStrNeon16Uop>(
                size, machInst, rMid, rn, 0, align);
        microOps[uopIdx++] = newNeonMemInst<MicroStrNeon16Uop>(
                size, machInst, rMid + 4, rn, 16, noAlign);
        break;
      case 3:
        microOps[uopIdx++] = newNeonMemInst<MicroStrNeon16Uop>(
                size, machInst, rMid, rn, 0, align);
        microOps[uopIdx++] = newNeonMemInst<MicroStrNeon8Uop>(
                size, machInst, rMid + 4, rn, 16, noAlign);
        break;
      case 2:
        microOps[uopIdx++] = newNeonMemInst<MicroStrNeon16Uop>(
                size, machInst, rMid, rn, 0, align);
        break;
      case 1:
        microOps[uopIdx++] = newNeonMemInst<MicroStrNeon8Uop>(
                size, machInst, rMid, rn, 0, align);
        break;
      default:
        // Unknown number of registers
        microOps[uopIdx++] = new Unknown(machInst);
    }
    if (wb) {
        if (rm != 15 && rm != 13) {
            microOps[uopIdx++] =
                new MicroAddUop(machInst, rn, rn, rm, 0, ArmISA::LSL);
        } else {
            microOps[uopIdx++] =
                new MicroAddiUop(machInst, rn, rn, regs * 8);
        }
    }
    assert(uopIdx == numMicroops);

    for (unsigned i = 0; i < numMicroops - 1; i++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(microOps[i].get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VstSingleOp::VstSingleOp(const char *mnem, ExtMachInst machInst,
                         OpClass __opClass, bool all, unsigned elems,
                         RegIndex rn, RegIndex vd, unsigned regs,
                         unsigned inc, uint32_t size, uint32_t align,
                         RegIndex rm, unsigned lane) :
    PredMacroOp(mnem, machInst, __opClass)
{
    assert(!all);
    assert(regs > 0 && regs <= 4);
    assert(regs % elems == 0);

    unsigned eBytes = (1 << size);
    unsigned storeSize = eBytes * elems;
    unsigned storeRegs M5_VAR_USED = (storeSize + sizeof(FloatRegBits) - 1) /
                         sizeof(FloatRegBits);

    assert(storeRegs > 0 && storeRegs <= 4);

    numMicroops = 1;
    bool wb = (rm != 15);

    if (wb) numMicroops++;
    numMicroops += (regs / elems);
    microOps = new StaticInstPtr[numMicroops];

    RegIndex ufp0 = NumFloatV7ArchRegs;

    unsigned uopIdx = 0;
    switch (elems) {
      case 4:
        assert(regs == 4);
        switch (size) {
          case 0:
            microOps[uopIdx++] = new MicroPackNeon8to2Uop<uint8_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          case 1:
            microOps[uopIdx++] = new MicroPackNeon8to2Uop<uint16_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          case 2:
            microOps[uopIdx++] = new MicroPackNeon8to4Uop<uint32_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          default:
            // Bad size
            microOps[uopIdx++] = new Unknown(machInst);
            break;
        }
        break;
      case 3:
        assert(regs == 3);
        switch (size) {
          case 0:
            microOps[uopIdx++] = new MicroPackNeon6to2Uop<uint8_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          case 1:
            microOps[uopIdx++] = new MicroPackNeon6to2Uop<uint16_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          case 2:
            microOps[uopIdx++] = new MicroPackNeon6to4Uop<uint32_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          default:
            // Bad size
            microOps[uopIdx++] = new Unknown(machInst);
            break;
        }
        break;
      case 2:
        assert(regs == 2);
        assert(storeRegs <= 2);
        switch (size) {
          case 0:
            microOps[uopIdx++] = new MicroPackNeon4to2Uop<uint8_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          case 1:
            microOps[uopIdx++] = new MicroPackNeon4to2Uop<uint16_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          case 2:
            microOps[uopIdx++] = new MicroPackNeon4to2Uop<uint32_t>(
                    machInst, ufp0, vd * 2, inc * 2, lane);
            break;
          default:
            // Bad size
            microOps[uopIdx++] = new Unknown(machInst);
            break;
        }
        break;
      case 1:
        assert(regs == 1 || (all && regs == 2));
        assert(storeRegs <= 2);
        for (unsigned offset = 0; offset < regs; offset++) {
            switch (size) {
              case 0:
                microOps[uopIdx++] = new MicroPackNeon2to2Uop<uint8_t>(
                        machInst, ufp0, (vd + offset) * 2, inc * 2, lane);
                break;
              case 1:
                microOps[uopIdx++] = new MicroPackNeon2to2Uop<uint16_t>(
                        machInst, ufp0, (vd + offset) * 2, inc * 2, lane);
                break;
              case 2:
                microOps[uopIdx++] = new MicroPackNeon2to2Uop<uint32_t>(
                        machInst, ufp0, (vd + offset) * 2, inc * 2, lane);
                break;
              default:
                // Bad size
                microOps[uopIdx++] = new Unknown(machInst);
                break;
            }
        }
        break;
      default:
        // Bad number of elements to unpack
        microOps[uopIdx++] = new Unknown(machInst);
    }
    switch (storeSize) {
      case 1:
        microOps[uopIdx++] = new MicroStrNeon1Uop<uint8_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 2:
        if (eBytes == 2) {
            microOps[uopIdx++] = new MicroStrNeon2Uop<uint16_t>(
                    machInst, ufp0, rn, 0, align);
        } else {
            microOps[uopIdx++] = new MicroStrNeon2Uop<uint8_t>(
                    machInst, ufp0, rn, 0, align);
        }
        break;
      case 3:
        microOps[uopIdx++] = new MicroStrNeon3Uop<uint8_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 4:
        switch (eBytes) {
          case 1:
            microOps[uopIdx++] = new MicroStrNeon4Uop<uint8_t>(
                    machInst, ufp0, rn, 0, align);
            break;
          case 2:
            microOps[uopIdx++] = new MicroStrNeon4Uop<uint16_t>(
                    machInst, ufp0, rn, 0, align);
            break;
          case 4:
            microOps[uopIdx++] = new MicroStrNeon4Uop<uint32_t>(
                    machInst, ufp0, rn, 0, align);
            break;
        }
        break;
      case 6:
        microOps[uopIdx++] = new MicroStrNeon6Uop<uint16_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 8:
        switch (eBytes) {
          case 2:
            microOps[uopIdx++] = new MicroStrNeon8Uop<uint16_t>(
                    machInst, ufp0, rn, 0, align);
            break;
          case 4:
            microOps[uopIdx++] = new MicroStrNeon8Uop<uint32_t>(
                    machInst, ufp0, rn, 0, align);
            break;
        }
        break;
      case 12:
        microOps[uopIdx++] = new MicroStrNeon12Uop<uint32_t>(
                machInst, ufp0, rn, 0, align);
        break;
      case 16:
        microOps[uopIdx++] = new MicroStrNeon16Uop<uint32_t>(
                machInst, ufp0, rn, 0, align);
        break;
      default:
        // Bad store size
        microOps[uopIdx++] = new Unknown(machInst);
    }
    if (wb) {
        if (rm != 15 && rm != 13) {
            microOps[uopIdx++] =
                new MicroAddUop(machInst, rn, rn, rm, 0, ArmISA::LSL);
        } else {
            microOps[uopIdx++] =
                new MicroAddiUop(machInst, rn, rn, storeSize);
        }
    }
    assert(uopIdx == numMicroops);

    for (unsigned i = 0; i < numMicroops - 1; i++) {
        MicroOp * uopPtr = dynamic_cast<MicroOp *>(microOps[i].get());
        assert(uopPtr);
        uopPtr->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VldMultOp64::VldMultOp64(const char *mnem, ExtMachInst machInst,
                         OpClass __opClass, RegIndex rn, RegIndex vd,
                         RegIndex rm, uint8_t eSize, uint8_t dataSize,
                         uint8_t numStructElems, uint8_t numRegs, bool wb) :
    PredMacroOp(mnem, machInst, __opClass)
{
    RegIndex vx = NumFloatV8ArchRegs / 4;
    RegIndex rnsp = (RegIndex) makeSP((IntRegIndex) rn);
    bool baseIsSP = isSP((IntRegIndex) rnsp);

    numMicroops = wb ? 1 : 0;

    int totNumBytes = numRegs * dataSize / 8;
    assert(totNumBytes <= 64);

    // The guiding principle here is that no more than 16 bytes can be
    // transferred at a time
    int numMemMicroops = totNumBytes / 16;
    int residuum = totNumBytes % 16;
    if (residuum)
        ++numMemMicroops;
    numMicroops += numMemMicroops;

    int numMarshalMicroops = numRegs / 2 + (numRegs % 2 ? 1 : 0);
    numMicroops += numMarshalMicroops;

    microOps = new StaticInstPtr[numMicroops];
    unsigned uopIdx = 0;
    uint32_t memaccessFlags = TLB::MustBeOne | (TLB::ArmFlags) eSize |
        TLB::AllowUnaligned;

    int i = 0;
    for(; i < numMemMicroops - 1; ++i) {
        microOps[uopIdx++] = new MicroNeonLoad64(
            machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags,
            baseIsSP, 16 /* accSize */, eSize);
    }
    microOps[uopIdx++] =  new MicroNeonLoad64(
        machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags, baseIsSP,
        residuum ? residuum : 16 /* accSize */, eSize);

    // Writeback microop: the post-increment amount is encoded in "Rm": a
    // 64-bit general register OR as '11111' for an immediate value equal to
    // the total number of bytes transferred (i.e. 8, 16, 24, 32, 48 or 64)
    if (wb) {
        if (rm != ((RegIndex) INTREG_X31)) {
            microOps[uopIdx++] = new MicroAddXERegUop(machInst, rnsp, rnsp, rm,
                                                      UXTX, 0);
        } else {
            microOps[uopIdx++] = new MicroAddXiUop(machInst, rnsp, rnsp,
                                                   totNumBytes);
        }
    }

    for (int i = 0; i < numMarshalMicroops; ++i) {
        microOps[uopIdx++] = new MicroDeintNeon64(
            machInst, vd + (RegIndex) (2 * i), vx, eSize, dataSize,
            numStructElems, numRegs, i /* step */);
    }

    assert(uopIdx == numMicroops);

    for (int i = 0; i < numMicroops - 1; ++i) {
        microOps[i]->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VstMultOp64::VstMultOp64(const char *mnem, ExtMachInst machInst,
                         OpClass __opClass, RegIndex rn, RegIndex vd,
                         RegIndex rm, uint8_t eSize, uint8_t dataSize,
                         uint8_t numStructElems, uint8_t numRegs, bool wb) :
    PredMacroOp(mnem, machInst, __opClass)
{
    RegIndex vx = NumFloatV8ArchRegs / 4;
    RegIndex rnsp = (RegIndex) makeSP((IntRegIndex) rn);
    bool baseIsSP = isSP((IntRegIndex) rnsp);

    numMicroops = wb ? 1 : 0;

    int totNumBytes = numRegs * dataSize / 8;
    assert(totNumBytes <= 64);

    // The guiding principle here is that no more than 16 bytes can be
    // transferred at a time
    int numMemMicroops = totNumBytes / 16;
    int residuum = totNumBytes % 16;
    if (residuum)
        ++numMemMicroops;
    numMicroops += numMemMicroops;

    int numMarshalMicroops = totNumBytes > 32 ? 2 : 1;
    numMicroops += numMarshalMicroops;

    microOps = new StaticInstPtr[numMicroops];
    unsigned uopIdx = 0;

    for(int i = 0; i < numMarshalMicroops; ++i) {
        microOps[uopIdx++] = new MicroIntNeon64(
            machInst, vx + (RegIndex) (2 * i), vd, eSize, dataSize,
            numStructElems, numRegs, i /* step */);
    }

    uint32_t memaccessFlags = TLB::MustBeOne | (TLB::ArmFlags) eSize |
        TLB::AllowUnaligned;

    int i = 0;
    for(; i < numMemMicroops - 1; ++i) {
        microOps[uopIdx++] = new MicroNeonStore64(
            machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags,
            baseIsSP, 16 /* accSize */, eSize);
    }
    microOps[uopIdx++] = new MicroNeonStore64(
        machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags, baseIsSP,
        residuum ? residuum : 16 /* accSize */, eSize);

    // Writeback microop: the post-increment amount is encoded in "Rm": a
    // 64-bit general register OR as '11111' for an immediate value equal to
    // the total number of bytes transferred (i.e. 8, 16, 24, 32, 48 or 64)
    if (wb) {
        if (rm != ((RegIndex) INTREG_X31)) {
            microOps[uopIdx++] = new MicroAddXERegUop(machInst, rnsp, rnsp, rm,
                                                      UXTX, 0);
        } else {
            microOps[uopIdx++] = new MicroAddXiUop(machInst, rnsp, rnsp,
                                                   totNumBytes);
        }
    }

    assert(uopIdx == numMicroops);

    for (int i = 0; i < numMicroops - 1; i++) {
        microOps[i]->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VldSingleOp64::VldSingleOp64(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, RegIndex rn, RegIndex vd,
                             RegIndex rm, uint8_t eSize, uint8_t dataSize,
                             uint8_t numStructElems, uint8_t index, bool wb,
                             bool replicate) :
    PredMacroOp(mnem, machInst, __opClass)
{
    RegIndex vx = NumFloatV8ArchRegs / 4;
    RegIndex rnsp = (RegIndex) makeSP((IntRegIndex) rn);
    bool baseIsSP = isSP((IntRegIndex) rnsp);

    numMicroops = wb ? 1 : 0;

    int eSizeBytes = 1 << eSize;
    int totNumBytes = numStructElems * eSizeBytes;
    assert(totNumBytes <= 64);

    // The guiding principle here is that no more than 16 bytes can be
    // transferred at a time
    int numMemMicroops = totNumBytes / 16;
    int residuum = totNumBytes % 16;
    if (residuum)
        ++numMemMicroops;
    numMicroops += numMemMicroops;

    int numMarshalMicroops = numStructElems / 2 + (numStructElems % 2 ? 1 : 0);
    numMicroops += numMarshalMicroops;

    microOps = new StaticInstPtr[numMicroops];
    unsigned uopIdx = 0;

    uint32_t memaccessFlags = TLB::MustBeOne | (TLB::ArmFlags) eSize |
        TLB::AllowUnaligned;

    int i = 0;
    for (; i < numMemMicroops - 1; ++i) {
        microOps[uopIdx++] = new MicroNeonLoad64(
            machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags,
            baseIsSP, 16 /* accSize */, eSize);
    }
    microOps[uopIdx++] = new MicroNeonLoad64(
        machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags, baseIsSP,
        residuum ? residuum : 16 /* accSize */, eSize);

    // Writeback microop: the post-increment amount is encoded in "Rm": a
    // 64-bit general register OR as '11111' for an immediate value equal to
    // the total number of bytes transferred (i.e. 8, 16, 24, 32, 48 or 64)
    if (wb) {
        if (rm != ((RegIndex) INTREG_X31)) {
            microOps[uopIdx++] = new MicroAddXERegUop(machInst, rnsp, rnsp, rm,
                                                      UXTX, 0);
        } else {
            microOps[uopIdx++] = new MicroAddXiUop(machInst, rnsp, rnsp,
                                                   totNumBytes);
        }
    }

    for(int i = 0; i < numMarshalMicroops; ++i) {
        microOps[uopIdx++] = new MicroUnpackNeon64(
            machInst, vd + (RegIndex) (2 * i), vx, eSize, dataSize,
            numStructElems, index, i /* step */, replicate);
    }

    assert(uopIdx == numMicroops);

    for (int i = 0; i < numMicroops - 1; i++) {
        microOps[i]->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
}

VstSingleOp64::VstSingleOp64(const char *mnem, ExtMachInst machInst,
                             OpClass __opClass, RegIndex rn, RegIndex vd,
                             RegIndex rm, uint8_t eSize, uint8_t dataSize,
                             uint8_t numStructElems, uint8_t index, bool wb,
                             bool replicate) :
    PredMacroOp(mnem, machInst, __opClass)
{
    RegIndex vx = NumFloatV8ArchRegs / 4;
    RegIndex rnsp = (RegIndex) makeSP((IntRegIndex) rn);
    bool baseIsSP = isSP((IntRegIndex) rnsp);

    numMicroops = wb ? 1 : 0;

    int eSizeBytes = 1 << eSize;
    int totNumBytes = numStructElems * eSizeBytes;
    assert(totNumBytes <= 64);

    // The guiding principle here is that no more than 16 bytes can be
    // transferred at a time
    int numMemMicroops = totNumBytes / 16;
    int residuum = totNumBytes % 16;
    if (residuum)
        ++numMemMicroops;
    numMicroops += numMemMicroops;

    int numMarshalMicroops = totNumBytes > 32 ? 2 : 1;
    numMicroops += numMarshalMicroops;

    microOps = new StaticInstPtr[numMicroops];
    unsigned uopIdx = 0;

    for(int i = 0; i < numMarshalMicroops; ++i) {
        microOps[uopIdx++] = new MicroPackNeon64(
            machInst, vx + (RegIndex) (2 * i), vd, eSize, dataSize,
            numStructElems, index, i /* step */, replicate);
    }

    uint32_t memaccessFlags = TLB::MustBeOne | (TLB::ArmFlags) eSize |
        TLB::AllowUnaligned;

    int i = 0;
    for(; i < numMemMicroops - 1; ++i) {
        microOps[uopIdx++] = new MicroNeonStore64(
            machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags,
            baseIsSP, 16 /* accsize */, eSize);
    }
    microOps[uopIdx++] = new MicroNeonStore64(
        machInst, vx + (RegIndex) i, rnsp, 16 * i, memaccessFlags, baseIsSP,
        residuum ? residuum : 16 /* accSize */, eSize);

    // Writeback microop: the post-increment amount is encoded in "Rm": a
    // 64-bit general register OR as '11111' for an immediate value equal to
    // the total number of bytes transferred (i.e. 8, 16, 24, 32, 48 or 64)
    if (wb) {
        if (rm != ((RegIndex) INTREG_X31)) {
            microOps[uopIdx++] = new MicroAddXERegUop(machInst, rnsp, rnsp, rm,
                                                      UXTX, 0);
        } else {
            microOps[uopIdx++] = new MicroAddXiUop(machInst, rnsp, rnsp,
                                                   totNumBytes);
        }
    }

    assert(uopIdx == numMicroops);

    for (int i = 0; i < numMicroops - 1; i++) {
        microOps[i]->setDelayedCommit();
    }
    microOps[numMicroops - 1]->setLastMicroop();
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
    if (count == 0 || count > NumFloatV7ArchRegs)
        warn_once("Bad offset field for VFP load/store multiple.\n");
    if (count == 0) {
        // Force there to be at least one microop so the macroop makes sense.
        writeback = true;
    }
    if (count > NumFloatV7ArchRegs)
        count = NumFloatV7ArchRegs;

    numMicroops = count * (single ? 1 : 2) + (writeback ? 1 : 0);
    microOps = new StaticInstPtr[numMicroops];

    int64_t addr = 0;

    if (!up)
        addr = 4 * offset;

    bool tempUp = up;
    for (int j = 0; j < count; j++) {
        if (load) {
            if (single) {
                microOps[i++] = new MicroLdrFpUop(machInst, vd++, rn,
                                                  tempUp, addr);
            } else {
                microOps[i++] = new MicroLdrDBFpUop(machInst, vd++, rn,
                                                    tempUp, addr);
                microOps[i++] = new MicroLdrDTFpUop(machInst, vd++, rn, tempUp,
                                                    addr + (up ? 4 : -4));
            }
        } else {
            if (single) {
                microOps[i++] = new MicroStrFpUop(machInst, vd++, rn,
                                                  tempUp, addr);
            } else {
                microOps[i++] = new MicroStrDBFpUop(machInst, vd++, rn,
                                                    tempUp, addr);
                microOps[i++] = new MicroStrDTFpUop(machInst, vd++, rn, tempUp,
                                                    addr + (up ? 4 : -4));
            }
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
MicroIntImmOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
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
MicroIntImmXOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
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
MicroSetPCCPSR::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    ss << "[PC,CPSR]";
    return ss.str();
}

std::string
MicroIntRegXOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, ura);
    ccprintf(ss, ", ");
    printReg(ss, urb);
    printExtendOperand(false, ss, (IntRegIndex)urc, type, shiftAmt);
    return ss.str();
}

std::string
MicroIntMov::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, ura);
    ss << ", ";
    printReg(ss, urb);
    return ss.str();
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
    printReg(ss, urc);
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
