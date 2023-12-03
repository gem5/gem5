/*
 * Copyright (c) 2018 ARM Limited
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

#ifndef __ARCH_ARM_SVE_MACROMEM_HH__
#define __ARCH_ARM_SVE_MACROMEM_HH__

#include "arch/arm/generated/decoder.hh"
#include "arch/arm/insts/pred_inst.hh"

namespace gem5
{

namespace ArmISA
{

template <typename Element, template <typename> class MicroopLdMemType,
          template <typename> class MicroopDeIntrlvType>
class SveLdStructSS : public PredMacroOp
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    RegIndex offset;
    uint8_t numregs;

  public:
    SveLdStructSS(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                  RegIndex _dest, RegIndex _gp, RegIndex _base,
                  RegIndex _offset, uint8_t _numregs)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest),
          gp(_gp),
          base(_base),
          offset(_offset),
          numregs(_numregs)
    {
        numMicroops = numregs * 2;

        microOps = new StaticInstPtr[numMicroops];

        for (int i = 0; i < numregs; ++i) {
            microOps[i] = new MicroopLdMemType<Element>(
                mnem, machInst, static_cast<RegIndex>(INTRLVREG0 + i), _gp,
                _base, _offset, _numregs, i);
        }
        for (int i = 0; i < numregs; ++i) {
            microOps[i + numregs] = new MicroopDeIntrlvType<Element>(
                mnem, machInst, static_cast<RegIndex>((_dest + i) % 32),
                _numregs, i, this);
        }

        microOps[0]->setFirstMicroop();
        microOps[numMicroops - 1]->setLastMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        for (int i = 0; i < numregs; ++i) {
            printVecReg(ss, (dest + i) % 32, true);
            if (i < numregs - 1)
                ccprintf(ss, ", ");
        }
        ccprintf(ss, "}, ");
        printVecPredReg(ss, gp);
        ccprintf(ss, "/z, [");
        printIntReg(ss, base);
        ccprintf(ss, ", ");
        printIntReg(ss, offset);
        ccprintf(ss, "]");
        return ss.str();
    }
};

template <typename Element, template <typename> class MicroopStMemType,
          template <typename> class MicroopIntrlvType>
class SveStStructSS : public PredMacroOp
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    RegIndex offset;
    uint8_t numregs;

  public:
    SveStStructSS(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                  RegIndex _dest, RegIndex _gp, RegIndex _base,
                  RegIndex _offset, uint8_t _numregs)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest),
          gp(_gp),
          base(_base),
          offset(_offset),
          numregs(_numregs)
    {
        numMicroops = numregs * 2;

        microOps = new StaticInstPtr[numMicroops];

        for (int i = 0; i < numregs; ++i) {
            microOps[i] = new MicroopIntrlvType<Element>(
                mnem, machInst, static_cast<RegIndex>(INTRLVREG0 + i), _dest,
                _numregs, i, this);
        }

        for (int i = 0; i < numregs; ++i) {
            microOps[i + numregs] = new MicroopStMemType<Element>(
                mnem, machInst, static_cast<RegIndex>(INTRLVREG0 + i), _gp,
                _base, _offset, _numregs, i);
        }

        microOps[0]->setFirstMicroop();
        microOps[numMicroops - 1]->setLastMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        for (int i = 0; i < numregs; ++i) {
            printVecReg(ss, (dest + i) % 32, true);
            if (i < numregs - 1)
                ccprintf(ss, ", ");
        }
        ccprintf(ss, "}, ");
        printVecPredReg(ss, gp);
        ccprintf(ss, ", [");
        printIntReg(ss, base);
        ccprintf(ss, ", ");
        printIntReg(ss, offset);
        ccprintf(ss, "]");
        return ss.str();
    }
};

template <typename Element, template <typename> class MicroopLdMemType,
          template <typename> class MicroopDeIntrlvType>
class SveLdStructSI : public PredMacroOp
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    int64_t imm;
    uint8_t numregs;

  public:
    SveLdStructSI(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                  RegIndex _dest, RegIndex _gp, RegIndex _base, int64_t _imm,
                  uint8_t _numregs)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest),
          gp(_gp),
          base(_base),
          imm(_imm),
          numregs(_numregs)
    {
        numMicroops = numregs * 2;

        microOps = new StaticInstPtr[numMicroops];

        for (int i = 0; i < numregs; ++i) {
            microOps[i] = new MicroopLdMemType<Element>(
                mnem, machInst, static_cast<RegIndex>(INTRLVREG0 + i), _gp,
                _base, _imm, _numregs, i);
        }
        for (int i = 0; i < numregs; ++i) {
            microOps[i + numregs] = new MicroopDeIntrlvType<Element>(
                mnem, machInst, static_cast<RegIndex>((_dest + i) % 32),
                _numregs, i, this);
        }

        microOps[0]->setFirstMicroop();
        microOps[numMicroops - 1]->setLastMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        for (int i = 0; i < numregs; ++i) {
            printVecReg(ss, (dest + i) % 32, true);
            if (i < numregs - 1)
                ccprintf(ss, ", ");
        }
        ccprintf(ss, "}, ");
        printVecPredReg(ss, gp);
        ccprintf(ss, "/z, [");
        printIntReg(ss, base);
        if (imm != 0) {
            ccprintf(ss, ", #%d, MUL VL", imm);
        }
        ccprintf(ss, "]");
        return ss.str();
    }
};

template <typename Element, template <typename> class MicroopStMemType,
          template <typename> class MicroopIntrlvType>
class SveStStructSI : public PredMacroOp
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    int64_t imm;
    uint8_t numregs;

  public:
    SveStStructSI(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                  RegIndex _dest, RegIndex _gp, RegIndex _base, int64_t _imm,
                  uint8_t _numregs)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest),
          gp(_gp),
          base(_base),
          imm(_imm),
          numregs(_numregs)
    {
        numMicroops = numregs * 2;

        microOps = new StaticInstPtr[numMicroops];

        for (int i = 0; i < numregs; ++i) {
            microOps[i] = new MicroopIntrlvType<Element>(
                mnem, machInst, static_cast<RegIndex>(INTRLVREG0 + i), _dest,
                _numregs, i, this);
        }

        for (int i = 0; i < numregs; ++i) {
            microOps[i + numregs] = new MicroopStMemType<Element>(
                mnem, machInst, static_cast<RegIndex>(INTRLVREG0 + i), _gp,
                _base, _imm, _numregs, i);
        }

        microOps[0]->setFirstMicroop();
        microOps[numMicroops - 1]->setLastMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        for (int i = 0; i < numregs; ++i) {
            printVecReg(ss, (dest + i) % 32, true);
            if (i < numregs - 1)
                ccprintf(ss, ", ");
        }
        ccprintf(ss, "}, ");
        printVecPredReg(ss, gp);
        ccprintf(ss, ", [");
        printIntReg(ss, base);
        if (imm != 0) {
            ccprintf(ss, ", #%d, MUL VL", imm);
        }
        ccprintf(ss, "]");
        return ss.str();
    }
};

template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType,
          template <typename> class FirstFaultWritebackMicroopType>
class SveIndexedMemVI : public PredMacroOp
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    uint64_t imm;

  public:
    SveIndexedMemVI(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _gp, RegIndex _base,
                    uint64_t _imm, bool firstFault)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest),
          gp(_gp),
          base(_base),
          imm(_imm)
    {
        bool isLoad = (__opClass == MemReadOp);
        assert(!firstFault || isLoad);

        int num_elems = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

        numMicroops = num_elems;
        if (isLoad) {
            if (firstFault) {
                numMicroops += 2;
            } else {
                numMicroops++;
            }
        }

        microOps = new StaticInstPtr[numMicroops];

        StaticInstPtr *uop = microOps;

        if (isLoad) {
            // The first microop of a gather load copies the source vector
            // register used for address calculation to an auxiliary register,
            // with all subsequent microops reading from the latter.  This is
            // needed to properly handle cases where the source vector
            // register is the same as the destination register
            *uop = new ArmISAInst::SveGatherLoadCpySrcVecMicroop(
                mnem, machInst, _base, this);
            uop++;
        }

        for (int i = 0; i < num_elems; i++, uop++) {
            *uop = new MicroopType<RegElemType, MemElemType>(
                mnem, machInst, __opClass, _dest, _gp,
                isLoad ? (RegIndex)VECREG_UREG0 : _base, _imm, i, num_elems,
                firstFault);
        }

        if (firstFault) {
            *uop = new FirstFaultWritebackMicroopType<RegElemType>(
                mnem, machInst, __opClass, num_elems, this);
        } else {
            --uop;
        }

        (*uop)->setLastMicroop();
        microOps[0]->setFirstMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        // TODO: add suffix to transfer and base registers
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        printVecReg(ss, dest, true);
        ccprintf(ss, "}, ");
        printVecPredReg(ss, gp);
        ccprintf(ss, "/z, [");
        printVecReg(ss, base, true);
        if (imm != 0) {
            ccprintf(ss, ", #%d", imm * sizeof(MemElemType));
        }
        ccprintf(ss, "]");
        return ss.str();
    }
};

template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType,
          template <typename> class FirstFaultWritebackMicroopType>
class SveIndexedMemSV : public PredMacroOp
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    RegIndex offset;

    bool offsetIs32;
    bool offsetIsSigned;
    bool offsetIsScaled;

  public:
    SveIndexedMemSV(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _gp, RegIndex _base,
                    RegIndex _offset, bool _offsetIs32, bool _offsetIsSigned,
                    bool _offsetIsScaled, bool firstFault)
        : PredMacroOp(mnem, machInst, __opClass),
          dest(_dest),
          gp(_gp),
          base(_base),
          offset(_offset),
          offsetIs32(_offsetIs32),
          offsetIsSigned(_offsetIsSigned),
          offsetIsScaled(_offsetIsScaled)
    {
        bool isLoad = (__opClass == MemReadOp);
        assert(!firstFault || isLoad);

        int num_elems = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

        numMicroops = num_elems;
        if (isLoad) {
            if (firstFault) {
                numMicroops += 2;
            } else {
                numMicroops++;
            }
        }

        microOps = new StaticInstPtr[numMicroops];

        StaticInstPtr *uop = microOps;

        if (isLoad) {
            // The first microop of a gather load copies the source vector
            // register used for address calculation to an auxiliary register,
            // with all subsequent microops reading from the latter.  This is
            // needed to properly handle cases where the source vector
            // register is the same as the destination register
            *uop = new ArmISAInst::SveGatherLoadCpySrcVecMicroop(
                mnem, machInst, _offset, this);
            uop++;
        }

        for (int i = 0; i < num_elems; i++, uop++) {
            *uop = new MicroopType<RegElemType, MemElemType>(
                mnem, machInst, __opClass, _dest, _gp, _base,
                isLoad ? (RegIndex)VECREG_UREG0 : _offset, _offsetIs32,
                _offsetIsSigned, _offsetIsScaled, i, num_elems, firstFault);
        }

        if (firstFault) {
            *uop = new FirstFaultWritebackMicroopType<RegElemType>(
                mnem, machInst, __opClass, num_elems, this);
        } else {
            --uop;
        }

        (*uop)->setLastMicroop();
        microOps[0]->setFirstMicroop();

        for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
            (*uop)->setDelayedCommit();
        }
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Execute method called when it shouldn't!");
        return NoFault;
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        // TODO: add suffix to transfer and base registers
        std::stringstream ss;
        printMnemonic(ss, "", false);
        ccprintf(ss, "{");
        printVecReg(ss, dest, true);
        ccprintf(ss, "}, ");
        printVecPredReg(ss, gp);
        ccprintf(ss, "/z, [");
        printIntReg(ss, base);
        ccprintf(ss, ", ");
        printVecReg(ss, offset, true);
        ccprintf(ss, "]");
        return ss.str();
    }
};

} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_SVE_MACROMEM_HH__
