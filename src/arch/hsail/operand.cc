/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
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
 * Author: Steve Reinhardt
 */

#include "arch/hsail/operand.hh"

using namespace Brig;

bool
BaseRegOperand::init(unsigned opOffset, const BrigObject *obj,
                     unsigned &maxRegIdx, char _regFileChar)
{
    regFileChar = _regFileChar;
    const BrigOperand *brigOp = obj->getOperand(opOffset);

    if (brigOp->kind != BRIG_KIND_OPERAND_REGISTER)
        return false;

    const BrigOperandRegister *brigRegOp = (const BrigOperandRegister*)brigOp;

    regIdx = brigRegOp->regNum;

    DPRINTF(GPUReg, "Operand: regNum: %d, kind: %d\n", regIdx,
            brigRegOp->regKind);

    maxRegIdx = std::max(maxRegIdx, regIdx);

    return true;
}

void
ListOperand::init(unsigned opOffset, const BrigObject *obj)
{
    const BrigOperand *brigOp = (const BrigOperand*)obj->getOperand(opOffset);

    switch (brigOp->kind) {
      case BRIG_KIND_OPERAND_CODE_LIST:
        {
            const BrigOperandCodeList *opList =
                (const BrigOperandCodeList*)brigOp;

            const Brig::BrigData *oprnd_data =
                obj->getBrigBaseData(opList->elements);

            // Note: for calls Dest list of operands could be size of 0.
            elementCount = oprnd_data->byteCount / 4;

            DPRINTF(GPUReg, "Operand Code List: # elements: %d\n",
                    elementCount);

            for (int i = 0; i < elementCount; ++i) {
                unsigned *data_offset =
                    (unsigned*)obj->getData(opList->elements + 4 * (i + 1));

                const BrigDirectiveVariable *p =
                    (const BrigDirectiveVariable*)obj->
                    getCodeSectionEntry(*data_offset);

                StorageElement *se = obj->currentCode->storageMap->
                    findSymbol(BRIG_SEGMENT_ARG, p);

                assert(se);
                callArgs.push_back(se);
            }
        }
        break;
      default:
        fatal("ListOperand: bad operand kind %d\n", brigOp->kind);
    }
}

std::string
ListOperand::disassemble()
{
    std::string res_str("");

    for (auto it : callArgs) {
        res_str += csprintf("%s ", it->name.c_str());
    }

    return res_str;
}

void
FunctionRefOperand::init(unsigned opOffset, const BrigObject *obj)
{
    const BrigOperand *baseOp = obj->getOperand(opOffset);

    if (baseOp->kind != BRIG_KIND_OPERAND_CODE_REF) {
        fatal("FunctionRefOperand: bad operand kind %d\n", baseOp->kind);
    }

    const BrigOperandCodeRef *brigOp = (const BrigOperandCodeRef*)baseOp;

    const BrigDirectiveExecutable *p =
        (const BrigDirectiveExecutable*)obj->getCodeSectionEntry(brigOp->ref);

    func_name = obj->getString(p->name);
}

std::string
FunctionRefOperand::disassemble()
{
    DPRINTF(GPUReg, "Operand Func-ref name: %s\n", func_name);

    return csprintf("%s", func_name);
}

bool
BaseRegOperand::init_from_vect(unsigned opOffset, const BrigObject *obj,
                               int at, unsigned &maxRegIdx, char _regFileChar)
{
    regFileChar = _regFileChar;
    const BrigOperand *brigOp = obj->getOperand(opOffset);

    if (brigOp->kind != BRIG_KIND_OPERAND_OPERAND_LIST)
        return false;


    const Brig::BrigOperandOperandList *brigRegVecOp =
         (const Brig::BrigOperandOperandList*)brigOp;

    unsigned *data_offset =
        (unsigned*)obj->getData(brigRegVecOp->elements + 4 * (at + 1));

    const BrigOperand *p =
        (const BrigOperand*)obj->getOperand(*data_offset);
    if (p->kind != BRIG_KIND_OPERAND_REGISTER) {
        return false;
    }

    const BrigOperandRegister *brigRegOp =(const BrigOperandRegister*)p;

    regIdx = brigRegOp->regNum;

    DPRINTF(GPUReg, "Operand: regNum: %d, kind: %d \n", regIdx,
            brigRegOp->regKind);

    maxRegIdx = std::max(maxRegIdx, regIdx);

    return true;
}

void
BaseRegOperand::initWithStrOffset(unsigned strOffset, const BrigObject *obj,
                     unsigned &maxRegIdx, char _regFileChar)
{
    const char *name = obj->getString(strOffset);
    char *endptr;
    regIdx = strtoul(name + 2, &endptr, 10);

    if (name[0] != '$' || name[1] != _regFileChar) {
        fatal("register operand parse error on \"%s\"\n", name);
    }

    maxRegIdx = std::max(maxRegIdx, regIdx);
}

unsigned SRegOperand::maxRegIdx;
unsigned DRegOperand::maxRegIdx;
unsigned CRegOperand::maxRegIdx;

std::string
SRegOperand::disassemble()
{
    return csprintf("$s%d", regIdx);
}

std::string
DRegOperand::disassemble()
{
    return csprintf("$d%d", regIdx);
}

std::string
CRegOperand::disassemble()
{
    return csprintf("$c%d", regIdx);
}

BrigRegOperandInfo
findRegDataType(unsigned opOffset, const BrigObject *obj)
{
    const BrigOperand *baseOp = obj->getOperand(opOffset);

    switch (baseOp->kind) {
      case BRIG_KIND_OPERAND_REGISTER:
        {
            const BrigOperandRegister *op = (BrigOperandRegister*)baseOp;

            return BrigRegOperandInfo((BrigKind16_t)baseOp->kind,
                                      (BrigRegisterKind)op->regKind);
        }
        break;

      case BRIG_KIND_OPERAND_OPERAND_LIST:
        {
             const BrigOperandOperandList *op =
                (BrigOperandOperandList*)baseOp;
             const BrigData *data_p = (BrigData*)obj->getData(op->elements);


             int num_operands = 0;
             BrigRegisterKind reg_kind = (BrigRegisterKind)0;
             for (int offset = 0; offset < data_p->byteCount; offset += 4) {
                 const BrigOperand *op_p = (const BrigOperand *)
                    obj->getOperand(((int *)data_p->bytes)[offset/4]);

                 if (op_p->kind == BRIG_KIND_OPERAND_REGISTER) {
                     const BrigOperandRegister *brigRegOp =
                        (const BrigOperandRegister*)op_p;
                     reg_kind = (BrigRegisterKind)brigRegOp->regKind;
                 } else if (op_p->kind == BRIG_KIND_OPERAND_CONSTANT_BYTES) {
                     uint16_t num_bytes =
                        ((Brig::BrigOperandConstantBytes*)op_p)->base.byteCount
                            - sizeof(BrigBase);
                     if (num_bytes == sizeof(uint32_t)) {
                         reg_kind = BRIG_REGISTER_KIND_SINGLE;
                     } else if (num_bytes == sizeof(uint64_t)) {
                         reg_kind = BRIG_REGISTER_KIND_DOUBLE;
                     } else {
                         fatal("OperandList: bad operand size %d\n", num_bytes);
                     }
                 } else {
                     fatal("OperandList: bad operand kind %d\n", op_p->kind);
                 }

                 num_operands++;
             }
             assert(baseOp->kind == BRIG_KIND_OPERAND_OPERAND_LIST);

             return BrigRegOperandInfo((BrigKind16_t)baseOp->kind, reg_kind);
        }
        break;

      case BRIG_KIND_OPERAND_ADDRESS:
        {
            const BrigOperandAddress *op = (BrigOperandAddress*)baseOp;

            if (!op->reg) {
                BrigType type = BRIG_TYPE_NONE;

                if (op->symbol) {
                    const BrigDirective *dir = (BrigDirective*)
                        obj->getCodeSectionEntry(op->symbol);

                    assert(dir->kind == BRIG_KIND_DIRECTIVE_VARIABLE);

                    const BrigDirectiveVariable *sym =
                       (const BrigDirectiveVariable*)dir;

                    type = (BrigType)sym->type;
                }
                return BrigRegOperandInfo(BRIG_KIND_OPERAND_ADDRESS,
                                          (BrigType)type);
            } else {
                const BrigOperandAddress *b = (const BrigOperandAddress*)baseOp;
                const BrigOperand *reg = obj->getOperand(b->reg);
                const BrigOperandRegister *rop = (BrigOperandRegister*)reg;

                return BrigRegOperandInfo(BRIG_KIND_OPERAND_REGISTER,
                                          (BrigRegisterKind)rop->regKind);
            }
        }
        break;

     default:
       fatal("AddrOperand: bad operand kind %d\n", baseOp->kind);
       break;
   }
}

void
AddrOperandBase::parseAddr(const BrigOperandAddress *op, const BrigObject *obj)
{
    assert(op->base.kind == BRIG_KIND_OPERAND_ADDRESS);

    const BrigDirective *d =
        (BrigDirective*)obj->getCodeSectionEntry(op->symbol);

    assert(d->kind == BRIG_KIND_DIRECTIVE_VARIABLE);
    const BrigDirectiveVariable *sym = (BrigDirectiveVariable*)d;
    name = obj->getString(sym->name);

    if (sym->segment != BRIG_SEGMENT_ARG) {
        storageElement =
            obj->currentCode->storageMap->findSymbol(sym->segment, name);
        assert(storageElement);
        offset = 0;
    } else {
        // sym->name does not work for BRIG_SEGMENT_ARG for the following case:
        //
        //     void foo(int a);
        //     void bar(double a);
        //
        //     foo(...) --> arg_u32 %param_p0;
        //                  st_arg_u32 $s0, [%param_p0];
        //                  call &foo (%param_p0);
        //     bar(...) --> arg_f64 %param_p0;
        //                  st_arg_u64 $d0, [%param_p0];
        //                  call &foo (%param_p0);
        //
        //  Both functions use the same variable name (param_p0)!!!
        //
        //  Maybe this is a bug in the compiler (I don't know).
        //
        // Solution:
        // Use directive pointer (BrigDirectiveVariable) to differentiate 2
        // versions of param_p0.
        //
        // Note this solution is kind of stupid, because we are pulling stuff
        // out of the brig binary via the directive pointer and putting it into
        // the symbol table, but now we are indexing the symbol table by the
        // brig directive pointer! It makes the symbol table sort of pointless.
        // But I don't want to mess with the rest of the infrastructure, so
        // let's go with this for now.
        //
        // When we update the compiler again, we should see if this problem goes
        // away. If so, we can fold some of this functionality into the code for
        // kernel arguments. If not, maybe we can index the symbol name on a
        // hash of the variable AND function name
        storageElement = obj->currentCode->
                 storageMap->findSymbol((Brig::BrigSegment)sym->segment, sym);

        assert(storageElement);
    }
}

uint64_t
AddrOperandBase::calcUniformBase()
{
    // start with offset, will be 0 if not specified
    uint64_t address = offset;

    // add in symbol value if specified
    if (storageElement) {
        address += storageElement->offset;
    }

    return address;
}

std::string
AddrOperandBase::disassemble(std::string reg_disassembly)
{
    std::string disasm;

    if (offset || reg_disassembly != "") {
        disasm += "[";

        if (reg_disassembly != "") {
            disasm += reg_disassembly;

            if (offset > 0) {
                disasm += "+";
            }
        }

        if (offset) {
            disasm += csprintf("%d", offset);
        }

        disasm += "]";
    } else if (name) {
        disasm += csprintf("[%s]", name);
    }

    return disasm;
}

void
NoRegAddrOperand::init(unsigned opOffset, const BrigObject *obj)
{
    const BrigOperand *baseOp = obj->getOperand(opOffset);

    if (baseOp->kind == BRIG_KIND_OPERAND_ADDRESS) {
        BrigOperandAddress *addrOp = (BrigOperandAddress*)baseOp;
        parseAddr(addrOp, obj);
        offset = (uint64_t(addrOp->offset.hi) << 32) |
                  uint64_t(addrOp->offset.lo);
    } else {
        fatal("NoRegAddrOperand: bad operand kind %d\n", baseOp->kind);
    }

}

std::string
NoRegAddrOperand::disassemble()
{
    return AddrOperandBase::disassemble(std::string(""));
}

void
LabelOperand::init(unsigned opOffset, const BrigObject *obj)
{
    const BrigOperandCodeRef *op =
        (const BrigOperandCodeRef*)obj->getOperand(opOffset);

    assert(op->base.kind == BRIG_KIND_OPERAND_CODE_REF);

    const BrigDirective *dir =
        (const BrigDirective*)obj->getCodeSectionEntry(op->ref);

    assert(dir->kind == BRIG_KIND_DIRECTIVE_LABEL);
    label = obj->currentCode->refLabel((BrigDirectiveLabel*)dir, obj);
}

uint32_t
LabelOperand::getTarget(Wavefront *w, int lane)
{
    return label->get();
}

std::string
LabelOperand::disassemble()
{
    return label->name;
}
