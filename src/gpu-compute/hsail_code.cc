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

#include "gpu-compute/hsail_code.hh"

#include "arch/gpu_types.hh"
#include "arch/hsail/Brig.h"
#include "arch/hsail/operand.hh"
#include "config/the_gpu_isa.hh"
#include "debug/BRIG.hh"
#include "debug/HSAILObject.hh"
#include "gpu-compute/brig_object.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/kernel_cfg.hh"

using namespace Brig;

int getBrigDataTypeBytes(BrigType16_t t);

HsailCode::HsailCode(const std::string &name_str)
    : HsaCode(name_str), private_size(-1), readonly_size(-1)
{
}

void
HsailCode::init(const BrigDirectiveExecutable *code_dir, const BrigObject *obj,
                StorageMap *objStorageMap)
{
    storageMap = objStorageMap;

    // set pointer so that decoding process can find this kernel context when
    // needed
    obj->currentCode = this;

    if (code_dir->base.kind != BRIG_KIND_DIRECTIVE_FUNCTION &&
        code_dir->base.kind != BRIG_KIND_DIRECTIVE_KERNEL) {
        fatal("unexpected directive kind %d inside kernel/function init\n",
              code_dir->base.kind);
    }

    DPRINTF(HSAILObject, "Initializing code, first code block entry is: %d\n",
            code_dir->firstCodeBlockEntry);

    // clear these static vars so we can properly track the max index
    // for this kernel
    SRegOperand::maxRegIdx = 0;
    DRegOperand::maxRegIdx = 0;
    CRegOperand::maxRegIdx = 0;
    setPrivateSize(0);

    const BrigBase *entryPtr = brigNext((BrigBase*)code_dir);
    const BrigBase *endPtr =
        obj->getCodeSectionEntry(code_dir->nextModuleEntry);

    // the instruction's byte address (relative to the base addr
    // of the code section)
    int inst_addr = 0;
    // the index that points to the instruction in the instruction
    // array
    int inst_idx = 0;
    std::vector<GPUStaticInst*> instructions;
    int funcarg_size_scope = 0;

    // walk through instructions in code section and directives in
    // directive section in parallel, processing directives that apply
    // when we reach the relevant code point.
    while (entryPtr < endPtr) {
        switch (entryPtr->kind) {
          case BRIG_KIND_DIRECTIVE_VARIABLE:
           {
                const BrigDirectiveVariable *sym =
                    (const BrigDirectiveVariable*)entryPtr;

                DPRINTF(HSAILObject,"Initializing code, directive is "
                        "kind_variable, symbol is: %s\n",
                        obj->getString(sym->name));

                StorageElement *se = storageMap->addSymbol(sym, obj);

                if (sym->segment == BRIG_SEGMENT_PRIVATE) {
                    setPrivateSize(se->size);
                } else { // spill
                    funcarg_size_scope += se->size;
                }
            }
            break;

          case BRIG_KIND_DIRECTIVE_LABEL:
            {
                const BrigDirectiveLabel *lbl =
                    (const BrigDirectiveLabel*)entryPtr;

                DPRINTF(HSAILObject,"Initializing code, directive is "
                        "kind_label, label is: %s \n",
                        obj->getString(lbl->name));

                labelMap.addLabel(lbl, inst_addr, obj);
            }
            break;

          case BRIG_KIND_DIRECTIVE_PRAGMA:
            {
                DPRINTF(HSAILObject, "Initializing code, directive "
                        "is kind_pragma\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_COMMENT:
            {
                DPRINTF(HSAILObject, "Initializing code, directive is "
                        "kind_comment\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_ARG_BLOCK_START:
            {
                DPRINTF(HSAILObject, "Initializing code, directive is "
                        "kind_arg_block_start\n");

                storageMap->resetOffset(BRIG_SEGMENT_ARG);
                funcarg_size_scope = 0;
            }
            break;

          case BRIG_KIND_DIRECTIVE_ARG_BLOCK_END:
            {
                DPRINTF(HSAILObject, "Initializing code, directive is "
                        "kind_arg_block_end\n");

                funcarg_size = funcarg_size < funcarg_size_scope ?
                                              funcarg_size_scope : funcarg_size;
            }
            break;

          case BRIG_KIND_DIRECTIVE_END:
            DPRINTF(HSAILObject, "Initializing code, dircetive is "
                    "kind_end\n");

            break;

          default:
            if (entryPtr->kind >= BRIG_KIND_INST_BEGIN &&
                entryPtr->kind <= BRIG_KIND_INST_END) {

                BrigInstBase *instPtr = (BrigInstBase*)entryPtr;
                TheGpuISA::MachInst machInst = { instPtr, obj };
                GPUStaticInst *iptr = decoder.decode(machInst);

                if (iptr) {
                    DPRINTF(HSAILObject, "Initializing code, processing inst "
                            "byte addr #%d idx %d: OPCODE=%d\n", inst_addr,
                            inst_idx, instPtr->opcode);

                    TheGpuISA::RawMachInst raw_inst = decoder.saveInst(iptr);
                    iptr->instNum(inst_idx);
                    iptr->instAddr(inst_addr);
                    _insts.push_back(raw_inst);
                    instructions.push_back(iptr);
                }
                inst_addr += sizeof(TheGpuISA::RawMachInst);
                ++inst_idx;
            } else if (entryPtr->kind >= BRIG_KIND_OPERAND_BEGIN &&
                       entryPtr->kind < BRIG_KIND_OPERAND_END) {
                warn("unexpected operand entry in code segment\n");
            } else {
                // there are surely some more cases we will need to handle,
                // but we'll deal with them as we find them.
                fatal("unexpected directive kind %d inside kernel scope\n",
                      entryPtr->kind);
            }
        }

        entryPtr = brigNext(entryPtr);
    }

    // compute Control Flow Graph for current kernel
    ControlFlowInfo::assignImmediatePostDominators(instructions);

    max_sreg = SRegOperand::maxRegIdx;
    max_dreg = DRegOperand::maxRegIdx;
    max_creg = CRegOperand::maxRegIdx;

    obj->currentCode = nullptr;
}

HsailCode::HsailCode(const std::string &name_str,
                     const BrigDirectiveExecutable *code_dir,
                     const BrigObject *obj, StorageMap *objStorageMap)
    : HsaCode(name_str), private_size(-1), readonly_size(-1)
{
    init(code_dir, obj, objStorageMap);
}

void
LabelMap::addLabel(const Brig::BrigDirectiveLabel *lblDir, int inst_index,
                   const BrigObject *obj)
{
    std::string lbl_name = obj->getString(lblDir->name);
    Label &lbl = map[lbl_name];

    if (lbl.defined()) {
        fatal("Attempt to redefine existing label %s\n", lbl_name);
    }

    lbl.define(lbl_name, inst_index);
    DPRINTF(HSAILObject, "label %s = %d\n", lbl_name, inst_index);
}

Label*
LabelMap::refLabel(const Brig::BrigDirectiveLabel *lblDir,
                   const BrigObject *obj)
{
    std::string name = obj->getString(lblDir->name);
    Label &lbl = map[name];
    lbl.checkName(name);

    return &lbl;
}

int
getBrigDataTypeBytes(BrigType16_t t)
{
    switch (t) {
      case BRIG_TYPE_S8:
      case BRIG_TYPE_U8:
      case BRIG_TYPE_B8:
        return 1;

      case BRIG_TYPE_S16:
      case BRIG_TYPE_U16:
      case BRIG_TYPE_B16:
      case BRIG_TYPE_F16:
        return 2;

      case BRIG_TYPE_S32:
      case BRIG_TYPE_U32:
      case BRIG_TYPE_B32:
      case BRIG_TYPE_F32:
        return 4;

      case BRIG_TYPE_S64:
      case BRIG_TYPE_U64:
      case BRIG_TYPE_B64:
      case BRIG_TYPE_F64:
        return 8;

      case BRIG_TYPE_B1:

      default:
        fatal("unhandled symbol data type %d", t);
        return 0;
    }
}

StorageElement*
StorageSpace::addSymbol(const BrigDirectiveVariable *sym,
                        const BrigObject *obj)
{
    const char *sym_name = obj->getString(sym->name);
    uint64_t size = 0;
    uint64_t offset = 0;

    if (sym->type & BRIG_TYPE_ARRAY) {
        size = getBrigDataTypeBytes(sym->type & ~BRIG_TYPE_ARRAY);
        size *= (((uint64_t)sym->dim.hi) << 32 | (uint64_t)sym->dim.lo);

        offset = roundUp(nextOffset, getBrigDataTypeBytes(sym->type &
                         ~BRIG_TYPE_ARRAY));
    } else {
        size = getBrigDataTypeBytes(sym->type);
        offset = roundUp(nextOffset, getBrigDataTypeBytes(sym->type));
    }

    nextOffset = offset + size;

    DPRINTF(HSAILObject, "Adding %s SYMBOL %s size %d offset 0x%x, init: %d\n",
            segmentNames[segment], sym_name, size, offset, sym->init);

    StorageElement* se = new StorageElement(sym_name, offset, size, sym);
    elements.push_back(se);
    elements_by_addr.insert(AddrRange(offset, offset + size - 1), se);
    elements_by_brigptr[sym] = se;

    return se;
}

StorageElement*
StorageSpace::findSymbol(std::string name)
{
    for (auto it : elements) {
        if (it->name == name) {
            return it;
        }
    }

    return nullptr;
}

StorageElement*
StorageSpace::findSymbol(uint64_t addr)
{
    assert(elements_by_addr.size() > 0);

    auto se = elements_by_addr.find(addr);

    if (se == elements_by_addr.end()) {
        return nullptr;
    } else {
        return se->second;
    }
}

StorageElement*
StorageSpace::findSymbol(const BrigDirectiveVariable *brigptr)
{
    assert(elements_by_brigptr.size() > 0);

    auto se = elements_by_brigptr.find(brigptr);

    if (se == elements_by_brigptr.end()) {
        return nullptr;
    } else {
        return se->second;
    }
}

StorageMap::StorageMap(StorageMap *outerScope)
    : outerScopeMap(outerScope)
{
    for (int i = 0; i < NumSegments; ++i)
        space[i] = new StorageSpace((BrigSegment)i);
}

StorageElement*
StorageMap::addSymbol(const BrigDirectiveVariable *sym, const BrigObject *obj)
{
    BrigSegment8_t segment = sym->segment;

    assert(segment >= Brig::BRIG_SEGMENT_FLAT);
    assert(segment < NumSegments);

    return space[segment]->addSymbol(sym, obj);
}

int
StorageMap::getSize(Brig::BrigSegment segment)
{
    assert(segment > Brig::BRIG_SEGMENT_GLOBAL);
    assert(segment < NumSegments);

    if (segment != Brig::BRIG_SEGMENT_GROUP &&
        segment != Brig::BRIG_SEGMENT_READONLY) {
        return space[segment]->getSize();
    } else {
        int ret = space[segment]->getSize();

        if (outerScopeMap) {
            ret += outerScopeMap->getSize(segment);
        }

        return ret;
    }
}

void
StorageMap::resetOffset(Brig::BrigSegment segment)
{
    space[segment]->resetOffset();
}

StorageElement*
StorageMap::findSymbol(BrigSegment segment, std::string name)
{
    StorageElement *se = space[segment]->findSymbol(name);

    if (se)
        return se;

    if (outerScopeMap)
        return outerScopeMap->findSymbol(segment, name);

    return nullptr;
}

StorageElement*
StorageMap::findSymbol(Brig::BrigSegment segment, uint64_t addr)
{
    StorageSpace *sp = space[segment];

    if (!sp) {
        // there is no memory in segment?
        return nullptr;
    }

    StorageElement *se = sp->findSymbol(addr);

    if (se)
        return se;

    if (outerScopeMap)
        return outerScopeMap->findSymbol(segment, addr);

    return nullptr;

}

StorageElement*
StorageMap::findSymbol(Brig::BrigSegment segment,
                       const BrigDirectiveVariable *brigptr)
{
    StorageSpace *sp = space[segment];

    if (!sp) {
        // there is no memory in segment?
        return nullptr;
    }

    StorageElement *se = sp->findSymbol(brigptr);

    if (se)
        return se;

    if (outerScopeMap)
        return outerScopeMap->findSymbol(segment, brigptr);

    return nullptr;

}
