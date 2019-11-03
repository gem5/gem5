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
 * Author: Steve Reinhardt, Anthony Gutierrez
 */

#include "gpu-compute/brig_object.hh"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <cstddef>
#include <cstdlib>

#include "arch/hsail/Brig.h"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/BRIG.hh"
#include "debug/HSAILObject.hh"
#include "debug/HSALoader.hh"

using namespace Brig;

std::vector<std::function<HsaObject*(const std::string&, int, uint8_t*)>>
    HsaObject::tryFileFuncs = { BrigObject::tryFile };

extern int getBrigDataTypeBytes(BrigType16_t t);

const char *BrigObject::sectionNames[] =
{
    "hsa_data",
    "hsa_code",
    "hsa_operand",
    ".shstrtab"
};

const char *segmentNames[] =
{
    "none",
    "flat",
    "global",
    "readonly",
    "kernarg",
    "group",
    "private",
    "spill",
    "args"
};

const uint8_t*
BrigObject::getSectionOffset(enum SectionIndex sec, int offs) const
{
    // allow offs == size for dummy end pointers
    assert(offs <= sectionInfo[sec].size);

    return sectionInfo[sec].ptr + offs;
}

const char*
BrigObject::getString(int offs) const
{
    return (const char*)(getSectionOffset(DataSectionIndex, offs) + 4);
}

const BrigBase*
BrigObject::getCodeSectionEntry(int offs) const
{
    return (const BrigBase*)getSectionOffset(CodeSectionIndex, offs);
}

const BrigData*
BrigObject::getBrigBaseData(int offs) const
{
    return (Brig::BrigData*)(getSectionOffset(DataSectionIndex, offs));
}

const uint8_t*
BrigObject::getData(int offs) const
{
    return getSectionOffset(DataSectionIndex, offs);
}

const BrigOperand*
BrigObject::getOperand(int offs) const
{
    return (const BrigOperand*)getSectionOffset(OperandsSectionIndex, offs);
}

unsigned
BrigObject::getOperandPtr(int offs, int index) const
{
    unsigned *op_offs = (unsigned*)(getData(offs + 4 * (index + 1)));

    return *op_offs;
}

const BrigInstBase*
BrigObject::getInst(int offs) const
{
    return (const BrigInstBase*)getSectionOffset(CodeSectionIndex, offs);
}

HsaCode*
BrigObject::getKernel(const std::string &name) const
{
    return nullptr;
}

HsaCode*
BrigObject::getFunction(const std::string &name) const
{
    for (int i = 0; i < functions.size(); ++i) {
        if (functions[i]->name() == name) {
            return functions[i];
        }
    }

    return nullptr;
}

void
BrigObject::processDirectives(const BrigBase *dirPtr, const BrigBase *endPtr,
                              StorageMap *storageMap)
{
    while (dirPtr < endPtr) {
        if (!dirPtr->byteCount) {
            fatal("Bad directive size 0\n");
        }

        // calculate next pointer now so we can override it if needed
        const BrigBase *nextDirPtr = brigNext(dirPtr);

        DPRINTF(HSAILObject, "Code section entry kind: #%x, byte count: %d\n",
                dirPtr->kind, dirPtr->byteCount);

        switch (dirPtr->kind) {
          case BRIG_KIND_DIRECTIVE_FUNCTION:
            {
                const BrigDirectiveExecutable *p M5_VAR_USED =
                    reinterpret_cast<const BrigDirectiveExecutable*>(dirPtr);

                DPRINTF(HSAILObject,"DIRECTIVE_FUNCTION: %s offset: "
                        "%d next: %d\n", getString(p->name),
                        p->firstCodeBlockEntry, p->nextModuleEntry);

                if (p->firstCodeBlockEntry != p->nextModuleEntry) {
                    // Function calls are not supported. We allow the BRIG
                    // object file to create stubs, but the function calls will
                    // not work properly if the application makes use of them.
                    warn("HSA function invocations are unsupported.\n");

                    const char *name = getString(p->name);

                    HsailCode *code_obj = nullptr;

                    for (int i = 0; i < functions.size(); ++i) {
                        if (functions[i]->name() == name) {
                            code_obj = functions[i];
                            break;
                        }
                    }

                    if (!code_obj) {
                        // create new local storage map for kernel-local symbols
                        code_obj = new HsailCode(name, p, this,
                                                 new StorageMap(storageMap));
                        functions.push_back(code_obj);
                    } else {
                        panic("Multiple definition of Function!!: %s\n",
                              getString(p->name));
                    }
                }

                nextDirPtr = getCodeSectionEntry(p->nextModuleEntry);
            }
            break;

          case BRIG_KIND_DIRECTIVE_KERNEL:
            {
                const BrigDirectiveExecutable *p =
                    reinterpret_cast<const BrigDirectiveExecutable*>(dirPtr);

                DPRINTF(HSAILObject,"DIRECTIVE_KERNEL: %s offset: %d count: "
                        "next: %d\n", getString(p->name),
                        p->firstCodeBlockEntry, p->nextModuleEntry);

                const char *name = getString(p->name);

                if (name[0] == '&')
                    name++;

                std::string str = name;
                char *temp;
                int len = str.length();

                if (str[len - 1] >= 'a' && str[len - 1] <= 'z') {
                    temp = new char[str.size() + 1];
                    std::copy(str.begin(), str.end() , temp);
                    temp[str.size()] = '\0';
                } else {
                    temp = new char[str.size()];
                    std::copy(str.begin(), str.end() - 1 , temp);
                    temp[str.size() - 1 ] = '\0';
                }

                std::string kernel_name = temp;
                delete[] temp;

                HsailCode *code_obj = nullptr;

                for (const auto &kernel : kernels) {
                    if (kernel->name() == kernel_name) {
                        code_obj = kernel;
                        break;
                    }
                }

                if (!code_obj) {
                    // create new local storage map for kernel-local symbols
                    code_obj = new HsailCode(kernel_name, p, this,
                                             new StorageMap(storageMap));

                    kernels.push_back(code_obj);
                }

                nextDirPtr = getCodeSectionEntry(p->nextModuleEntry);
            }
            break;

          case BRIG_KIND_DIRECTIVE_VARIABLE:
            {
                const BrigDirectiveVariable *p =
                    reinterpret_cast<const BrigDirectiveVariable*>(dirPtr);

                uint64_t readonlySize_old =
                    storageMap->getSize(BRIG_SEGMENT_READONLY);

                StorageElement* se = storageMap->addSymbol(p, this);

                DPRINTF(HSAILObject, "DIRECTIVE_VARIABLE, symbol %s\n",
                        getString(p->name));

                if (p->segment == BRIG_SEGMENT_READONLY) {
                    // readonly memory has initialization data
                    uint8_t* readonlyData_old = readonlyData;

                    readonlyData =
                        new uint8_t[storageMap->getSize(BRIG_SEGMENT_READONLY)];

                    if (p->init) {
                        if ((p->type == BRIG_TYPE_ROIMG) ||
                            (p->type == BRIG_TYPE_WOIMG) ||
                            (p->type == BRIG_TYPE_SAMP) ||
                            (p->type == BRIG_TYPE_SIG32) ||
                            (p->type == BRIG_TYPE_SIG64)) {
                            panic("Read only data type not supported: %s\n",
                                  getString(p->name));
                        }

                        const BrigOperand *brigOp = getOperand(p->init);
                        assert(brigOp->kind ==
                               BRIG_KIND_OPERAND_CONSTANT_BYTES);

                        const Brig::BrigData *operand_data M5_VAR_USED =
                            getBrigBaseData(((BrigOperandConstantBytes*)
                                            brigOp)->bytes);

                        assert((operand_data->byteCount / 4) > 0);

                        uint8_t *symbol_data =
                            (uint8_t*)getData(((BrigOperandConstantBytes*)
                                              brigOp)->bytes + 4);

                        // copy the old data and add the new data
                        if (readonlySize_old > 0) {
                            memcpy(readonlyData, readonlyData_old,
                                   readonlySize_old);
                        }

                        memcpy(readonlyData + se->offset, symbol_data,
                               se->size);

                        delete[] readonlyData_old;
                   }
                }
            }
            break;

          case BRIG_KIND_DIRECTIVE_LABEL:
            {
              const BrigDirectiveLabel M5_VAR_USED *p =
                    reinterpret_cast<const BrigDirectiveLabel*>(dirPtr);

              panic("Label directives cannot be at the module level: %s\n",
                    getString(p->name));

            }
            break;

          case BRIG_KIND_DIRECTIVE_COMMENT:
            {
              const BrigDirectiveComment M5_VAR_USED *p =
                  reinterpret_cast<const BrigDirectiveComment*>(dirPtr);

              DPRINTF(HSAILObject, "DIRECTIVE_COMMENT: %s\n",
                      getString(p->name));
            }
            break;

          case BRIG_KIND_DIRECTIVE_LOC:
            {
                DPRINTF(HSAILObject, "BRIG_DIRECTIVE_LOC\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_MODULE:
            {
                const BrigDirectiveModule M5_VAR_USED *p =
                    reinterpret_cast<const BrigDirectiveModule*>(dirPtr);

                DPRINTF(HSAILObject, "BRIG_DIRECTIVE_MODULE: %s\n",
                        getString(p->name));
            }
            break;

          case BRIG_KIND_DIRECTIVE_CONTROL:
            {
                DPRINTF(HSAILObject, "DIRECTIVE_CONTROL\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_PRAGMA:
            {
                DPRINTF(HSAILObject, "DIRECTIVE_PRAGMA\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_EXTENSION:
            {
                DPRINTF(HSAILObject, "DIRECTIVE_EXTENSION\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_ARG_BLOCK_START:
            {
                DPRINTF(HSAILObject, "DIRECTIVE_ARG_BLOCK_START\n");
            }
            break;

          case BRIG_KIND_DIRECTIVE_ARG_BLOCK_END:
            {
                DPRINTF(HSAILObject, "DIRECTIVE_ARG_BLOCK_END\n");
            }
            break;
          default:
            if (dirPtr->kind >= BRIG_KIND_INST_BEGIN &&
                dirPtr->kind <= BRIG_KIND_INST_END)
                break;

            if (dirPtr->kind >= BRIG_KIND_OPERAND_BEGIN &&
                dirPtr->kind <= BRIG_KIND_OPERAND_END)
                break;

            warn("Unknown Brig directive kind: %d\n", dirPtr->kind);
            break;
        }

        dirPtr = nextDirPtr;
    }
}

HsaObject*
BrigObject::tryFile(const std::string &fname, int len, uint8_t *fileData)
{
    const char *brig_ident = "HSA BRIG";

    if (memcmp(brig_ident, fileData, MODULE_IDENTIFICATION_LENGTH))
        return nullptr;

    return new BrigObject(fname, len, fileData);
}

BrigObject::BrigObject(const std::string &fname, int len, uint8_t *fileData)
    : HsaObject(fname), storageMap(new StorageMap())
{
    const char *brig_ident = "HSA BRIG";
    BrigModuleHeader *mod_hdr = (BrigModuleHeader*)fileData;

    fatal_if(memcmp(brig_ident, mod_hdr, MODULE_IDENTIFICATION_LENGTH),
             "%s is not a BRIG file\n", fname);

    if (mod_hdr->brigMajor != BRIG_VERSION_BRIG_MAJOR ||
        mod_hdr->brigMinor != BRIG_VERSION_BRIG_MINOR) {
        fatal("%s: BRIG version mismatch, %d.%d != %d.%d\n",
              fname, mod_hdr->brigMajor, mod_hdr->brigMinor,
              BRIG_VERSION_BRIG_MAJOR, BRIG_VERSION_BRIG_MINOR);
    }

    fatal_if(mod_hdr->sectionCount != NumSectionIndices, "%s: BRIG section "
             "count (%d) != expected value (%d)\n", fname,
             mod_hdr->sectionCount, NumSectionIndices);

    for (int i = 0; i < NumSectionIndices; ++i) {
        sectionInfo[i].ptr = nullptr;
    }

    uint64_t *sec_idx_table = (uint64_t*)(fileData + mod_hdr->sectionIndex);
    for (int sec_idx = 0; sec_idx < mod_hdr->sectionCount; ++sec_idx) {
        uint8_t *sec_hdr_byte_ptr = fileData + sec_idx_table[sec_idx];
        BrigSectionHeader *sec_hdr = (BrigSectionHeader*)sec_hdr_byte_ptr;

        // It doesn't look like cprintf supports string precision values,
        // but if this breaks, the right answer is to fix that
        DPRINTF(HSAILObject, "found section %.*s\n", sec_hdr->nameLength,
                sec_hdr->name);

        sectionInfo[sec_idx].ptr = new uint8_t[sec_hdr->byteCount];
        memcpy(sectionInfo[sec_idx].ptr, sec_hdr_byte_ptr, sec_hdr->byteCount);
        sectionInfo[sec_idx].size = sec_hdr->byteCount;
    }

    BrigSectionHeader *code_hdr =
        (BrigSectionHeader*)sectionInfo[CodeSectionIndex].ptr;

    DPRINTF(HSAILObject, "Code section hdr, count: %d, hdr count: %d, "
            "name len: %d\n", code_hdr->byteCount, code_hdr->headerByteCount,
            code_hdr->nameLength);

    // start at offset 4 to skip initial null entry (see Brig spec)
    processDirectives(getCodeSectionEntry(code_hdr->headerByteCount),
                      getCodeSectionEntry(sectionInfo[CodeSectionIndex].size),
                      storageMap);

    delete[] fileData;

    DPRINTF(HSALoader, "BRIG object %s loaded.\n", fname);
}

BrigObject::~BrigObject()
{
    for (int i = 0; i < NumSectionIndices; ++i)
        if (sectionInfo[i].ptr)
            delete[] sectionInfo[i].ptr;
}
