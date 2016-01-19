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

#ifndef __BRIG_OBJECT_HH__
#define __BRIG_OBJECT_HH__

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "arch/hsail/Brig.h"
#include "gpu-compute/hsa_object.hh"
#include "gpu-compute/hsail_code.hh"

class LabelMap;
class StorageMap;

/* @class BrigObject
 * this class implements the BRIG loader object, and
 * is used when the simulator directly executes HSAIL.
 * this class is responsible for extracting all
 * information about kernels contained in BRIG format
 * and converts them to HsailCode objects that are
 * usable by the simulator and emulated runtime.
 */

class BrigObject final : public HsaObject
{
  public:
    enum SectionIndex
    {
        DataSectionIndex,
        CodeSectionIndex,
        OperandsSectionIndex,
        NumSectionIndices
    };

    static const char *sectionNames[];

    struct SectionInfo
    {
        uint8_t *ptr;
        int size;
    };

    static HsaObject* tryFile(const std::string &fname, int len,
                              uint8_t *fileData);

    SectionInfo sectionInfo[NumSectionIndices];
    const uint8_t *getSectionOffset(enum SectionIndex sec, int offs) const;

    std::vector<HsailCode*> kernels;
    std::vector<HsailCode*> functions;
    std::string kern_block_name;

    void processDirectives(const Brig::BrigBase *dirPtr,
                           const Brig::BrigBase *endPtr,
                           StorageMap *storageMap);

    BrigObject(const std::string &fname, int len, uint8_t *fileData);
    ~BrigObject();

    // eventually these will need to be per-kernel not per-object-file
    StorageMap *storageMap;
    LabelMap *labelMap;

    const char* getString(int offs) const;
    const Brig::BrigData* getBrigBaseData(int offs) const;
    const uint8_t* getData(int offs) const;
    const Brig::BrigBase* getCodeSectionEntry(int offs) const;
    const Brig::BrigOperand* getOperand(int offs) const;
    unsigned getOperandPtr(int offs, int index) const;
    const Brig::BrigInstBase* getInst(int offs) const;

    HsaCode* getKernel(const std::string &name) const override;
    HsaCode* getFunction(const std::string &name) const override;

    int numKernels() const override { return kernels.size(); }

    HsaCode* getKernel(int i) const override { return kernels[i]; }

    // pointer to the current kernel/function we're processing, so elements
    // under construction can reference it.  kinda ugly, but easier
    // than passing it all over for the few places it's needed.
    mutable HsailCode *currentCode;
};

// Utility function to bump Brig item pointer to next element given
// item size in bytes.  Really just an add but with lots of casting.
template<typename T>
T*
brigNext(T *ptr)
{
    Brig::BrigBase *base_ptr = (Brig::BrigBase*)ptr;
    int size = base_ptr->byteCount;
    assert(size);

    return (T*)((uint8_t*)ptr + size);
}

#endif // __BRIG_OBJECT_HH__
