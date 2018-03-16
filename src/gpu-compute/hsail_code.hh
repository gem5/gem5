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

#ifndef __HSAIL_CODE_HH__
#define __HSAIL_CODE_HH__

#include <cassert>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "arch/gpu_decoder.hh"
#include "arch/hsail/Brig.h"
#include "base/addr_range_map.hh"
#include "base/intmath.hh"
#include "config/the_gpu_isa.hh"
#include "gpu-compute/hsa_code.hh"
#include "gpu-compute/hsa_kernel_info.hh"
#include "gpu-compute/misc.hh"

class BrigObject;
class GPUStaticInst;

inline int
popcount(uint64_t src, int sz)
{
    int cnt = 0;

    for (int i = 0; i < sz; ++i) {
        if (src & 1)
            ++cnt;
        src >>= 1;
    }

    return cnt;
}

inline int
firstbit(uint64_t src, int sz)
{
    int i;

    for (i = 0; i < sz; ++i) {
        if (src & 1)
            break;
        src >>= 1;
    }

    return i;
}

inline int
lastbit(uint64_t src, int sz)
{
    int i0 = -1;

    for (int i = 0; i < sz; ++i) {
        if (src & 1)
            i0 = i;
        src >>= 1;
    }

    return i0;
}

inline int
signbit(uint64_t src, int sz)
{
    int i0 = -1;

    if (src & (1 << (sz - 1))) {
        for (int i = 0; i < sz - 1; ++i) {
            if (!(src & 1))
                i0 = i;
            src >>= 1;
        }
    } else {
        for (int i = 0; i < sz - 1; ++i) {
            if (src & 1)
                i0 = i;
            src >>= 1;
        }
    }

    return i0;
}

inline uint64_t
bitrev(uint64_t src, int sz)
{
    uint64_t r = 0;

    for (int i = 0; i < sz; ++i) {
        r <<= 1;
        if (src & 1)
            r |= 1;
        src >>= 1;
    }

    return r;
}

inline uint64_t
mul_hi(uint32_t a, uint32_t b)
{
    return ((uint64_t)a * (uint64_t)b) >> 32;
}

inline uint64_t
mul_hi(int32_t a, int32_t b)
{
    return ((int64_t)a * (int64_t)b) >> 32;
}

inline uint64_t
mul_hi(uint64_t a, uint64_t b)
{
    return ((uint64_t)a * (uint64_t)b) >> 32;
}

inline uint64_t
mul_hi(int64_t a, int64_t b)
{
    return ((int64_t)a * (int64_t)b) >> 32;
}

inline uint64_t
mul_hi(double a, double b)
{
    return 0;
}

class Label
{
  public:
    std::string name;
    int value;

    Label() : value(-1)
    {
    }

    bool defined() { return value != -1; }

    void
    checkName(std::string &_name)
    {
        if (name.empty()) {
            name = _name;
        } else {
            assert(name == _name);
        }
    }

    void
    define(std::string &_name, int _value)
    {
        assert(!defined());
        assert(_value != -1);
        value = _value;
        checkName(_name);
    }

    int
    get()
    {
        assert(defined());
        return value;
    }
};

class LabelMap
{
    std::map<std::string, Label> map;

  public:
    LabelMap() { }

    void addLabel(const Brig::BrigDirectiveLabel *lbl, int inst_index,
                  const BrigObject *obj);

    Label *refLabel(const Brig::BrigDirectiveLabel *lbl,
                    const BrigObject *obj);
};

const int NumSegments = Brig::BRIG_SEGMENT_AMD_GCN;

extern const char *segmentNames[];

class StorageElement
{
  public:
    std::string name;
    uint64_t offset;

    uint64_t size;
    const Brig::BrigDirectiveVariable *brigSymbol;
    StorageElement(const char *_name, uint64_t _offset, int _size,
                   const Brig::BrigDirectiveVariable *sym)
        : name(_name), offset(_offset), size(_size), brigSymbol(sym)
    {
    }
};

class StorageSpace
{
    typedef std::map<const Brig::BrigDirectiveVariable*, StorageElement*>
            DirVarToSE_map;

    std::list<StorageElement*> elements;
    AddrRangeMap<StorageElement*> elements_by_addr;
    DirVarToSE_map elements_by_brigptr;

    uint64_t nextOffset;

  public:
    StorageSpace(Brig::BrigSegment _class) : nextOffset(0)
    {
    }

    StorageElement *addSymbol(const Brig::BrigDirectiveVariable *sym,
                              const BrigObject *obj);

    StorageElement* findSymbol(std::string name);
    StorageElement* findSymbol(uint64_t addr);
    StorageElement* findSymbol(const Brig::BrigDirectiveVariable *brigptr);

    int getSize() { return nextOffset; }
    void resetOffset() { nextOffset = 0; }
};

class StorageMap
{
    StorageMap *outerScopeMap;
    StorageSpace *space[NumSegments];

  public:
    StorageMap(StorageMap *outerScope = nullptr);

    StorageElement *addSymbol(const Brig::BrigDirectiveVariable *sym,
                              const BrigObject *obj);

    StorageElement* findSymbol(Brig::BrigSegment segment, std::string name);
    StorageElement* findSymbol(Brig::BrigSegment segment, uint64_t addr);

    StorageElement* findSymbol(Brig::BrigSegment segment,
                               const Brig::BrigDirectiveVariable *brigptr);

    // overloaded version to avoid casting
    StorageElement*
    findSymbol(Brig::BrigSegment8_t segment, std::string name)
    {
        return findSymbol((Brig::BrigSegment)segment, name);
    }

    int getSize(Brig::BrigSegment segment);
    void resetOffset(Brig::BrigSegment segment);
};

typedef enum
{
    BT_DEFAULT,
    BT_B8,
    BT_U8,
    BT_U16,
    BT_U32,
    BT_U64,
    BT_S8,
    BT_S16,
    BT_S32,
    BT_S64,
    BT_F16,
    BT_F32,
    BT_F64,
    BT_NULL
} base_type_e;

/* @class HsailCode
 * the HsailCode class is used to store information
 * about HSA kernels stored in the BRIG format. it holds
 * all information about a kernel, function, or variable
 * symbol and provides methods for accessing that
 * information.
 */

class HsailCode final : public HsaCode
{
  public:
    TheGpuISA::Decoder decoder;

    StorageMap *storageMap;
    LabelMap labelMap;
    uint32_t kernarg_start;
    uint32_t kernarg_end;
    int32_t private_size;

    int32_t readonly_size;

    // We track the maximum register index used for each register
    // class when we load the code so we can size the register files
    // appropriately (i.e., one more than the max index).
    uint32_t max_creg;    // maximum c-register index
    uint32_t max_sreg;    // maximum s-register index
    uint32_t max_dreg;    // maximum d-register index

    HsailCode(const std::string &name_str,
              const Brig::BrigDirectiveExecutable *code_dir,
              const BrigObject *obj,
              StorageMap *objStorageMap);

    // this version is used to create a placeholder when
    // we encounter a kernel-related directive before the
    // kernel itself
    HsailCode(const std::string &name_str);

    void init(const Brig::BrigDirectiveExecutable *code_dir,
              const BrigObject *obj, StorageMap *objStorageMap);

    void
    generateHsaKernelInfo(HsaKernelInfo *hsaKernelInfo) const
    {
        hsaKernelInfo->sRegCount = max_sreg + 1;
        hsaKernelInfo->dRegCount = max_dreg + 1;
        hsaKernelInfo->cRegCount = max_creg + 1;

        hsaKernelInfo->static_lds_size = getSize(Brig::BRIG_SEGMENT_GROUP);

        hsaKernelInfo->private_mem_size =
            roundUp(getSize(Brig::BRIG_SEGMENT_PRIVATE), 8);

        hsaKernelInfo->spill_mem_size =
            roundUp(getSize(Brig::BRIG_SEGMENT_SPILL), 8);
    }

    int
    getSize(MemorySegment segment) const
    {
        Brig::BrigSegment brigSeg;

        switch (segment) {
          case MemorySegment::NONE:
            brigSeg = Brig::BRIG_SEGMENT_NONE;
            break;
          case MemorySegment::FLAT:
            brigSeg = Brig::BRIG_SEGMENT_FLAT;
            break;
          case MemorySegment::GLOBAL:
            brigSeg = Brig::BRIG_SEGMENT_GLOBAL;
            break;
          case MemorySegment::READONLY:
            brigSeg = Brig::BRIG_SEGMENT_READONLY;
            break;
          case MemorySegment::KERNARG:
            brigSeg = Brig::BRIG_SEGMENT_KERNARG;
            break;
          case MemorySegment::GROUP:
            brigSeg = Brig::BRIG_SEGMENT_GROUP;
            break;
          case MemorySegment::PRIVATE:
            brigSeg = Brig::BRIG_SEGMENT_PRIVATE;
            break;
          case MemorySegment::SPILL:
            brigSeg = Brig::BRIG_SEGMENT_SPILL;
            break;
          case MemorySegment::ARG:
            brigSeg = Brig::BRIG_SEGMENT_ARG;
            break;
          case MemorySegment::EXTSPACE0:
            brigSeg = Brig::BRIG_SEGMENT_AMD_GCN;
            break;
          default:
            fatal("Unknown BrigSegment type.\n");
        }

        return getSize(brigSeg);
    }

  private:
    int
    getSize(Brig::BrigSegment segment) const
    {
        if (segment == Brig::BRIG_SEGMENT_PRIVATE) {
            // with the code generated by new HSA compiler the assertion
            // does not hold anymore..
            //assert(private_size != -1);
            return private_size;
        } else {
            return storageMap->getSize(segment);
        }
    }

  public:
    StorageElement*
    findSymbol(Brig::BrigSegment segment, uint64_t addr)
    {
        return storageMap->findSymbol(segment, addr);
    }

    void
    setPrivateSize(int32_t _private_size)
    {
        private_size = _private_size;
    }

    Label*
    refLabel(const Brig::BrigDirectiveLabel *lbl, const BrigObject *obj)
    {
        return labelMap.refLabel(lbl, obj);
    }
};

#endif // __HSAIL_CODE_HH__
