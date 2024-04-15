/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#ifndef __DEV_AMDGPU_AMDGPU_VM_HH__
#define __DEV_AMDGPU_AMDGPU_VM_HH__

#include <vector>

#include "arch/amdgpu/vega/pagetable_walker.hh"
#include "base/intmath.hh"
#include "dev/amdgpu/amdgpu_defines.hh"
#include "mem/packet.hh"
#include "mem/translation_gen.hh"
#include "sim/serialize.hh"

/**
 * MMIO offsets for graphics register bus manager (GRBM). These values were
 * taken from linux header files. The header files can be found here:
 *
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *      drivers/gpu/drm/amd/include/ asic_reg/gc/gc_9_0_offset.h
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *      drivers/gpu/drm/amd/include/ asic_reg/mmhub/mmhub_1_0_offset.h
 */

#define mmVM_INVALIDATE_ENG17_ACK 0x08c6
#define mmVM_CONTEXT0_PAGE_TABLE_BASE_ADDR_LO32 0x08eb
#define mmVM_CONTEXT0_PAGE_TABLE_BASE_ADDR_HI32 0x08ec
#define mmVM_CONTEXT0_PAGE_TABLE_START_ADDR_LO32 0x090b
#define mmVM_CONTEXT0_PAGE_TABLE_START_ADDR_HI32 0x090c
#define mmVM_CONTEXT0_PAGE_TABLE_END_ADDR_LO32 0x092b
#define mmVM_CONTEXT0_PAGE_TABLE_END_ADDR_HI32 0x092c

#define mmMC_VM_FB_OFFSET 0x096b
#define mmMC_VM_FB_LOCATION_BASE 0x0980
#define mmMC_VM_FB_LOCATION_TOP 0x0981
#define mmMC_VM_AGP_TOP 0x0982
#define mmMC_VM_AGP_BOT 0x0983
#define mmMC_VM_AGP_BASE 0x0984
#define mmMC_VM_SYSTEM_APERTURE_LOW_ADDR 0x0985
#define mmMC_VM_SYSTEM_APERTURE_HIGH_ADDR 0x0986

#define mmMMHUB_VM_INVALIDATE_ENG17_SEM 0x06e2
#define mmMMHUB_VM_INVALIDATE_ENG17_REQ 0x06f4
#define mmMMHUB_VM_INVALIDATE_ENG17_ACK 0x0706
#define mmMMHUB_VM_FB_LOCATION_BASE 0x082c
#define mmMMHUB_VM_FB_LOCATION_TOP 0x082d

#define VEGA10_FB_LOCATION_BASE 0x6a0b0
#define VEGA10_FB_LOCATION_TOP 0x6a0b4

#define MI100_MEM_SIZE_REG 0x0378c
#define MI100_FB_LOCATION_BASE 0x6ac00
#define MI100_FB_LOCATION_TOP 0x6ac04

#define MI200_MEM_SIZE_REG 0x0378c
#define MI200_FB_LOCATION_BASE 0x6b300
#define MI200_FB_LOCATION_TOP 0x6b304

// AMD GPUs support 16 different virtual address spaces
static constexpr int AMDGPU_VM_COUNT = 16;

// These apertures have a fixed page size
static constexpr int AMDGPU_AGP_PAGE_SIZE = 4096;
static constexpr int AMDGPU_GART_PAGE_SIZE = 4096;
static constexpr int AMDGPU_MMHUB_PAGE_SIZE = 4096;

// Vega page size can be any power of 2 between 4kB and 1GB.
static constexpr int AMDGPU_USER_PAGE_SIZE = 4096;

namespace gem5
{

typedef enum :
    int
{
    NBIO_MMIO_RANGE,
    MMHUB_MMIO_RANGE,
    GFX_MMIO_RANGE,
    GRBM_MMIO_RANGE,
    IH_MMIO_RANGE,
    NUM_MMIO_RANGES
} mmio_range_t;

class AMDGPUDevice;

class AMDGPUVM : public Serializable
{
  private:
    AMDGPUDevice *gpuDevice;

    typedef struct GEM5_PACKED
    {
        // Page table addresses: from (Base + Start) to (End)
        union
        {
            struct
            {
                uint32_t ptBaseL;
                uint32_t ptBaseH;
            };

            Addr ptBase;
        };

        union
        {
            struct
            {
                uint32_t ptStartL;
                uint32_t ptStartH;
            };

            Addr ptStart;
        };

        union
        {
            struct
            {
                uint32_t ptEndL;
                uint32_t ptEndH;
            };

            Addr ptEnd;
        };
    } AMDGPUVMContext;

    typedef struct AMDGPUSysVMContext : AMDGPUVMContext
    {
        Addr agpBase;
        Addr agpTop;
        Addr agpBot;
        Addr fbBase;
        Addr fbTop;
        Addr fbOffset;
        Addr sysAddrL;
        Addr sysAddrH;
    } AMDGPUSysVMContext;

    AMDGPUSysVMContext vmContext0;
    std::vector<AMDGPUVMContext> vmContexts;

    // MMHUB aperture. These addresses mirror the framebuffer, so addresses
    // can be calculated by subtracting the base address.
    uint64_t mmhubBase = 0x0;
    uint64_t mmhubTop = 0x0;

    /**
     * List of TLBs associated with the GPU device. This is used for flushing
     * the TLBs upon a driver request.
     */
    std::vector<VegaISA::GpuTLB *> gpu_tlbs;

    std::array<AddrRange, NUM_MMIO_RANGES> mmioRanges;

  public:
    AMDGPUVM();

    void
    setGPUDevice(AMDGPUDevice *gpu_device)
    {
        gpuDevice = gpu_device;
    }

    /**
     * Return base address of GART table in framebuffer.
     */
    Addr gartBase();
    /**
     * Return size of GART in number of PTEs.
     */
    Addr gartSize();

    bool
    inGARTRange(Addr paddr)
    {
        return ((paddr >= gartBase()) && (paddr <= (gartBase() + gartSize())));
    }

    /**
     * Copy of GART table. Typically resides in device memory, however we use
     * a copy in gem5 to simplify the interface.
     */
    std::unordered_map<uint64_t, uint64_t> gartTable;

    void readMMIO(PacketPtr pkt, Addr offset);
    void writeMMIO(PacketPtr pkt, Addr offset);

    /**
     * Methods for resolving apertures
     */
    bool
    inAGP(Addr vaddr)
    {
        return ((vaddr >= vmContext0.agpBot) && (vaddr <= vmContext0.agpTop));
    }

    Addr
    getAGPBot()
    {
        return vmContext0.agpBot;
    }

    Addr
    getAGPTop()
    {
        return vmContext0.agpTop;
    }

    Addr
    getAGPBase()
    {
        return vmContext0.agpBase;
    }

    bool
    inMMHUB(Addr vaddr)
    {
        return ((vaddr >= getMMHUBBase()) && (vaddr <= getMMHUBTop()));
    }

    Addr
    getMMHUBBase()
    {
        return mmhubBase;
    }

    Addr
    getMMHUBTop()
    {
        return mmhubTop;
    }

    void
    setMMHUBBase(Addr base)
    {
        mmhubBase = base;
    }

    void
    setMMHUBTop(Addr top)
    {
        mmhubTop = top;
    }

    bool
    inFB(Addr vaddr)
    {
        return ((vaddr >= vmContext0.fbBase) && (vaddr <= vmContext0.fbTop));
    }

    Addr
    getFBBase()
    {
        return vmContext0.fbBase;
    }

    Addr
    getFBTop()
    {
        return vmContext0.fbTop;
    }

    Addr
    getFBOffset()
    {
        return vmContext0.fbOffset;
    }

    bool
    inSys(Addr vaddr)
    {
        return ((vaddr >= vmContext0.sysAddrL) &&
                (vaddr <= vmContext0.sysAddrH));
    }

    Addr
    getSysAddrRangeLow()
    {
        return vmContext0.sysAddrL;
    }

    Addr
    getSysAddrRangeHigh()
    {
        return vmContext0.sysAddrH;
    }

    void setMMIOAperture(mmio_range_t mmio_aperture, AddrRange range);
    const AddrRange &getMMIOAperture(Addr addr);
    AddrRange getMMIORange(mmio_range_t mmio_aperture);

    // Getting mapped aperture base addresses
    Addr
    getFrameAperture(Addr addr)
    {
        if (addr < gartBase()) {
            warn_once("Accessing unsupported frame apperture!\n");
            return ~0;
        } else if (gartBase() <= addr && addr < (gartBase() + gartSize())) {
            return gartBase();
        } else {
            warn_once("Accessing unsupported frame apperture!\n");
            return ~0;
        }
    }

    /**
     * Page table base/start accessors for user VMIDs.
     */
    void
    setPageTableBase(uint16_t vmid, Addr ptBase)
    {
        vmContexts[vmid].ptBase = ptBase;
    }

    Addr
    getPageTableBase(uint16_t vmid)
    {
        assert(vmid > 0 && vmid < vmContexts.size());
        return vmContexts[vmid].ptBase;
    }

    Addr
    getPageTableStart(uint16_t vmid)
    {
        assert(vmid > 0 && vmid < vmContexts.size());
        return vmContexts[vmid].ptStart;
    }

    /**
     * Control methods for TLBs associated with the GPU device.
     */
    void registerTLB(VegaISA::GpuTLB *tlb);
    void invalidateTLBs();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Translation range generators
     *
     * AGP - Legacy interface to device memory. Addr range is set via MMIO
     * GART - Legacy privledged translation table. Table in device memory
     * MMHUB - Shadow range of VRAM
     */
    class AGPTranslationGen : public TranslationGen
    {
      private:
        AMDGPUVM *vm;

        void translate(Range &range) const override;

      public:
        AGPTranslationGen(AMDGPUVM *_vm, Addr vaddr, Addr size)
            : TranslationGen(vaddr, size), vm(_vm)
        {}
    };

    class GARTTranslationGen : public TranslationGen
    {
      private:
        AMDGPUVM *vm;

        void translate(Range &range) const override;

      public:
        GARTTranslationGen(AMDGPUVM *_vm, Addr vaddr, Addr size)
            : TranslationGen(vaddr, size), vm(_vm)
        {}
    };

    class MMHUBTranslationGen : public TranslationGen
    {
      private:
        AMDGPUVM *vm;

        void translate(Range &range) const override;

      public:
        MMHUBTranslationGen(AMDGPUVM *_vm, Addr vaddr, Addr size)
            : TranslationGen(vaddr, size), vm(_vm)
        {}
    };

    class UserTranslationGen : public TranslationGen
    {
      private:
        AMDGPUVM *vm;
        VegaISA::Walker *walker;
        int vmid;

        void translate(Range &range) const override;

      public:
        UserTranslationGen(AMDGPUVM *_vm, VegaISA::Walker *_walker, int _vmid,
                           Addr vaddr, Addr size)
            : TranslationGen(vaddr, size),
              vm(_vm),
              walker(_walker),
              vmid(_vmid)
        {}
    };
};

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_VM_HH__
