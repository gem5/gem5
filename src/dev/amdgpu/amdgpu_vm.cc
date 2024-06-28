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

#include "dev/amdgpu/amdgpu_vm.hh"

#include "arch/amdgpu/vega/pagetable_walker.hh"
#include "arch/amdgpu/vega/tlb.hh"
#include "arch/generic/mmu.hh"
#include "base/trace.hh"
#include "debug/AMDGPUDevice.hh"
#include "dev/amdgpu/amdgpu_defines.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "mem/packet_access.hh"

namespace gem5
{

AMDGPUVM::AMDGPUVM()
{
    // Zero out contexts
    memset(&vmContext0, 0, sizeof(AMDGPUSysVMContext));

    vmContexts.resize(AMDGPU_VM_COUNT);
    for (int i = 0; i < AMDGPU_VM_COUNT; ++i) {
        memset(&vmContexts[0], 0, sizeof(AMDGPUVMContext));
    }

    for (int i = 0; i < NUM_MMIO_RANGES; ++i) {
        mmioRanges[i] = AddrRange();
    }
}

void
AMDGPUVM::setMMIOAperture(mmio_range_t mmio_aperture, AddrRange range)
{
    mmioRanges[mmio_aperture] = range;
}

AddrRange
AMDGPUVM::getMMIORange(mmio_range_t mmio_aperture)
{
    return mmioRanges[mmio_aperture];
}

const AddrRange&
AMDGPUVM::getMMIOAperture(Addr offset)
{
    for (int i = 0; i < NUM_MMIO_RANGES; ++i) {
        if (mmioRanges[i].contains(offset)) {
            return mmioRanges[i];
        }
    }

    // Default to NBIO
    return mmioRanges[NBIO_MMIO_RANGE];
}

Addr
AMDGPUVM::gartBase()
{
    return vmContext0.ptBase;
}

Addr
AMDGPUVM::gartSize()
{
    return vmContext0.ptEnd - vmContext0.ptStart;
}

void
AMDGPUVM::readMMIO(PacketPtr pkt, Addr offset)
{
    uint32_t value = pkt->getLE<uint32_t>();

    switch (offset) {
      // MMHUB MMIOs
      case mmMMHUB_VM_INVALIDATE_ENG17_SEM:
        DPRINTF(AMDGPUDevice, "Marking invalidate ENG17 SEM acquired\n");
        pkt->setLE<uint32_t>(1);
        break;
      case mmMMHUB_VM_INVALIDATE_ENG17_ACK:
        // This is only used by driver initialization and only expects an ACK
        // for VMID 0 which is the first bit in the response.
        DPRINTF(AMDGPUDevice, "Telling driver invalidate ENG17 is complete\n");
        pkt->setLE<uint32_t>(1);
        break;
      case mmMMHUB_VM_FB_LOCATION_BASE:
        mmhubBase = ((Addr)bits(value, 23, 0) << 24);
        DPRINTF(AMDGPUDevice, "MMHUB FB base set to %#x\n", mmhubBase);
        break;
      case mmMMHUB_VM_FB_LOCATION_TOP:
        mmhubTop = ((Addr)bits(value, 23, 0) << 24) | 0xFFFFFFULL;
        DPRINTF(AMDGPUDevice, "MMHUB FB top set to %#x\n", mmhubTop);
        break;
      // GRBM MMIOs
      case mmVM_INVALIDATE_ENG17_ACK:
        DPRINTF(AMDGPUDevice, "Overwritting invalidation ENG17 ACK\n");
        pkt->setLE<uint32_t>(1);
        break;
      default:
        DPRINTF(AMDGPUDevice, "GPUVM read of unknown MMIO %#x\n", offset);
        break;
    }
}

void
AMDGPUVM::writeMMIO(PacketPtr pkt, Addr offset)
{
    switch (offset) {
      // VMID0 MMIOs
      case mmVM_CONTEXT0_PAGE_TABLE_BASE_ADDR_LO32:
        vmContext0.ptBaseL = pkt->getLE<uint32_t>();
        // Clear extra bits not part of address
        vmContext0.ptBaseL = insertBits(vmContext0.ptBaseL, 0, 0, 0);
        break;
      case mmVM_CONTEXT0_PAGE_TABLE_BASE_ADDR_HI32:
        vmContext0.ptBaseH = pkt->getLE<uint32_t>();
        break;
      case mmVM_CONTEXT0_PAGE_TABLE_START_ADDR_LO32:
        vmContext0.ptStartL = pkt->getLE<uint32_t>();
        break;
      case mmVM_CONTEXT0_PAGE_TABLE_START_ADDR_HI32:
        vmContext0.ptStartH = pkt->getLE<uint32_t>();
        break;
      case mmVM_CONTEXT0_PAGE_TABLE_END_ADDR_LO32:
        vmContext0.ptEndL = pkt->getLE<uint32_t>();
        break;
      case mmVM_CONTEXT0_PAGE_TABLE_END_ADDR_HI32:
        vmContext0.ptEndH = pkt->getLE<uint32_t>();
        break;
      case mmMC_VM_AGP_TOP: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.agpTop = (((Addr)bits(val, 23, 0)) << 24) | 0xffffff;
        } break;
      case mmMC_VM_AGP_BOT: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.agpBot = ((Addr)bits(val, 23, 0)) << 24;
        } break;
      case mmMC_VM_AGP_BASE: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.agpBase = ((Addr)bits(val, 23, 0)) << 24;
        } break;
      case mmMC_VM_FB_LOCATION_TOP: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.fbTop = (((Addr)bits(val, 23, 0)) << 24) | 0xffffff;
        } break;
      case mmMC_VM_FB_LOCATION_BASE: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.fbBase = ((Addr)bits(val, 23, 0)) << 24;
        } break;
      case mmMC_VM_FB_OFFSET: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.fbOffset = ((Addr)bits(val, 23, 0)) << 24;
        } break;
      case mmMC_VM_SYSTEM_APERTURE_LOW_ADDR: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.sysAddrL = ((Addr)bits(val, 29, 0)) << 18;
        } break;
      case mmMC_VM_SYSTEM_APERTURE_HIGH_ADDR: {
        uint32_t val = pkt->getLE<uint32_t>();
        vmContext0.sysAddrH = ((Addr)bits(val, 29, 0)) << 18;
        } break;
      default:
        break;
    }
}

void
AMDGPUVM::registerTLB(VegaISA::GpuTLB *tlb)
{
    DPRINTF(AMDGPUDevice, "Registered a TLB with device\n");
    gpu_tlbs.push_back(tlb);
}

void
AMDGPUVM::invalidateTLBs()
{
    DPRINTF(AMDGPUDevice, "Invalidating all TLBs\n");
    for (auto &tlb : gpu_tlbs) {
        tlb->invalidateAll();
        DPRINTF(AMDGPUDevice, " ... TLB invalidated\n");
    }
}

void
AMDGPUVM::serialize(CheckpointOut &cp) const
{
    Addr vm0PTBase = vmContext0.ptBase;
    Addr vm0PTStart = vmContext0.ptStart;
    Addr vm0PTEnd = vmContext0.ptEnd;
    uint64_t gartTableSize;
    SERIALIZE_SCALAR(vm0PTBase);
    SERIALIZE_SCALAR(vm0PTStart);
    SERIALIZE_SCALAR(vm0PTEnd);

    SERIALIZE_SCALAR(vmContext0.agpBase);
    SERIALIZE_SCALAR(vmContext0.agpTop);
    SERIALIZE_SCALAR(vmContext0.agpBot);
    SERIALIZE_SCALAR(vmContext0.fbBase);
    SERIALIZE_SCALAR(vmContext0.fbTop);
    SERIALIZE_SCALAR(vmContext0.fbOffset);
    SERIALIZE_SCALAR(vmContext0.sysAddrL);
    SERIALIZE_SCALAR(vmContext0.sysAddrH);

    SERIALIZE_SCALAR(mmhubBase);
    SERIALIZE_SCALAR(mmhubTop);

    Addr ptBase[AMDGPU_VM_COUNT];
    Addr ptStart[AMDGPU_VM_COUNT];
    Addr ptEnd[AMDGPU_VM_COUNT];
    for (int i = 0; i < AMDGPU_VM_COUNT; i++) {
        ptBase[i] = vmContexts[i].ptBase;
        ptStart[i] = vmContexts[i].ptStart;
        ptEnd[i] = vmContexts[i].ptEnd;
    }
    SERIALIZE_ARRAY(ptBase, AMDGPU_VM_COUNT);
    SERIALIZE_ARRAY(ptStart, AMDGPU_VM_COUNT);
    SERIALIZE_ARRAY(ptEnd, AMDGPU_VM_COUNT);

    gartTableSize = gartTable.size();
    uint64_t* gartTableKey = new uint64_t[gartTableSize];
    uint64_t* gartTableValue = new uint64_t[gartTableSize];
    SERIALIZE_SCALAR(gartTableSize);
    int i = 0;
    for (auto it = gartTable.begin(); it != gartTable.end(); ++it) {
        gartTableKey[i] = it->first;
        gartTableValue[i] = it->second;
        i++;
    }
    SERIALIZE_ARRAY(gartTableKey, gartTableSize);
    SERIALIZE_ARRAY(gartTableValue, gartTableSize);
    delete[] gartTableKey;
    delete[] gartTableValue;
}

void
AMDGPUVM::unserialize(CheckpointIn &cp)
{
    // Unserialize requires fields not be packed
    Addr vm0PTBase;
    Addr vm0PTStart;
    Addr vm0PTEnd;
    uint64_t gartTableSize, *gartTableKey, *gartTableValue;
    UNSERIALIZE_SCALAR(vm0PTBase);
    UNSERIALIZE_SCALAR(vm0PTStart);
    UNSERIALIZE_SCALAR(vm0PTEnd);
    vmContext0.ptBase = vm0PTBase;
    vmContext0.ptStart = vm0PTStart;
    vmContext0.ptEnd = vm0PTEnd;

    UNSERIALIZE_SCALAR(vmContext0.agpBase);
    UNSERIALIZE_SCALAR(vmContext0.agpTop);
    UNSERIALIZE_SCALAR(vmContext0.agpBot);
    UNSERIALIZE_SCALAR(vmContext0.fbBase);
    UNSERIALIZE_SCALAR(vmContext0.fbTop);
    UNSERIALIZE_SCALAR(vmContext0.fbOffset);
    UNSERIALIZE_SCALAR(vmContext0.sysAddrL);
    UNSERIALIZE_SCALAR(vmContext0.sysAddrH);

    UNSERIALIZE_SCALAR(mmhubBase);
    UNSERIALIZE_SCALAR(mmhubTop);

    Addr ptBase[AMDGPU_VM_COUNT];
    Addr ptStart[AMDGPU_VM_COUNT];
    Addr ptEnd[AMDGPU_VM_COUNT];
    UNSERIALIZE_ARRAY(ptBase, AMDGPU_VM_COUNT);
    UNSERIALIZE_ARRAY(ptStart, AMDGPU_VM_COUNT);
    UNSERIALIZE_ARRAY(ptEnd, AMDGPU_VM_COUNT);
    for (int i = 0; i < AMDGPU_VM_COUNT; i++) {
        vmContexts[i].ptBase = ptBase[i];
        vmContexts[i].ptStart = ptStart[i];
        vmContexts[i].ptEnd = ptEnd[i];
    }
    UNSERIALIZE_SCALAR(gartTableSize);
    gartTableKey = new uint64_t[gartTableSize];
    gartTableValue = new uint64_t[gartTableSize];
    UNSERIALIZE_ARRAY(gartTableKey, gartTableSize);
    UNSERIALIZE_ARRAY(gartTableValue, gartTableSize);
    for (uint64_t i = 0; i < gartTableSize; i++) {
        gartTable[gartTableKey[i]] = gartTableValue[i];
    }
    delete[] gartTableKey;
    delete[] gartTableValue;
}

void
AMDGPUVM::AGPTranslationGen::translate(Range &range) const
{
    assert(vm->inAGP(range.vaddr));

    Addr next = roundUp(range.vaddr, AMDGPU_AGP_PAGE_SIZE);
    if (next == range.vaddr)
        next += AMDGPU_AGP_PAGE_SIZE;

    range.size = std::min(range.size, next - range.vaddr);
    range.paddr = range.vaddr - vm->getAGPBot() + vm->getAGPBase();

    DPRINTF(AMDGPUDevice, "AMDGPUVM: AGP translation %#lx -> %#lx\n",
            range.vaddr, range.paddr);
}

void
AMDGPUVM::GARTTranslationGen::translate(Range &range) const
{
    Addr next = roundUp(range.vaddr, AMDGPU_GART_PAGE_SIZE);
    if (next == range.vaddr)
        next += AMDGPU_GART_PAGE_SIZE;
    range.size = std::min(range.size, next - range.vaddr);

    Addr gart_addr = bits(range.vaddr, 63, 12);

    // This table is a bit hard to iterate over. If we cross a page, the next
    // PTE is not necessarily the next entry but actually 7 entries away.
    Addr lsb = bits(gart_addr, 2, 0);
    gart_addr += lsb * 7;

    // GART is a single level translation, so the value at the "virtual" addr
    // is the PTE containing the physical address.
    auto result = vm->gartTable.find(gart_addr);
    if (result == vm->gartTable.end()) {
        // There is no reason to fault as there is no recovery mechanism for
        // invalid GART entries. Simply panic in this case
        warn("GART translation for %p not found", range.vaddr);

        // Some PM4 packets have register addresses which we ignore. In that
        // case just return the vaddr rather than faulting.
        range.paddr = range.vaddr;
    } else {
        Addr pte = result->second;
        Addr lower_bits = bits(range.vaddr, 11, 0);
        range.paddr = (bits(pte, 47, 12) << 12) | lower_bits;
    }

    DPRINTF(AMDGPUDevice, "AMDGPUVM: GART translation %#lx -> %#lx\n",
            range.vaddr, range.paddr);
}

void
AMDGPUVM::MMHUBTranslationGen::translate(Range &range) const
{
    assert(vm->inMMHUB(range.vaddr));

    Addr next = roundUp(range.vaddr, AMDGPU_MMHUB_PAGE_SIZE);
    if (next == range.vaddr)
        next += AMDGPU_MMHUB_PAGE_SIZE;

    range.size = std::min(range.size, next - range.vaddr);
    range.paddr = range.vaddr - vm->getMMHUBBase();

    DPRINTF(AMDGPUDevice, "AMDGPUVM: MMHUB translation %#lx -> %#lx\n",
            range.vaddr, range.paddr);
}

void
AMDGPUVM::UserTranslationGen::translate(Range &range) const
{
    // Get base address of the page table for this vmid
    Addr base = vm->getPageTableBase(vmid);
    Addr start = vm->getPageTableStart(vmid);
    DPRINTF(AMDGPUDevice, "User tl base %#lx start %#lx walker %p\n",
            base, start, walker);

    bool system_bit;
    unsigned logBytes;
    Addr paddr = range.vaddr;
    Fault fault = walker->startFunctional(base, paddr, logBytes,
                                          BaseMMU::Mode::Read, system_bit);
    if (fault != NoFault) {
        fatal("User translation fault");
    }

    // GPU page size is variable. Use logBytes to determine size.
    const Addr page_size = 1 << logBytes;
    Addr next = roundUp(range.vaddr, page_size);
    if (next == range.vaddr) {
        // We don't know the size of the next page, use default.
        next += AMDGPU_USER_PAGE_SIZE;
    }

    // If we are not in system/host memory, change the address to the MMHUB
    // aperture. This is mapped to the same backing memory as device memory.
    if (!system_bit) {
        paddr += vm->getMMHUBBase();
        assert(vm->inMMHUB(paddr));
    }

    range.size = std::min(range.size, next - range.vaddr);
    range.paddr = paddr;
}

} // namespace gem5
