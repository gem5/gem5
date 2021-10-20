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

#ifndef __DEV_AMDGPU_AMDGPU_DEVICE_HH__
#define __DEV_AMDGPU_AMDGPU_DEVICE_HH__

#include <map>

#include "base/bitunion.hh"
#include "dev/amdgpu/amdgpu_defines.hh"
#include "dev/amdgpu/mmio_reader.hh"
#include "dev/io_device.hh"
#include "dev/pci/device.hh"
#include "params/AMDGPUDevice.hh"

namespace gem5
{

class AMDGPUInterruptHandler;

/**
 * Device model for an AMD GPU. This models the interface between the PCI bus
 * and the various IP blocks behind it. It translates requests to the various
 * BARs and sends them to the appropriate IP block. BAR0 requests are VRAM
 * requests that go to device memory, BAR2 are doorbells which are decoded and
 * sent to the corresponding IP block. BAR5 is the MMIO interface which writes
 * data values to registers controlling the IP blocks.
 */
class AMDGPUDevice : public PciDevice
{
  private:
    /**
     * Convert a PCI packet into a response
     */
    void dispatchAccess(PacketPtr pkt, bool read);

    /**
     * Helper methods to handle specific BAR read/writes. Offset is the
     * address of the packet - base address of the BAR.
     *
     * read/writeFrame are used for BAR0 requests
     * read/writeDoorbell are used for BAR2 requests
     * read/writeMMIO are used for BAR5 requests
     */
    void readFrame(PacketPtr pkt, Addr offset);
    void readDoorbell(PacketPtr pkt, Addr offset);
    void readMMIO(PacketPtr pkt, Addr offset);

    void writeFrame(PacketPtr pkt, Addr offset);
    void writeDoorbell(PacketPtr pkt, Addr offset);
    void writeMMIO(PacketPtr pkt, Addr offset);

    /**
     * Structures to hold registers, doorbells, and some frame memory
     */
    using GPURegMap = std::unordered_map<uint32_t, uint64_t>;
    GPURegMap frame_regs;
    GPURegMap regs;
    std::unordered_map<uint32_t, QueueType> doorbells;

    /**
     * VGA ROM methods
     */
    AddrRange romRange;
    bool isROM(Addr addr) const { return romRange.contains(addr); }
    void readROM(PacketPtr pkt);

    std::array<uint8_t, ROM_SIZE> rom;

    /**
     * MMIO reader to populate device registers map.
     */
    AMDMMIOReader mmioReader;

    /**
     * Blocks of the GPU
     */
    AMDGPUInterruptHandler *deviceIH;

    /**
     * Initial checkpoint support variables.
     */
    bool checkpoint_before_mmios;
    int init_interrupt_count;

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
    } VMContext; // VM Context

    typedef struct SysVMContext : VMContext
    {
        Addr agpBase;
        Addr agpTop;
        Addr agpBot;
        Addr fbBase;
        Addr fbTop;
        Addr fbOffset;
        Addr sysAddrL;
        Addr sysAddrH;
    } SysVMContext; // System VM Context

    SysVMContext vmContext0;
    std::vector<VMContext> vmContexts;

  public:
    AMDGPUDevice(const AMDGPUDeviceParams &p);

    /**
     * Methods inherited from PciDevice
     */
    void intrPost();

    Tick writeConfig(PacketPtr pkt) override;
    Tick readConfig(PacketPtr pkt) override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    AddrRangeList getAddrRanges() const override;

    /**
     * Checkpoint support
     */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Methods related to translations and system/device memory.
     */
    RequestorID vramRequestorId() { return 0; }

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

    Addr
    getMmioAperture(Addr addr)
    {
        // Aperture ranges:
        // NBIO               0x0     - 0x4280
        // IH                 0x4280  - 0x4980
        // SDMA0              0x4980  - 0x5180
        // SDMA1              0x5180  - 0x5980
        // GRBM               0x8000  - 0xD000
        // GFX                0x28000 - 0x3F000
        // MMHUB              0x68000 - 0x6a120

        if (IH_BASE <= addr && addr < IH_BASE + IH_SIZE)
            return IH_BASE;
        else if (SDMA0_BASE <= addr && addr < SDMA0_BASE + SDMA_SIZE)
            return SDMA0_BASE;
        else if (SDMA1_BASE <= addr && addr < SDMA1_BASE + SDMA_SIZE)
            return SDMA1_BASE;
        else if (GRBM_BASE <= addr && addr < GRBM_BASE + GRBM_SIZE)
            return GRBM_BASE;
        else if (GFX_BASE <= addr && addr < GFX_BASE + GFX_SIZE)
            return GFX_BASE;
        else if (MMHUB_BASE <= addr && addr < MMHUB_BASE + MMHUB_SIZE)
            return MMHUB_BASE;
        else {
            warn_once("Accessing unsupported MMIO aperture! Assuming NBIO\n");
            return NBIO_BASE;
        }
    }

    /**
     * Setters to set values from other GPU blocks.
     */
    void setDoorbellType(uint32_t offset, QueueType qt);
};

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_DEVICE_HH__
