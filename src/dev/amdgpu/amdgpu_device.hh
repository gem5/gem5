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
#include "dev/amdgpu/amdgpu_gfx.hh"
#include "dev/amdgpu/amdgpu_nbio.hh"
#include "dev/amdgpu/amdgpu_vm.hh"
#include "dev/amdgpu/memory_manager.hh"
#include "dev/amdgpu/mmio_reader.hh"
#include "dev/io_device.hh"
#include "dev/pci/device.hh"
#include "enums/GfxVersion.hh"
#include "params/AMDGPUDevice.hh"

namespace gem5
{

class AMDGPUInterruptHandler;
class SDMAEngine;

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
    GPURegMap regs;
    std::unordered_map<uint32_t, QueueType> doorbells;

    /**
     * VGA ROM methods
     */
    AddrRange romRange;
    bool isROM(Addr addr) const { return romRange.contains(addr); }
    void readROM(PacketPtr pkt);
    void writeROM(PacketPtr pkt);

    std::array<uint8_t, ROM_SIZE> rom;

    /**
     * MMIO reader to populate device registers map.
     */
    AMDMMIOReader mmioReader;

    /**
     * Blocks of the GPU
     */
    AMDGPUNbio nbio;
    AMDGPUGfx gfx;
    AMDGPUMemoryManager *gpuMemMgr;
    AMDGPUInterruptHandler *deviceIH;
    AMDGPUVM gpuvm;
    PM4PacketProcessor *pm4PktProc;
    GPUCommandProcessor *cp;

    // SDMAs mapped by doorbell offset
    std::unordered_map<uint32_t, SDMAEngine *> sdmaEngs;
    // SDMAs mapped by ID
    std::unordered_map<uint32_t, SDMAEngine *> sdmaIds;
    // SDMA ID to MMIO range
    std::unordered_map<uint32_t, AddrRange> sdmaMmios;
    // SDMA ID to function
    typedef void (SDMAEngine::*sdmaFuncPtr)(uint32_t);
    std::unordered_map<uint32_t, sdmaFuncPtr> sdmaFunc;

    /**
     * Initial checkpoint support variables.
     */
    bool checkpoint_before_mmios;
    int init_interrupt_count;

    // VMIDs data structures
    // map of pasids to vmids
    std::unordered_map<uint16_t, uint16_t> idMap;
    // map of doorbell offsets to vmids
    std::unordered_map<Addr, uint16_t> doorbellVMIDMap;
    // map of vmid to all queue ids using that vmid
    std::unordered_map<uint16_t, std::set<int>> usedVMIDs;
    // last vmid allocated by map_process PM4 packet
    uint16_t _lastVMID;

    /*
     * Backing store for GPU memory / framebuffer / VRAM
     */
    memory::PhysicalMemory deviceMem;

    /* Device information */
    GfxVersion gfx_version = GfxVersion::gfx900;

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
     * Get handles to GPU blocks.
     */
    AMDGPUInterruptHandler* getIH() { return deviceIH; }
    SDMAEngine* getSDMAById(int id);
    SDMAEngine* getSDMAEngine(Addr offset);
    AMDGPUVM &getVM() { return gpuvm; }
    AMDGPUMemoryManager* getMemMgr() { return gpuMemMgr; }
    GPUCommandProcessor* CP() { return cp; }

    /**
     * Set handles to GPU blocks.
     */
    void setDoorbellType(uint32_t offset, QueueType qt);
    void setSDMAEngine(Addr offset, SDMAEngine *eng);

    /**
     * Register value getter/setter. Used by other GPU blocks to change
     * values from incoming driver/user packets.
     */
    bool haveRegVal(uint32_t addr);
    uint32_t getRegVal(uint32_t addr);
    void setRegVal(uint32_t addr, uint32_t value);

    /**
     * Methods related to translations and system/device memory.
     */
    RequestorID vramRequestorId() { return gpuMemMgr->getRequestorID(); }

    /* HW context stuff */
    uint16_t lastVMID() { return _lastVMID; }
    uint16_t allocateVMID(uint16_t pasid);
    void deallocateVmid(uint16_t vmid);
    void deallocatePasid(uint16_t pasid);
    void deallocateAllQueues();
    void mapDoorbellToVMID(Addr doorbell, uint16_t vmid);
    uint16_t getVMID(Addr doorbell) { return doorbellVMIDMap[doorbell]; }
    std::unordered_map<uint16_t, std::set<int>>& getUsedVMIDs();
    void insertQId(uint16_t vmid, int id);

    /* Device information */
    GfxVersion getGfxVersion() const { return gfx_version; }
};

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_DEVICE_HH__
