/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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
#include "dev/amdgpu/mmio_reader.hh"
#include "dev/io_device.hh"
#include "dev/pci/device.hh"
#include "params/AMDGPUDevice.hh"

namespace gem5
{

/* Names of BARs used by the device. */
constexpr int FRAMEBUFFER_BAR = 0;
constexpr int DOORBELL_BAR = 2;
constexpr int MMIO_BAR = 5;

/* By default the X86 kernel expects the vga ROM at 0xc0000. */
constexpr uint32_t VGA_ROM_DEFAULT = 0xc0000;
constexpr uint32_t ROM_SIZE = 0x20000;        // 128kB

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
     * Device registers - Maps register address to register value
     */
    std::unordered_map<uint32_t, uint64_t> regs;

    bool checkpoint_before_mmios;
    int init_interrupt_count;

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
};

} // namespace gem5

#endif // __DEV_AMDGPU_AMDGPU_DEVICE_HH__
