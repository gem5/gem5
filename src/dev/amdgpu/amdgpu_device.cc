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

#include "dev/amdgpu/amdgpu_device.hh"

#include <fstream>

#include "debug/AMDGPUDevice.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AMDGPUDevice.hh"
#include "sim/byteswap.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

AMDGPUDevice::AMDGPUDevice(const AMDGPUDeviceParams &p)
    : PciDevice(p), checkpoint_before_mmios(p.checkpoint_before_mmios),
      init_interrupt_count(0)
{
    // Loading the rom binary dumped from hardware.
    std::ifstream romBin;
    romBin.open(p.rom_binary, std::ios::binary);
    romBin.read((char *)rom.data(), ROM_SIZE);
    romBin.close();

    if (config.expansionROM) {
        romRange = RangeSize(config.expansionROM, ROM_SIZE);
    } else {
        romRange = RangeSize(VGA_ROM_DEFAULT, ROM_SIZE);
    }

    if (p.trace_file != "") {
        mmioReader.readMMIOTrace(p.trace_file);
    }
}

void
AMDGPUDevice::readROM(PacketPtr pkt)
{
    Addr rom_offset = pkt->getAddr() & (ROM_SIZE - 1);
    uint64_t rom_data = 0;

    memcpy(&rom_data, rom.data() + rom_offset, pkt->getSize());
    pkt->setUintX(rom_data, ByteOrder::little);

    DPRINTF(AMDGPUDevice, "Read from addr %#x on ROM offset %#x data: %#x\n",
            pkt->getAddr(), rom_offset, rom_data);
}

AddrRangeList
AMDGPUDevice::getAddrRanges() const
{
    AddrRangeList ranges = PciDevice::getAddrRanges();
    AddrRangeList ret_ranges;
    ret_ranges.push_back(romRange);

    // If the range starts at zero assume OS hasn't assigned it yet. Do not
    // return ranges starting with zero as they will surely overlap with
    // another range causing the I/O crossbar to fatal.
    for (auto & r : ranges) {
        if (r.start() != 0) {
            ret_ranges.push_back(r);
        }
    }

    return ret_ranges;
}

Tick
AMDGPUDevice::readConfig(PacketPtr pkt)
{
    [[maybe_unused]] int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    DPRINTF(AMDGPUDevice, "Read Config: from offset: %#x size: %#x "
            "data: %#x\n", offset, pkt->getSize(), config.data[offset]);

    Tick delay = PciDevice::readConfig(pkt);

    // Before sending MMIOs the driver sends three interrupts in a row.
    // Use this to trigger creating a checkpoint to restore in timing mode.
    // This is only necessary until we can create a "hole" in the KVM VM
    // around the VGA ROM region such that KVM exits and sends requests to
    // this device rather than the KVM VM.
    if (checkpoint_before_mmios) {
        if (offset == PCI0_INTERRUPT_PIN) {
            if (++init_interrupt_count == 3) {
                DPRINTF(AMDGPUDevice, "Checkpointing before first MMIO\n");
                exitSimLoop("checkpoint", 0, curTick() + delay + 1);
            }
        } else {
            init_interrupt_count = 0;
        }
    }

    return delay;
}

Tick
AMDGPUDevice::writeConfig(PacketPtr pkt)
{
    [[maybe_unused]] int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    DPRINTF(AMDGPUDevice, "Write Config: from offset: %#x size: %#x "
            "data: %#x\n", offset, pkt->getSize(),
            pkt->getUintX(ByteOrder::little));

    return PciDevice::writeConfig(pkt);
}

void
AMDGPUDevice::dispatchAccess(PacketPtr pkt, bool read)
{
    DPRINTF(AMDGPUDevice, "%s from addr %#x size: %#x data: %#x\n",
            read ? "Read" : "Write", pkt->getAddr(), pkt->getSize(),
            pkt->getUintX(ByteOrder::little));

    pkt->makeAtomicResponse();
}

void
AMDGPUDevice::readFrame(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Read framebuffer address %#lx\n", offset);

    mmioReader.readFromTrace(pkt, FRAMEBUFFER_BAR, offset);

    /* Handle special counter addresses in framebuffer. */
    if (offset == 0xa28000) {
        /* Counter addresses expect the read to return previous value + 1. */
        if (regs.find(pkt->getAddr()) == regs.end()) {
            regs[pkt->getAddr()] = 1;
        } else {
            regs[pkt->getAddr()]++;
        }

        pkt->setUintX(regs[pkt->getAddr()], ByteOrder::little);
    }
}

void
AMDGPUDevice::readDoorbell(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Read doorbell %#lx\n", offset);
    mmioReader.readFromTrace(pkt, DOORBELL_BAR, offset);
}

void
AMDGPUDevice::readMMIO(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Read MMIO %#lx\n", offset);
    mmioReader.readFromTrace(pkt, MMIO_BAR, offset);
}

void
AMDGPUDevice::writeFrame(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote framebuffer address %#lx\n", offset);
    mmioReader.writeFromTrace(pkt, FRAMEBUFFER_BAR, offset);
}

void
AMDGPUDevice::writeDoorbell(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote doorbell %#lx\n", offset);
    mmioReader.writeFromTrace(pkt, DOORBELL_BAR, offset);
}

void
AMDGPUDevice::writeMMIO(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote MMIO %#lx\n", offset);
    mmioReader.writeFromTrace(pkt, MMIO_BAR, offset);
}

Tick
AMDGPUDevice::read(PacketPtr pkt)
{
    if (isROM(pkt->getAddr())) {
        readROM(pkt);
    } else {
        int barnum = -1;
        Addr offset = 0;
        getBAR(pkt->getAddr(), barnum, offset);

        switch (barnum) {
          case FRAMEBUFFER_BAR:
              readFrame(pkt, offset);
              break;
          case DOORBELL_BAR:
              readDoorbell(pkt, offset);
              break;
          case MMIO_BAR:
              readMMIO(pkt, offset);
              break;
          default:
            panic("Request with address out of mapped range!");
        }
    }

    dispatchAccess(pkt, true);
    return pioDelay;
}

Tick
AMDGPUDevice::write(PacketPtr pkt)
{
    int barnum = -1;
    Addr offset = 0;
    getBAR(pkt->getAddr(), barnum, offset);

    switch (barnum) {
      case FRAMEBUFFER_BAR:
          writeFrame(pkt, offset);
          break;
      case DOORBELL_BAR:
          writeDoorbell(pkt, offset);
          break;
      case MMIO_BAR:
          writeMMIO(pkt, offset);
          break;
      default:
        panic("Request with address out of mapped range!");
    }

    // Record only if there is non-zero value, or a value to be overwritten.
    // Reads return 0 by default.
    uint64_t data = pkt->getUintX(ByteOrder::little);

    if (data || regs.find(pkt->getAddr()) != regs.end())
        regs[pkt->getAddr()] = data;

    dispatchAccess(pkt, false);

    return pioDelay;
}

void
AMDGPUDevice::serialize(CheckpointOut &cp) const
{
    // Serialize the PciDevice base class
    PciDevice::serialize(cp);
}

void
AMDGPUDevice::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    PciDevice::unserialize(cp);
}

} // namespace gem5
