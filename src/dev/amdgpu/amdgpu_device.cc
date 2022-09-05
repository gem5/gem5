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

#include "dev/amdgpu/amdgpu_device.hh"

#include <fstream>

#include "debug/AMDGPUDevice.hh"
#include "dev/amdgpu/amdgpu_vm.hh"
#include "dev/amdgpu/interrupt_handler.hh"
#include "dev/amdgpu/pm4_packet_processor.hh"
#include "dev/amdgpu/sdma_engine.hh"
#include "dev/hsa/hw_scheduler.hh"
#include "gpu-compute/gpu_command_processor.hh"
#include "gpu-compute/shader.hh"
#include "mem/abstract_mem.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AMDGPUDevice.hh"
#include "sim/byteswap.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

AMDGPUDevice::AMDGPUDevice(const AMDGPUDeviceParams &p)
    : PciDevice(p), gpuMemMgr(p.memory_manager), deviceIH(p.device_ih),
      sdma0(p.sdma0), sdma1(p.sdma1), pm4PktProc(p.pm4_pkt_proc), cp(p.cp),
      checkpoint_before_mmios(p.checkpoint_before_mmios),
      init_interrupt_count(0), _lastVMID(0),
      deviceMem(name() + ".deviceMem", p.memories, false, "", false)
{
    // Loading the rom binary dumped from hardware.
    std::ifstream romBin;
    romBin.open(p.rom_binary, std::ios::binary);
    romBin.read((char *)rom.data(), ROM_SIZE);
    romBin.close();

    // System pointer needs to be explicitly set for device memory since
    // DRAMCtrl uses it to get (1) cache line size and (2) the mem mode.
    // Note this means the cache line size is system wide.
    for (auto& m : p.memories) {
        m->system(p.system);

        // Add to system's device memory map.
        p.system->addDeviceMemory(gpuMemMgr->getRequestorID(), m);
    }

    if (config.expansionROM) {
        romRange = RangeSize(config.expansionROM, ROM_SIZE);
    } else {
        romRange = RangeSize(VGA_ROM_DEFAULT, ROM_SIZE);
    }

    if (p.trace_file != "") {
        mmioReader.readMMIOTrace(p.trace_file);
    }

    sdma0->setGPUDevice(this);
    sdma0->setId(0);
    sdma1->setGPUDevice(this);
    sdma1->setId(1);
    deviceIH->setGPUDevice(this);
    pm4PktProc->setGPUDevice(this);
    cp->hsaPacketProc().setGPUDevice(this);
    cp->setGPUDevice(this);
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

    /*
     * Return data for frame reads in priority order: (1) Special addresses
     * first, ignoring any writes from driver. (2) GART addresses written
     * to frame_regs in writeFrame. (3) Any other address from device backing
     * store / abstract memory class functionally.
     */
    if (offset == 0xa28000) {
        /*
         * Handle special counter addresses in framebuffer. These counter
         * addresses expect the read to return previous value + 1.
         */
        if (regs.find(pkt->getAddr()) == regs.end()) {
            regs[pkt->getAddr()] = 1;
        } else {
            regs[pkt->getAddr()]++;
        }

        pkt->setUintX(regs[pkt->getAddr()], ByteOrder::little);
    } else if (frame_regs.find(offset) != frame_regs.end()) {
        /* If the driver wrote something, use that value over the trace. */
        pkt->setUintX(frame_regs[offset], ByteOrder::little);
    } else {
        /*
         * Read the value from device memory. This must be done functionally
         * because this method is called by the PCIDevice::read method which
         * is a non-timing read.
         */
        RequestPtr req = std::make_shared<Request>(offset, pkt->getSize(), 0,
                                                   vramRequestorId());
        PacketPtr readPkt = Packet::createRead(req);
        uint8_t *dataPtr = new uint8_t[pkt->getSize()];
        readPkt->dataDynamic(dataPtr);

        auto system = cp->shader()->gpuCmdProc.system();
        system->getDeviceMemory(readPkt)->access(readPkt);

        pkt->setUintX(readPkt->getUintX(ByteOrder::little), ByteOrder::little);
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
    Addr aperture = gpuvm.getMmioAperture(offset);
    Addr aperture_offset = offset - aperture;

    // By default read from MMIO trace. Overwrite the packet for a select
    // few more dynamic MMIOs.
    DPRINTF(AMDGPUDevice, "Read MMIO %#lx\n", offset);
    mmioReader.readFromTrace(pkt, MMIO_BAR, offset);

    switch (aperture) {
      case NBIO_BASE:
        switch (aperture_offset) {
          // This is a PCIe status register. At some point during driver init
          // the driver checks that interrupts are enabled. This is only
          // checked once, so if the MMIO trace does not exactly line up with
          // what the driver is doing in gem5, this may still have the first
          // bit zero causing driver to fail. Therefore, we always set this
          // bit to one as there is no harm to do so.
          case 0x3c: // mmPCIE_DATA2 << 2
            uint32_t value = pkt->getLE<uint32_t>() | 0x1;
            DPRINTF(AMDGPUDevice, "Marking interrupts enabled: %#lx\n", value);
            pkt->setLE<uint32_t>(value);
            break;
        } break;
      case GRBM_BASE:
        gpuvm.readMMIO(pkt, aperture_offset >> GRBM_OFFSET_SHIFT);
        break;
      case MMHUB_BASE:
        gpuvm.readMMIO(pkt, aperture_offset >> MMHUB_OFFSET_SHIFT);
        break;
      default:
        break;
    }
}

void
AMDGPUDevice::writeFrame(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote framebuffer address %#lx\n", offset);

    Addr aperture = gpuvm.getFrameAperture(offset);
    Addr aperture_offset = offset - aperture;

    // Record the value
    frame_regs[offset] = pkt->getUintX(ByteOrder::little);
    if (aperture == gpuvm.gartBase()) {
        frame_regs[aperture_offset] = pkt->getLE<uint32_t>();
        DPRINTF(AMDGPUDevice, "GART translation %p -> %p\n", aperture_offset,
            bits(frame_regs[aperture_offset], 48, 12));
        gpuvm.gartTable[aperture_offset] = pkt->getLE<uint32_t>();
    }
}

void
AMDGPUDevice::writeDoorbell(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote doorbell %#lx\n", offset);

    if (doorbells.find(offset) != doorbells.end()) {
        QueueType q_type = doorbells[offset];
        DPRINTF(AMDGPUDevice, "Doorbell offset %p queue: %d\n",
                              offset, q_type);
        switch (q_type) {
          case Compute:
            pm4PktProc->process(pm4PktProc->getQueue(offset),
                                pkt->getLE<uint64_t>());
          break;
          case Gfx:
            pm4PktProc->process(pm4PktProc->getQueue(offset, true),
                                pkt->getLE<uint64_t>());
          break;
          case SDMAGfx: {
            SDMAEngine *sdmaEng = getSDMAEngine(offset);
            sdmaEng->processGfx(pkt->getLE<uint64_t>());
          } break;
          case SDMAPage: {
            SDMAEngine *sdmaEng = getSDMAEngine(offset);
            sdmaEng->processPage(pkt->getLE<uint64_t>());
          } break;
          case ComputeAQL: {
            cp->hsaPacketProc().hwScheduler()->write(offset,
                pkt->getLE<uint64_t>() + 1);
            pm4PktProc->updateReadIndex(offset, pkt->getLE<uint64_t>() + 1);
          } break;
          case InterruptHandler:
            deviceIH->updateRptr(pkt->getLE<uint32_t>());
            break;
          case RLC: {
            SDMAEngine *sdmaEng = getSDMAEngine(offset);
            sdmaEng->processRLC(offset, pkt->getLE<uint64_t>());
          } break;
          default:
            panic("Write to unkown queue type!");
        }
    } else {
        warn("Unknown doorbell offset: %lx\n", offset);
    }
}

void
AMDGPUDevice::writeMMIO(PacketPtr pkt, Addr offset)
{
    Addr aperture = gpuvm.getMmioAperture(offset);
    Addr aperture_offset = offset - aperture;

    DPRINTF(AMDGPUDevice, "Wrote MMIO %#lx\n", offset);

    switch (aperture) {
      /* Write a register to the first System DMA. */
      case SDMA0_BASE:
        sdma0->writeMMIO(pkt, aperture_offset >> SDMA_OFFSET_SHIFT);
        break;
      /* Write a register to the second System DMA. */
      case SDMA1_BASE:
        sdma1->writeMMIO(pkt, aperture_offset >> SDMA_OFFSET_SHIFT);
        break;
      /* Write a general register to the graphics register bus manager. */
      case GRBM_BASE:
        gpuvm.writeMMIO(pkt, aperture_offset >> GRBM_OFFSET_SHIFT);
        pm4PktProc->writeMMIO(pkt, aperture_offset >> GRBM_OFFSET_SHIFT);
        break;
      /* Write a register to the interrupt handler. */
      case IH_BASE:
        deviceIH->writeMMIO(pkt, aperture_offset >> IH_OFFSET_SHIFT);
        break;
      default:
        DPRINTF(AMDGPUDevice, "Unknown MMIO aperture for %#x\n", offset);
        break;
    }
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
          gpuMemMgr->writeRequest(offset, pkt->getPtr<uint8_t>(),
                                  pkt->getSize(), 0, nullptr);
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

    DPRINTF(AMDGPUDevice, "PCI Write to %#lx data %#lx\n",
                            pkt->getAddr(), data);

    if (data || regs.find(pkt->getAddr()) != regs.end())
        regs[pkt->getAddr()] = data;

    dispatchAccess(pkt, false);

    return pioDelay;
}

uint32_t
AMDGPUDevice::getRegVal(uint32_t addr)
{
    return regs[addr];
}
void
AMDGPUDevice::setRegVal(uint32_t addr, uint32_t value)
{
    DPRINTF(AMDGPUDevice, "Setting register 0x%lx to %x\n",
            addr, value);
    regs[addr] = value;
}

void
AMDGPUDevice::setDoorbellType(uint32_t offset, QueueType qt)
{
    DPRINTF(AMDGPUDevice, "Setting doorbell type for %x\n", offset);
    doorbells[offset] = qt;
}

void
AMDGPUDevice::setSDMAEngine(Addr offset, SDMAEngine *eng)
{
    sdmaEngs[offset] = eng;
}

SDMAEngine*
AMDGPUDevice::getSDMAById(int id)
{
    /**
     * PM4 packets selected SDMAs using an integer ID. This method simply maps
     * the integer ID to a pointer to the SDMA and checks for invalid IDs.
     */
    switch (id) {
        case 0:
            return sdma0;
            break;
        case 1:
            return sdma1;
            break;
        default:
            panic("No SDMA with id %d\n", id);
            break;
    }

    return nullptr;
}

SDMAEngine*
AMDGPUDevice::getSDMAEngine(Addr offset)
{
    return sdmaEngs[offset];
}

void
AMDGPUDevice::intrPost()
{
    PciDevice::intrPost();
}

void
AMDGPUDevice::serialize(CheckpointOut &cp) const
{
    // Serialize the PciDevice base class
    PciDevice::serialize(cp);

    uint64_t regs_size = regs.size();
    uint64_t doorbells_size = doorbells.size();
    uint64_t sdma_engs_size = sdmaEngs.size();

    SERIALIZE_SCALAR(regs_size);
    SERIALIZE_SCALAR(doorbells_size);
    SERIALIZE_SCALAR(sdma_engs_size);

    // Make a c-style array of the regs to serialize
    uint32_t reg_addrs[regs_size];
    uint64_t reg_values[regs_size];
    uint32_t doorbells_offset[doorbells_size];
    QueueType doorbells_queues[doorbells_size];
    uint32_t sdma_engs_offset[sdma_engs_size];
    int sdma_engs[sdma_engs_size];

    int idx = 0;
    for (auto & it : regs) {
        reg_addrs[idx] = it.first;
        reg_values[idx] = it.second;
        ++idx;
    }

    idx = 0;
    for (auto & it : doorbells) {
        doorbells_offset[idx] = it.first;
        doorbells_queues[idx] = it.second;
        ++idx;
    }

    idx = 0;
    for (auto & it : sdmaEngs) {
        sdma_engs_offset[idx] = it.first;
        sdma_engs[idx] = it.second == sdma0 ? 0 : 1;
        ++idx;
    }

    SERIALIZE_ARRAY(reg_addrs, sizeof(reg_addrs)/sizeof(reg_addrs[0]));
    SERIALIZE_ARRAY(reg_values, sizeof(reg_values)/sizeof(reg_values[0]));
    SERIALIZE_ARRAY(doorbells_offset, sizeof(doorbells_offset)/
        sizeof(doorbells_offset[0]));
    SERIALIZE_ARRAY(doorbells_queues, sizeof(doorbells_queues)/
        sizeof(doorbells_queues[0]));
    SERIALIZE_ARRAY(sdma_engs_offset, sizeof(sdma_engs_offset)/
        sizeof(sdma_engs_offset[0]));
    SERIALIZE_ARRAY(sdma_engs, sizeof(sdma_engs)/sizeof(sdma_engs[0]));

    // Serialize the device memory
    deviceMem.serializeSection(cp, "deviceMem");
}

void
AMDGPUDevice::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    PciDevice::unserialize(cp);

    uint64_t regs_size = 0;
    uint64_t doorbells_size = 0;
    uint64_t sdma_engs_size = 0;

    UNSERIALIZE_SCALAR(regs_size);
    UNSERIALIZE_SCALAR(doorbells_size);
    UNSERIALIZE_SCALAR(sdma_engs_size);

    if (regs_size > 0) {
        uint32_t reg_addrs[regs_size];
        uint64_t reg_values[regs_size];

        UNSERIALIZE_ARRAY(reg_addrs, sizeof(reg_addrs)/sizeof(reg_addrs[0]));
        UNSERIALIZE_ARRAY(reg_values,
                          sizeof(reg_values)/sizeof(reg_values[0]));

        for (int idx = 0; idx < regs_size; ++idx) {
            regs.insert(std::make_pair(reg_addrs[idx], reg_values[idx]));
        }
    }

    if (doorbells_size > 0) {
        uint32_t doorbells_offset[doorbells_size];
        QueueType doorbells_queues[doorbells_size];

        UNSERIALIZE_ARRAY(doorbells_offset, sizeof(doorbells_offset)/
                sizeof(doorbells_offset[0]));
        UNSERIALIZE_ARRAY(doorbells_queues, sizeof(doorbells_queues)/
                sizeof(doorbells_queues[0]));

        for (int idx = 0; idx < doorbells_size; ++idx) {
            regs.insert(std::make_pair(doorbells_offset[idx],
                      doorbells_queues[idx]));
            doorbells[doorbells_offset[idx]] = doorbells_queues[idx];
        }
    }

    if (sdma_engs_size > 0) {
        uint32_t sdma_engs_offset[sdma_engs_size];
        int sdma_engs[sdma_engs_size];

        UNSERIALIZE_ARRAY(sdma_engs_offset, sizeof(sdma_engs_offset)/
            sizeof(sdma_engs_offset[0]));
        UNSERIALIZE_ARRAY(sdma_engs, sizeof(sdma_engs)/sizeof(sdma_engs[0]));

        for (int idx = 0; idx < sdma_engs_size; ++idx) {
            SDMAEngine *sdma = sdma_engs[idx] == 0 ? sdma0 : sdma1;
            sdmaEngs.insert(std::make_pair(sdma_engs_offset[idx], sdma));
        }
    }

    // Unserialize the device memory
    deviceMem.unserializeSection(cp, "deviceMem");
}

uint16_t
AMDGPUDevice::allocateVMID(uint16_t pasid)
{
    for (uint16_t vmid = 1; vmid < AMDGPU_VM_COUNT; vmid++) {
        auto result = usedVMIDs.find(vmid);
        if (result == usedVMIDs.end()) {
            idMap.insert(std::make_pair(pasid, vmid));
            usedVMIDs[vmid] = {};
            _lastVMID = vmid;
            return vmid;
        }
    }
    panic("All VMIDs have been assigned");
}

void
AMDGPUDevice::deallocateVmid(uint16_t vmid)
{
    usedVMIDs.erase(vmid);
}

void
AMDGPUDevice::deallocatePasid(uint16_t pasid)
{
    auto result = idMap.find(pasid);
    assert(result != idMap.end());
    if (result == idMap.end()) return;
    uint16_t vmid = result->second;

    idMap.erase(result);
    usedVMIDs.erase(vmid);
}

void
AMDGPUDevice::deallocateAllQueues()
{
    idMap.erase(idMap.begin(), idMap.end());
    usedVMIDs.erase(usedVMIDs.begin(), usedVMIDs.end());

    sdma0->deallocateRLCQueues();
    sdma1->deallocateRLCQueues();
}

void
AMDGPUDevice::mapDoorbellToVMID(Addr doorbell, uint16_t vmid)
{
    doorbellVMIDMap[doorbell] = vmid;
}

std::unordered_map<uint16_t, std::set<int>>&
AMDGPUDevice::getUsedVMIDs()
{
    return usedVMIDs;
}

void
AMDGPUDevice::insertQId(uint16_t vmid, int id)
{
    usedVMIDs[vmid].insert(id);
}

} // namespace gem5
