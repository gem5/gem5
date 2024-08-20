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
#include "dev/amdgpu/amdgpu_nbio.hh"
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
      cp(p.cp), checkpoint_before_mmios(p.checkpoint_before_mmios),
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

    if (p.device_name == "Vega10") {
        gfx_version = GfxVersion::gfx900;
    } else if (p.device_name == "MI100") {
        gfx_version = GfxVersion::gfx908;
    } else if (p.device_name == "MI200") {
        gfx_version = GfxVersion::gfx90a;
    } else if (p.device_name == "MI300X") {
        gfx_version = GfxVersion::gfx942;
    } else {
        panic("Unknown GPU device %s\n", p.device_name);
    }

    if (p.trace_file != "") {
        mmioReader.readMMIOTrace(p.trace_file);
    }

    int sdma_id = 0;
    for (auto& s : p.sdmas) {
        s->setGPUDevice(this);
        s->setId(sdma_id);
        sdmaIds.insert({sdma_id, s});
        sdmaMmios.insert({sdma_id,
                          RangeSize(s->getMmioBase(), s->getMmioSize())});
        DPRINTF(AMDGPUDevice, "SDMA%d has MMIO range %s\n", sdma_id,
                sdmaMmios[sdma_id].to_string().c_str());
        sdma_id++;
    }

    // Map SDMA MMIO addresses to functions
    sdmaFunc.insert({0x81, &SDMAEngine::setGfxBaseLo});
    sdmaFunc.insert({0x82, &SDMAEngine::setGfxBaseHi});
    sdmaFunc.insert({0x88, &SDMAEngine::setGfxRptrHi});
    sdmaFunc.insert({0x89, &SDMAEngine::setGfxRptrLo});
    sdmaFunc.insert({0x92, &SDMAEngine::setGfxDoorbellLo});
    sdmaFunc.insert({0xab, &SDMAEngine::setGfxDoorbellOffsetLo});
    sdmaFunc.insert({0x80, &SDMAEngine::setGfxSize});
    sdmaFunc.insert({0xb2, &SDMAEngine::setGfxWptrLo});
    sdmaFunc.insert({0xb3, &SDMAEngine::setGfxWptrHi});
    if (p.device_name == "Vega10") {
        sdmaFunc.insert({0xe1, &SDMAEngine::setPageBaseLo});
        sdmaFunc.insert({0xe9, &SDMAEngine::setPageRptrLo});
        sdmaFunc.insert({0xe8, &SDMAEngine::setPageRptrHi});
        sdmaFunc.insert({0xf2, &SDMAEngine::setPageDoorbellLo});
        sdmaFunc.insert({0x10b, &SDMAEngine::setPageDoorbellOffsetLo});
        sdmaFunc.insert({0xe0, &SDMAEngine::setPageSize});
        sdmaFunc.insert({0x113, &SDMAEngine::setPageWptrLo});
    } else if (p.device_name == "MI100" || p.device_name == "MI200"
            || p.device_name == "MI300X") {
        sdmaFunc.insert({0xd9, &SDMAEngine::setPageBaseLo});
        sdmaFunc.insert({0xe1, &SDMAEngine::setPageRptrLo});
        sdmaFunc.insert({0xe0, &SDMAEngine::setPageRptrHi});
        sdmaFunc.insert({0xea, &SDMAEngine::setPageDoorbellLo});
        sdmaFunc.insert({0xd8, &SDMAEngine::setPageDoorbellOffsetLo});
        sdmaFunc.insert({0x10b, &SDMAEngine::setPageWptrLo});
    } else {
        panic("Unknown GPU device %s\n", p.device_name);
    }

    // Setup PM4 packet processors and sanity check IDs
    std::set<int> pm4_ids;
    for (auto& pm4 : p.pm4_pkt_procs) {
        pm4->setGPUDevice(this);
        fatal_if(pm4_ids.count(pm4->getIpId()),
                "Two PM4s with same IP IDs is not allowed");
        pm4_ids.insert(pm4->getIpId());
        pm4PktProcs.insert({pm4->getIpId(), pm4});

        pm4Ranges.insert({pm4->getMMIORange(), pm4});
    }

    // There should be at least one PM4 packet processor with ID 0
    fatal_if(!pm4PktProcs.count(0), "No default PM4 processor found");

    deviceIH->setGPUDevice(this);
    cp->hsaPacketProc().setGPUDevice(this);
    cp->setGPUDevice(this);
    nbio.setGPUDevice(this);

    // Address aperture for device memory. We tell this to the driver and
    // could possibly be anything, but these are the values used by hardware.
    uint64_t mmhubBase = 0x8000ULL << 24;
    uint64_t mmhubTop = 0x83ffULL << 24;
    uint64_t mem_size = 0x3ff0; // 16 GB of memory

    gpuvm.setMMHUBBase(mmhubBase);
    gpuvm.setMMHUBTop(mmhubTop);

    // Map other MMIO apertures based on gfx version. This must be done before
    // any calls to get/setRegVal.
    // NBIO               0x0     - 0x4280
    // IH                 0x4280  - 0x4980
    // GRBM               0x8000  - 0xC000
    // GFX                0x28000 - 0x3F000
    // MMHUB              0x68000 - 0x6a120
    gpuvm.setMMIOAperture(NBIO_MMIO_RANGE, AddrRange(0x0, 0x4280));
    gpuvm.setMMIOAperture(IH_MMIO_RANGE,   AddrRange(0x4280, 0x4980));
    gpuvm.setMMIOAperture(GRBM_MMIO_RANGE, AddrRange(0x8000, 0xC000));
    gpuvm.setMMIOAperture(GFX_MMIO_RANGE,  AddrRange(0x28000, 0x3F000));
    gpuvm.setMMIOAperture(MMHUB_MMIO_RANGE,  AddrRange(0x68000, 0x6A120));

    // These are hardcoded register values to return what the driver expects
    setRegVal(AMDGPU_MP0_SMN_C2PMSG_33, 0x80000000);

    // There are different registers for different GPUs, so we set the value
    // based on the GPU type specified by the user.
    if (p.device_name == "Vega10") {
        setRegVal(VEGA10_FB_LOCATION_BASE, mmhubBase >> 24);
        setRegVal(VEGA10_FB_LOCATION_TOP, mmhubTop >> 24);
    } else if (p.device_name == "MI100") {
        setRegVal(MI100_FB_LOCATION_BASE, mmhubBase >> 24);
        setRegVal(MI100_FB_LOCATION_TOP, mmhubTop >> 24);
        setRegVal(MI100_MEM_SIZE_REG, mem_size);
    } else if (p.device_name == "MI200") {
        // This device can have either 64GB or 128GB of device memory.
        // This limits to 16GB for simulation.
        setRegVal(MI200_FB_LOCATION_BASE, mmhubBase >> 24);
        setRegVal(MI200_FB_LOCATION_TOP, mmhubTop >> 24);
        setRegVal(MI200_MEM_SIZE_REG, mem_size);
    } else if (p.device_name == "MI300X") {
        setRegVal(MI200_FB_LOCATION_BASE, mmhubBase >> 24);
        setRegVal(MI200_FB_LOCATION_TOP, mmhubTop >> 24);
        setRegVal(MI200_MEM_SIZE_REG, mem_size);
    } else {
        panic("Unknown GPU device %s\n", p.device_name);
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

void
AMDGPUDevice::writeROM(PacketPtr pkt)
{
    assert(isROM(pkt->getAddr()));

    Addr rom_offset = pkt->getAddr() - romRange.start();
    uint64_t rom_data = pkt->getUintX(ByteOrder::little);

    memcpy(rom.data() + rom_offset, &rom_data, pkt->getSize());

    DPRINTF(AMDGPUDevice, "Write to addr %#x on ROM offset %#x data: %#x\n",
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
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;

    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDevice::readConfig(pkt);
    } else {
        if (offset >= PXCAP_BASE && offset < (PXCAP_BASE + sizeof(PXCAP))) {
            int pxcap_offset = offset - PXCAP_BASE;

            switch (pkt->getSize()) {
                case sizeof(uint8_t):
                    pkt->setLE<uint8_t>(pxcap.data[pxcap_offset]);
                    DPRINTF(AMDGPUDevice,
                        "Read PXCAP:  dev %#x func %#x reg %#x 1 bytes: data "
                        "= %#x\n", _busAddr.dev, _busAddr.func, pxcap_offset,
                        (uint32_t)pkt->getLE<uint8_t>());
                    break;
                case sizeof(uint16_t):
                    pkt->setLE<uint16_t>(
                        *(uint16_t*)&pxcap.data[pxcap_offset]);
                    DPRINTF(AMDGPUDevice,
                        "Read PXCAP:  dev %#x func %#x reg %#x 2 bytes: data "
                        "= %#x\n", _busAddr.dev, _busAddr.func, pxcap_offset,
                        (uint32_t)pkt->getLE<uint16_t>());
                    break;
                case sizeof(uint32_t):
                    pkt->setLE<uint32_t>(
                        *(uint32_t*)&pxcap.data[pxcap_offset]);
                    DPRINTF(AMDGPUDevice,
                        "Read PXCAP:  dev %#x func %#x reg %#x 4 bytes: data "
                        "= %#x\n",_busAddr.dev, _busAddr.func, pxcap_offset,
                        (uint32_t)pkt->getLE<uint32_t>());
                    break;
                default:
                    panic("Invalid access size (%d) for amdgpu PXCAP %#x\n",
                          pkt->getSize(), pxcap_offset);
            }
            pkt->makeAtomicResponse();
        } else {
            warn("Device specific offset %d not implemented!\n", offset);
        }
    }

    // Before sending MMIOs the driver sends three interrupts in a row.
    // Use this to trigger creating a checkpoint to restore in timing mode.
    // This is only necessary until we can create a "hole" in the KVM VM
    // around the VGA ROM region such that KVM exits and sends requests to
    // this device rather than the KVM VM.
    if (checkpoint_before_mmios) {
        if (offset == PCI0_INTERRUPT_PIN) {
            if (++init_interrupt_count == 3) {
                DPRINTF(AMDGPUDevice, "Checkpointing before first MMIO\n");
                exitSimLoop("checkpoint", 0, curTick() + configDelay + 1);
            }
        } else {
            init_interrupt_count = 0;
        }
    }

    return configDelay;
}

Tick
AMDGPUDevice::writeConfig(PacketPtr pkt)
{
    [[maybe_unused]] int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    DPRINTF(AMDGPUDevice, "Write Config: from offset: %#x size: %#x "
            "data: %#x\n", offset, pkt->getSize(),
            pkt->getUintX(ByteOrder::little));

    if (offset < PCI_DEVICE_SPECIFIC)
        return PciDevice::writeConfig(pkt);


    if (offset >= PXCAP_BASE && offset < (PXCAP_BASE + sizeof(PXCAP))) {
        uint8_t *pxcap_data = &(pxcap.data[0]);
        int pxcap_offset = offset - PXCAP_BASE;

        DPRINTF(AMDGPUDevice, "Writing PXCAP offset %d size %d\n",
                pxcap_offset, pkt->getSize());

        memcpy(pxcap_data + pxcap_offset, pkt->getConstPtr<void>(),
               pkt->getSize());
    }

    pkt->makeAtomicResponse();

    return configDelay;
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
     * first, ignoring any writes from driver. (2) Any other address from
     * device backing store / abstract memory class functionally.
     */
    if (nbio.readFrame(pkt, offset)) {
        return;
    }

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
    delete readPkt;
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
    AddrRange aperture = gpuvm.getMMIOAperture(offset);
    Addr aperture_offset = offset - aperture.start();

    // By default read from MMIO trace. Overwrite the packet for a select
    // few more dynamic MMIOs.
    DPRINTF(AMDGPUDevice, "Read MMIO %#lx\n", offset);
    mmioReader.readFromTrace(pkt, MMIO_BAR, offset);

    if (aperture == gpuvm.getMMIORange(NBIO_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "NBIO base\n");
        nbio.readMMIO(pkt, aperture_offset);
    } else if (aperture == gpuvm.getMMIORange(GRBM_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "GRBM base\n");
        gpuvm.readMMIO(pkt, aperture_offset >> GRBM_OFFSET_SHIFT);
    } else if (aperture == gpuvm.getMMIORange(GFX_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "GFX base\n");
        gfx.readMMIO(pkt, aperture_offset);
    } else if (aperture == gpuvm.getMMIORange(MMHUB_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "MMHUB base\n");
        gpuvm.readMMIO(pkt, aperture_offset >> MMHUB_OFFSET_SHIFT);
    } else {
        DPRINTF(AMDGPUDevice, "Unknown MMIO aperture for read %#x\n", offset);
    }
}

void
AMDGPUDevice::writeFrame(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote framebuffer address %#lx\n", offset);

    for (auto& cu: CP()->shader()->cuList) {
        auto system = CP()->shader()->gpuCmdProc.system();
        Addr aligned_addr = offset & ~(system->cacheLineSize() - 1);
        cu->sendInvL2(aligned_addr);
    }

    Addr aperture = gpuvm.getFrameAperture(offset);
    Addr aperture_offset = offset - aperture;

    // Record the value
    if (aperture == gpuvm.gartBase()) {
        gpuvm.gartTable[aperture_offset] = pkt->getUintX(ByteOrder::little);
        DPRINTF(AMDGPUDevice, "GART translation %p -> %p\n", aperture_offset,
                gpuvm.gartTable[aperture_offset]);
    }

    nbio.writeFrame(pkt, offset);

    /*
     * Write the value to device memory. This must be done functionally
     * because this method is called by the PCIDevice::write method which
     * is a non-timing write.
     */
    RequestPtr req = std::make_shared<Request>(offset, pkt->getSize(), 0,
                                               vramRequestorId());
    PacketPtr writePkt = Packet::createWrite(req);
    uint8_t *dataPtr = new uint8_t[pkt->getSize()];
    std::memcpy(dataPtr, pkt->getPtr<uint8_t>(),
                pkt->getSize() * sizeof(uint8_t));
    writePkt->dataDynamic(dataPtr);

    auto system = cp->shader()->gpuCmdProc.system();
    system->getDeviceMemory(writePkt)->access(writePkt);

    delete writePkt;
}

void
AMDGPUDevice::writeDoorbell(PacketPtr pkt, Addr offset)
{
    DPRINTF(AMDGPUDevice, "Wrote doorbell %#lx\n", offset);

    if (doorbells.find(offset) != doorbells.end()) {
        QueueType q_type = doorbells[offset].qtype;
        int ip_id = doorbells[offset].ip_id;
        DPRINTF(AMDGPUDevice, "Doorbell offset %p queue: %d\n",
                              offset, q_type);
        switch (q_type) {
          case Compute:
            assert(pm4PktProcs.count(ip_id));
            pm4PktProcs[ip_id]->process(
                pm4PktProcs[ip_id]->getQueue(offset),
                pkt->getLE<uint64_t>());
          break;
          case Gfx:
            assert(pm4PktProcs.count(ip_id));
            pm4PktProcs[ip_id]->process(
                pm4PktProcs[ip_id]->getQueue(offset, true),
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
            assert(pm4PktProcs.count(ip_id));
            cp->hsaPacketProc().hwScheduler()->write(offset,
                pkt->getLE<uint64_t>() + 1);
            pm4PktProcs[ip_id]->updateReadIndex(offset,
                pkt->getLE<uint64_t>() + 1);
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
        warn("Unknown doorbell offset: %lx. Saving to pending doorbells.\n",
             offset);

        // We have to ACK the PCI packet immediately, so create a copy of the
        // packet here to send again. The packet data contains the value of
        // the doorbell to write so we need to copy that as the original
        // packet gets deleted after the PCI write() method returns.
        RequestPtr pending_req(pkt->req);
        PacketPtr pending_pkt = Packet::createWrite(pending_req);
        uint8_t *pending_data = new uint8_t[pkt->getSize()];
        memcpy(pending_data, pkt->getPtr<uint8_t>(), pkt->getSize());
        pending_pkt->dataDynamic(pending_data);

        pendingDoorbellPkts.emplace(offset, pending_pkt);
    }
}

void
AMDGPUDevice::writeMMIO(PacketPtr pkt, Addr offset)
{
    AddrRange aperture = gpuvm.getMMIOAperture(offset);
    Addr aperture_offset = offset - aperture.start();

    DPRINTF(AMDGPUDevice, "Wrote MMIO %#lx\n", offset);

    // Check SDMA functions first, then fallback to MMIO ranges.
    for (int idx = 0; idx < sdmaIds.size(); ++idx) {
        if (sdmaMmios[idx].contains(offset)) {
            Addr sdma_offset = (offset - sdmaMmios[idx].start()) >> 2;
            if (sdmaFunc.count(sdma_offset)) {
                DPRINTF(AMDGPUDevice, "Calling SDMA%d MMIO function %lx\n",
                        idx, sdma_offset);
                sdmaFuncPtr mptr = sdmaFunc[sdma_offset];
                (getSDMAById(idx)->*mptr)(pkt->getLE<uint32_t>());
            } else {
                DPRINTF(AMDGPUDevice, "Unknown SDMA%d MMIO: %#lx\n", idx,
                        sdma_offset);
            }

            return;
        }
    }

    // Check PM4s next, returning to avoid duplicate writes.
    for (auto& [range, pm4_proc] : pm4Ranges) {
        if (range.contains(offset)) {
            // PM4 MMIOs are offset based on the MMIO range start
            Addr ip_offset = offset - range.start();
            pm4_proc->writeMMIO(pkt, ip_offset >> GRBM_OFFSET_SHIFT);

            return;
        }
    }

    if (aperture == gpuvm.getMMIORange(GRBM_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "GRBM base\n");
        gpuvm.writeMMIO(pkt, aperture_offset >> GRBM_OFFSET_SHIFT);
    } else if (aperture == gpuvm.getMMIORange(IH_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "IH base\n");
        deviceIH->writeMMIO(pkt, aperture_offset >> IH_OFFSET_SHIFT);
    } else if (aperture == gpuvm.getMMIORange(NBIO_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "NBIO base\n");
        nbio.writeMMIO(pkt, aperture_offset);
    } else if (aperture == gpuvm.getMMIORange(GFX_MMIO_RANGE)) {
        DPRINTF(AMDGPUDevice, "GFX base\n");
        gfx.writeMMIO(pkt, aperture_offset);
    } else {
        DPRINTF(AMDGPUDevice, "Unknown MMIO aperture for write %#x\n", offset);
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
    if (isROM(pkt->getAddr())) {
        writeROM(pkt);

        dispatchAccess(pkt, false);

        return pioDelay;
    }

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

    DPRINTF(AMDGPUDevice, "PCI Write to %#lx data %#lx\n",
                            pkt->getAddr(), data);

    dispatchAccess(pkt, false);

    return pioDelay;
}

void
AMDGPUDevice::processPendingDoorbells(uint32_t offset)
{
    if (pendingDoorbellPkts.count(offset)) {
        DPRINTF(AMDGPUDevice, "Sending pending doorbell %x\n", offset);
        writeDoorbell(pendingDoorbellPkts[offset], offset);
        delete pendingDoorbellPkts[offset];
        pendingDoorbellPkts.erase(offset);
    }
}

uint32_t
AMDGPUDevice::getRegVal(uint64_t addr)
{
    // This is somewhat of a guess based on amdgpu_device_mm_access
    // in amdgpu_device.c in the ROCk driver. If bit 32 is 1 then
    // assume VRAM and use full address, otherwise assume register
    // address and only user lower 31 bits.
    Addr fixup_addr = bits(addr, 31, 31) ? addr : addr & 0x7fffffff;

    uint32_t pkt_data = 0;
    RequestPtr request = std::make_shared<Request>(fixup_addr,
            sizeof(uint32_t), 0 /* flags */, vramRequestorId());
    PacketPtr pkt = Packet::createRead(request);
    pkt->dataStatic((uint8_t *)&pkt_data);
    readMMIO(pkt, addr);
    DPRINTF(AMDGPUDevice, "Getting register 0x%lx = %x\n",
            fixup_addr, pkt->getLE<uint32_t>());

    pkt_data = pkt->getLE<uint32_t>();
    delete pkt;

    return pkt_data;
}

void
AMDGPUDevice::setRegVal(uint64_t addr, uint32_t value)
{
    DPRINTF(AMDGPUDevice, "Setting register 0x%lx to %x\n",
            addr, value);

    uint32_t pkt_data = value;
    RequestPtr request = std::make_shared<Request>(addr,
            sizeof(uint32_t), 0 /* flags */, vramRequestorId());
    PacketPtr pkt = Packet::createWrite(request);
    pkt->dataStatic((uint8_t *)&pkt_data);
    writeMMIO(pkt, addr);
    delete pkt;
}

void
AMDGPUDevice::setDoorbellType(uint32_t offset, QueueType qt, int ip_id)
{
    DPRINTF(AMDGPUDevice, "Setting doorbell type for %x\n", offset);
    doorbells[offset].qtype = qt;
    doorbells[offset].ip_id = ip_id;
}

void
AMDGPUDevice::unsetDoorbell(uint32_t offset)
{
    doorbells.erase(offset);
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
    assert(sdmaIds.count(id));

    return sdmaIds[id];
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

    uint64_t doorbells_size = doorbells.size();
    uint64_t sdma_engs_size = sdmaEngs.size();
    uint64_t used_vmid_map_size = usedVMIDs.size();

    SERIALIZE_SCALAR(doorbells_size);
    SERIALIZE_SCALAR(sdma_engs_size);
    // Save the number of vmids used
    SERIALIZE_SCALAR(used_vmid_map_size);

    // Make a c-style array of the regs to serialize
    uint32_t doorbells_offset[doorbells_size];
    QueueType doorbells_queues[doorbells_size];
    int doorbells_ip_ids[doorbells_size];
    uint32_t sdma_engs_offset[sdma_engs_size];
    int sdma_engs[sdma_engs_size];
    int used_vmids[used_vmid_map_size];
    int used_queue_id_sizes[used_vmid_map_size];
    std::vector<int> used_vmid_sets;

    int idx = 0;
    for (auto & it : doorbells) {
        doorbells_offset[idx] = it.first;
        doorbells_queues[idx] = it.second.qtype;
        doorbells_ip_ids[idx] = it.second.ip_id;
        ++idx;
    }

    idx = 0;
    for (auto & it : sdmaEngs) {
        sdma_engs_offset[idx] = it.first;
        sdma_engs[idx] = it.second->getId();
        ++idx;
    }

    idx = 0;
    for (auto & it : usedVMIDs) {
        used_vmids[idx] = it.first;
        used_queue_id_sizes[idx] = it.second.size();
        std::vector<int> set_vector(it.second.begin(), it.second.end());
        used_vmid_sets.insert(used_vmid_sets.end(),
                set_vector.begin(), set_vector.end());
        ++idx;
    }

    int num_queue_id = used_vmid_sets.size();
    int* vmid_array = new int[num_queue_id];
    std::copy(used_vmid_sets.begin(), used_vmid_sets.end(), vmid_array);

    SERIALIZE_ARRAY(doorbells_offset, sizeof(doorbells_offset)/
        sizeof(doorbells_offset[0]));
    SERIALIZE_ARRAY(doorbells_queues, sizeof(doorbells_queues)/
        sizeof(doorbells_queues[0]));
    SERIALIZE_ARRAY(doorbells_ip_ids, sizeof(doorbells_ip_ids)/
        sizeof(doorbells_ip_ids[0]));
    SERIALIZE_ARRAY(sdma_engs_offset, sizeof(sdma_engs_offset)/
        sizeof(sdma_engs_offset[0]));
    SERIALIZE_ARRAY(sdma_engs, sizeof(sdma_engs)/sizeof(sdma_engs[0]));
    // Save the vmids used in an array
    SERIALIZE_ARRAY(used_vmids, sizeof(used_vmids)/sizeof(used_vmids[0]));
    // Save the size of the set of queue ids mapped to each vmid
    SERIALIZE_ARRAY(used_queue_id_sizes,
            sizeof(used_queue_id_sizes)/sizeof(used_queue_id_sizes[0]));
    // Save all the queue ids used for all the vmids
    SERIALIZE_ARRAY(vmid_array, num_queue_id);
    // Save the total number of queue idsused
    SERIALIZE_SCALAR(num_queue_id);

    // Serialize the device memory
    deviceMem.serializeSection(cp, "deviceMem");
    gpuvm.serializeSection(cp, "GPUVM");

    delete[] vmid_array;
}

void
AMDGPUDevice::unserialize(CheckpointIn &cp)
{
    // Unserialize the PciDevice base class
    PciDevice::unserialize(cp);

    uint64_t doorbells_size = 0;
    uint64_t sdma_engs_size = 0;
    uint64_t used_vmid_map_size = 0;

    UNSERIALIZE_SCALAR(doorbells_size);
    UNSERIALIZE_SCALAR(sdma_engs_size);
    UNSERIALIZE_SCALAR(used_vmid_map_size);


    if (doorbells_size > 0) {
        uint32_t doorbells_offset[doorbells_size];
        QueueType doorbells_queues[doorbells_size];
        int doorbells_ip_ids[doorbells_size];

        UNSERIALIZE_ARRAY(doorbells_offset, sizeof(doorbells_offset)/
                sizeof(doorbells_offset[0]));
        UNSERIALIZE_ARRAY(doorbells_queues, sizeof(doorbells_queues)/
                sizeof(doorbells_queues[0]));
        UNSERIALIZE_ARRAY(doorbells_ip_ids, sizeof(doorbells_ip_ids)/
                sizeof(doorbells_ip_ids[0]));

        for (int idx = 0; idx < doorbells_size; ++idx) {
            doorbells[doorbells_offset[idx]].qtype = doorbells_queues[idx];
            doorbells[doorbells_offset[idx]].ip_id = doorbells_ip_ids[idx];
        }
    }

    if (sdma_engs_size > 0) {
        uint32_t sdma_engs_offset[sdma_engs_size];
        int sdma_engs[sdma_engs_size];

        UNSERIALIZE_ARRAY(sdma_engs_offset, sizeof(sdma_engs_offset)/
            sizeof(sdma_engs_offset[0]));
        UNSERIALIZE_ARRAY(sdma_engs, sizeof(sdma_engs)/sizeof(sdma_engs[0]));

        for (int idx = 0; idx < sdma_engs_size; ++idx) {
            int sdma_id = sdma_engs[idx];
            assert(sdmaIds.count(sdma_id));
            SDMAEngine *sdma = sdmaIds[sdma_id];
            sdmaEngs.insert(std::make_pair(sdma_engs_offset[idx], sdma));
        }
    }

    if (used_vmid_map_size > 0) {
        int used_vmids[used_vmid_map_size];
        int used_queue_id_sizes[used_vmid_map_size];
        int num_queue_id = 0;
        std::vector<int> used_vmid_sets;
        // Extract the total number of queue ids used
        UNSERIALIZE_SCALAR(num_queue_id);
        int* vmid_array = new int[num_queue_id];
        // Extract the number of vmids used
        UNSERIALIZE_ARRAY(used_vmids, used_vmid_map_size);
        // Extract the size of the queue id set for each vmid
        UNSERIALIZE_ARRAY(used_queue_id_sizes, used_vmid_map_size);
        // Extract all the queue ids used
        UNSERIALIZE_ARRAY(vmid_array, num_queue_id);
        // Populate the usedVMIDs map with the queue ids per vm
        int idx = 0;
        for (int it = 0; it < used_vmid_map_size; it++) {
            int vmid = used_vmids[it];
            int vmid_set_size = used_queue_id_sizes[it];
            for (int j = 0; j < vmid_set_size; j++) {
                usedVMIDs[vmid].insert(vmid_array[idx + j]);
            }
            idx += vmid_set_size;
        }
        delete[] vmid_array;
    }

    // Unserialize the device memory
    deviceMem.unserializeSection(cp, "deviceMem");
    gpuvm.unserializeSection(cp, "GPUVM");
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
AMDGPUDevice::deallocateAllQueues(bool unmap_static)
{
    idMap.erase(idMap.begin(), idMap.end());
    usedVMIDs.erase(usedVMIDs.begin(), usedVMIDs.end());

    for (auto& it : sdmaEngs) {
        it.second->deallocateRLCQueues(unmap_static);
    }

    // "All" queues implicitly refers to all user queues. User queues begin at
    // doorbell address 0x4000, so unmap any queue at or above that address.
    for (auto [offset, vmid] : doorbellVMIDMap) {
        if (offset >= 0x4000) {
            doorbells.erase(offset);
        }
    }
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
