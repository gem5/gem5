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
 *
 */

#include "dev/amdgpu/interrupt_handler.hh"

#include "debug/AMDGPUDevice.hh"
#include "dev/amdgpu/ih_mmio.hh"
#include "mem/packet_access.hh"

// For generating interrupts, the object causing interrupt communicates with
// the Interrupt Handler (IH), which submits a 256-bit Interrupt packet to the
// system memory. The location where the IH submits the packet is the
// IH Ring buffer in the system memory. The IH updates the Write Pointer
// and the host consumes the ring buffer and once done, updates the Read
// Pointer through the doorbell.

// IH_RB_BaseAddr, IH_RB_WptrAddr (Lo/Hi), IH_RB_RptrAddr (Lo/Hi), etc. are
// not GART addresses but system dma addresses and thus don't require
// translations through the GART table.

namespace gem5
{

AMDGPUInterruptHandler::AMDGPUInterruptHandler(
    const AMDGPUInterruptHandlerParams &p)
    : DmaDevice(p)
{
    memset(&regs, 0, sizeof(AMDGPUIHRegs));
}

AddrRangeList
AMDGPUInterruptHandler::getAddrRanges() const
{
    AddrRangeList ranges;
    return ranges;
}

void
AMDGPUInterruptHandler::intrPost()
{
    if (gpuDevice)
        gpuDevice->intrPost();
}

void
AMDGPUInterruptHandler::prepareInterruptCookie(ContextID cntxt_id,
                                               uint32_t ring_id,
                                               uint32_t client_id,
                                               uint32_t source_id,
                                               unsigned node_id)
{
    assert(client_id == SOC15_IH_CLIENTID_RLC ||
           client_id == SOC15_IH_CLIENTID_SDMA0 ||
           client_id == SOC15_IH_CLIENTID_SDMA1 ||
           client_id == SOC15_IH_CLIENTID_SDMA2 ||
           client_id == SOC15_IH_CLIENTID_SDMA3 ||
           client_id == SOC15_IH_CLIENTID_SDMA4 ||
           client_id == SOC15_IH_CLIENTID_SDMA5 ||
           client_id == SOC15_IH_CLIENTID_SDMA6 ||
           client_id == SOC15_IH_CLIENTID_SDMA7 ||
           client_id == SOC15_IH_CLIENTID_GRBM_CP);
    assert(source_id == CP_EOP || source_id == TRAP_ID);

    /**
     * Setup the fields in the interrupt cookie (see header file for more
     * detail on the fields). The timestamp here is a bogus value. It seems
     * the driver does not really care what this value is. Additionally the
     * model does not currently have anything to keep track of time. It is
     * possible that tick/cycle count can be used in the future if this ends
     * up being important. The remaining fields are passed from whichever
     * block is sending the interrupt.
     */
    AMDGPUInterruptCookie *cookie = new AMDGPUInterruptCookie();
    memset(cookie, 0, sizeof(AMDGPUInterruptCookie));

    // Currently only one process is supported and the first pasid from driver
    // is always 0x8000. In the future this can be obtained from the PM4
    // MAP_PROCESS packet and may need to be passed to this function.
    //
    // On a related note, leave vmid fields alone as they are only used for
    // memory exceptions. Memory exceptions are not supported on gfx900.
    cookie->pasid = 0x8000;
    cookie->timestamp_Lo = 0x40;
    cookie->clientId = client_id;
    cookie->sourceId = source_id;
    cookie->ringId = ring_id;
    cookie->nodeId = node_id;
    cookie->source_data_dw1 = cntxt_id;
    interruptQueue.push(cookie);
}

void
AMDGPUInterruptHandler::DmaEvent::process()
{
    if (data == 1) {
        DPRINTF(AMDGPUDevice, "Completed interrupt cookie write\n");
        deviceIh->submitWritePointer();
    } else if (data == 2) {
        DPRINTF(AMDGPUDevice, "Completed interrupt write pointer update\n");
        deviceIh->intrPost();
    } else {
        fatal("Interrupt Handler DMA event returned bad value: %d\n", data);
    }
}

void
AMDGPUInterruptHandler::submitWritePointer()
{
    uint8_t *dataPtr = new uint8_t[sizeof(uint32_t)];
    regs.IH_Wptr += sizeof(AMDGPUInterruptCookie);
    Addr paddr = regs.WptrAddr;
    std::memcpy(dataPtr, &regs.IH_Wptr, sizeof(uint32_t));

    dmaEvent = new AMDGPUInterruptHandler::DmaEvent(this, 2);
    dmaWrite(paddr, sizeof(uint32_t), dmaEvent, dataPtr);
}

void
AMDGPUInterruptHandler::submitInterruptCookie()
{
    assert(!interruptQueue.empty());
    auto cookie = interruptQueue.front();
    size_t cookieSize = sizeof(AMDGPUInterruptCookie);

    uint8_t *dataPtr = new uint8_t[cookieSize];
    std::memcpy(dataPtr, cookie, cookieSize);
    Addr paddr = regs.baseAddr + regs.IH_Wptr;

    DPRINTF(AMDGPUDevice, "InterruptHandler rptr: 0x%x wptr: 0x%x\n",
            regs.IH_Rptr, regs.IH_Wptr);
    dmaEvent = new AMDGPUInterruptHandler::DmaEvent(this, 1);
    dmaWrite(paddr, cookieSize, dmaEvent, dataPtr);

    interruptQueue.pop();
}

void
AMDGPUInterruptHandler::writeMMIO(PacketPtr pkt, Addr mmio_offset)
{
    switch (mmio_offset) {
    case mmIH_RB_CNTL:
        setCntl(pkt->getLE<uint32_t>());
        break;
    case mmIH_RB_BASE:
        setBase(pkt->getLE<uint32_t>());
        break;
    case mmIH_RB_BASE_HI:
        setBaseHi(pkt->getLE<uint32_t>());
        break;
    case mmIH_RB_RPTR:
        setRptr(pkt->getLE<uint32_t>());
        break;
    case mmIH_RB_WPTR:
        setWptr(pkt->getLE<uint32_t>());
        break;
    case mmIH_RB_WPTR_ADDR_LO:
        setWptrAddrLo(pkt->getLE<uint32_t>());
        break;
    case mmIH_RB_WPTR_ADDR_HI:
        setWptrAddrHi(pkt->getLE<uint32_t>());
        break;
    case mmIH_DOORBELL_RPTR:
        setDoorbellOffset(pkt->getLE<uint32_t>());
        if (bits(pkt->getLE<uint32_t>(), 28, 28)) {
            gpuDevice->setDoorbellType(getDoorbellOffset() << 2,
                                       InterruptHandler);
        }
        break;
    default:
        DPRINTF(AMDGPUDevice, "IH Unknown MMIO %#x\n", mmio_offset);
        break;
    }
}

void
AMDGPUInterruptHandler::setCntl(const uint32_t &data)
{
    regs.IH_Cntl = data;
}

void
AMDGPUInterruptHandler::setBase(const uint32_t &data)
{
    regs.baseAddr = data;
    regs.baseAddr <<= 8;
}

void
AMDGPUInterruptHandler::setBaseHi(const uint32_t &data)
{
    regs.baseAddr |= static_cast<uint64_t>(data) << 40;
}

void
AMDGPUInterruptHandler::setRptr(const uint32_t &data)
{
    regs.IH_Rptr = data;
}

void
AMDGPUInterruptHandler::setWptr(const uint32_t &data)
{
    regs.IH_Wptr = data;
}

void
AMDGPUInterruptHandler::setWptrAddrLo(const uint32_t &data)
{
    regs.IH_Wptr_Addr_Lo = data;
    regs.WptrAddr |= regs.IH_Wptr_Addr_Lo;
}

void
AMDGPUInterruptHandler::setWptrAddrHi(const uint32_t &data)
{
    regs.IH_Wptr_Addr_Hi = data;
    regs.WptrAddr |= ((uint64_t)regs.IH_Wptr_Addr_Hi) << 32;
}

void
AMDGPUInterruptHandler::setDoorbellOffset(const uint32_t &data)
{
    regs.IH_Doorbell = data & 0x3ffffff;
}

void
AMDGPUInterruptHandler::updateRptr(const uint32_t &data)
{
    regs.IH_Rptr = data; // update ring buffer rptr offset
}

void
AMDGPUInterruptHandler::serialize(CheckpointOut &cp) const
{
    uint32_t ih_cntl = regs.IH_Cntl;
    uint32_t ih_base = regs.IH_Base;
    uint32_t ih_base_hi = regs.IH_Base_Hi;
    Addr ih_baseAddr = regs.baseAddr;
    uint32_t ih_rptr = regs.IH_Rptr;
    uint32_t ih_wptr = regs.IH_Wptr;
    uint32_t ih_wptr_addr_lo = regs.IH_Wptr_Addr_Lo;
    uint32_t ih_wptr_addr_hi = regs.IH_Wptr_Addr_Hi;
    Addr ih_wptrAddr = regs.WptrAddr;
    uint32_t ih_doorbellOffset = regs.IH_Doorbell;

    SERIALIZE_SCALAR(ih_cntl);
    SERIALIZE_SCALAR(ih_base);
    SERIALIZE_SCALAR(ih_base_hi);
    SERIALIZE_SCALAR(ih_baseAddr);
    SERIALIZE_SCALAR(ih_rptr);
    SERIALIZE_SCALAR(ih_wptr);
    SERIALIZE_SCALAR(ih_wptr_addr_lo);
    SERIALIZE_SCALAR(ih_wptr_addr_hi);
    SERIALIZE_SCALAR(ih_wptrAddr);
    SERIALIZE_SCALAR(ih_doorbellOffset);
}

void
AMDGPUInterruptHandler::unserialize(CheckpointIn &cp)
{
    uint32_t ih_cntl;
    uint32_t ih_base;
    uint32_t ih_base_hi;
    Addr ih_baseAddr;
    uint32_t ih_rptr;
    uint32_t ih_wptr;
    uint32_t ih_wptr_addr_lo;
    uint32_t ih_wptr_addr_hi;
    Addr ih_wptrAddr;
    uint32_t ih_doorbellOffset;

    UNSERIALIZE_SCALAR(ih_cntl);
    UNSERIALIZE_SCALAR(ih_base);
    UNSERIALIZE_SCALAR(ih_base_hi);
    UNSERIALIZE_SCALAR(ih_baseAddr);
    UNSERIALIZE_SCALAR(ih_rptr);
    UNSERIALIZE_SCALAR(ih_wptr);
    UNSERIALIZE_SCALAR(ih_wptr_addr_lo);
    UNSERIALIZE_SCALAR(ih_wptr_addr_hi);
    UNSERIALIZE_SCALAR(ih_wptrAddr);
    UNSERIALIZE_SCALAR(ih_doorbellOffset);

    regs.IH_Cntl = ih_cntl;
    regs.IH_Base = ih_base;
    regs.IH_Base_Hi = ih_base_hi;
    regs.baseAddr = ih_baseAddr;
    regs.IH_Rptr = ih_rptr;
    regs.IH_Wptr = ih_wptr;
    regs.IH_Wptr_Addr_Lo = ih_wptr_addr_lo;
    regs.IH_Wptr_Addr_Hi = ih_wptr_addr_hi;
    regs.WptrAddr = ih_wptrAddr;
    regs.IH_Doorbell = ih_doorbellOffset;
}

} // namespace gem5
