/*
 * Copyright (c) 2021 Huawei International
 * Copyright (c) 2016-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dev/riscv/vio_mmio.hh"

#include "debug/VirtIOMMIO.hh"
#include "dev/riscv/hifive.hh"
#include "mem/packet_access.hh"
#include "params/RiscvMmioVirtIO.hh"

namespace gem5
{

namespace RiscvISA
{

MmioVirtIO::MmioVirtIO(const RiscvMmioVirtIOParams &params)
    : PlicIntDevice(params),
      hostFeaturesSelect(0),
      guestFeaturesSelect(0),
      pageSize(0),
      interruptStatus(0),
      vio(*params.vio)
{
    vio.registerKickCallback([this]() { kick(); });
}

MmioVirtIO::~MmioVirtIO() {}

Tick
MmioVirtIO::read(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pioAddr;
    const unsigned size(pkt->getSize());

    DPRINTF(VirtIOMMIO, "Reading %u bytes @ 0x%x:\n", size, offset);

    // Forward device configuration writes to the device VirtIO model
    if (offset >= OFF_CONFIG) {
        vio.readConfig(pkt, offset - OFF_CONFIG);
        return 0;
    }
    panic_if(size != 4, "Unexpected read size: %u\n", size);

    const uint32_t value = read(offset);
    DPRINTF(VirtIOMMIO, "    value: 0x%x\n", value);
    pkt->makeResponse();
    pkt->setLE<uint32_t>(value);

    return 0;
}

uint32_t
MmioVirtIO::read(Addr offset)
{
    switch (offset) {
    case OFF_MAGIC:
        return MAGIC;

    case OFF_VERSION:
        return VERSION;

    case OFF_DEVICE_ID:
        return vio.deviceId;

    case OFF_VENDOR_ID:
        return VENDOR_ID;

    case OFF_HOST_FEATURES:
        // We only implement 32 bits of this register
        if (hostFeaturesSelect == 0)
            return vio.deviceFeatures;
        else
            return 0;

    case OFF_HOST_FEATURES_SELECT:
        return hostFeaturesSelect;

    case OFF_GUEST_FEATURES:
        // We only implement 32 bits of this register
        if (guestFeaturesSelect == 0)
            return vio.getGuestFeatures();
        else
            return 0;

    case OFF_GUEST_FEATURES_SELECT:
        return hostFeaturesSelect;

    case OFF_GUEST_PAGE_SIZE:
        return pageSize;

    case OFF_QUEUE_SELECT:
        return vio.getQueueSelect();

    case OFF_QUEUE_NUM_MAX:
        return vio.getQueueSize();

    case OFF_QUEUE_NUM:
        // TODO: We don't support queue resizing, so ignore this for now.
        return vio.getQueueSize();

    case OFF_QUEUE_ALIGN:
        // TODO: Implement this once we support other alignment sizes
        return VirtQueue::ALIGN_SIZE;

    case OFF_QUEUE_PFN:
        return vio.getQueueAddress();

    case OFF_INTERRUPT_STATUS:
        return interruptStatus;

    case OFF_STATUS:
        return vio.getDeviceStatus();

        // Write-only registers
    case OFF_QUEUE_NOTIFY:
    case OFF_INTERRUPT_ACK:
        warn("Guest is trying to read to write-only register 0x%\n", offset);
        return 0;

    default:
        panic("Unhandled read offset (0x%x)\n", offset);
    }
}

Tick
MmioVirtIO::write(PacketPtr pkt)
{
    const Addr offset = pkt->getAddr() - pioAddr;
    const unsigned size(pkt->getSize());

    DPRINTF(VirtIOMMIO, "Writing %u bytes @ 0x%x:\n", size, offset);

    // Forward device configuration writes to the device VirtIO model
    if (offset >= OFF_CONFIG) {
        vio.writeConfig(pkt, offset - OFF_CONFIG);
        return 0;
    }

    panic_if(size != 4, "Unexpected write size @ 0x%x: %u\n", offset, size);
    DPRINTF(VirtIOMMIO, "    value: 0x%x\n", pkt->getLE<uint32_t>());
    pkt->makeResponse();
    write(offset, pkt->getLE<uint32_t>());
    return 0;
}

void
MmioVirtIO::write(Addr offset, uint32_t value)
{
    switch (offset) {
    case OFF_HOST_FEATURES_SELECT:
        hostFeaturesSelect = value;
        return;

    case OFF_GUEST_FEATURES:
        if (guestFeaturesSelect == 0) {
            vio.setGuestFeatures(value);
        } else if (value != 0) {
            warn("Setting unimplemented guest features register %u: %u\n",
                 guestFeaturesSelect, value);
        }
        return;

    case OFF_GUEST_FEATURES_SELECT:
        guestFeaturesSelect = value;
        return;

    case OFF_GUEST_PAGE_SIZE:
        // TODO: We only support 4096 byte pages at the moment
        panic_if(value != VirtQueue::ALIGN_SIZE,
                 "Unhandled VirtIO page size: %u", value);
        pageSize = value;
        return;

    case OFF_QUEUE_SELECT:
        vio.setQueueSelect(value);
        return;

    case OFF_QUEUE_NUM:
        // TODO: We don't support queue resizing, so ignore this for now.
        warn_once("Ignoring queue resize hint. Requested size: %u\n", value);
        return;

    case OFF_QUEUE_ALIGN:
        // TODO: We currently only support the hard-coded 4k alignment used
        // in legacy VirtIO.
        panic_if(value != VirtQueue::ALIGN_SIZE,
                 "Unhandled VirtIO alignment size: %u", value);
        return;

    case OFF_QUEUE_PFN:
        vio.setQueueAddress(value);
        return;

    case OFF_QUEUE_NOTIFY:
        vio.onNotify(value);
        return;

    case OFF_INTERRUPT_ACK:
        setInterrupts(interruptStatus & (~value));
        return;

    case OFF_STATUS:
        panic_if(value > 0xff, "Unexpected status: 0x%x\n", value);
        vio.setDeviceStatus(value);
        return;

        /* Read-only registers */
    case OFF_MAGIC:
    case OFF_VERSION:
    case OFF_DEVICE_ID:
    case OFF_VENDOR_ID:
    case OFF_HOST_FEATURES:
    case OFF_QUEUE_NUM_MAX:
    case OFF_INTERRUPT_STATUS:
        warn("Guest is trying to write to read-only register 0x%\n", offset);
        return;

    default:
        panic("Unhandled read offset (0x%x)\n", offset);
    }
}

void
MmioVirtIO::kick()
{
    DPRINTF(VirtIOMMIO, "kick(): Sending interrupt...\n");
    setInterrupts(interruptStatus | INT_USED_RING);
}

void
MmioVirtIO::setInterrupts(uint32_t value)
{
    const uint32_t old_ints = interruptStatus;
    interruptStatus = value;

    if (!old_ints && interruptStatus) {
        platform->postPciInt(_interruptID);
    } else if (old_ints && !interruptStatus) {
        platform->clearPciInt(_interruptID);
    }
}

} // namespace RiscvISA

} // namespace gem5
