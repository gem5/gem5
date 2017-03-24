/*
 * Copyright (c) 2014, 2017 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#include "dev/virtio/pci.hh"

#include "base/bitfield.hh"
#include "debug/VIOIface.hh"
#include "mem/packet_access.hh"
#include "params/PciVirtIO.hh"

PciVirtIO::PciVirtIO(const Params *params)
    : PciDevice(params), queueNotify(0), interruptDeliveryPending(false),
      vio(*params->vio), callbackKick(this)
{
    // Override the subsystem ID with the device ID from VirtIO
    config.subsystemID = htole(vio.deviceId);

    // The kernel driver expects the BAR size to be an exact power of
    // two. Nothing else is supported. Therefore, we need to force
    // that alignment here. We do not touch vio.configSize as this is
    // used to check accesses later on.
    BARSize[0] = alignToPowerOfTwo(BAR0_SIZE_BASE + vio.configSize);

    vio.registerKickCallback(&callbackKick);
}

PciVirtIO::~PciVirtIO()
{
}

Tick
PciVirtIO::read(PacketPtr pkt)
{
    const unsigned M5_VAR_USED size(pkt->getSize());
    int bar;
    Addr offset;
    if (!getBAR(pkt->getAddr(), bar, offset))
        panic("Invalid PCI memory access to unmapped memory.\n");
    assert(bar == 0);

    DPRINTF(VIOIface, "Reading offset 0x%x [len: %i]\n", offset, size);

    // Forward device configuration writes to the device VirtIO model
    if (offset >= OFF_VIO_DEVICE) {
        vio.readConfig(pkt, offset - OFF_VIO_DEVICE);
        return 0;
    }

    pkt->makeResponse();

    switch(offset) {
      case OFF_DEVICE_FEATURES:
        DPRINTF(VIOIface, "   DEVICE_FEATURES request\n");
        assert(size == sizeof(uint32_t));
        pkt->set<uint32_t>(vio.deviceFeatures);
        break;

      case OFF_GUEST_FEATURES:
        DPRINTF(VIOIface, "   GUEST_FEATURES request\n");
        assert(size == sizeof(uint32_t));
        pkt->set<uint32_t>(vio.getGuestFeatures());
        break;

      case OFF_QUEUE_ADDRESS:
        DPRINTF(VIOIface, "   QUEUE_ADDRESS request\n");
        assert(size == sizeof(uint32_t));
        pkt->set<uint32_t>(vio.getQueueAddress());
        break;

      case OFF_QUEUE_SIZE:
        DPRINTF(VIOIface, "   QUEUE_SIZE request\n");
        assert(size == sizeof(uint16_t));
        pkt->set<uint16_t>(vio.getQueueSize());
        break;

      case OFF_QUEUE_SELECT:
        DPRINTF(VIOIface, "   QUEUE_SELECT\n");
        assert(size == sizeof(uint16_t));
        pkt->set<uint16_t>(vio.getQueueSelect());
        break;

      case OFF_QUEUE_NOTIFY:
        DPRINTF(VIOIface, "   QUEUE_NOTIFY request\n");
        assert(size == sizeof(uint16_t));
        pkt->set<uint16_t>(queueNotify);
        break;

      case OFF_DEVICE_STATUS:
        DPRINTF(VIOIface, "   DEVICE_STATUS request\n");
        assert(size == sizeof(uint8_t));
        pkt->set<uint8_t>(vio.getDeviceStatus());
        break;

      case OFF_ISR_STATUS: {
          DPRINTF(VIOIface, "   ISR_STATUS\n");
          assert(size == sizeof(uint8_t));
          const uint8_t isr_status(interruptDeliveryPending ? 1 : 0);
          if (interruptDeliveryPending) {
              interruptDeliveryPending = false;
              intrClear();
          }
          pkt->set<uint8_t>(isr_status);
      } break;

      default:
        panic("Unhandled read offset (0x%x)\n", offset);
    }

    return 0;
}

Tick
PciVirtIO::write(PacketPtr pkt)
{
    const unsigned M5_VAR_USED size(pkt->getSize());
    int bar;
    Addr offset;
    if (!getBAR(pkt->getAddr(), bar, offset))
        panic("Invalid PCI memory access to unmapped memory.\n");
    assert(bar == 0);

    DPRINTF(VIOIface, "Writing offset 0x%x [len: %i]\n", offset, size);

    // Forward device configuration writes to the device VirtIO model
    if (offset >= OFF_VIO_DEVICE) {
        vio.writeConfig(pkt, offset - OFF_VIO_DEVICE);
        return 0;
    }

    pkt->makeResponse();

    switch(offset) {
      case OFF_DEVICE_FEATURES:
        warn("Guest tried to write device features.");
        break;

      case OFF_GUEST_FEATURES:
        DPRINTF(VIOIface, "   WRITE GUEST_FEATURES request\n");
        assert(size == sizeof(uint32_t));
        vio.setGuestFeatures(pkt->get<uint32_t>());
        break;

      case OFF_QUEUE_ADDRESS:
        DPRINTF(VIOIface, "   WRITE QUEUE_ADDRESS\n");
        assert(size == sizeof(uint32_t));
        vio.setQueueAddress(pkt->get<uint32_t>());
        break;

      case OFF_QUEUE_SIZE:
        panic("Guest tried to write queue size.");
        break;

      case OFF_QUEUE_SELECT:
        DPRINTF(VIOIface, "   WRITE QUEUE_SELECT\n");
        assert(size == sizeof(uint16_t));
        vio.setQueueSelect(pkt->get<uint16_t>());
        break;

      case OFF_QUEUE_NOTIFY:
        DPRINTF(VIOIface, "   WRITE QUEUE_NOTIFY\n");
        assert(size == sizeof(uint16_t));
        queueNotify = pkt->get<uint16_t>();
        vio.onNotify(queueNotify);
        break;

      case OFF_DEVICE_STATUS: {
          assert(size == sizeof(uint8_t));
          uint8_t status(pkt->get<uint8_t>());
          DPRINTF(VIOIface, "VirtIO set status: 0x%x\n", status);
          vio.setDeviceStatus(status);
      } break;

      case OFF_ISR_STATUS:
        warn("Guest tried to write ISR status.");
        break;

      default:
        panic("Unhandled read offset (0x%x)\n", offset);
    }

    return 0;
}

void
PciVirtIO::kick()
{
    DPRINTF(VIOIface, "kick(): Sending interrupt...\n");
    interruptDeliveryPending = true;
    intrPost();
}

PciVirtIO *
PciVirtIOParams::create()
{
    return new PciVirtIO(this);
}
