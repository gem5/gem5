/*
 * Copyright (c) 2014 ARM Limited
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

#ifndef __DEV_VIRTIO_PCI_HH__
#define __DEV_VIRTIO_PCI_HH__

#include "base/statistics.hh"
#include "dev/virtio/base.hh"
#include "dev/pci/device.hh"

struct PciVirtIOParams;

class PciVirtIO : public PciDevice
{
  public:
    typedef PciVirtIOParams Params;
    PciVirtIO(const Params *params);
    virtual ~PciVirtIO();

    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);

    void kick();

  protected:
    /** @{ */
    /** Offsets into VirtIO header (BAR0 relative). */

    static const Addr OFF_DEVICE_FEATURES = 0x00;
    static const Addr OFF_GUEST_FEATURES = 0x04;
    static const Addr OFF_QUEUE_ADDRESS = 0x08;
    static const Addr OFF_QUEUE_SIZE = 0x0C;
    static const Addr OFF_QUEUE_SELECT = 0x0E;
    static const Addr OFF_QUEUE_NOTIFY = 0x10;
    static const Addr OFF_DEVICE_STATUS = 0x12;
    static const Addr OFF_ISR_STATUS = 0x13;
    static const Addr OFF_VIO_DEVICE = 0x14;

    /** @} */

    static const Addr BAR0_SIZE_BASE = OFF_VIO_DEVICE;


    VirtIODeviceBase::QueueID queueNotify;

    bool interruptDeliveryPending;

    VirtIODeviceBase &vio;

    MakeCallback<PciVirtIO, &PciVirtIO::kick> callbackKick;
};

#endif // __DEV_VIRTIO_PCI_HH__
