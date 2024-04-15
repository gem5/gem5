/*
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

#ifndef __DEV_ARM_VIO_MMIO_HH__
#define __DEV_ARM_VIO_MMIO_HH__

#include "dev/io_device.hh"
#include "dev/virtio/base.hh"

namespace gem5
{

class ArmInterruptPin;
struct MmioVirtIOParams;

class MmioVirtIO : public BasicPioDevice
{
  public:
    MmioVirtIO(const MmioVirtIOParams &params);
    virtual ~MmioVirtIO();

  protected: // BasicPioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  protected:
    /** @{ */
    /** Offsets into VirtIO MMIO space. */

    enum : Addr
    {
        OFF_MAGIC = 0x00,
        OFF_VERSION = 0x04,
        OFF_DEVICE_ID = 0x08,
        OFF_VENDOR_ID = 0x0C,
        OFF_HOST_FEATURES = 0x10,
        OFF_HOST_FEATURES_SELECT = 0x14,
        OFF_GUEST_FEATURES = 0x20,
        OFF_GUEST_FEATURES_SELECT = 0x24,
        OFF_GUEST_PAGE_SIZE = 0x28,
        OFF_QUEUE_SELECT = 0x30,
        OFF_QUEUE_NUM_MAX = 0x34,
        OFF_QUEUE_NUM = 0x38,
        OFF_QUEUE_ALIGN = 0x3C,
        OFF_QUEUE_PFN = 0x40,
        OFF_QUEUE_NOTIFY = 0x50,
        OFF_INTERRUPT_STATUS = 0x60,
        OFF_INTERRUPT_ACK = 0x64,
        OFF_STATUS = 0x70,
        OFF_CONFIG = 0x100,
    };

    /** @} */

    enum
    {
        INT_USED_RING = 1 << 0,
        INT_CONFIG = 1 << 1,
    };

    static const uint32_t MAGIC = 0x74726976;
    static const uint32_t VERSION = 1;
    static const uint32_t VENDOR_ID = 0x1AF4;

    uint32_t read(Addr offset);
    void write(Addr offset, uint32_t value);

    void kick();
    void setInterrupts(uint32_t value);

    uint32_t hostFeaturesSelect;
    uint32_t guestFeaturesSelect;
    uint32_t pageSize;
    uint32_t interruptStatus;

  protected: // Params
    VirtIODeviceBase &vio;
    ArmInterruptPin *const interrupt;
};

} // namespace gem5

#endif // __DEV_ARM_VIO_MMIO_HH__
