/*
 * Copyright (c) 2026 Kuan-Wei Chiu <visitorckw@gmail.com>
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

#ifndef __DEV_VIRTIO_RTC_HH__
#define __DEV_VIRTIO_RTC_HH__

#include <chrono>
#include <cstdint>
#include <string>

#include "base/compiler.hh"
#include "dev/virtio/base.hh"

namespace gem5
{

struct VirtIORtcParams;

/**
 * VirtIO RTC
 *
 * @see https://github.com/rustyrussell/virtio-spec
 * @see http://docs.oasis-open.org/virtio/virtio/v1.0/virtio-v1.0.html
 */
class VirtIORtc : public VirtIODeviceBase
{
  public:
    typedef VirtIORtcParams Params;
    VirtIORtc(const Params &params);
    virtual ~VirtIORtc();

    void readConfig(PacketPtr pkt, Addr cfgOffset);

  protected:
    /** VirtIO device ID */
    static const DeviceId ID_RTC = 0x11;

    enum MsgType : uint16_t
    {
        VIO_RTC_REQ_READ = 0x0001,
        VIO_RTC_REQ_READ_CROSS = 0x0002,
        VIO_RTC_REQ_CFG = 0x1000,
        VIO_RTC_REQ_CLOCK_CAP = 0x1001,
        VIO_RTC_REQ_CROSS_CAP = 0x1002,
        VIO_RTC_REQ_READ_ALARM = 0x1003,
        VIO_RTC_REQ_SET_ALARM = 0x1004,
        VIO_RTC_REQ_SET_ALARM_ENABLED = 0x1005,
    };

    enum Status : uint8_t
    {
        VIO_RTC_S_OK = 0,
        VIO_RTC_S_EOPNOTSUPP = 2,
        VIO_RTC_S_ENODEV = 3,
        VIO_RTC_S_EINVAL = 4,
        VIO_RTC_S_EIO = 5,
    };

    enum ClockType : uint8_t
    {
        VIO_RTC_CLOCK_UTC = 0,
        VIO_RTC_CLOCK_TAI = 1,
        VIO_RTC_CLOCK_MONOTONIC = 2,
        VIO_RTC_CLOCK_UTC_SMEARED = 3,
        VIO_RTC_CLOCK_UTC_MAYBE_SMEARED = 4,
    };

    enum SmearType : uint8_t
    {
        VIO_RTC_SMEAR_UNSPECIFIED = 0,
        VIO_RTC_SMEAR_NOON_LINEAR = 1,
        VIO_RTC_SMEAR_UTC_SLS = 2,
    };

    struct GEM5_PACKED ReqHead
    {
        uint16_t msg_type;
        uint8_t reserved[6];
    };

    struct GEM5_PACKED RespHead
    {
        uint8_t status;
        uint8_t reserved[7];
    };

    struct GEM5_PACKED ReqRead
    {
        ReqHead head;
        uint16_t clock_id;
        uint8_t reserved[6];
    };

    struct GEM5_PACKED RespRead
    {
        RespHead head;
        uint64_t clock_reading;
    };

    struct GEM5_PACKED ReqCfg
    { ReqHead head; };

    struct GEM5_PACKED RespCfg
    {
        RespHead head;
        uint16_t num_clocks;
        uint8_t reserved[6];
    };

    struct GEM5_PACKED ReqClockCap
    {
        ReqHead head;
        uint16_t clock_id;
        uint8_t reserved[6];
    };

    struct GEM5_PACKED RespClockCap
    {
        RespHead head;
        uint8_t type;
        uint8_t leap_second_smearing;
        uint8_t flags;
        uint8_t reserved[5];
    };

    struct GEM5_PACKED ReqCrossCap
    {
        ReqHead head;
        uint16_t clock_id;
        uint8_t hw_counter;
        uint8_t reserved[5];
    };

    struct GEM5_PACKED RespCrossCap
    {
        RespHead head;
        uint8_t flags;
        uint8_t reserved[7];
    };

  protected:
    /**
     * Virtqueue for RTC requests from the guest.
     */
    class RtcQueue : public VirtQueue
    {
      public:
        RtcQueue(PortProxy &proxy, ByteOrder bo, uint16_t size,
                 VirtIORtc &_parent);
        virtual ~RtcQueue() {}

        void
        onNotify()
        { tryProcess(); }

        /** Try to process pending requests in the virtqueue. */
        void tryProcess();

        std::string
        name() const
        { return parent.name() + ".qReq"; }

      protected:
        VirtIORtc &parent;
    };
    /** Request queue */
    RtcQueue qReq;

    uint64_t getNanoseconds() const;
};

} // namespace gem5

#endif // __DEV_VIRTIO_RTC_HH__
