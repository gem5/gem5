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

#include "dev/virtio/rtc.hh"

#include <cstring>

#include "base/trace.hh"
#include "debug/VIORtc.hh"
#include "params/VirtIORtc.hh"
#include "sim/system.hh"

namespace gem5
{

VirtIORtc::VirtIORtc(const Params &params)
    : VirtIODeviceBase(params, ID_RTC, 0, 0),
      qReq(params.system->physProxy, byteOrder, params.qSize, *this)
{ registerQueue(qReq); }

VirtIORtc::~VirtIORtc()
{}

VirtIORtc::RtcQueue::RtcQueue(PortProxy &proxy, ByteOrder bo, uint16_t size,
                              VirtIORtc &_parent)
    : VirtQueue(proxy, bo, size), parent(_parent)
{}

void
VirtIORtc::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    // There are no configuration for RTC device
    pkt->makeResponse();
}

uint64_t
VirtIORtc::getNanoseconds() const
{
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

void
VirtIORtc::RtcQueue::tryProcess()
{
    DPRINTF(VIORtc, "try process\n");

    VirtDescriptor *d;
    while ((d = consumeDescriptor())) {
        DPRINTF(VIORtc, "Got descriptor (len: %i)\n", d->size());

        ReqHead reqHead;
        d->chainRead(0, (uint8_t *)&reqHead, sizeof(reqHead));
        uint16_t msgType = gtoh(reqHead.msg_type, byteOrder);

        DPRINTF(VIORtc, "Request msg_type: 0x%x\n", msgType);

        size_t offResp = d->size();
        size_t respLen = 0;

        switch (msgType) {
            case VIO_RTC_REQ_CFG: {
                RespCfg resp;
                std::memset(&resp, 0, sizeof(resp));
                resp.head.status = VIO_RTC_S_OK;
                resp.num_clocks = htog((uint16_t)1, byteOrder);

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            case VIO_RTC_REQ_CLOCK_CAP: {
                ReqClockCap req;
                d->chainRead(0, (uint8_t *)&req, sizeof(req));
                uint16_t clkId = gtoh(req.clock_id, byteOrder);

                RespClockCap resp;
                std::memset(&resp, 0, sizeof(resp));

                if (clkId == 0) {
                    resp.head.status = VIO_RTC_S_OK;
                    resp.type = VIO_RTC_CLOCK_UTC_SMEARED;
                    resp.leap_second_smearing = VIO_RTC_SMEAR_NOON_LINEAR;
                    resp.flags = 0;
                } else {
                    resp.head.status = VIO_RTC_S_ENODEV;
                }

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            case VIO_RTC_REQ_CROSS_CAP: {
                RespCrossCap resp;
                std::memset(&resp, 0, sizeof(resp));
                resp.head.status = VIO_RTC_S_OK;
                resp.flags = 0;

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            case VIO_RTC_REQ_READ: {
                ReqRead req;
                d->chainRead(0, (uint8_t *)&req, sizeof(req));
                uint16_t clkId = gtoh(req.clock_id, byteOrder);

                RespRead resp;
                std::memset(&resp, 0, sizeof(resp));

                if (clkId == 0) {
                    resp.head.status = VIO_RTC_S_OK;
                    uint64_t ns = parent.getNanoseconds();
                    resp.clock_reading = htog(ns, byteOrder);
                    DPRINTF(VIORtc, "Read clock %d: %llu ns\n", clkId,
                            (unsigned long long)ns);
                } else {
                    resp.head.status = VIO_RTC_S_ENODEV;
                }

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            default: {
                warn("VirtIORtc: Unsupported request type 0x%x\n", msgType);
                RespHead resp;
                std::memset(&resp, 0, sizeof(resp));
                resp.status = VIO_RTC_S_EOPNOTSUPP;

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }
        }

        // Tell the guest that we are done with this descriptor.
        produceDescriptor(d, respLen);
        parent.kick();
    }
}

} // namespace gem5
