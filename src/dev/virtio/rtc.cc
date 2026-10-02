/*
 * Copyright (c) 2026 Kuan-Wei Chiu <visitorckw@gmail.com>
 * All rights reserved
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

#include <algorithm>
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
{
    registerQueue(qReq);
}

VirtIORtc::~VirtIORtc()
{}

VirtIORtc::RtcQueue::RtcQueue(PortProxy &proxy, ByteOrder bo, uint16_t size,
                              VirtIORtc &_parent)
    : VirtQueue(proxy, bo, size), parent(_parent)
{}

void
VirtIORtc::readConfig(PacketPtr pkt, Addr cfgOffset)
{
    // There is no configuration space for the RTC device
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

        size_t inSize = 0;
        size_t outSize = 0;
        const VirtDescriptor *c = d;
        for (; c && c->isIncoming(); c = c->next()) {
            inSize += c->size();
        }
        for (; c && c->isOutgoing(); c = c->next()) {
            outSize += c->size();
        }

        uint16_t msgType = 0;
        if (!c && inSize >= sizeof(ReqHead)) {
            ReqHead reqHead;
            d->chainRead(0, (uint8_t *)&reqHead, sizeof(reqHead));
            msgType = gtoh(reqHead.msg_type, byteOrder);
            DPRINTF(VIORtc, "Request msg_type: 0x%x\n", msgType);
        }

        size_t offResp = inSize;
        size_t respLen = 0;
        uint8_t errStatus = VIO_RTC_S_EINVAL;

        switch (msgType) {
            case VIO_RTC_REQ_CFG: {
                if (outSize < sizeof(RespCfg)) {
                    break;
                }

                RespCfg resp;
                std::memset(&resp, 0, sizeof(resp));
                resp.head.status = VIO_RTC_S_OK;
                resp.num_clocks = htog((uint16_t)1, byteOrder);

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            case VIO_RTC_REQ_CLOCK_CAP: {
                if (inSize < sizeof(ReqClockCap) ||
                    outSize < sizeof(RespClockCap)) {
                    break;
                }

                ReqClockCap req;
                d->chainRead(0, (uint8_t *)&req, sizeof(req));
                uint16_t clkId = gtoh(req.clock_id, byteOrder);

                RespClockCap resp;
                std::memset(&resp, 0, sizeof(resp));

                if (clkId == 0) {
                    resp.head.status = VIO_RTC_S_OK;
                    // Report smeared UTC so the Linux virtio-rtc driver
                    // registers an RTC class device.
                    resp.type = VIO_RTC_CLOCK_UTC_SMEARED;
                    resp.leap_second_smearing = VIO_RTC_SMEAR_UNSPECIFIED;
                    resp.flags = 0;
                } else {
                    resp.head.status = VIO_RTC_S_ENODEV;
                }

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            case VIO_RTC_REQ_CROSS_CAP: {
                if (inSize < sizeof(ReqCrossCap) ||
                    outSize < sizeof(RespCrossCap)) {
                    break;
                }

                ReqCrossCap req;
                d->chainRead(0, (uint8_t *)&req, sizeof(req));
                uint16_t clkId = gtoh(req.clock_id, byteOrder);

                RespCrossCap resp;
                std::memset(&resp, 0, sizeof(resp));

                if (clkId != 0) {
                    resp.head.status = VIO_RTC_S_ENODEV;
                } else if (req.hw_counter == VIO_RTC_COUNTER_ARM_VCT ||
                           req.hw_counter == VIO_RTC_COUNTER_X86_TSC) {
                    resp.head.status = VIO_RTC_S_OK;
                    resp.flags = 0;
                } else {
                    resp.head.status = VIO_RTC_S_EOPNOTSUPP;
                }

                d->chainWrite(offResp, (const uint8_t *)&resp, sizeof(resp));
                respLen = sizeof(resp);
                break;
            }

            case VIO_RTC_REQ_READ: {
                if (inSize < sizeof(ReqRead) || outSize < sizeof(RespRead)) {
                    break;
                }

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
                if (!c && inSize >= sizeof(ReqHead)) {
                    warn("VirtIORtc: Unsupported request type 0x%x\n",
                         msgType);
                    errStatus = VIO_RTC_S_EOPNOTSUPP;
                }
                break;
            }
        }

        if (respLen == 0 && outSize > 0) {
            RespHead resp;
            std::memset(&resp, 0, sizeof(resp));
            resp.status = errStatus;
            respLen = std::min(outSize, sizeof(resp));
            d->chainWrite(offResp, (const uint8_t *)&resp, respLen);
        }

        // Tell the guest that we are done with this descriptor.
        produceDescriptor(d, respLen);
        parent.kick();
    }
}

} // namespace gem5
