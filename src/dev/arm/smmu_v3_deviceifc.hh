/*
 * Copyright (c) 2013, 2018-2019 ARM Limited
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

#ifndef __DEV_ARM_SMMU_V3_DEVICEIFC_HH__
#define __DEV_ARM_SMMU_V3_DEVICEIFC_HH__

#include <list>

#include "dev/arm/smmu_v3_caches.hh"
#include "dev/arm/smmu_v3_defs.hh"
#include "dev/arm/smmu_v3_events.hh"
#include "dev/arm/smmu_v3_ports.hh"
#include "dev/arm/smmu_v3_proc.hh"
#include "params/SMMUv3DeviceInterface.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

class SMMUTranslationProcess;
class SMMUv3;
class SMMUDevicePort;

class SMMUv3DeviceInterface : public ClockedObject
{
  protected:
    friend class SMMUTranslationProcess;

  public:
    SMMUv3 *smmu;
    SMMUTLB* microTLB;
    SMMUTLB* mainTLB;

    const bool microTLBEnable;
    const bool mainTLBEnable;

    SMMUSemaphore devicePortSem;
    SMMUSemaphore microTLBSem;
    SMMUSemaphore mainTLBSem;

    const Cycles microTLBLat;
    const Cycles mainTLBLat;

    SMMUDevicePort *devicePort;
    SMMUATSDevicePort  atsDevicePort;
    SMMUATSMemoryPort atsMemPort;

    // in bytes
    const unsigned portWidth;

    unsigned wrBufSlotsRemaining;
    unsigned xlateSlotsRemaining;
    unsigned pendingMemAccesses;

    const bool prefetchEnable;
    const bool prefetchReserveLastWay;

    std::list<SMMUTranslationProcess *> duplicateReqs;
    SMMUSignal duplicateReqRemoved;

    std::list<SMMUTranslationProcess *> dependentReads[SMMU_MAX_TRANS_ID];
    std::list<SMMUTranslationProcess *> dependentWrites[SMMU_MAX_TRANS_ID];
    SMMUSignal dependentReqRemoved;

    // Receiving translation requests from the requestor device
    Tick recvAtomic(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
    void schedTimingResp(PacketPtr pkt);

    Tick atsRecvAtomic(PacketPtr pkt);
    bool atsRecvTimingReq(PacketPtr pkt);
    bool atsRecvTimingResp(PacketPtr pkt);
    void schedAtsTimingResp(PacketPtr pkt);

    void scheduleDeviceRetry();
    void sendDeviceRetry();
    void atsSendDeviceRetry();

    bool deviceNeedsRetry;
    bool atsDeviceNeedsRetry;

    SMMUDeviceRetryEvent sendDeviceRetryEvent;
    EventWrapper<
        SMMUv3DeviceInterface,
        &SMMUv3DeviceInterface::atsSendDeviceRetry> atsSendDeviceRetryEvent;

    Port& getPort(const std::string &name, PortID id) override;

  public:
    PARAMS(SMMUv3DeviceInterface);
    SMMUv3DeviceInterface(const Params &p);

    ~SMMUv3DeviceInterface()
    {
        delete microTLB;
        delete mainTLB;
    }

    DrainState drain() override;

    void setSMMU(SMMUv3 *_smmu) { smmu = _smmu; }
    void sendRange();
};

} // namespace gem5

#endif /* __DEV_ARM_SMMU_V3_DEVICEIFC_HH__ */
