/*
 * Copyright (c) 2013, 2018-2020, 2024 ARM Limited
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

#ifndef __DEV_ARM_SMMU_V3_HH__
#define __DEV_ARM_SMMU_V3_HH__

#include <list>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "base/statistics.hh"
#include "dev/arm/smmu_v3_caches.hh"
#include "dev/arm/smmu_v3_cmdexec.hh"
#include "dev/arm/smmu_v3_defs.hh"
#include "dev/arm/smmu_v3_deviceifc.hh"
#include "dev/arm/smmu_v3_events.hh"
#include "dev/arm/smmu_v3_ports.hh"
#include "dev/arm/smmu_v3_proc.hh"
#include "mem/packet.hh"
#include "params/SMMUv3.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

/**
 * @file:
 * This is an implementation of the SMMUv3 architecture.
 *
 * What can it do?
 * - Single-stage and nested translation with 4k or 64k granule.  16k would
 *   be straightforward to add.
 * - Large pages are supported.
 * - Works with any gem5 device as long as it is issuing packets with a
 *   valid (Sub)StreamId
 *
 * What it can't do?
 * - Fragment stage 1 page when the underlying stage 2 page is smaller.  S1
 *   page size > S2 page size is not supported
 * - Invalidations take zero time. This wouldn't be hard to fix.
 * - Checkpointing is not supported
 * - Stall/resume for faulting transactions is not supported
 */

namespace gem5
{

class ArmInterruptPin;

class SMMUTranslationProcess;

class SMMUv3 : public ClockedObject
{
  protected:
    friend class SMMUProcess;
    friend class SMMUTranslationProcess;
    friend class SMMUCommandExecProcess;
    friend class SMMUv3DeviceInterface;

    const System &system;
    const RequestorID requestorId;

    SMMURequestPort requestPort;
    SMMUTableWalkPort tableWalkPort;
    SMMUControlPort controlPort;

    // This could be nullptr if wired implementation of the
    // event queue interrupt is not supported
    ArmInterruptPin *const eventqInterrupt;

    ARMArchTLB tlb;
    ConfigCache configCache;
    IPACache ipaCache;
    WalkCache walkCache;

    const bool tlbEnable;
    const bool configCacheEnable;
    const bool ipaCacheEnable;
    const bool walkCacheEnable;
    bool tableWalkPortEnable;

    const bool walkCacheNonfinalEnable;
    const unsigned walkCacheS1Levels;
    const unsigned walkCacheS2Levels;
    const unsigned requestPortWidth; // in bytes

    SMMUSemaphore tlbSem;
    SMMUSemaphore ifcSmmuSem;
    SMMUSemaphore smmuIfcSem;
    SMMUSemaphore configSem;
    SMMUSemaphore ipaSem;
    SMMUSemaphore walkSem;
    SMMUSemaphore requestPortSem;

    SMMUSemaphore transSem; // max N transactions in SMMU
    SMMUSemaphore ptwSem;   // max N concurrent PTWs
    SMMUSemaphore cycleSem; // max 1 table walk per cycle

    // Timing parameters
    const Cycles tlbLat;
    const Cycles ifcSmmuLat;
    const Cycles smmuIfcLat;
    const Cycles configLat;
    const Cycles ipaLat;
    const Cycles walkLat;

    // Stats
    struct SMMUv3Stats : public statistics::Group
    {
        SMMUv3Stats(statistics::Group *parent);
        statistics::Scalar steL1Fetches;
        statistics::Scalar steFetches;
        statistics::Scalar cdL1Fetches;
        statistics::Scalar cdFetches;
        statistics::Distribution translationTimeDist;
        statistics::Distribution ptwTimeDist;
    } stats;

    std::vector<SMMUv3DeviceInterface *> deviceInterfaces;

    SMMUCommandExecProcess commandExecutor;

    const AddrRange regsMap;
    SMMURegs regs;

    bool inSecureBlock(uint32_t offs) const;

    std::queue<SMMUAction> packetsToRetry;
    std::queue<SMMUAction> packetsTableWalkToRetry;

    void scheduleDeviceRetries();

    SMMUAction runProcess(SMMUProcess *proc, PacketPtr pkt);
    SMMUAction runProcessAtomic(SMMUProcess *proc, PacketPtr pkt);
    SMMUAction runProcessTiming(SMMUProcess *proc, PacketPtr pkt);

    void processCommands();
    MemberEventWrapper<&SMMUv3::processCommands> processCommandsEvent;

    void processCommand(const SMMUCommand &cmd);

  public:
    SMMUv3(const SMMUv3Params &p);

    virtual ~SMMUv3() {}

    virtual void init() override;

    Tick recvAtomic(PacketPtr pkt, PortID id);
    bool recvTimingReq(PacketPtr pkt, PortID id);
    bool recvTimingResp(PacketPtr pkt);
    void recvReqRetry();

    bool tableWalkRecvTimingResp(PacketPtr pkt);
    void tableWalkRecvReqRetry();

    Tick readControl(PacketPtr pkt);
    Tick writeControl(PacketPtr pkt);

    DrainState drain() override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    virtual Port &getPort(const std::string &name,
                          PortID id = InvalidPortID) override;
};

} // namespace gem5

#endif /* __DEV_ARM_SMMU_V3_HH__ */
