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

#ifndef __DEV_ARM_SMMU_V3_PORTS_HH__
#define __DEV_ARM_SMMU_V3_PORTS_HH__

#include "mem/qport.hh"
#include "mem/tport.hh"

namespace gem5
{

class SMMUv3;
class SMMUv3DeviceInterface;

class SMMURequestPort : public RequestPort
{
  protected:
    SMMUv3 &smmu;

    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();

  public:
    SMMURequestPort(const std::string &_name, SMMUv3 &_smmu);

    virtual ~SMMURequestPort() {}
};

// Separate request port to send MMU initiated requests on
class SMMUTableWalkPort : public RequestPort
{
  protected:
    SMMUv3 &smmu;

    virtual bool recvTimingResp(PacketPtr pkt);
    virtual void recvReqRetry();

  public:
    SMMUTableWalkPort(const std::string &_name, SMMUv3 &_smmu);

    virtual ~SMMUTableWalkPort() {}
};

class SMMUDevicePort : public QueuedResponsePort
{
  protected:
    SMMUv3DeviceInterface &ifc;
    RespPacketQueue respQueue;

    virtual void recvFunctional(PacketPtr pkt);
    virtual Tick recvAtomic(PacketPtr pkt);
    virtual bool recvTimingReq(PacketPtr pkt);

  public:
    SMMUDevicePort(const std::string &_name, SMMUv3DeviceInterface &_ifc,
                   PortID _id = InvalidPortID);

    virtual ~SMMUDevicePort() {}

    virtual AddrRangeList
    getAddrRanges() const
    {
        return AddrRangeList{ AddrRange(0, UINT64_MAX) };
    }
};

class SMMUControlPort : public SimpleTimingPort
{
  protected:
    SMMUv3 &smmu;
    AddrRange addrRange;

    virtual Tick recvAtomic(PacketPtr pkt);
    virtual AddrRangeList getAddrRanges() const;

  public:
    SMMUControlPort(const std::string &_name, SMMUv3 &_smmu,
                    AddrRange _addrRange);

    virtual ~SMMUControlPort() {}
};

class SMMUATSMemoryPort : public QueuedRequestPort
{
  protected:
    SMMUv3DeviceInterface &ifc;
    ReqPacketQueue reqQueue;
    SnoopRespPacketQueue snoopRespQueue;

    virtual bool recvTimingResp(PacketPtr pkt);

  public:
    SMMUATSMemoryPort(const std::string &_name, SMMUv3DeviceInterface &_ifc);

    virtual ~SMMUATSMemoryPort() {}
};

class SMMUATSDevicePort : public QueuedResponsePort
{
  protected:
    SMMUv3DeviceInterface &ifc;
    RespPacketQueue respQueue;

    virtual void recvFunctional(PacketPtr pkt);
    virtual Tick recvAtomic(PacketPtr pkt);
    virtual bool recvTimingReq(PacketPtr pkt);

    virtual AddrRangeList
    getAddrRanges() const
    {
        return AddrRangeList();
    }

  public:
    SMMUATSDevicePort(const std::string &_name, SMMUv3DeviceInterface &_ifc);

    virtual ~SMMUATSDevicePort() {}
};

} // namespace gem5

#endif /* __DEV_ARM_SMMU_V3_PORTS_HH__ */
