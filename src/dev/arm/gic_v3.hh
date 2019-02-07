/*
 * Copyright (c) 2018 Metempsy Technology Consulting
 * All rights reserved.
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
 * Authors: Jairo Balart
 */

#ifndef __DEV_ARM_GICV3_H__
#define __DEV_ARM_GICV3_H__

#include "dev/arm/base_gic.hh"
#include "params/Gicv3.hh"

class Gicv3CPUInterface;
class Gicv3Distributor;
class Gicv3Redistributor;

class Gicv3 : public BaseGic
{
  protected:

    typedef Gicv3Params Params;
    Gicv3Distributor * distributor;
    std::vector<Gicv3Redistributor *> redistributors;
    std::vector<Gicv3CPUInterface *> cpuInterfaces;
    AddrRange distRange;
    AddrRange redistRange;
    AddrRangeList addrRanges;

  public:

    // Special interrupt IDs, as per SPEC 2.2.1 section
    static const int INTID_SECURE = 1020;
    static const int INTID_NONSECURE = 1021;
    static const int INTID_SPURIOUS = 1023;

    // Number of Software Generated Interrupts
    static const int SGI_MAX = 16;
    // Number of Private Peripheral Interrupts
    static const int PPI_MAX = 16;

    // Interrupt states for PPIs, SGIs and SPIs, as per SPEC 4.1.2 section
    typedef enum {
        INT_INACTIVE,
        INT_PENDING,
        INT_ACTIVE,
        INT_ACTIVE_PENDING,
    } IntStatus;

    // Interrupt groups, as per SPEC section 4.6
    typedef enum {
        G0S,
        G1S,
        G1NS,
    } GroupId;

    typedef enum {
        INT_LEVEL_SENSITIVE,
        INT_EDGE_TRIGGERED,
    } IntTriggerType;

  protected:

    void clearInt(uint32_t int_id) override;
    void clearPPInt(uint32_t int_id, uint32_t cpu) override;

    inline AddrRangeList
    getAddrRanges() const override
    {
        return addrRanges;
    }

    void init() override;
    void initState() override;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Tick read(PacketPtr pkt) override;
    void reset();
    void sendInt(uint32_t int_id) override;
    void sendPPInt(uint32_t int_id, uint32_t cpu) override;
    void serialize(CheckpointOut & cp) const override;
    void unserialize(CheckpointIn & cp) override;
    Tick write(PacketPtr pkt) override;

  public:

    Gicv3(const Params * p);
    void deassertInt(uint32_t cpu, ArmISA::InterruptTypes int_type);

    inline Gicv3CPUInterface *
    getCPUInterface(int cpu_id) const
    {
        assert(cpu_id < cpuInterfaces.size() and cpuInterfaces[cpu_id]);
        return cpuInterfaces[cpu_id];
    }

    inline Gicv3Distributor *
    getDistributor() const
    {
        return distributor;
    }

    inline Gicv3Redistributor *
    getRedistributor(ContextID context_id) const
    {
        assert(context_id < redistributors.size() and
               redistributors[context_id]);
        return redistributors[context_id];
    }

    Gicv3Redistributor *
    getRedistributorByAffinity(uint32_t affinity) const;
    void postInt(uint32_t cpu, ArmISA::InterruptTypes int_type);
};

#endif //__DEV_ARM_GICV3_H__
