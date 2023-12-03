/*
 * Copyright (c) 2019, 2022 Arm Limited
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
 */

#ifndef __DEV_ARM_GICV3_H__
#define __DEV_ARM_GICV3_H__

#include "arch/arm/interrupts.hh"
#include "arch/arm/types.hh"
#include "dev/arm/base_gic.hh"
#include "params/Gicv3.hh"

namespace gem5
{

class Gicv3CPUInterface;
class Gicv3Distributor;
class Gicv3Redistributor;
class Gicv3Its;

class Gicv3Registers
{
  public:
    virtual uint32_t readDistributor(Addr daddr) = 0;
    virtual uint32_t readRedistributor(const ArmISA::Affinity &aff,
                                       Addr daddr) = 0;
    virtual RegVal readCpu(const ArmISA::Affinity &aff,
                           ArmISA::MiscRegIndex misc_reg) = 0;

    virtual void writeDistributor(Addr daddr, uint32_t data) = 0;
    virtual void writeRedistributor(const ArmISA::Affinity &aff, Addr daddr,
                                    uint32_t data) = 0;
    virtual void writeCpu(const ArmISA::Affinity &aff,
                          ArmISA::MiscRegIndex misc_reg, RegVal data) = 0;

  protected:
    static void copyDistRegister(Gicv3Registers *from, Gicv3Registers *to,
                                 Addr daddr);
    static void copyRedistRegister(Gicv3Registers *from, Gicv3Registers *to,
                                   const ArmISA::Affinity &aff, Addr daddr);
    static void copyCpuRegister(Gicv3Registers *from, Gicv3Registers *to,
                                const ArmISA::Affinity &aff,
                                ArmISA::MiscRegIndex misc_reg);
    static void clearRedistRegister(Gicv3Registers *to,
                                    const ArmISA::Affinity &aff, Addr daddr);
    static void copyRedistRange(Gicv3Registers *from, Gicv3Registers *to,
                                const ArmISA::Affinity &aff, Addr daddr,
                                size_t size);
    static void copyDistRange(Gicv3Registers *from, Gicv3Registers *to,
                              Addr daddr, size_t size);
    static void clearDistRange(Gicv3Registers *to, Addr daddr, size_t size);
};

class Gicv3 : public BaseGic, public Gicv3Registers
{
  protected:
    friend class Gicv3CPUInterface;
    friend class Gicv3Redistributor;
    friend class Gicv3Distributor;

    Gicv3Distributor *distributor;
    std::vector<Gicv3Redistributor *> redistributors;
    std::vector<Gicv3CPUInterface *> cpuInterfaces;
    Gicv3Its *its;
    AddrRange distRange;
    AddrRange redistRange;
    AddrRangeList addrRanges;
    uint64_t redistSize;

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
    enum IntStatus
    {
        INT_INACTIVE,
        INT_PENDING,
        INT_ACTIVE,
        INT_ACTIVE_PENDING,
    };

    // Interrupt groups, as per SPEC section 4.6
    enum GroupId
    {
        G0S,
        G1S,
        G1NS,
    };

    enum IntTriggerType
    {
        INT_LEVEL_SENSITIVE,
        INT_EDGE_TRIGGERED,
    };

  protected:
    void clearInt(uint32_t int_id) override;
    void clearPPInt(uint32_t int_id, uint32_t cpu) override;

    inline AddrRangeList
    getAddrRanges() const override
    {
        return addrRanges;
    }

    void init() override;

    PARAMS(Gicv3);

    Tick read(PacketPtr pkt) override;
    void reset();
    void sendInt(uint32_t int_id) override;
    void sendPPInt(uint32_t int_id, uint32_t cpu) override;
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    Tick write(PacketPtr pkt) override;
    bool supportsVersion(GicVersion version) override;

    template <typename... Args>
    void
    reserved(const char *fmt, Args... args) const
    {
        if (params().reserved_is_res0) {
            warn(fmt, args...);
        } else {
            panic(fmt, args...);
        }
    }

  public:
    Gicv3(const Params &p);
    void deassertInt(uint32_t cpu, ArmISA::InterruptTypes int_type);
    void deassertAll(uint32_t cpu);
    bool haveAsserted(uint32_t cpu) const;

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

    Gicv3CPUInterface *
    getCPUInterfaceByAffinity(const ArmISA::Affinity &aff) const;

    Gicv3Redistributor *
    getRedistributorByAffinity(const ArmISA::Affinity &aff) const;

    Gicv3Redistributor *getRedistributorByAddr(Addr address) const;

    void postInt(uint32_t cpu, ArmISA::InterruptTypes int_type);

    void update();

  protected: // GIC state transfer
    void copyGicState(Gicv3Registers *from, Gicv3Registers *to);

  public: // Gicv3Registers
    uint32_t readDistributor(Addr daddr) override;
    uint32_t readRedistributor(const ArmISA::Affinity &aff,
                               Addr daddr) override;
    RegVal readCpu(const ArmISA::Affinity &aff,
                   ArmISA::MiscRegIndex misc_reg) override;

    void writeDistributor(Addr daddr, uint32_t data) override;
    void writeRedistributor(const ArmISA::Affinity &aff, Addr daddr,
                            uint32_t data) override;
    void writeCpu(const ArmISA::Affinity &aff, ArmISA::MiscRegIndex misc_reg,
                  RegVal data) override;
};

} // namespace gem5

#endif //__DEV_ARM_GICV3_H__
