/*
 * Copyright (c) 2015-2017, 2021-2022 Arm Limited
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

#ifndef __ARCH_ARM_KVM_GIC_HH__
#define __ARCH_ARM_KVM_GIC_HH__

#include "arch/arm/system.hh"
#include "cpu/kvm/device.hh"
#include "cpu/kvm/vm.hh"
#include "dev/arm/gic_v2.hh"
#include "dev/arm/gic_v3.hh"
#include "dev/platform.hh"

#include "params/MuxingKvmGicV2.hh"
#include "params/MuxingKvmGicV3.hh"

namespace gem5
{

/**
 * KVM in-kernel GIC abstraction
 *
 * This class defines a high-level interface to the KVM in-kernel GIC
 * model. It exposes an API that is similar to that of
 * software-emulated GIC models in gem5.
 */
class KvmKernelGic
{
  public:
    /**
     * Instantiate a KVM in-kernel GIC model.
     *
     * This constructor instantiates an in-kernel GIC model and wires
     * it up to the virtual memory system.
     *
     * @param vm KVM VM representing this system
     * @param cpu_addr GIC CPU interface base address
     * @param dist_addr GIC distributor base address
     * @param it_lines Number of interrupt lines to support
     */
    KvmKernelGic(KvmVM &vm, uint32_t dev, unsigned it_lines);
    virtual ~KvmKernelGic();

    KvmKernelGic(const KvmKernelGic &other) = delete;
    KvmKernelGic(const KvmKernelGic &&other) = delete;
    KvmKernelGic &operator=(const KvmKernelGic &&rhs) = delete;
    KvmKernelGic &operator=(const KvmKernelGic &rhs) = delete;

    virtual void
    init()
    {}

  public:
    /**
     * @{
     * @name In-kernel GIC API
     */

    /**
     * Raise a shared peripheral interrupt
     *
     * @param spi SPI number
     */
    void setSPI(unsigned spi);
    /**
     * Clear a shared peripheral interrupt
     *
     * @param spi SPI number
     */
    void clearSPI(unsigned spi);

    /**
     * Raise a private peripheral interrupt
     *
     * @param vcpu KVM virtual CPU number
     * @parma ppi PPI interrupt number
     */
    void setPPI(unsigned vcpu, unsigned ppi);

    /**
     * Clear a private peripheral interrupt
     *
     * @param vcpu KVM virtual CPU number
     * @parma ppi PPI interrupt number
     */
    void clearPPI(unsigned vcpu, unsigned ppi);

    /* @} */

  protected:
    /**
     * Update the kernel's VGIC interrupt state
     *
     * @param type Interrupt type (KVM_ARM_IRQ_TYPE_PPI/KVM_ARM_IRQ_TYPE_SPI)
     * @param vcpu CPU id within KVM (ignored for SPIs)
     * @param irq Interrupt number
     * @param high True to signal an interrupt, false to clear it.
     */
    void setIntState(unsigned type, unsigned vcpu, unsigned irq, bool high);

    /** KVM VM in the parent system */
    KvmVM &vm;

    /** Kernel interface to the GIC */
    KvmDevice kdev;
};

class KvmKernelGicV2 : public KvmKernelGic, public GicV2Registers
{
  public:
    /**
     * Instantiate a KVM in-kernel GICv2 model.
     *
     * This constructor instantiates an in-kernel GICv2 model and wires
     * it up to the virtual memory system.
     *
     * @param vm KVM VM representing this system
     * @param params MuxingKvmGicV2 params
     */
    KvmKernelGicV2(KvmVM &vm, const MuxingKvmGicV2Params &params);

  public: // GicV2Registers
    uint32_t readDistributor(ContextID ctx, Addr daddr) override;
    uint32_t readCpu(ContextID ctx, Addr daddr) override;

    void writeDistributor(ContextID ctx, Addr daddr, uint32_t data) override;
    void writeCpu(ContextID ctx, Addr daddr, uint32_t data) override;

  protected:
    /**
     * Get value of GIC register "from" a cpu
     *
     * @param group Distributor or CPU (KVM_DEV_ARM_VGIC_GRP_{DIST,CPU}_REGS)
     * @param vcpu CPU id within KVM
     * @param offset register offset
     */
    uint32_t getGicReg(unsigned group, unsigned vcpu, unsigned offset);

    /**
     * Set value of GIC register "from" a cpu
     *
     * @param group Distributor or CPU (KVM_DEV_ARM_VGIC_GRP_{DIST,CPU}_REGS)
     * @param vcpu CPU id within KVM
     * @param offset register offset
     * @param value value to set register to
     */
    void setGicReg(unsigned group, unsigned vcpu, unsigned offset,
                   unsigned value);

  private:
    /** Address range for the CPU interfaces */
    const AddrRange cpuRange;
    /** Address range for the distributor */
    const AddrRange distRange;
};

class KvmKernelGicV3 : public KvmKernelGic, public Gicv3Registers
{
  public:
    /**
     * Instantiate a KVM in-kernel GICv3 model.
     *
     * This constructor instantiates an in-kernel GICv3 model and wires
     * it up to the virtual memory system.
     *
     * @param vm KVM VM representing this system
     * @param params MuxingKvmGicV3 parameters
     */
    KvmKernelGicV3(KvmVM &vm, const MuxingKvmGicV3Params &params);

    void init() override;

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

  protected:
    /**
     * Get value of GIC register "from" a cpu
     *
     * @param group Distributor or CPU (KVM_DEV_ARM_VGIC_GRP_{DIST,CPU}_REGS)
     * @param mpidr CPU affinity numbers
     * @param offset register offset
     */
    template <typename Ret>
    Ret getGicReg(unsigned group, unsigned mpidr, unsigned offset);

    /**
     * Set value of GIC register "from" a cpu
     *
     * @param group Distributor or CPU (KVM_DEV_ARM_VGIC_GRP_{DIST,CPU}_REGS)
     * @param mpidr CPU affinity numbers
     * @param offset register offset
     * @param value value to set register to
     */
    template <typename Arg>
    void setGicReg(unsigned group, unsigned mpidr, unsigned offset, Arg value);

  private:
    /** Address range for the redistributor */
    const AddrRange redistRange;
    /** Address range for the distributor */
    const AddrRange distRange;
};

struct GicV2Types
{
    using SimGic = GicV2;
    using KvmGic = KvmKernelGicV2;
    using Params = MuxingKvmGicV2Params;
};

struct GicV3Types
{
    using SimGic = Gicv3;
    using KvmGic = KvmKernelGicV3;
    using Params = MuxingKvmGicV3Params;
};

template <class Types>
class MuxingKvmGic : public Types::SimGic
{
    using SimGic = typename Types::SimGic;
    using KvmGic = typename Types::KvmGic;
    using Params = typename Types::Params;

  public: // SimObject / Serializable / Drainable
    MuxingKvmGic(const Params &p);

    void startup() override;
    DrainState drain() override;
    void drainResume() override;

  public: // PioDevice
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  public: // GicV2
    void sendInt(uint32_t num) override;
    void clearInt(uint32_t num) override;

    void sendPPInt(uint32_t num, uint32_t cpu) override;
    void clearPPInt(uint32_t num, uint32_t cpu) override;

  protected: // BaseGic
    bool blockIntUpdate() const override;

  protected:
    /** System this interrupt controller belongs to */
    System &system;

    /** Kernel GIC device */
    KvmKernelGic *kernelGic;

  private:
    bool usingKvm;

    /** Multiplexing implementation */
    void fromGicToKvm();
    void fromKvmToGic();
};

} // namespace gem5

#endif // __ARCH_ARM_KVM_GIC_HH__
