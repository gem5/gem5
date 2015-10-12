/*
 * Copyright (c) 2015 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#ifndef __ARCH_ARM_KVM_GIC_HH__
#define __ARCH_ARM_KVM_GIC_HH__

#include "arch/arm/system.hh"
#include "cpu/kvm/device.hh"
#include "cpu/kvm/vm.hh"
#include "dev/arm/base_gic.hh"
#include "dev/platform.hh"

class KvmGicParams;

/**
 * In-kernel GIC model.
 *
 * When using a KVM-based CPU model, it is possible to offload GIC
 * emulation to the kernel. This reduces some overheads when the guest
 * accesses the GIC and makes it possible to use in-kernel
 * architected/generic timer emulation.
 *
 * This device uses interfaces with the kernel GicV2 model that is
 * documented in Documentation/virtual/kvm/devices/arm-vgic.txt in the
 * Linux kernel sources.
 *
 * This GIC model has the following known limitations:
 * <ul>
 *   <li>Checkpointing is not supported.
 *   <li>This model only works with kvm. Simulated CPUs are not
 *       supported since this would require the kernel to inject
 *       interrupt into the simulated CPU.
 * </ul>
 *
 * @warn This GIC model cannot be used with simulated CPUs!
 */
class KvmGic : public BaseGic
{
  public: // SimObject / Serializable / Drainable
    KvmGic(const KvmGicParams *p);
    ~KvmGic();

    void startup() override { verifyMemoryMode(); }
    void drainResume() override { verifyMemoryMode(); }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(Checkpoint *cp, const std::string &sec)  override;

  public: // PioDevice
    AddrRangeList getAddrRanges() const { return addrRanges; }
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

  public: // BaseGic
    void sendInt(uint32_t num) override;
    void clearInt(uint32_t num) override;

    void sendPPInt(uint32_t num, uint32_t cpu) override;
    void clearPPInt(uint32_t num, uint32_t cpu) override;

  protected:
    /**
     * Do memory mode sanity checks
     *
     * This method only really exists to warn users that try to switch
     * to a simulate CPU. There is no fool proof method to detect
     * simulated CPUs, but checking that we're in atomic mode and
     * bypassing caches should be robust enough.
     */
    void verifyMemoryMode() const;

    /**
     * Update the kernel's VGIC interrupt state
     *
     * @param type Interrupt type (KVM_ARM_IRQ_TYPE_PPI/KVM_ARM_IRQ_TYPE_SPI)
     * @param vcpu CPU id within KVM (ignored for SPIs)
     * @param irq Interrupt number
     * @param high True to signal an interrupt, false to clear it.
     */
    void setIntState(uint8_t type, uint8_t vcpu, uint16_t irq, bool high);

    /** System this interrupt controller belongs to */
    System &system;
    /** VM for this system */
    KvmVM &vm;
    /** Kernel interface to the GIC */
    KvmDevice kdev;

    /** Address range for the distributor interface */
    const AddrRange distRange;
    /** Address range for the CPU interfaces */
    const AddrRange cpuRange;
    /** Union of all memory  */
    const AddrRangeList addrRanges;
};

#endif // __ARCH_ARM_KVM_GIC_HH__
