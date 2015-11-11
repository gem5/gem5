/*
 * Copyright (c) 2013 Andreas Sandberg
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
 *
 * Authors: Andreas Sandberg
 */

#ifndef __CPU_KVM_X86_CPU_HH__
#define __CPU_KVM_X86_CPU_HH__

#include "cpu/kvm/base.hh"
#include "cpu/kvm/vm.hh"
#include "params/X86KvmCPU.hh"

/**
 * x86 implementation of a KVM-based hardware virtualized CPU.
 */
class X86KvmCPU : public BaseKvmCPU
{
  public:
    X86KvmCPU(X86KvmCPUParams *params);
    virtual ~X86KvmCPU();

    void startup();

    /** @{ */
    void dump() const override;
    void dumpFpuRegs() const;
    void dumpIntRegs() const;
    void dumpSpecRegs() const;
    void dumpDebugRegs() const;
    void dumpXCRs() const;
    void dumpXSave() const;
    void dumpVCpuEvents() const;
    void dumpMSRs() const;
    /** @} */

  protected:
    typedef std::vector<struct kvm_msr_entry> KvmMSRVector;

    Tick kvmRun(Tick ticks);

    /**
     * Run the virtual CPU until draining completes.
     *
     * In addition to the base functionality provided by
     * BaseKvmCPU::kvmRunDrain(), this method handles x86-specific
     * cases where there are pending interrupt events in the virtual
     * CPU.  These are handled by requesting an interrupt window if
     * interrupts are pending (causing the vCPU to execute until
     * interrupts can be delivered again).
     *
     * @see BaseKvmCPU::kvmRunDrain()
     * @see archIsDrained()
     *
     * @return Number of ticks executed
     */
    Tick kvmRunDrain();

    /** Wrapper that synchronizes state in kvm_run */
    Tick kvmRunWrapper(Tick ticks);

    uint64_t getHostCycles() const;

    /**
     * Methods to access CPUID information using the extended
     * API. Only available if Kvm::capExtendedCPUID() is true.
     *
     * @{
     */
    void setCPUID(const struct kvm_cpuid2 &cpuid);
    void setCPUID(const Kvm::CPUIDVector &cpuid);
    /** @} */

    /**
     * Methods to access MSRs in the guest.
     *
     * @{
     */
    void setMSRs(const struct kvm_msrs &msrs);
    void setMSRs(const KvmMSRVector &msrs);
    void getMSRs(struct kvm_msrs &msrs) const;
    void setMSR(uint32_t index, uint64_t value);
    uint64_t getMSR(uint32_t index) const;
    /** @} */

    /**
     * Get a list of MSRs supported by both gem5 and KVM.
     *
     * @note This method uses an internal cache and only generates the
     * MSR list once.
     *
     * @return reference to a list of msr indices
     */
    const Kvm::MSRIndexVector &getMsrIntersection() const;

    /**
     * Wrappers around KVM's state transfer methods.
     *
     * @{
     */
    void getDebugRegisters(struct kvm_debugregs &regs) const;
    void setDebugRegisters(const struct kvm_debugregs &regs);
    void getXCRs(struct kvm_xcrs &regs) const;
    void setXCRs(const struct kvm_xcrs &regs);
    void getXSave(struct kvm_xsave &xsave) const;
    void setXSave(const struct kvm_xsave &xsave);
    void getVCpuEvents(struct kvm_vcpu_events &events) const;
    void setVCpuEvents(const struct kvm_vcpu_events &events);
    /** @} */

    void updateKvmState();
    void updateThreadContext();

    /**
     * Inject pending interrupts from gem5 into the virtual CPU.
     */
    void deliverInterrupts();

    /**
     * Handle x86 legacy IO (in/out)
     */
    Tick handleKvmExitIO();

    Tick handleKvmExitIRQWindowOpen();

    /**
     * Check if there are pending events in the vCPU that prevents it
     * from being drained.
     *
     * There are cases after interrupt injection where the interrupt
     * is still pending in the guest. This method detects such cases
     * and requests additional draining.
     *
     * @return False if there are pending events in the guest, True
     * otherwise.
     */
    bool archIsDrained() const;

  private:
    /**
     * Support routines to update the state of the KVM CPU from gem5's
     * state representation.
     *
     * @{
     */
    /** Update integer registers */
    void updateKvmStateRegs();
    /** Update control registers (CRx, segments, etc.) */
    void updateKvmStateSRegs();
    /**
     * Update FPU and SIMD registers
     *
     * This method uses the appropriate (depending on availability and
     * user configuration) kernel API by calling
     * updateKvmStateFPULegacy() or updateKvmStateFPUXSave().
     *
     * @see updateKvmStateFPULegacy()
     * @see updateKvmStateFPUXSave()
     */
    void updateKvmStateFPU();
    /**
     * Update FPU and SIMD registers using the legacy API
     *
     * @note This method should normally only be called by
     * updateKvmStateFPU() which automatically chooses between
     * available APIs.
     */
    void updateKvmStateFPULegacy();
    /**
     * Update FPU and SIMD registers using the XSave API
     *
     * @note This method should normally only be called by
     * updateKvmStateFPU() which automatically chooses between
     * available APIs.
     */
    void updateKvmStateFPUXSave();
    /** Update MSR registers */
    void updateKvmStateMSRs();
    /** @} */

    /**
     * Support routines to update the state of gem5's thread context from
     * KVM's state representation.
     *
     * @{
     */
    /** Update integer registers */
    void updateThreadContextRegs(const struct kvm_regs &regs,
                                 const struct kvm_sregs &sregs);
    /** Update control registers (CRx, segments, etc.) */
    void updateThreadContextSRegs(const struct kvm_sregs &sregs);
    /** Update FPU and SIMD registers using the legacy API */
    void updateThreadContextFPU(const struct kvm_fpu &fpu);
    /** Update FPU and SIMD registers using the XSave API */
    void updateThreadContextXSave(const struct kvm_xsave &kxsave);
    /** Update MSR registers */
    void updateThreadContextMSRs();
    /** @} */

    /** Transfer gem5's CPUID values into the virtual CPU. */
    void updateCPUID();

    /**
     * Handle a 32-bit IO access that should be mapped to a MiscReg.
     *
     * @note This method can only be called on when handling IO after
     * a KVM_EXIT_IO.
     *
     * @param miscreg Register to map the current IO access to.
     */
    void handleIOMiscReg32(int miscreg);

    /** Cached intersection of supported MSRs */
    mutable Kvm::MSRIndexVector cachedMsrIntersection;

    /** @{ */
    /** Kvm::capDebugRegs() available? */
    bool haveDebugRegs;
    /** Kvm::capXSave() available? */
    bool haveXSave;
    /**
     * Should the XSave interface be used to sync the FPU and SIMD
     * registers?
     */
    bool useXSave;
    /** Kvm::capXCRs() available? */
    bool haveXCRs;
    /** @} */
};

#endif
