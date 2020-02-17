/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2012, 2015 ARM Limited
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

#ifndef __CPU_KVM_KVMVM_HH__
#define __CPU_KVM_KVMVM_HH__

#include <vector>

#include "base/addr_range.hh"
#include "sim/sim_object.hh"

// forward declarations
struct KvmVMParams;
class BaseKvmCPU;
class System;

/**
 * @defgroup KvmInterrupts KVM Interrupt handling.
 *
 * These methods control interrupt delivery to the guest system.
 */

/**
 * @defgroup KvmIoctl KVM low-level ioctl interface.
 *
 * These methods provide a low-level interface to the underlying KVM
 * layer.
 */

/**
 * KVM parent interface
 *
 * The main Kvm object is used to provide functionality that is not
 * specific to a VM or CPU. For example, it allows checking of the
 * optional features and creation of VM containers.
 */
class Kvm
{
    friend class KvmVM;

  public:
    virtual ~Kvm();

    Kvm *create();

    /** Get the version of the KVM API implemented by the kernel. */
    int getAPIVersion() const { return apiVersion; }
    /**
     * Get the size of the MMAPed parameter area used to communicate
     * vCPU parameters between the kernel and userspace. This area,
     * amongst other things, contains the kvm_run data structure.
     */
    int getVCPUMMapSize() const { return vcpuMMapSize; }

    /** @{ */
    /** Support for KvmVM::setUserMemoryRegion() */
    bool capUserMemory() const;
    /** Support for KvmVM::setTSSAddress() */
    bool capSetTSSAddress() const;
    /** Support for BaseKvmCPU::setCPUID2 and getSupportedCPUID(). */
    bool capExtendedCPUID() const;
    /** Support for BaseKvmCPU::kvmNonMaskableInterrupt(). */
    bool capUserNMI() const;

    /**
     * Check if coalesced MMIO is supported and which page in the
     * MMAP'ed structure it stores requests in.
     *
     * @return Offset (in pages) into the mmap'ed vCPU area where the
     * MMIO buffer is stored. 0 if unsupported.
     */
    int capCoalescedMMIO() const;

    /**
     * Attempt to determine how many memory slots are available. If it can't
     * be determined, this function returns 0.
     */
    int capNumMemSlots() const;

    /**
     * Support for reading and writing single registers.
     *
     * @see BaseKvmCPU::getOneReg(), and BaseKvmCPU::setOneReg()
     */
    bool capOneReg() const;

    /**
     * Support for creating an in-kernel IRQ chip model.
     *
     * @see KvmVM::createIRQChip()
     */
    bool capIRQChip() const;

    /** Support for getting and setting the kvm_vcpu_events structure. */
    bool capVCPUEvents() const;

    /** Support for getting and setting the kvm_debugregs structure. */
    bool capDebugRegs() const;

    /** Support for getting and setting the x86 XCRs. */
    bool capXCRs() const;

    /** Support for getting and setting the kvm_xsave structure. */
    bool capXSave() const;
    /** @} */

#if defined(__i386__) || defined(__x86_64__)
  public: // x86-specific
    /**
     * @{
     * @name X86-specific APIs
     */

    typedef std::vector<struct kvm_cpuid_entry2> CPUIDVector;
    typedef std::vector<uint32_t> MSRIndexVector;

    /**
     * Get the CPUID features supported by the hardware and Kvm.
     *
     * @note Requires capExtendedCPUID().
     *
     * @return False if the allocation is too small, true on success.
     */
    bool getSupportedCPUID(struct kvm_cpuid2 &cpuid) const;

    /**
     * Get the CPUID features supported by the hardware and Kvm.
     *
     * @note Requires capExtendedCPUID().
     *
     * @note This method uses an internal cache to minimize the number
     * of calls into the kernel.
     *
     * @return Reference to cached MSR index list.
     */
    const CPUIDVector &getSupportedCPUID() const;

    /**
     * Get the MSRs supported by the hardware and Kvm.
     *
     * @return False if the allocation is too small, true on success.
     */
    bool getSupportedMSRs(struct kvm_msr_list &msrs) const;

    /**
     * Get the MSRs supported by the hardware and Kvm.
     *
     * @note This method uses an internal cache to minimize the number
     * of calls into the kernel.
     *
     * @return Reference to cached MSR index list.
     */
    const MSRIndexVector &getSupportedMSRs() const;

  private: // x86-specific
    /** Cached vector of supported CPUID entries. */
    mutable CPUIDVector supportedCPUIDCache;

    /** Cached vector of supported MSRs. */
    mutable MSRIndexVector supportedMSRCache;


    /** @} */
#endif

  protected:
    /**
     * Check for the presence of an extension to the KVM API.
     *
     * The return value depends on the extension, but is always zero
     * if it is unsupported or positive otherwise. Some extensions use
     * the return value provide additional data about the extension.
     *
     * @return 0 if the extension is unsupported, positive integer
     * otherwise.
     */
    int checkExtension(int extension) const;

    /**
     * @addtogroup KvmIoctl
     * @{
     */
    /**
     * Main VM ioctl interface.
     *
     * @param request KVM request
     * @param p1 Optional request parameter
     *
     * @return -1 on error (error number in errno), ioctl dependent
     * value otherwise.
     */
    int ioctl(int request, long p1) const;
    int ioctl(int request, void *p1) const {
        return ioctl(request, (long)p1);
    }
    int ioctl(int request) const {
        return ioctl(request, 0L);
    }
    /** @} */

  private:
    // This object is a singleton, so prevent instantiation.
    Kvm();

    // Prevent copying
    Kvm(const Kvm &kvm);
    // Prevent assignment
    Kvm &operator=(const Kvm &kvm);

    /**
     * Create a KVM Virtual Machine
     *
     * @return File descriptor pointing to the VM
     */
    int createVM();

    /** KVM VM file descriptor */
    int kvmFD;
    /** KVM API version */
    int apiVersion;
    /** Size of the MMAPed vCPU parameter area. */
    int vcpuMMapSize;

    /** Singleton instance */
    static Kvm *instance;
};

/**
 * KVM VM container
 *
 * A KVM VM container normally contains all the CPUs in a shared
 * memory machine. The VM container handles things like physical
 * memory and to some extent interrupts. Normally, the VM API is only
 * used for interrupts when the PIC is emulated by the kernel, which
 * is a feature we do not use. However, some architectures (notably
 * ARM) use the VM interface to deliver interrupts to specific CPUs as
 * well.
 *
 * VM initialization is a bit different from that of other
 * SimObjects. When we initialize the VM, we discover all physical
 * memory mappings in the system. Since AbstractMem::unserialize
 * re-maps the guests memory, we need to make sure that this is done
 * after the memory has been re-mapped, but before the vCPUs are
 * initialized (KVM requires memory mappings to be setup before CPUs
 * can be created). Normally, we would just initialize the VM in
 * init() or startup(), however, we can not use init() since this is
 * called before AbstractMem::unserialize() and we can not use
 * startup() since it must be called before BaseKvmCPU::startup() and
 * the simulator framework does not guarantee call order. We therefore
 * call cpuStartup() from BaseKvmCPU::startup() instead and execute
 * the initialization code once when the first CPU in the VM is
 * starting.
 */
class KvmVM : public SimObject
{
    friend class BaseKvmCPU;

  public:
    KvmVM(KvmVMParams *params);
    virtual ~KvmVM();

    void notifyFork();

    /**
     * Setup a shared three-page memory region used by the internals
     * of KVM. This is currently only needed by x86 implementations.
     *
     * @param tss_address Physical address of the start of the TSS
     */
    void setTSSAddress(Addr tss_address);

    /** @{ */
    /**
     * Request coalescing MMIO for a memory range.
     *
     * @param start Physical start address in guest
     * @param size Size of the MMIO region
     */
    void coalesceMMIO(Addr start, int size);

    /**
     * Request coalescing MMIO for a memory range.
     *
     * @param range Coalesced MMIO range
     */
    void coalesceMMIO(const AddrRange &range);
    /** @} */

    /**
     * @addtogroup KvmInterrupts
     * @{
     */
    /**
     * Create an in-kernel interrupt  controller
     *
     * @note This functionality depends on Kvm::capIRQChip().
     */
    void createIRQChip();

    /**
     * Set the status of an IRQ line using KVM_IRQ_LINE.
     *
     * @note This ioctl is usually only used if the interrupt
     * controller is emulated by the kernel (i.e., after calling
     * createIRQChip()). Some architectures (e.g., ARM) use it instead
     * of BaseKvmCPU::kvmInterrupt().
     *
     * @param irq Interrupt number
     * @param high Line level (true for high, false for low)
     */
    void setIRQLine(uint32_t irq, bool high);

    /**
     * Is in-kernel IRQ chip emulation enabled?
     */
    bool hasKernelIRQChip() const { return _hasKernelIRQChip; }

    /**
     * Tell the VM and VCPUs to use an in-kernel IRQ chip for
     * interrupt delivery.
     *
     * @note This is set automatically if the IRQ chip is created
     * using the KvmVM::createIRQChip() API.
     */
    void enableKernelIRQChip() { _hasKernelIRQChip = true; }
    /** @} */

    struct MemSlot
    {
        MemSlot(uint32_t _num) : num(_num)
        {}
        MemSlot() : num(-1)
        {}

        int32_t num;
    };

    /**
     *  Allocate a memory slot within the VM.
     */
    const MemSlot allocMemSlot(uint64_t size);

    /**
     * Setup a region of physical memory in the guest
     *
     * @param slot KVM memory slot ID returned by allocMemSlot
     * @param host_addr Memory allocation backing the memory
     * @param guest_addr Address in the guest
     * @param flags Flags (see the KVM API documentation)
     */
    void setupMemSlot(const MemSlot slot, void *host_addr, Addr guest_addr,
                      uint32_t flags);

    /**
     * Disable a memory slot.
     */
    void disableMemSlot(const MemSlot slot);

    /**
     *  Free a previously allocated memory slot.
     */
    void freeMemSlot(const MemSlot slot);

    /**
     * Create an in-kernel device model.
     *
     * @param type Device type (KVM_DEV_TYPE_xxx)
     * @param flags Creation flags (KVM_CREATE_DEVICE_xxx)
     * @return Device file descriptor
     */
    int createDevice(uint32_t type, uint32_t flags = 0);

    /** Global KVM interface */
    Kvm *kvm;

    /**
     * Initialize system pointer. Invoked by system object.
     */
    void setSystem(System *s);

    /**
      * Get the VCPUID for a given context
      */
    long contextIdToVCpuId(ContextID ctx) const;

#if defined(__aarch64__)
  public: // ARM-specific
    /**
     * Ask the kernel for the preferred CPU target to simulate.
     *
     * When creating an ARM vCPU in Kvm, we need to initialize it with
     * a call to BaseArmKvmCPU::kvmArmVCpuInit(). When calling this
     * function, we need to know what type of CPU the host has. This
     * call sets up the kvm_vcpu_init structure with the values the
     * kernel wants.
     *
     * @param[out] target Target structure to initialize.
     */
    void kvmArmPreferredTarget(struct kvm_vcpu_init &target) const;

#endif

  protected:
    /**
     * VM CPU initialization code.
     *
     * This method is called from BaseKvmCPU::startup() when a CPU in
     * the VM executes its BaseKvmCPU::startup() method. The first
     * time method is executed on a VM, it calls the delayedStartup()
     * method.
     */
    void cpuStartup();

    /**
     * Delayed initialization, executed once before the first CPU
     * starts.
     *
     * This method provides a way to do VM initialization once before
     * the first CPU in a VM starts. It is needed since some resources
     * (e.g., memory mappings) can change in the normal
     * SimObject::startup() path. Since the call order of
     * SimObject::startup() is not guaranteed, we simply defer some
     * initialization until a CPU is about to start.
     */
    void delayedStartup();


    /** @{ */
    /**
     * Setup a region of physical memory in the guest
     *
     * @param slot KVM memory slot ID (must be unique)
     * @param host_addr Memory allocation backing the memory
     * @param guest_addr Address in the guest
     * @param len Size of the allocation in bytes
     * @param flags Flags (see the KVM API documentation)
     */
    void setUserMemoryRegion(uint32_t slot,
                             void *host_addr, Addr guest_addr,
                             uint64_t len, uint32_t flags);
    /** @} */

    /**
     * Create a new vCPU within a VM.
     *
     * @param vcpuID ID of the new CPU within the VM.
     * @return File descriptor referencing the CPU.
     */
    int createVCPU(long vcpuID);

    /**
     * Allocate a new vCPU ID within the VM.
     *
     * The returned vCPU ID is guaranteed to be unique within the
     * VM. New IDs are allocated sequentially starting from 0.
     *
     * @return ID of the new vCPU
     */
    long allocVCPUID();

    /**
     * @addtogroup KvmIoctl
     * @{
     */
    /**
     * KVM VM ioctl interface.
     *
     * @param request KVM VM request
     * @param p1 Optional request parameter
     *
     * @return -1 on error (error number in errno), ioctl dependent
     * value otherwise.
     */
    int ioctl(int request, long p1) const;
    int ioctl(int request, void *p1) const {
        return ioctl(request, (long)p1);
    }
    int ioctl(int request) const {
        return ioctl(request, 0L);
    }
    /**@}*/

  private:
    // Prevent copying
    KvmVM(const KvmVM &vm);
    // Prevent assignment
    KvmVM &operator=(const KvmVM &vm);

    System *system;

    /** KVM VM file descriptor */
    int vmFD;

    /** Has delayedStartup() already been called? */
    bool started;

    /** Do we have in-kernel IRQ-chip emulation enabled? */
    bool _hasKernelIRQChip;

    /** Next unallocated vCPU ID */
    long nextVCPUID;

    /**
     *  Structures tracking memory slots.
     */
    class MemorySlot
    {
      public:
        uint64_t size;
        uint32_t slot;
        bool active;
    };
    std::vector<MemorySlot> memorySlots;
    uint32_t maxMemorySlot;
};

#endif
