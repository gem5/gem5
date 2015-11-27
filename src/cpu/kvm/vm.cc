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
 *
 * Authors: Andreas Sandberg
 */

#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <memory>

#include "cpu/kvm/vm.hh"
#include "debug/Kvm.hh"
#include "params/KvmVM.hh"
#include "sim/system.hh"

#define EXPECTED_KVM_API_VERSION 12

#if EXPECTED_KVM_API_VERSION != KVM_API_VERSION
#error Unsupported KVM version
#endif

Kvm *Kvm::instance = NULL;

Kvm::Kvm()
    : kvmFD(-1), apiVersion(-1), vcpuMMapSize(0)
{
    kvmFD = ::open("/dev/kvm", O_RDWR);
    if (kvmFD == -1)
        fatal("KVM: Failed to open /dev/kvm\n");

    apiVersion = ioctl(KVM_GET_API_VERSION);
    if (apiVersion != EXPECTED_KVM_API_VERSION)
        fatal("KVM: Incompatible API version\n");

    vcpuMMapSize = ioctl(KVM_GET_VCPU_MMAP_SIZE);
    if (vcpuMMapSize == -1)
        panic("KVM: Failed to get virtual CPU MMAP size\n");
}

Kvm::~Kvm()
{
    close(kvmFD);
}

Kvm *
Kvm::create()
{
    if (!instance)
        instance = new Kvm();

    return instance;
}

bool
Kvm::capUserMemory() const
{
    return checkExtension(KVM_CAP_USER_MEMORY) != 0;
}

bool
Kvm::capSetTSSAddress() const
{
    return checkExtension(KVM_CAP_SET_TSS_ADDR) != 0;
}

bool
Kvm::capExtendedCPUID() const
{
    return checkExtension(KVM_CAP_EXT_CPUID) != 0;
}

bool
Kvm::capUserNMI() const
{
#ifdef KVM_CAP_USER_NMI
    return checkExtension(KVM_CAP_USER_NMI) != 0;
#else
    return false;
#endif
}

int
Kvm::capCoalescedMMIO() const
{
    return checkExtension(KVM_CAP_COALESCED_MMIO);
}

int
Kvm::capNumMemSlots() const
{
#ifdef KVM_CAP_NR_MEMSLOTS
    return checkExtension(KVM_CAP_NR_MEMSLOTS);
#else
    return 0;
#endif
}

bool
Kvm::capOneReg() const
{
#ifdef KVM_CAP_ONE_REG
    return checkExtension(KVM_CAP_ONE_REG) != 0;
#else
    return false;
#endif
}

bool
Kvm::capIRQChip() const
{
    return checkExtension(KVM_CAP_IRQCHIP) != 0;
}

bool
Kvm::capVCPUEvents() const
{
#ifdef KVM_CAP_VCPU_EVENTS
    return checkExtension(KVM_CAP_VCPU_EVENTS) != 0;
#else
    return false;
#endif
}

bool
Kvm::capDebugRegs() const
{
#ifdef KVM_CAP_DEBUGREGS
    return checkExtension(KVM_CAP_DEBUGREGS) != 0;
#else
    return false;
#endif
}

bool
Kvm::capXCRs() const
{
#ifdef KVM_CAP_XCRS
    return checkExtension(KVM_CAP_XCRS) != 0;
#else
    return false;
#endif
}

bool
Kvm::capXSave() const
{
#ifdef KVM_CAP_XSAVE
    return checkExtension(KVM_CAP_XSAVE) != 0;
#else
    return false;
#endif
}


#if defined(__i386__) || defined(__x86_64__)
bool
Kvm::getSupportedCPUID(struct kvm_cpuid2 &cpuid) const
{
    if (ioctl(KVM_GET_SUPPORTED_CPUID, (void *)&cpuid) == -1) {
        if (errno == E2BIG)
            return false;
        else
            panic("KVM: Failed to get supported CPUID (errno: %i)\n", errno);
    } else
        return true;
}

const Kvm::CPUIDVector &
Kvm::getSupportedCPUID() const
{
    if (supportedCPUIDCache.empty()) {
        std::unique_ptr<struct kvm_cpuid2> cpuid;
        int i(1);
        do {
            cpuid.reset((struct kvm_cpuid2 *)operator new(
                            sizeof(kvm_cpuid2) + i * sizeof(kvm_cpuid_entry2)));

            cpuid->nent = i;
            ++i;
        } while (!getSupportedCPUID(*cpuid));
        supportedCPUIDCache.assign(cpuid->entries,
                                   cpuid->entries + cpuid->nent);
    }

    return supportedCPUIDCache;
}

bool
Kvm::getSupportedMSRs(struct kvm_msr_list &msrs) const
{
    if (ioctl(KVM_GET_MSR_INDEX_LIST, (void *)&msrs) == -1) {
        if (errno == E2BIG)
            return false;
        else
            panic("KVM: Failed to get supported CPUID (errno: %i)\n", errno);
    } else
        return true;
}

const Kvm::MSRIndexVector &
Kvm::getSupportedMSRs() const
{
    if (supportedMSRCache.empty()) {
        std::unique_ptr<struct kvm_msr_list> msrs;
        int i(0);
        do {
            msrs.reset((struct kvm_msr_list *)operator new(
                           sizeof(kvm_msr_list) + i * sizeof(uint32_t)));

            msrs->nmsrs = i;
            ++i;
        } while (!getSupportedMSRs(*msrs));
        supportedMSRCache.assign(msrs->indices, msrs->indices + msrs->nmsrs);
    }

    return supportedMSRCache;
}

#endif // x86-specific


int
Kvm::checkExtension(int extension) const
{
    int ret = ioctl(KVM_CHECK_EXTENSION, extension);
    if (ret == -1)
        panic("KVM: ioctl failed when checking for extension\n");
    return ret;
}

int
Kvm::ioctl(int request, long p1) const
{
    assert(kvmFD != -1);

    return ::ioctl(kvmFD, request, p1);
}

int
Kvm::createVM()
{
    int vmFD;

    vmFD = ioctl(KVM_CREATE_VM);
    if (vmFD == -1)
        panic("Failed to create KVM VM\n");

    return vmFD;
}


KvmVM::KvmVM(KvmVMParams *params)
    : SimObject(params),
      kvm(new Kvm()), system(params->system),
      vmFD(kvm->createVM()),
      started(false),
      nextVCPUID(0)
{
    maxMemorySlot = kvm->capNumMemSlots();
    /* If we couldn't determine how memory slots there are, guess 32. */
    if (!maxMemorySlot)
        maxMemorySlot = 32;
    /* Setup the coalesced MMIO regions */
    for (int i = 0; i < params->coalescedMMIO.size(); ++i)
        coalesceMMIO(params->coalescedMMIO[i]);
}

KvmVM::~KvmVM()
{
    if (vmFD != -1)
        close(vmFD);

    if (kvm)
        delete kvm;
}

void
KvmVM::notifyFork()
{
    if (vmFD != -1) {
        if (close(vmFD) == -1)
            warn("kvm VM: notifyFork failed to close vmFD\n");

        vmFD = -1;

        delete kvm;
        kvm = NULL;
    }
}

void
KvmVM::cpuStartup()
{
    if (started)
        return;
    started = true;

    delayedStartup();
}

void
KvmVM::delayedStartup()
{
    const std::vector<std::pair<AddrRange, uint8_t*> >&memories(
        system->getPhysMem().getBackingStore());

    DPRINTF(Kvm, "Mapping %i memory region(s)\n", memories.size());
    for (int slot(0); slot < memories.size(); ++slot) {
        const AddrRange &range(memories[slot].first);
        void *pmem(memories[slot].second);

        if (pmem) {
            DPRINTF(Kvm, "Mapping region: 0x%p -> 0x%llx [size: 0x%llx]\n",
                    pmem, range.start(), range.size());

            if (range.interleaved()) {
                panic("Tried to map an interleaved memory range into "
                      "a KVM VM.\n");
            }

            const MemSlot slot = allocMemSlot(range.size());
            setupMemSlot(slot, pmem, range.start(), 0/* flags */);
        } else {
            DPRINTF(Kvm, "Zero-region not mapped: [0x%llx]\n", range.start());
            hack("KVM: Zero memory handled as IO\n");
        }
    }
}

const KvmVM::MemSlot
KvmVM::allocMemSlot(uint64_t size)
{
    if (!size)
        panic("Memory slots must have non-zero size.\n");

    std::vector<MemorySlot>::iterator pos;
    for (pos = memorySlots.begin(); pos != memorySlots.end(); pos++) {
        if (!pos->size) {
            pos->size = size;
            pos->active = false;
            return pos->slot;
        }
    }

    uint32_t nextSlot = memorySlots.size();
    if (nextSlot > maxMemorySlot)
        panic("Out of memory slots.\n");

    MemorySlot slot;
    slot.size = size;
    slot.slot = nextSlot;
    slot.active = false;

    memorySlots.push_back(slot);
    return MemSlot(slot.slot);
}

void
KvmVM::setupMemSlot(const KvmVM::MemSlot num, void *host_addr, Addr guest,
                    uint32_t flags)
{
    MemorySlot &slot = memorySlots.at(num.num);
    slot.active = true;
    setUserMemoryRegion(num.num, host_addr, guest, slot.size, flags);
}

void
KvmVM::disableMemSlot(const KvmVM::MemSlot num)
{
    MemorySlot &slot = memorySlots.at(num.num);
    if (slot.active)
        setUserMemoryRegion(num.num, NULL, 0, 0, 0);
    slot.active = false;
}

void
KvmVM::freeMemSlot(const KvmVM::MemSlot num)
{
    disableMemSlot(num.num);
    MemorySlot &slot = memorySlots.at(num.num);
    slot.size = 0;
}

void
KvmVM::setUserMemoryRegion(uint32_t slot,
                           void *host_addr, Addr guest_addr,
                           uint64_t len, uint32_t flags)
{
    struct kvm_userspace_memory_region m;

    memset(&m, 0, sizeof(m));
    m.slot = slot;
    m.flags = flags;
    m.guest_phys_addr = (uint64_t)guest_addr;
    m.memory_size = len;
    m.userspace_addr = (__u64)host_addr;

    if (ioctl(KVM_SET_USER_MEMORY_REGION, (void *)&m) == -1) {
        panic("Failed to setup KVM memory region:\n"
              "\tHost Address: 0x%p\n"
              "\tGuest Address: 0x%llx\n",
              "\tSize: %ll\n",
              "\tFlags: 0x%x\n",
              m.userspace_addr, m.guest_phys_addr,
              m.memory_size, m.flags);
    }
}

void
KvmVM::coalesceMMIO(const AddrRange &range)
{
    coalesceMMIO(range.start(), range.size());
}

void
KvmVM::coalesceMMIO(Addr start, int size)
{
    struct kvm_coalesced_mmio_zone zone;

    zone.addr = start;
    zone.size = size;
    zone.pad = 0;

    DPRINTF(Kvm, "KVM: Registering coalesced MMIO region [0x%x, 0x%x]\n",
            zone.addr, zone.addr + zone.size - 1);
    if (ioctl(KVM_REGISTER_COALESCED_MMIO, (void *)&zone) == -1)
        panic("KVM: Failed to register coalesced MMIO region (%i)\n",
              errno);
}

void
KvmVM::setTSSAddress(Addr tss_address)
{
    if (ioctl(KVM_SET_TSS_ADDR, (unsigned long)tss_address) == -1)
        panic("KVM: Failed to set VM TSS address\n");
}

void
KvmVM::createIRQChip()
{
    if (_hasKernelIRQChip)
        panic("KvmVM::createIRQChip called twice.\n");

    if (ioctl(KVM_CREATE_IRQCHIP) != -1) {
        _hasKernelIRQChip = true;
    } else {
        warn("KVM: Failed to create in-kernel IRQ chip (errno: %i)\n",
             errno);
        _hasKernelIRQChip = false;
    }
}

void
KvmVM::setIRQLine(uint32_t irq, bool high)
{
    struct kvm_irq_level kvm_level;

    kvm_level.irq = irq;
    kvm_level.level = high ? 1 : 0;

    if (ioctl(KVM_IRQ_LINE, &kvm_level) == -1)
        panic("KVM: Failed to set IRQ line level (errno: %i)\n",
              errno);
}

int
KvmVM::createDevice(uint32_t type, uint32_t flags)
{
#if defined(KVM_CREATE_DEVICE)
    struct kvm_create_device dev = { type, 0, flags };

    if (ioctl(KVM_CREATE_DEVICE, &dev) == -1) {
        panic("KVM: Failed to create device (errno: %i)\n",
              errno);
    }

    return dev.fd;
#else
    panic("Kernel headers don't support KVM_CREATE_DEVICE\n");
#endif
}

int
KvmVM::createVCPU(long vcpuID)
{
    int fd;

    fd = ioctl(KVM_CREATE_VCPU, vcpuID);
    if (fd == -1)
        panic("KVM: Failed to create virtual CPU");

    return fd;
}

long
KvmVM::allocVCPUID()
{
    return nextVCPUID++;
}

#if defined(__aarch64__)
void
KvmVM::kvmArmPreferredTarget(struct kvm_vcpu_init &target) const
{
    if (ioctl(KVM_ARM_PREFERRED_TARGET, &target) == -1) {
        panic("KVM: Failed to get ARM preferred CPU target (errno: %i)\n",
              errno);
    }
}
#endif

int
KvmVM::ioctl(int request, long p1) const
{
    assert(vmFD != -1);

    return ::ioctl(vmFD, request, p1);
}


KvmVM *
KvmVMParams::create()
{
    static bool created = false;
    if (created)
        warn_once("Use of multiple KvmVMs is currently untested!\n");

    created = true;

    return new KvmVM(this);
}
