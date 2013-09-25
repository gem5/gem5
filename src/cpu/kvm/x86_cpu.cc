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

#include <linux/kvm.h>

#include <algorithm>
#include <cerrno>
#include <memory>

#include "arch/x86/regs/msr.hh"
#include "arch/x86/cpuid.hh"
#include "arch/x86/utility.hh"
#include "arch/registers.hh"
#include "cpu/kvm/base.hh"
#include "cpu/kvm/x86_cpu.hh"
#include "debug/Drain.hh"
#include "debug/Kvm.hh"
#include "debug/KvmContext.hh"
#include "debug/KvmIO.hh"
#include "debug/KvmInt.hh"

using namespace X86ISA;

#define MSR_TSC 0x10

#define IO_PCI_CONF_ADDR 0xCF8
#define IO_PCI_CONF_DATA_BASE 0xCFC

#define FOREACH_IREG()                          \
    do {                                        \
        APPLY_IREG(rax, INTREG_RAX);            \
        APPLY_IREG(rbx, INTREG_RBX);            \
        APPLY_IREG(rcx, INTREG_RCX);            \
        APPLY_IREG(rdx, INTREG_RDX);            \
        APPLY_IREG(rsi, INTREG_RSI);            \
        APPLY_IREG(rdi, INTREG_RDI);            \
        APPLY_IREG(rsp, INTREG_RSP);            \
        APPLY_IREG(rbp, INTREG_RBP);            \
        APPLY_IREG(r8, INTREG_R8);              \
        APPLY_IREG(r9, INTREG_R9);              \
        APPLY_IREG(r10, INTREG_R10);            \
        APPLY_IREG(r11, INTREG_R11);            \
        APPLY_IREG(r12, INTREG_R12);            \
        APPLY_IREG(r13, INTREG_R13);            \
        APPLY_IREG(r14, INTREG_R14);            \
        APPLY_IREG(r15, INTREG_R15);            \
    } while(0)

#define FOREACH_SREG()                                  \
    do {                                                \
        APPLY_SREG(cr0, MISCREG_CR0);                   \
        APPLY_SREG(cr2, MISCREG_CR2);                   \
        APPLY_SREG(cr3, MISCREG_CR3);                   \
        APPLY_SREG(cr4, MISCREG_CR4);                   \
        APPLY_SREG(cr8, MISCREG_CR8);                   \
        APPLY_SREG(efer, MISCREG_EFER);                 \
        APPLY_SREG(apic_base, MISCREG_APIC_BASE);       \
    } while(0)

#define FOREACH_DREG()                          \
    do {                                        \
        APPLY_DREG(db[0], MISCREG_DR0);         \
        APPLY_DREG(db[1], MISCREG_DR1);         \
        APPLY_DREG(db[2], MISCREG_DR2);         \
        APPLY_DREG(db[3], MISCREG_DR3);         \
        APPLY_DREG(dr6, MISCREG_DR6);           \
        APPLY_DREG(dr7, MISCREG_DR7);           \
    } while(0)

#define FOREACH_SEGMENT()                                       \
    do {                                                        \
        APPLY_SEGMENT(cs, MISCREG_CS - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(ds, MISCREG_DS - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(es, MISCREG_ES - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(fs, MISCREG_FS - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(gs, MISCREG_GS - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(ss, MISCREG_SS - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(tr, MISCREG_TR - MISCREG_SEG_SEL_BASE);   \
        APPLY_SEGMENT(ldt, MISCREG_TSL - MISCREG_SEG_SEL_BASE); \
    } while(0)

#define FOREACH_DTABLE()                                        \
    do {                                                        \
        APPLY_DTABLE(gdt, MISCREG_TSG - MISCREG_SEG_SEL_BASE);  \
        APPLY_DTABLE(idt, MISCREG_IDTR - MISCREG_SEG_SEL_BASE); \
    } while(0)

template<typename STRUCT, typename ENTRY>
static STRUCT *newVarStruct(size_t entries)
{
    return (STRUCT *)operator new(sizeof(STRUCT) + entries * sizeof(ENTRY));
}

static void
dumpKvm(const struct kvm_regs &regs)
{
    inform("KVM register state:\n");

#define APPLY_IREG(kreg, mreg)                  \
    inform("\t" # kreg ": 0x%llx\n", regs.kreg)

    FOREACH_IREG();

#undef APPLY_IREG

    inform("\trip: 0x%llx\n", regs.rip);
    inform("\trflags: 0x%llx\n", regs.rflags);
}

static void
dumpKvm(const char *reg_name, const struct kvm_segment &seg)
{
    inform("\t%s: @0x%llx+%x [sel: 0x%x, type: 0x%x]\n"
           "\t\tpres.: %u, dpl: %u, db: %u, s: %u, l: %u, g: %u, avl: %u, unus.: %u\n",
           reg_name,
           seg.base, seg.limit, seg.selector, seg.type,
           seg.present, seg.dpl, seg.db, seg.s, seg.l, seg.g, seg.avl, seg.unusable);
}

static void
dumpKvm(const char *reg_name, const struct kvm_dtable &dtable)
{
    inform("\t%s: @0x%llx+%x\n",
           reg_name, dtable.base, dtable.limit);
}

static void
dumpKvm(const struct kvm_sregs &sregs)
{
#define APPLY_SREG(kreg, mreg)                          \
    inform("\t" # kreg ": 0x%llx\n", sregs.kreg);
#define APPLY_SEGMENT(kreg, idx)                \
    dumpKvm(# kreg, sregs.kreg);
#define APPLY_DTABLE(kreg, idx)                 \
    dumpKvm(# kreg, sregs.kreg);

    inform("Special registers:\n");
    FOREACH_SEGMENT();
    FOREACH_SREG();
    FOREACH_DTABLE();

    inform("Interrupt Bitmap:");
    for (int i = 0; i < KVM_NR_INTERRUPTS; i += 64)
        inform("  0x%.8x", sregs.interrupt_bitmap[i / 64]);

#undef APPLY_SREG
#undef APPLY_SEGMENT
#undef APPLY_DTABLE
}

#ifdef KVM_GET_DEBUGREGS
static void
dumpKvm(const struct kvm_debugregs &regs)
{
    inform("KVM debug state:\n");

#define APPLY_DREG(kreg, mreg)                  \
    inform("\t" # kreg ": 0x%llx\n", regs.kreg)

    FOREACH_DREG();

#undef APPLY_DREG

    inform("\tflags: 0x%llx\n", regs.flags);
}
#endif

static void
dumpKvm(const struct kvm_fpu &fpu)
{
    inform("FPU registers:\n");
    inform("\tfcw: 0x%x\n", fpu.fcw);
    inform("\tfsw: 0x%x\n", fpu.fsw);
    inform("\tftwx: 0x%x\n", fpu.ftwx);
    inform("\tlast_opcode: 0x%x\n", fpu.last_opcode);
    inform("\tlast_ip: 0x%x\n", fpu.last_ip);
    inform("\tlast_dp: 0x%x\n", fpu.last_dp);
    inform("\tmxcsr: 0x%x\n", fpu.mxcsr);
    inform("\tFP Stack:\n");
    for (int i = 0; i < 8; ++i) {
        const bool empty(!((fpu.ftwx >> i) & 0x1));
        char hex[33];
        for (int j = 0; j < 16; ++j)
            snprintf(&hex[j*2], 3, "%.2x", fpu.fpr[i][j]);
        inform("\t\t%i: 0x%s%s\n", i, hex, empty ? " (e)" : "");
    }
    inform("\tXMM registers:\n");
    for (int i = 0; i < 16; ++i) {
        char hex[33];
        for (int j = 0; j < 16; ++j)
            snprintf(&hex[j*2], 3, "%.2x", fpu.xmm[i][j]);
        inform("\t\t%i: 0x%s\n", i, hex);
    }
}

static void
dumpKvm(const struct kvm_msrs &msrs)
{
    inform("MSRs:\n");

    for (int i = 0; i < msrs.nmsrs; ++i) {
        const struct kvm_msr_entry &e(msrs.entries[i]);

        inform("\t0x%x: 0x%x\n", e.index, e.data);
    }
}

static void
dumpKvm(const struct kvm_xcrs &regs)
{
    inform("KVM XCR registers:\n");

    inform("\tFlags: 0x%x\n", regs.flags);
    for (int i = 0; i < regs.nr_xcrs; ++i) {
        inform("\tXCR[0x%x]: 0x%x\n",
               regs.xcrs[i].xcr,
               regs.xcrs[i].value);
    }
}

static void
dumpKvm(const struct kvm_xsave &xsave)
{
    inform("KVM XSAVE:\n");

    Trace::dump((Tick)-1, "xsave.region",
                xsave.region, sizeof(xsave.region));
}

static void
dumpKvm(const struct kvm_vcpu_events &events)
{
    inform("vCPU events:\n");

    inform("\tException: [inj: %i, nr: %i, has_ec: %i, ec: %i]\n",
           events.exception.injected, events.exception.nr,
           events.exception.has_error_code, events.exception.error_code);

    inform("\tInterrupt: [inj: %i, nr: %i, soft: %i]\n",
           events.interrupt.injected, events.interrupt.nr,
           events.interrupt.soft);

    inform("\tNMI: [inj: %i, pending: %i, masked: %i]\n",
           events.nmi.injected, events.nmi.pending,
           events.nmi.masked);

    inform("\tSIPI vector: 0x%x\n", events.sipi_vector);
    inform("\tFlags: 0x%x\n", events.flags);
}

X86KvmCPU::X86KvmCPU(X86KvmCPUParams *params)
    : BaseKvmCPU(params)
{
    Kvm &kvm(vm.kvm);

    if (!kvm.capSetTSSAddress())
        panic("KVM: Missing capability (KVM_CAP_SET_TSS_ADDR)\n");
    if (!kvm.capExtendedCPUID())
        panic("KVM: Missing capability (KVM_CAP_EXT_CPUID)\n");
    if (!kvm.capUserNMI())
        warn("KVM: Missing capability (KVM_CAP_USER_NMI)\n");
    if (!kvm.capVCPUEvents())
        warn("KVM: Missing capability (KVM_CAP_VCPU_EVENTS)\n");

    haveDebugRegs = kvm.capDebugRegs();
    haveXSave = kvm.capXSave();
    haveXCRs = kvm.capXCRs();
}

X86KvmCPU::~X86KvmCPU()
{
}

void
X86KvmCPU::startup()
{
    BaseKvmCPU::startup();

    updateCPUID();

    io_req.setThreadContext(tc->contextId(), 0);

    // TODO: Do we need to create an identity mapped TSS area? We
    // should call kvm.vm.setTSSAddress() here in that case. It should
    // only be needed for old versions of the virtualization
    // extensions. We should make sure that the identity range is
    // reserved in the e820 memory map in that case.
}

void
X86KvmCPU::dump()
{
    dumpIntRegs();
    dumpFpuRegs();
    dumpSpecRegs();
    dumpDebugRegs();
    dumpXCRs();
    dumpVCpuEvents();
    dumpMSRs();
    dumpXSave();
}

void
X86KvmCPU::dumpFpuRegs() const
{
    struct kvm_fpu fpu;
    getFPUState(fpu);
    dumpKvm(fpu);
}

void
X86KvmCPU::dumpIntRegs() const
{
    struct kvm_regs regs;
    getRegisters(regs);
    dumpKvm(regs);
}

void
X86KvmCPU::dumpSpecRegs() const
{
    struct kvm_sregs sregs;
    getSpecialRegisters(sregs);
    dumpKvm(sregs);
}

void
X86KvmCPU::dumpDebugRegs() const
{
    if (haveDebugRegs) {
#ifdef KVM_GET_DEBUGREGS
        struct kvm_debugregs dregs;
        getDebugRegisters(dregs);
        dumpKvm(dregs);
#endif
    } else {
        inform("Debug registers not supported by kernel.\n");
    }
}

void
X86KvmCPU::dumpXCRs() const
{
    if (haveXCRs) {
        struct kvm_xcrs xcrs;
        getXCRs(xcrs);
        dumpKvm(xcrs);
    } else {
        inform("XCRs not supported by kernel.\n");
    }
}

void
X86KvmCPU::dumpXSave() const
{
    if (haveXSave) {
        struct kvm_xsave xsave;
        getXSave(xsave);
        dumpKvm(xsave);
    } else {
        inform("XSave not supported by kernel.\n");
    }
}

void
X86KvmCPU::dumpVCpuEvents() const
{
    struct kvm_vcpu_events events;
    getVCpuEvents(events);
    dumpKvm(events);
}

void
X86KvmCPU::dumpMSRs() const
{
    const Kvm::MSRIndexVector &supported_msrs(vm.kvm.getSupportedMSRs());
    std::unique_ptr<struct kvm_msrs> msrs(
        newVarStruct<struct kvm_msrs, struct kvm_msr_entry>(
            supported_msrs.size()));

    msrs->nmsrs = supported_msrs.size();
    for (int i = 0; i < supported_msrs.size(); ++i) {
        struct kvm_msr_entry &e(msrs->entries[i]);
        e.index = supported_msrs[i];
        e.reserved = 0;
        e.data = 0;
    }
    getMSRs(*msrs.get());

    dumpKvm(*msrs.get());
}

void
X86KvmCPU::updateKvmState()
{
    updateKvmStateRegs();
    updateKvmStateSRegs();
    updateKvmStateFPU();
    updateKvmStateMSRs();

    DPRINTF(KvmContext, "X86KvmCPU::updateKvmState():\n");
    if (DTRACE(KvmContext))
        dump();
}

void
X86KvmCPU::updateKvmStateRegs()
{
    struct kvm_regs regs;

#define APPLY_IREG(kreg, mreg) regs.kreg = tc->readIntReg(mreg)
    FOREACH_IREG();
#undef APPLY_IREG

    regs.rip = tc->instAddr();

    /* You might think that setting regs.rflags to the contents
     * MISCREG_RFLAGS here would suffice. In that case you're
     * mistaken. We need to reconstruct it from a bunch of ucode
     * registers and wave a dead chicken over it (aka mask out and set
     * reserved bits) to get it to work.
     */
    regs.rflags = X86ISA::getRFlags(tc);

    setRegisters(regs);
}

static inline void
setKvmSegmentReg(ThreadContext *tc, struct kvm_segment &kvm_seg,
                 const int index)
{
    SegAttr attr(tc->readMiscRegNoEffect(MISCREG_SEG_ATTR(index)));

    kvm_seg.base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(index));
    kvm_seg.limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(index));
    kvm_seg.selector = tc->readMiscRegNoEffect(MISCREG_SEG_SEL(index));
    kvm_seg.type = attr.type;
    kvm_seg.present = attr.present;
    kvm_seg.dpl = attr.dpl;
    kvm_seg.db = attr.defaultSize;
    kvm_seg.s = attr.system;
    kvm_seg.l = attr.longMode;
    kvm_seg.g = attr.granularity;
    kvm_seg.avl = attr.avl;

    // A segment is unusable when the selector is zero. There is a
    // attr.unusable flag in gem5, but it seems unused.
    //
    // TODO: Are there corner cases where this doesn't work?
    kvm_seg.unusable = (kvm_seg.selector == 0);
}

static inline void
setKvmDTableReg(ThreadContext *tc, struct kvm_dtable &kvm_dtable,
                const int index)
{
    kvm_dtable.base = tc->readMiscRegNoEffect(MISCREG_SEG_BASE(index));
    kvm_dtable.limit = tc->readMiscRegNoEffect(MISCREG_SEG_LIMIT(index));
}

void
X86KvmCPU::updateKvmStateSRegs()
{
    struct kvm_sregs sregs;

#define APPLY_SREG(kreg, mreg) sregs.kreg = tc->readMiscRegNoEffect(mreg)
#define APPLY_SEGMENT(kreg, idx) setKvmSegmentReg(tc, sregs.kreg, idx)
#define APPLY_DTABLE(kreg, idx) setKvmDTableReg(tc, sregs.kreg, idx)

    FOREACH_SREG();
    FOREACH_SEGMENT();
    FOREACH_DTABLE();

#undef APPLY_SREG
#undef APPLY_SEGMENT
#undef APPLY_DTABLE

    // Clear the interrupt bitmap
    memset(&sregs.interrupt_bitmap, 0, sizeof(sregs.interrupt_bitmap));

    setSpecialRegisters(sregs);
}
void
X86KvmCPU::updateKvmStateFPU()
{
    warn_once("X86KvmCPU::updateKvmStateFPU not implemented\n");
}

void
X86KvmCPU::updateKvmStateMSRs()
{
    KvmMSRVector msrs;

    const Kvm::MSRIndexVector &indices(getMsrIntersection());

    for (auto it = indices.cbegin(); it != indices.cend(); ++it) {
        struct kvm_msr_entry e;

        e.index = *it;
        e.reserved = 0;
        e.data = tc->readMiscReg(msrMap.at(*it));
        DPRINTF(KvmContext, "Adding MSR: idx: 0x%x, data: 0x%x\n",
                e.index, e.data);

        msrs.push_back(e);
    }

    setMSRs(msrs);
}

void
X86KvmCPU::updateThreadContext()
{
    DPRINTF(KvmContext, "X86KvmCPU::updateThreadContext():\n");
    if (DTRACE(KvmContext))
        dump();

    updateThreadContextRegs();
    updateThreadContextSRegs();
    updateThreadContextFPU();
    updateThreadContextMSRs();

    // The M5 misc reg caches some values from other
    // registers. Writing to it with side effects causes it to be
    // updated from its source registers.
    tc->setMiscReg(MISCREG_M5_REG, 0);
}

void
X86KvmCPU::updateThreadContextRegs()
{
    struct kvm_regs regs;
    getRegisters(regs);

#define APPLY_IREG(kreg, mreg) tc->setIntReg(mreg, regs.kreg)

    FOREACH_IREG();

#undef APPLY_IREG

    tc->pcState(PCState(regs.rip));

    // Flags are spread out across multiple semi-magic registers so we
    // need some special care when updating them.
    X86ISA::setRFlags(tc, regs.rflags);
}


inline void
setContextSegment(ThreadContext *tc, const struct kvm_segment &kvm_seg,
                  const int index)
{
    SegAttr attr(0);

    attr.type = kvm_seg.type;
    attr.present = kvm_seg.present;
    attr.dpl = kvm_seg.dpl;
    attr.defaultSize = kvm_seg.db;
    attr.system = kvm_seg.s;
    attr.longMode = kvm_seg.l;
    attr.granularity = kvm_seg.g;
    attr.avl = kvm_seg.avl;
    attr.unusable = kvm_seg.unusable;

    // We need some setMiscReg magic here to keep the effective base
    // addresses in sync. We need an up-to-date version of EFER, so
    // make sure this is called after the sregs have been synced.
    tc->setMiscReg(MISCREG_SEG_BASE(index), kvm_seg.base);
    tc->setMiscReg(MISCREG_SEG_LIMIT(index), kvm_seg.limit);
    tc->setMiscReg(MISCREG_SEG_SEL(index), kvm_seg.selector);
    tc->setMiscReg(MISCREG_SEG_ATTR(index), attr);
}

inline void
setContextSegment(ThreadContext *tc, const struct kvm_dtable &kvm_dtable,
                  const int index)
{
    // We need some setMiscReg magic here to keep the effective base
    // addresses in sync. We need an up-to-date version of EFER, so
    // make sure this is called after the sregs have been synced.
    tc->setMiscReg(MISCREG_SEG_BASE(index), kvm_dtable.base);
    tc->setMiscReg(MISCREG_SEG_LIMIT(index), kvm_dtable.limit);
}

void
X86KvmCPU::updateThreadContextSRegs()
{
    struct kvm_sregs sregs;
    getSpecialRegisters(sregs);

    assert(getKvmRunState()->apic_base == sregs.apic_base);
    assert(getKvmRunState()->cr8 == sregs.cr8);

#define APPLY_SREG(kreg, mreg) tc->setMiscRegNoEffect(mreg, sregs.kreg)
#define APPLY_SEGMENT(kreg, idx) setContextSegment(tc, sregs.kreg, idx)
#define APPLY_DTABLE(kreg, idx) setContextSegment(tc, sregs.kreg, idx)
    FOREACH_SREG();
    FOREACH_SEGMENT();
    FOREACH_DTABLE();
#undef APPLY_SREG
#undef APPLY_SEGMENT
#undef APPLY_DTABLE
}

void
X86KvmCPU::updateThreadContextFPU()
{
    warn_once("X86KvmCPU::updateThreadContextFPU not implemented\n");
}

void
X86KvmCPU::updateThreadContextMSRs()
{
    const Kvm::MSRIndexVector &msrs(getMsrIntersection());

    std::unique_ptr<struct kvm_msrs> kvm_msrs(
        newVarStruct<struct kvm_msrs, struct kvm_msr_entry>(msrs.size()));
    struct kvm_msr_entry *entry;

    // Create a list of MSRs to read
    kvm_msrs->nmsrs = msrs.size();
    entry = &kvm_msrs->entries[0];
    for (auto it = msrs.cbegin(); it != msrs.cend(); ++it, ++entry) {
        entry->index = *it;
        entry->reserved = 0;
        entry->data = 0;
    }

    getMSRs(*kvm_msrs.get());

    // Update M5's state
    entry = &kvm_msrs->entries[0];
    for (int i = 0; i < kvm_msrs->nmsrs; ++i, ++entry) {
        DPRINTF(KvmContext, "Setting M5 MSR: idx: 0x%x, data: 0x%x\n",
                entry->index, entry->data);

        tc->setMiscReg(X86ISA::msrMap.at(entry->index), entry->data);
    }
}

void
X86KvmCPU::deliverInterrupts()
{
    syncThreadContext();

    Fault fault(interrupts->getInterrupt(tc));
    interrupts->updateIntrInfo(tc);

    X86Interrupt *x86int(dynamic_cast<X86Interrupt *>(fault.get()));
    if (x86int) {
        struct kvm_interrupt kvm_int;
        kvm_int.irq = x86int->getVector();

        DPRINTF(KvmInt, "Delivering interrupt: %s (%u)\n",
                fault->name(), kvm_int.irq);

        kvmInterrupt(kvm_int);
    } else if (dynamic_cast<NonMaskableInterrupt *>(fault.get())) {
        DPRINTF(KvmInt, "Delivering NMI\n");
        kvmNonMaskableInterrupt();
    } else {
        panic("KVM: Unknown interrupt type\n");
    }

}

Tick
X86KvmCPU::kvmRun(Tick ticks)
{
    struct kvm_run &kvm_run(*getKvmRunState());

    if (interrupts->checkInterruptsRaw()) {
        if (kvm_run.ready_for_interrupt_injection) {
            // KVM claims that it is ready for an interrupt. It might
            // be lying if we just updated rflags and disabled
            // interrupts (e.g., by doing a CPU handover). Let's sync
            // the thread context and check if there are /really/
            // interrupts that should be delivered now.
            syncThreadContext();
            if (interrupts->checkInterrupts(tc)) {
                DPRINTF(KvmInt,
                        "M5 has pending interrupts, delivering interrupt.\n");

                deliverInterrupts();
            } else {
                DPRINTF(KvmInt,
                        "Interrupt delivery delayed due to KVM confusion.\n");
                kvm_run.request_interrupt_window = 1;
            }
        } else if (!kvm_run.request_interrupt_window) {
            DPRINTF(KvmInt,
                    "M5 has pending interrupts, requesting interrupt "
                    "window.\n");
            kvm_run.request_interrupt_window = 1;
        }
    } else {
        kvm_run.request_interrupt_window = 0;
    }

    return kvmRunWrapper(ticks);
}

Tick
X86KvmCPU::kvmRunDrain()
{
    struct kvm_run &kvm_run(*getKvmRunState());

    if (!archIsDrained()) {
        DPRINTF(Drain, "kvmRunDrain: Architecture code isn't drained\n");

        // Tell KVM to find a suitable place to deliver interrupts. This
        // should ensure that pending interrupts have been delivered and
        // things are reasonably consistent (i.e., no interrupts pending
        // in the guest).
        kvm_run.request_interrupt_window = 1;

        // Limit the run to 1 millisecond. That is hopefully enough to
        // reach an interrupt window. Otherwise, we'll just try again
        // later.
        return kvmRunWrapper(1 * SimClock::Float::ms);
    } else {
        DPRINTF(Drain, "kvmRunDrain: Delivering pending IO\n");

        return kvmRunWrapper(0);
    }
}

Tick
X86KvmCPU::kvmRunWrapper(Tick ticks)
{
    struct kvm_run &kvm_run(*getKvmRunState());

    // Synchronize the APIC base and CR8 here since they are present
    // in the kvm_run struct, which makes the synchronization really
    // cheap.
    kvm_run.apic_base = tc->readMiscReg(MISCREG_APIC_BASE);
    kvm_run.cr8 = tc->readMiscReg(MISCREG_CR8);

    const Tick run_ticks(BaseKvmCPU::kvmRun(ticks));

    tc->setMiscReg(MISCREG_APIC_BASE, kvm_run.apic_base);
    kvm_run.cr8 = tc->readMiscReg(MISCREG_CR8);

    return run_ticks;
}

uint64_t
X86KvmCPU::getHostCycles() const
{
    return getMSR(MSR_TSC);
}

void
X86KvmCPU::handleIOMiscReg32(int miscreg)
{
    struct kvm_run &kvm_run(*getKvmRunState());
    const uint16_t port(kvm_run.io.port);

    assert(kvm_run.exit_reason == KVM_EXIT_IO);

    if (kvm_run.io.size != 4) {
        panic("Unexpected IO size (%u) for address 0x%x.\n",
              kvm_run.io.size, port);
    }

    if (kvm_run.io.count != 1) {
        panic("Unexpected IO count (%u) for address 0x%x.\n",
              kvm_run.io.count, port);
    }

    uint32_t *data((uint32_t *)getGuestData(kvm_run.io.data_offset));
    if (kvm_run.io.direction == KVM_EXIT_IO_OUT)
        tc->setMiscReg(miscreg, *data);
    else
        *data = tc->readMiscRegNoEffect(miscreg);
}

Tick
X86KvmCPU::handleKvmExitIO()
{
    struct kvm_run &kvm_run(*getKvmRunState());
    bool isWrite(kvm_run.io.direction == KVM_EXIT_IO_OUT);
    unsigned char *guestData(getGuestData(kvm_run.io.data_offset));
    Tick delay(0);
    uint16_t port(kvm_run.io.port);
    Addr pAddr;
    const int count(kvm_run.io.count);

    assert(kvm_run.io.direction == KVM_EXIT_IO_IN ||
           kvm_run.io.direction == KVM_EXIT_IO_OUT);

    DPRINTF(KvmIO, "KVM-x86: Handling IO instruction (%s) (port: 0x%x)\n",
            (isWrite ? "out" : "in"), kvm_run.io.port);

    /* Vanilla gem5 handles PCI discovery in the TLB(!). Since we
     * don't use the TLB component, we need to intercept and handle
     * the PCI configuration space IO ports here.
     *
     * The IO port PCI discovery mechanism uses one address register
     * and one data register. We map the address register to a misc
     * reg and use that to re-route data register accesses to the
     * right location in the PCI configuration space.
     */
    if (port == IO_PCI_CONF_ADDR) {
        handleIOMiscReg32(MISCREG_PCI_CONFIG_ADDRESS);
        return 0;
    } else if ((port & ~0x3) == IO_PCI_CONF_DATA_BASE) {
        Addr pciConfigAddr(tc->readMiscRegNoEffect(MISCREG_PCI_CONFIG_ADDRESS));
        if (pciConfigAddr & 0x80000000) {
            pAddr = X86ISA::x86PciConfigAddress((pciConfigAddr & 0x7ffffffc) |
                                                (port & 0x3));
        } else {
            pAddr = X86ISA::x86IOAddress(port);
        }
    } else {
        pAddr = X86ISA::x86IOAddress(port);
    }

    io_req.setPhys(pAddr, kvm_run.io.size, Request::UNCACHEABLE,
                   dataMasterId());

    const MemCmd cmd(isWrite ? MemCmd::WriteReq : MemCmd::ReadReq);
    for (int i = 0; i < count; ++i) {
        Packet pkt(&io_req, cmd);

        pkt.dataStatic(guestData);
        delay += dataPort.sendAtomic(&pkt);

        guestData += kvm_run.io.size;
    }

    return delay;
}

Tick
X86KvmCPU::handleKvmExitIRQWindowOpen()
{
    // We don't need to do anything here since this is caught the next
    // time we execute kvmRun(). We still overload the exit event to
    // silence the warning about an unhandled exit event.
    return 0;
}

bool
X86KvmCPU::archIsDrained() const
{
    struct kvm_vcpu_events events;

    getVCpuEvents(events);

    // We could probably handle this in a by re-inserting interrupts
    // that are pending into gem5 on a drain. However, that would
    // probably be tricky to do reliably, so we'll just prevent a
    // drain if there is anything pending in the
    // guest. X86KvmCPU::kvmRunDrain() minimizes the amount of code
    // executed in the guest by requesting an interrupt window if
    // there are pending interrupts.
    const bool pending_events(events.exception.injected ||
                              events.interrupt.injected ||
                              events.nmi.injected || events.nmi.pending);

    if (pending_events) {
        DPRINTF(Drain, "archIsDrained: Pending events: %s %s %s %s\n",
                events.exception.injected ? "exception" : "",
                events.interrupt.injected ? "interrupt" : "",
                events.nmi.injected ? "nmi[i]" : "",
                events.nmi.pending ? "nmi[p]" : "");
    }

    return !pending_events;
}

static struct kvm_cpuid_entry2
makeKvmCpuid(uint32_t function, uint32_t index,
             CpuidResult &result)
{
    struct kvm_cpuid_entry2 e;
    e.function = function;
    e.index = index;
    e.flags = 0;
    e.eax = (uint32_t)result.rax;
    e.ebx = (uint32_t)result.rbx;
    e.ecx = (uint32_t)result.rcx;
    e.edx = (uint32_t)result.rdx;

    return e;
}

void
X86KvmCPU::updateCPUID()
{
    Kvm::CPUIDVector m5_supported;

    /* TODO: We currently don't support any of the functions that
     * iterate through data structures in the CPU using an index. It's
     * currently not a problem since M5 doesn't expose any of them at
     * the moment.
     */

    /* Basic features */
    CpuidResult func0;
    X86ISA::doCpuid(tc, 0x0, 0, func0);
    for (uint32_t function = 0; function <= func0.rax; ++function) {
        CpuidResult cpuid;
        uint32_t idx(0);

        X86ISA::doCpuid(tc, function, idx, cpuid);
        m5_supported.push_back(makeKvmCpuid(function, idx, cpuid));
    }

    /* Extended features */
    CpuidResult efunc0;
    X86ISA::doCpuid(tc, 0x80000000, 0, efunc0);
    for (uint32_t function = 0x80000000; function <= efunc0.rax; ++function) {
        CpuidResult cpuid;
        uint32_t idx(0);

        X86ISA::doCpuid(tc, function, idx, cpuid);
        m5_supported.push_back(makeKvmCpuid(function, idx, cpuid));
    }

    setCPUID(m5_supported);
}

void
X86KvmCPU::setCPUID(const struct kvm_cpuid2 &cpuid)
{
    if (ioctl(KVM_SET_CPUID2, (void *)&cpuid) == -1)
        panic("KVM: Failed to set guest CPUID2 (errno: %i)\n",
              errno);
}

void
X86KvmCPU::setCPUID(const Kvm::CPUIDVector &cpuid)
{
    std::unique_ptr<struct kvm_cpuid2> kvm_cpuid(
        newVarStruct<struct kvm_cpuid2, struct kvm_cpuid_entry2>(cpuid.size()));

    kvm_cpuid->nent = cpuid.size();
    std::copy(cpuid.begin(), cpuid.end(), kvm_cpuid->entries);

    setCPUID(*kvm_cpuid);
}

void
X86KvmCPU::setMSRs(const struct kvm_msrs &msrs)
{
    if (ioctl(KVM_SET_MSRS, (void *)&msrs) == -1)
        panic("KVM: Failed to set guest MSRs (errno: %i)\n",
              errno);
}

void
X86KvmCPU::setMSRs(const KvmMSRVector &msrs)
{
    std::unique_ptr<struct kvm_msrs> kvm_msrs(
        newVarStruct<struct kvm_msrs, struct kvm_msr_entry>(msrs.size()));

    kvm_msrs->nmsrs = msrs.size();
    std::copy(msrs.begin(), msrs.end(), kvm_msrs->entries);

    setMSRs(*kvm_msrs);
}

void
X86KvmCPU::getMSRs(struct kvm_msrs &msrs) const
{
    if (ioctl(KVM_GET_MSRS, (void *)&msrs) == -1)
        panic("KVM: Failed to get guest MSRs (errno: %i)\n",
              errno);
}


void
X86KvmCPU::setMSR(uint32_t index, uint64_t value)
{
    std::unique_ptr<struct kvm_msrs> kvm_msrs(
        newVarStruct<struct kvm_msrs, struct kvm_msr_entry>(1));
    struct kvm_msr_entry &entry(kvm_msrs->entries[0]);

    kvm_msrs->nmsrs = 1;
    entry.index = index;
    entry.reserved = 0;
    entry.data = value;

    setMSRs(*kvm_msrs.get());
}

uint64_t
X86KvmCPU::getMSR(uint32_t index) const
{
    std::unique_ptr<struct kvm_msrs> kvm_msrs(
        newVarStruct<struct kvm_msrs, struct kvm_msr_entry>(1));
    struct kvm_msr_entry &entry(kvm_msrs->entries[0]);

    kvm_msrs->nmsrs = 1;
    entry.index = index;
    entry.reserved = 0;
    entry.data = 0;

    getMSRs(*kvm_msrs.get());
    return entry.data;
}

const Kvm::MSRIndexVector &
X86KvmCPU::getMsrIntersection() const
{
    if (cachedMsrIntersection.empty()) {
        const Kvm::MSRIndexVector &kvm_msrs(vm.kvm.getSupportedMSRs());

        DPRINTF(Kvm, "kvm-x86: Updating MSR intersection\n");
        for (auto it = kvm_msrs.cbegin(); it != kvm_msrs.cend(); ++it) {
            if (X86ISA::msrMap.find(*it) != X86ISA::msrMap.end()) {
                cachedMsrIntersection.push_back(*it);
                DPRINTF(Kvm, "kvm-x86: Adding MSR 0x%x\n", *it);
            } else {
                warn("kvm-x86: MSR (0x%x) unsupported by gem5. Skipping.\n",
                     *it);
            }
        }
    }

    return cachedMsrIntersection;
}

void
X86KvmCPU::getDebugRegisters(struct kvm_debugregs &regs) const
{
#ifdef KVM_GET_DEBUGREGS
    if (ioctl(KVM_GET_DEBUGREGS, &regs) == -1)
        panic("KVM: Failed to get guest debug registers\n");
#else
    panic("KVM: Unsupported getDebugRegisters call.\n");
#endif
}

void
X86KvmCPU::setDebugRegisters(const struct kvm_debugregs &regs)
{
#ifdef KVM_SET_DEBUGREGS
    if (ioctl(KVM_SET_DEBUGREGS, (void *)&regs) == -1)
        panic("KVM: Failed to set guest debug registers\n");
#else
    panic("KVM: Unsupported setDebugRegisters call.\n");
#endif
}

void
X86KvmCPU::getXCRs(struct kvm_xcrs &regs) const
{
    if (ioctl(KVM_GET_XCRS, &regs) == -1)
        panic("KVM: Failed to get guest debug registers\n");
}

void
X86KvmCPU::setXCRs(const struct kvm_xcrs &regs)
{
    if (ioctl(KVM_SET_XCRS, (void *)&regs) == -1)
        panic("KVM: Failed to set guest debug registers\n");
}

void
X86KvmCPU::getXSave(struct kvm_xsave &xsave) const
{
    if (ioctl(KVM_GET_XSAVE, &xsave) == -1)
        panic("KVM: Failed to get guest debug registers\n");
}

void
X86KvmCPU::setXSave(const struct kvm_xsave &xsave)
{
    if (ioctl(KVM_SET_XSAVE, (void *)&xsave) == -1)
        panic("KVM: Failed to set guest debug registers\n");
}


void
X86KvmCPU::getVCpuEvents(struct kvm_vcpu_events &events) const
{
    if (ioctl(KVM_GET_VCPU_EVENTS, &events) == -1)
        panic("KVM: Failed to get guest debug registers\n");
}

void
X86KvmCPU::setVCpuEvents(const struct kvm_vcpu_events &events)
{
    if (ioctl(KVM_SET_VCPU_EVENTS, (void *)&events) == -1)
        panic("KVM: Failed to set guest debug registers\n");
}

X86KvmCPU *
X86KvmCPUParams::create()
{
    return new X86KvmCPU(this);
}
