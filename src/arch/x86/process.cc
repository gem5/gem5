/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 *          Ali Saidi
 */

#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/segment.hh"
#include "arch/x86/isa_traits.hh"
#include "arch/x86/process.hh"
#include "arch/x86/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

using namespace std;
using namespace X86ISA;

static const int ArgumentReg[] = {
    INTREG_RDI,
    INTREG_RSI,
    INTREG_RDX,
    //This argument register is r10 for syscalls and rcx for C.
    INTREG_R10W,
    //INTREG_RCX,
    INTREG_R8W,
    INTREG_R9W
};
static const int NumArgumentRegs = sizeof(ArgumentReg) / sizeof(const int);
static const int ArgumentReg32[] = {
    INTREG_EBX,
    INTREG_ECX,
    INTREG_EDX,
    INTREG_ESI,
    INTREG_EDI,
};
static const int NumArgumentRegs32 = sizeof(ArgumentReg) / sizeof(const int);

X86LiveProcess::X86LiveProcess(LiveProcessParams * params, ObjectFile *objFile,
        SyscallDesc *_syscallDescs, int _numSyscallDescs) :
    LiveProcess(params, objFile), syscallDescs(_syscallDescs),
    numSyscallDescs(_numSyscallDescs)
{
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, VMPageSize);
}

X86_64LiveProcess::X86_64LiveProcess(LiveProcessParams *params,
        ObjectFile *objFile, SyscallDesc *_syscallDescs,
        int _numSyscallDescs) :
    X86LiveProcess(params, objFile, _syscallDescs, _numSyscallDescs)
{

    vsyscallPage.base = 0xffffffffff600000ULL;
    vsyscallPage.size = VMPageSize;
    vsyscallPage.vtimeOffset = 0x400;
    vsyscallPage.vgettimeofdayOffset = 0x0;

    // Set up stack. On X86_64 Linux, stack goes from the top of memory
    // downward, less the hole for the kernel address space plus one page
    // for undertermined purposes.
    stack_base = (Addr)0x7FFFFFFFF000ULL;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    // Set up region for mmaps. This was determined empirically and may not
    // always be correct.
    mmap_start = mmap_end = (Addr)0x2aaaaaaab000ULL;
}

void
I386LiveProcess::syscall(int64_t callnum, ThreadContext *tc)
{
    TheISA::PCState pc = tc->pcState();
    Addr eip = pc.pc();
    if (eip >= vsyscallPage.base &&
            eip < vsyscallPage.base + vsyscallPage.size) {
        pc.npc(vsyscallPage.base + vsyscallPage.vsysexitOffset);
        tc->pcState(pc);
    }
    X86LiveProcess::syscall(callnum, tc);
}


I386LiveProcess::I386LiveProcess(LiveProcessParams *params,
        ObjectFile *objFile, SyscallDesc *_syscallDescs,
        int _numSyscallDescs) :
    X86LiveProcess(params, objFile, _syscallDescs, _numSyscallDescs)
{
    _gdtStart = ULL(0xffffd000);
    _gdtSize = VMPageSize;

    vsyscallPage.base = 0xffffe000ULL;
    vsyscallPage.size = VMPageSize;
    vsyscallPage.vsyscallOffset = 0x400;
    vsyscallPage.vsysexitOffset = 0x410;

    stack_base = _gdtStart;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    // Set up region for mmaps. This was determined empirically and may not
    // always be correct.
    mmap_start = mmap_end = (Addr)0xf7ffe000ULL;
}

SyscallDesc*
X86LiveProcess::getDesc(int callnum)
{
    if (callnum < 0 || callnum >= numSyscallDescs)
        return NULL;
    return &syscallDescs[callnum];
}

void
X86_64LiveProcess::initState()
{
    X86LiveProcess::initState();

    argsInit(sizeof(uint64_t), VMPageSize);

       // Set up the vsyscall page for this process.
    allocateMem(vsyscallPage.base, vsyscallPage.size);
    uint8_t vtimeBlob[] = {
        0x48,0xc7,0xc0,0xc9,0x00,0x00,0x00,    // mov    $0xc9,%rax
        0x0f,0x05,                             // syscall
        0xc3                                   // retq
    };
    initVirtMem.writeBlob(vsyscallPage.base + vsyscallPage.vtimeOffset,
            vtimeBlob, sizeof(vtimeBlob));

    uint8_t vgettimeofdayBlob[] = {
        0x48,0xc7,0xc0,0x60,0x00,0x00,0x00,    // mov    $0x60,%rax
        0x0f,0x05,                             // syscall
        0xc3                                   // retq
    };
    initVirtMem.writeBlob(vsyscallPage.base + vsyscallPage.vgettimeofdayOffset,
            vgettimeofdayBlob, sizeof(vgettimeofdayBlob));

    for (int i = 0; i < contextIds.size(); i++) {
        ThreadContext * tc = system->getThreadContext(contextIds[i]);

        SegAttr dataAttr = 0;
        dataAttr.dpl = 3;
        dataAttr.unusable = 0;
        dataAttr.defaultSize = 1;
        dataAttr.longMode = 1;
        dataAttr.avl = 0;
        dataAttr.granularity = 1;
        dataAttr.present = 1;
        dataAttr.type = 3;
        dataAttr.writable = 1;
        dataAttr.readable = 1;
        dataAttr.expandDown = 0;
        dataAttr.system = 1;

        //Initialize the segment registers.
        for(int seg = 0; seg < NUM_SEGMENTREGS; seg++) {
            tc->setMiscRegNoEffect(MISCREG_SEG_BASE(seg), 0);
            tc->setMiscRegNoEffect(MISCREG_SEG_EFF_BASE(seg), 0);
            tc->setMiscRegNoEffect(MISCREG_SEG_ATTR(seg), dataAttr);
        }

        SegAttr csAttr = 0;
        csAttr.dpl = 3;
        csAttr.unusable = 0;
        csAttr.defaultSize = 0;
        csAttr.longMode = 1;
        csAttr.avl = 0;
        csAttr.granularity = 1;
        csAttr.present = 1;
        csAttr.type = 10;
        csAttr.writable = 0;
        csAttr.readable = 1;
        csAttr.expandDown = 0;
        csAttr.system = 1;

        tc->setMiscRegNoEffect(MISCREG_CS_ATTR, csAttr);

        Efer efer = 0;
        efer.sce = 1; // Enable system call extensions.
        efer.lme = 1; // Enable long mode.
        efer.lma = 1; // Activate long mode.
        efer.nxe = 1; // Enable nx support.
        efer.svme = 0; // Disable svm support for now. It isn't implemented.
        efer.ffxsr = 1; // Turn on fast fxsave and fxrstor.
        tc->setMiscReg(MISCREG_EFER, efer);

        //Set up the registers that describe the operating mode.
        CR0 cr0 = 0;
        cr0.pg = 1; // Turn on paging.
        cr0.cd = 0; // Don't disable caching.
        cr0.nw = 0; // This is bit is defined to be ignored.
        cr0.am = 0; // No alignment checking
        cr0.wp = 0; // Supervisor mode can write read only pages
        cr0.ne = 1;
        cr0.et = 1; // This should always be 1
        cr0.ts = 0; // We don't do task switching, so causing fp exceptions
                    // would be pointless.
        cr0.em = 0; // Allow x87 instructions to execute natively.
        cr0.mp = 1; // This doesn't really matter, but the manual suggests
                    // setting it to one.
        cr0.pe = 1; // We're definitely in protected mode.
        tc->setMiscReg(MISCREG_CR0, cr0);

        tc->setMiscReg(MISCREG_MXCSR, 0x1f80);
    }
}

void
I386LiveProcess::initState()
{
    X86LiveProcess::initState();

    argsInit(sizeof(uint32_t), VMPageSize);

    /* 
     * Set up a GDT for this process. The whole GDT wouldn't really be for
     * this process, but the only parts we care about are.
     */
    allocateMem(_gdtStart, _gdtSize);
    uint64_t zero = 0;
    assert(_gdtSize % sizeof(zero) == 0);
    for (Addr gdtCurrent = _gdtStart;
            gdtCurrent < _gdtStart + _gdtSize; gdtCurrent += sizeof(zero)) {
        initVirtMem.write(gdtCurrent, zero);
    }

    // Set up the vsyscall page for this process.
    allocateMem(vsyscallPage.base, vsyscallPage.size);
    uint8_t vsyscallBlob[] = {
        0x51,       // push %ecx
        0x52,       // push %edp
        0x55,       // push %ebp
        0x89, 0xe5, // mov %esp, %ebp
        0x0f, 0x34  // sysenter
    };
    initVirtMem.writeBlob(vsyscallPage.base + vsyscallPage.vsyscallOffset,
            vsyscallBlob, sizeof(vsyscallBlob));

    uint8_t vsysexitBlob[] = {
        0x5d,       // pop %ebp
        0x5a,       // pop %edx
        0x59,       // pop %ecx
        0xc3        // ret
    };
    initVirtMem.writeBlob(vsyscallPage.base + vsyscallPage.vsysexitOffset,
            vsysexitBlob, sizeof(vsysexitBlob));

    for (int i = 0; i < contextIds.size(); i++) {
        ThreadContext * tc = system->getThreadContext(contextIds[i]);

        SegAttr dataAttr = 0;
        dataAttr.dpl = 3;
        dataAttr.unusable = 0;
        dataAttr.defaultSize = 1;
        dataAttr.longMode = 0;
        dataAttr.avl = 0;
        dataAttr.granularity = 1;
        dataAttr.present = 1;
        dataAttr.type = 3;
        dataAttr.writable = 1;
        dataAttr.readable = 1;
        dataAttr.expandDown = 0;
        dataAttr.system = 1;

        //Initialize the segment registers.
        for(int seg = 0; seg < NUM_SEGMENTREGS; seg++) {
            tc->setMiscRegNoEffect(MISCREG_SEG_BASE(seg), 0);
            tc->setMiscRegNoEffect(MISCREG_SEG_EFF_BASE(seg), 0);
            tc->setMiscRegNoEffect(MISCREG_SEG_ATTR(seg), dataAttr);
            tc->setMiscRegNoEffect(MISCREG_SEG_SEL(seg), 0xB);
            tc->setMiscRegNoEffect(MISCREG_SEG_LIMIT(seg), (uint32_t)(-1));
        }

        SegAttr csAttr = 0;
        csAttr.dpl = 3;
        csAttr.unusable = 0;
        csAttr.defaultSize = 1;
        csAttr.longMode = 0;
        csAttr.avl = 0;
        csAttr.granularity = 1;
        csAttr.present = 1;
        csAttr.type = 0xa;
        csAttr.writable = 0;
        csAttr.readable = 1;
        csAttr.expandDown = 0;
        csAttr.system = 1;

        tc->setMiscRegNoEffect(MISCREG_CS_ATTR, csAttr);

        tc->setMiscRegNoEffect(MISCREG_TSG_BASE, _gdtStart);
        tc->setMiscRegNoEffect(MISCREG_TSG_EFF_BASE, _gdtStart);
        tc->setMiscRegNoEffect(MISCREG_TSG_LIMIT, _gdtStart + _gdtSize - 1);

        // Set the LDT selector to 0 to deactivate it.
        tc->setMiscRegNoEffect(MISCREG_TSL, 0);

        Efer efer = 0;
        efer.sce = 1; // Enable system call extensions.
        efer.lme = 1; // Enable long mode.
        efer.lma = 0; // Deactivate long mode.
        efer.nxe = 1; // Enable nx support.
        efer.svme = 0; // Disable svm support for now. It isn't implemented.
        efer.ffxsr = 1; // Turn on fast fxsave and fxrstor.
        tc->setMiscReg(MISCREG_EFER, efer);

        //Set up the registers that describe the operating mode.
        CR0 cr0 = 0;
        cr0.pg = 1; // Turn on paging.
        cr0.cd = 0; // Don't disable caching.
        cr0.nw = 0; // This is bit is defined to be ignored.
        cr0.am = 0; // No alignment checking
        cr0.wp = 0; // Supervisor mode can write read only pages
        cr0.ne = 1;
        cr0.et = 1; // This should always be 1
        cr0.ts = 0; // We don't do task switching, so causing fp exceptions
                    // would be pointless.
        cr0.em = 0; // Allow x87 instructions to execute natively.
        cr0.mp = 1; // This doesn't really matter, but the manual suggests
                    // setting it to one.
        cr0.pe = 1; // We're definitely in protected mode.
        tc->setMiscReg(MISCREG_CR0, cr0);

        tc->setMiscReg(MISCREG_MXCSR, 0x1f80);
    }
}

template<class IntType>
void
X86LiveProcess::argsInit(int pageSize,
        std::vector<AuxVector<IntType> > extraAuxvs)
{
    int intSize = sizeof(IntType);

    typedef AuxVector<IntType> auxv_t;
    std::vector<auxv_t> auxv = extraAuxvs;

    string filename;
    if(argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    //We want 16 byte alignment
    uint64_t align = 16;

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    enum X86CpuFeature {
        X86_OnboardFPU = 1 << 0,
        X86_VirtualModeExtensions = 1 << 1,
        X86_DebuggingExtensions = 1 << 2,
        X86_PageSizeExtensions = 1 << 3,

        X86_TimeStampCounter = 1 << 4,
        X86_ModelSpecificRegisters = 1 << 5,
        X86_PhysicalAddressExtensions = 1 << 6,
        X86_MachineCheckExtensions = 1 << 7,

        X86_CMPXCHG8Instruction = 1 << 8,
        X86_OnboardAPIC = 1 << 9,
        X86_SYSENTER_SYSEXIT = 1 << 11,

        X86_MemoryTypeRangeRegisters = 1 << 12,
        X86_PageGlobalEnable = 1 << 13,
        X86_MachineCheckArchitecture = 1 << 14,
        X86_CMOVInstruction = 1 << 15,

        X86_PageAttributeTable = 1 << 16,
        X86_36BitPSEs = 1 << 17,
        X86_ProcessorSerialNumber = 1 << 18,
        X86_CLFLUSHInstruction = 1 << 19,

        X86_DebugTraceStore = 1 << 21,
        X86_ACPIViaMSR = 1 << 22,
        X86_MultimediaExtensions = 1 << 23,

        X86_FXSAVE_FXRSTOR = 1 << 24,
        X86_StreamingSIMDExtensions = 1 << 25,
        X86_StreamingSIMDExtensions2 = 1 << 26,
        X86_CPUSelfSnoop = 1 << 27,

        X86_HyperThreading = 1 << 28,
        X86_AutomaticClockControl = 1 << 29,
        X86_IA64Processor = 1 << 30
    };

    //Setup the auxilliary vectors. These will already have endian conversion.
    //Auxilliary vectors are loaded only for elf formatted executables.
    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if(elfObject)
    {
        uint64_t features =
            X86_OnboardFPU |
            X86_VirtualModeExtensions |
            X86_DebuggingExtensions |
            X86_PageSizeExtensions |
            X86_TimeStampCounter |
            X86_ModelSpecificRegisters |
            X86_PhysicalAddressExtensions |
            X86_MachineCheckExtensions |
            X86_CMPXCHG8Instruction |
            X86_OnboardAPIC |
            X86_SYSENTER_SYSEXIT |
            X86_MemoryTypeRangeRegisters |
            X86_PageGlobalEnable |
            X86_MachineCheckArchitecture |
            X86_CMOVInstruction |
            X86_PageAttributeTable |
            X86_36BitPSEs |
//            X86_ProcessorSerialNumber |
            X86_CLFLUSHInstruction |
//            X86_DebugTraceStore |
//            X86_ACPIViaMSR |
            X86_MultimediaExtensions |
            X86_FXSAVE_FXRSTOR |
            X86_StreamingSIMDExtensions |
            X86_StreamingSIMDExtensions2 |
//            X86_CPUSelfSnoop |
//            X86_HyperThreading |
//            X86_AutomaticClockControl |
//            X86_IA64Processor |
            0;

        //Bits which describe the system hardware capabilities
        //XXX Figure out what these should be
        auxv.push_back(auxv_t(M5_AT_HWCAP, features));
        //The system page size
        auxv.push_back(auxv_t(M5_AT_PAGESZ, X86ISA::VMPageSize));
        //Frequency at which times() increments
        //Defined to be 100 in the kernel source.
        auxv.push_back(auxv_t(M5_AT_CLKTCK, 100));
        // For statically linked executables, this is the virtual address of the
        // program header tables if they appear in the executable image
        auxv.push_back(auxv_t(M5_AT_PHDR, elfObject->programHeaderTable()));
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(M5_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(M5_AT_PHNUM, elfObject->programHeaderCount()));
        //This is the address of the elf "interpreter", It should be set
        //to 0 for regular executables. It should be something else
        //(not sure what) for dynamic libraries.
        auxv.push_back(auxv_t(M5_AT_BASE, 0));

        //XXX Figure out what this should be.
        auxv.push_back(auxv_t(M5_AT_FLAGS, 0));
        //The entry point to the program
        auxv.push_back(auxv_t(M5_AT_ENTRY, objFile->entryPoint()));
        //Different user and group IDs
        auxv.push_back(auxv_t(M5_AT_UID, uid()));
        auxv.push_back(auxv_t(M5_AT_EUID, euid()));
        auxv.push_back(auxv_t(M5_AT_GID, gid()));
        auxv.push_back(auxv_t(M5_AT_EGID, egid()));
        //Whether to enable "secure mode" in the executable
        auxv.push_back(auxv_t(M5_AT_SECURE, 0));
        //The address of 16 "random" bytes.
        auxv.push_back(auxv_t(M5_AT_RANDOM, 0));
        //The name of the program
        auxv.push_back(auxv_t(M5_AT_EXECFN, 0));
        //The platform string
        auxv.push_back(auxv_t(M5_AT_PLATFORM, 0));
    }

    //Figure out how big the initial stack needs to be

    // A sentry NULL void pointer at the top of the stack.
    int sentry_size = intSize;

    //This is the name of the file which is present on the initial stack
    //It's purpose is to let the user space linker examine the original file.
    int file_name_size = filename.size() + 1;

    const int numRandomBytes = 16;
    int aux_data_size = numRandomBytes;

    string platform = "x86_64";
    aux_data_size += platform.size() + 1;

    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    //The info_block needs to be padded so it's size is a multiple of the
    //alignment mask. Also, it appears that there needs to be at least some
    //padding, so if the size is already a multiple, we need to increase it
    //anyway.
    int base_info_block_size =
        sentry_size + file_name_size + env_data_size + arg_data_size;

    int info_block_size = roundUp(base_info_block_size, align);

    int info_block_padding = info_block_size - base_info_block_size;

    //Each auxilliary vector is two 8 byte words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;

    //Figure out the size of the contents of the actual initial frame
    int frame_size =
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size;

    //There needs to be padding after the auxiliary vector data so that the
    //very bottom of the stack is aligned properly.
    int partial_size = frame_size + aux_data_size;
    int aligned_partial_size = roundUp(partial_size, align);
    int aux_padding = aligned_partial_size - partial_size;

    int space_needed =
        info_block_size +
        aux_data_size +
        aux_padding +
        frame_size;

    stack_min = stack_base - space_needed;
    stack_min = roundDown(stack_min, align);
    stack_size = stack_base - stack_min;

    // map memory
    allocateMem(roundDown(stack_min, pageSize), roundUp(stack_size, pageSize));

    // map out initial stack contents
    IntType sentry_base = stack_base - sentry_size;
    IntType file_name_base = sentry_base - file_name_size;
    IntType env_data_base = file_name_base - env_data_size;
    IntType arg_data_base = env_data_base - arg_data_size;
    IntType aux_data_base = arg_data_base - info_block_padding - aux_data_size;
    IntType auxv_array_base = aux_data_base - aux_array_size - aux_padding;
    IntType envp_array_base = auxv_array_base - envp_array_size;
    IntType argv_array_base = envp_array_base - argv_array_size;
    IntType argc_base = argv_array_base - argc_size;

    DPRINTF(Stack, "The addresses of items on the initial stack:\n");
    DPRINTF(Stack, "0x%x - file name\n", file_name_base);
    DPRINTF(Stack, "0x%x - env data\n", env_data_base);
    DPRINTF(Stack, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Stack, "0x%x - aux data\n", aux_data_base);
    DPRINTF(Stack, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Stack, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Stack, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Stack, "0x%x - argc \n", argc_base);
    DPRINTF(Stack, "0x%x - stack min\n", stack_min);

    // write contents to stack

    // figure out argc
    IntType argc = argv.size();
    IntType guestArgc = X86ISA::htog(argc);

    //Write out the sentry void *
    IntType sentry_NULL = 0;
    initVirtMem.writeBlob(sentry_base,
            (uint8_t*)&sentry_NULL, sentry_size);

    //Write the file name
    initVirtMem.writeString(file_name_base, filename.c_str());

    //Fix up the aux vectors which point to data
    assert(auxv[auxv.size() - 3].a_type == M5_AT_RANDOM);
    auxv[auxv.size() - 3].a_val = aux_data_base;
    assert(auxv[auxv.size() - 2].a_type == M5_AT_EXECFN);
    auxv[auxv.size() - 2].a_val = argv_array_base;
    assert(auxv[auxv.size() - 1].a_type == M5_AT_PLATFORM);
    auxv[auxv.size() - 1].a_val = aux_data_base + numRandomBytes;

    //Copy the aux stuff
    for(int x = 0; x < auxv.size(); x++)
    {
        initVirtMem.writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].a_type), intSize);
        initVirtMem.writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].a_val), intSize);
    }
    //Write out the terminating zeroed auxilliary vector
    const uint64_t zero = 0;
    initVirtMem.writeBlob(auxv_array_base + 2 * intSize * auxv.size(),
            (uint8_t*)&zero, 2 * intSize);

    initVirtMem.writeString(aux_data_base, platform.c_str());

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    initVirtMem.writeBlob(argc_base, (uint8_t*)&guestArgc, intSize);

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    //Set the stack pointer register
    tc->setIntReg(StackPointerReg, stack_min);

    // There doesn't need to be any segment base added in since we're dealing
    // with the flat segmentation model.
    tc->pcState(objFile->entryPoint());

    //Align the "stack_min" to a page boundary.
    stack_min = roundDown(stack_min, pageSize);

//    num_processes++;
}

void
X86_64LiveProcess::argsInit(int intSize, int pageSize)
{
    std::vector<AuxVector<uint64_t> > extraAuxvs;
    extraAuxvs.push_back(AuxVector<uint64_t>(M5_AT_SYSINFO_EHDR,
                vsyscallPage.base));
    X86LiveProcess::argsInit<uint64_t>(pageSize, extraAuxvs);
}

void
I386LiveProcess::argsInit(int intSize, int pageSize)
{
    std::vector<AuxVector<uint32_t> > extraAuxvs;
    //Tell the binary where the vsyscall part of the vsyscall page is.
    extraAuxvs.push_back(AuxVector<uint32_t>(M5_AT_SYSINFO,
                vsyscallPage.base + vsyscallPage.vsyscallOffset));
    extraAuxvs.push_back(AuxVector<uint32_t>(M5_AT_SYSINFO_EHDR,
                vsyscallPage.base));
    X86LiveProcess::argsInit<uint32_t>(pageSize, extraAuxvs);
}

void
X86LiveProcess::setSyscallReturn(ThreadContext *tc, SyscallReturn retval)
{
    tc->setIntReg(INTREG_RAX, retval.encodedValue());
}

X86ISA::IntReg
X86_64LiveProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < NumArgumentRegs);
    return tc->readIntReg(ArgumentReg[i++]);
}

void
X86_64LiveProcess::setSyscallArg(ThreadContext *tc, int i, X86ISA::IntReg val)
{
    assert(i < NumArgumentRegs);
    return tc->setIntReg(ArgumentReg[i], val);
}

X86ISA::IntReg
I386LiveProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < NumArgumentRegs32);
    return tc->readIntReg(ArgumentReg32[i++]);
}

X86ISA::IntReg
I386LiveProcess::getSyscallArg(ThreadContext *tc, int &i, int width)
{
    assert(width == 32 || width == 64);
    assert(i < NumArgumentRegs);
    uint64_t retVal = tc->readIntReg(ArgumentReg32[i++]) & mask(32);
    if (width == 64)
        retVal |= ((uint64_t)tc->readIntReg(ArgumentReg[i++]) << 32);
    return retVal;
}

void
I386LiveProcess::setSyscallArg(ThreadContext *tc, int i, X86ISA::IntReg val)
{
    assert(i < NumArgumentRegs);
    return tc->setIntReg(ArgumentReg[i], val);
}
