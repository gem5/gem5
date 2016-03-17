/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#include "arch/sparc/asi.hh"
#include "arch/sparc/handlers.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/process.hh"
#include "arch/sparc/registers.hh"
#include "arch/sparc/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "sim/process_impl.hh"
#include "sim/system.hh"

using namespace std;
using namespace SparcISA;

static const int FirstArgumentReg = 8;


SparcLiveProcess::SparcLiveProcess(LiveProcessParams * params,
        ObjectFile *objFile, Addr _StackBias)
    : LiveProcess(params, objFile), StackBias(_StackBias)
{

    // XXX all the below need to be updated for SPARC - Ali
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, PageBytes);

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    // Initialize these to 0s
    fillStart = 0;
    spillStart = 0;
}

void
SparcLiveProcess::handleTrap(int trapNum, ThreadContext *tc)
{
    PCState pc = tc->pcState();
    switch (trapNum) {
      case 0x01: // Software breakpoint
        warn("Software breakpoint encountered at pc %#x.\n", pc.pc());
        break;
      case 0x02: // Division by zero
        warn("Software signaled a division by zero at pc %#x.\n", pc.pc());
        break;
      case 0x03: // Flush window trap
        flushWindows(tc);
        break;
      case 0x04: // Clean windows
        warn("Ignoring process request for clean register "
                "windows at pc %#x.\n", pc.pc());
        break;
      case 0x05: // Range check
        warn("Software signaled a range check at pc %#x.\n", pc.pc());
        break;
      case 0x06: // Fix alignment
        warn("Ignoring process request for os assisted unaligned accesses "
                "at pc %#x.\n", pc.pc());
        break;
      case 0x07: // Integer overflow
        warn("Software signaled an integer overflow at pc %#x.\n", pc.pc());
        break;
      case 0x32: // Get integer condition codes
        warn("Ignoring process request to get the integer condition codes "
                "at pc %#x.\n", pc.pc());
        break;
      case 0x33: // Set integer condition codes
        warn("Ignoring process request to set the integer condition codes "
                "at pc %#x.\n", pc.pc());
        break;
      default:
        panic("Unimplemented trap to operating system: trap number %#x.\n", trapNum);
    }
}

void
SparcLiveProcess::initState()
{
    LiveProcess::initState();

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    // From the SPARC ABI

    // Setup default FP state
    tc->setMiscRegNoEffect(MISCREG_FSR, 0);

    tc->setMiscRegNoEffect(MISCREG_TICK, 0);

    /*
     * Register window management registers
     */

    // No windows contain info from other programs
    // tc->setMiscRegNoEffect(MISCREG_OTHERWIN, 0);
    tc->setIntReg(NumIntArchRegs + 6, 0);
    // There are no windows to pop
    // tc->setMiscRegNoEffect(MISCREG_CANRESTORE, 0);
    tc->setIntReg(NumIntArchRegs + 4, 0);
    // All windows are available to save into
    // tc->setMiscRegNoEffect(MISCREG_CANSAVE, NWindows - 2);
    tc->setIntReg(NumIntArchRegs + 3, NWindows - 2);
    // All windows are "clean"
    // tc->setMiscRegNoEffect(MISCREG_CLEANWIN, NWindows);
    tc->setIntReg(NumIntArchRegs + 5, NWindows);
    // Start with register window 0
    tc->setMiscReg(MISCREG_CWP, 0);
    // Always use spill and fill traps 0
    // tc->setMiscRegNoEffect(MISCREG_WSTATE, 0);
    tc->setIntReg(NumIntArchRegs + 7, 0);
    // Set the trap level to 0
    tc->setMiscRegNoEffect(MISCREG_TL, 0);
    // Set the ASI register to something fixed
    tc->setMiscReg(MISCREG_ASI, ASI_PRIMARY);

    /*
     * T1 specific registers
     */
    // Turn on the icache, dcache, dtb translation, and itb translation.
    tc->setMiscRegNoEffect(MISCREG_MMU_LSU_CTRL, 15);
}

void
Sparc32LiveProcess::initState()
{
    SparcLiveProcess::initState();

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    // The process runs in user mode with 32 bit addresses
    PSTATE pstate = 0;
    pstate.ie = 1;
    pstate.am = 1;
    tc->setMiscReg(MISCREG_PSTATE, pstate);

    argsInit(32 / 8, PageBytes);
}

void
Sparc64LiveProcess::initState()
{
    SparcLiveProcess::initState();

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    // The process runs in user mode
    PSTATE pstate = 0;
    pstate.ie = 1;
    tc->setMiscReg(MISCREG_PSTATE, pstate);

    argsInit(sizeof(IntReg), PageBytes);
}

template<class IntType>
void
SparcLiveProcess::argsInit(int pageSize)
{
    int intSize = sizeof(IntType);

    typedef AuxVector<IntType> auxv_t;

    std::vector<auxv_t> auxv;

    string filename;
    if (argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    // Even for a 32 bit process, the ABI says we still need to
    // maintain double word alignment of the stack pointer.
    uint64_t align = 16;

    // Patch the ld_bias for dynamic executables.
    updateBias();

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    enum hardwareCaps
    {
        M5_HWCAP_SPARC_FLUSH = 1,
        M5_HWCAP_SPARC_STBAR = 2,
        M5_HWCAP_SPARC_SWAP = 4,
        M5_HWCAP_SPARC_MULDIV = 8,
        M5_HWCAP_SPARC_V9 = 16,
        // This one should technically only be set
        // if there is a cheetah or cheetah_plus tlb,
        // but we'll use it all the time
        M5_HWCAP_SPARC_ULTRA3 = 32
    };

    const int64_t hwcap =
        M5_HWCAP_SPARC_FLUSH |
        M5_HWCAP_SPARC_STBAR |
        M5_HWCAP_SPARC_SWAP |
        M5_HWCAP_SPARC_MULDIV |
        M5_HWCAP_SPARC_V9 |
        M5_HWCAP_SPARC_ULTRA3;

    // Setup the auxilliary vectors. These will already have endian conversion.
    // Auxilliary vectors are loaded only for elf formatted executables.
    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if (elfObject) {
        // Bits which describe the system hardware capabilities
        auxv.push_back(auxv_t(M5_AT_HWCAP, hwcap));
        // The system page size
        auxv.push_back(auxv_t(M5_AT_PAGESZ, SparcISA::PageBytes));
        // Defined to be 100 in the kernel source.
        // Frequency at which times() increments
        auxv.push_back(auxv_t(M5_AT_CLKTCK, 100));
        // For statically linked executables, this is the virtual address of the
        // program header tables if they appear in the executable image
        auxv.push_back(auxv_t(M5_AT_PHDR, elfObject->programHeaderTable()));
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(M5_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(M5_AT_PHNUM, elfObject->programHeaderCount()));
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.push_back(auxv_t(M5_AT_BASE, getBias()));
        // This is hardwired to 0 in the elf loading code in the kernel
        auxv.push_back(auxv_t(M5_AT_FLAGS, 0));
        // The entry point to the program
        auxv.push_back(auxv_t(M5_AT_ENTRY, objFile->entryPoint()));
        // Different user and group IDs
        auxv.push_back(auxv_t(M5_AT_UID, uid()));
        auxv.push_back(auxv_t(M5_AT_EUID, euid()));
        auxv.push_back(auxv_t(M5_AT_GID, gid()));
        auxv.push_back(auxv_t(M5_AT_EGID, egid()));
        // Whether to enable "secure mode" in the executable
        auxv.push_back(auxv_t(M5_AT_SECURE, 0));
    }

    // Figure out how big the initial stack needs to be

    // The unaccounted for 8 byte 0 at the top of the stack
    int sentry_size = 8;

    // This is the name of the file which is present on the initial stack
    // It's purpose is to let the user space linker examine the original file.
    int file_name_size = filename.size() + 1;

    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    // The info_block.
    int base_info_block_size =
        sentry_size + file_name_size + env_data_size + arg_data_size;

    int info_block_size = roundUp(base_info_block_size, align);

    int info_block_padding = info_block_size - base_info_block_size;

    // Each auxilliary vector is two words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;
    int window_save_size = intSize * 16;

    // Figure out the size of the contents of the actual initial frame
    int frame_size =
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size +
        window_save_size;

    // There needs to be padding after the auxiliary vector data so that the
    // very bottom of the stack is aligned properly.
    int aligned_partial_size = roundUp(frame_size, align);
    int aux_padding = aligned_partial_size - frame_size;

    int space_needed =
        info_block_size +
        aux_padding +
        frame_size;

    stack_min = stack_base - space_needed;
    stack_min = roundDown(stack_min, align);
    stack_size = stack_base - stack_min;

    // Allocate space for the stack
    allocateMem(roundDown(stack_min, pageSize), roundUp(stack_size, pageSize));

    // map out initial stack contents
    IntType sentry_base = stack_base - sentry_size;
    IntType file_name_base = sentry_base - file_name_size;
    IntType env_data_base = file_name_base - env_data_size;
    IntType arg_data_base = env_data_base - arg_data_size;
    IntType auxv_array_base = arg_data_base -
        info_block_padding - aux_array_size - aux_padding;
    IntType envp_array_base = auxv_array_base - envp_array_size;
    IntType argv_array_base = envp_array_base - argv_array_size;
    IntType argc_base = argv_array_base - argc_size;
#if TRACING_ON
    IntType window_save_base = argc_base - window_save_size;
#endif

    DPRINTF(Stack, "The addresses of items on the initial stack:\n");
    DPRINTF(Stack, "%#x - sentry NULL\n", sentry_base);
    DPRINTF(Stack, "filename = %s\n", filename);
    DPRINTF(Stack, "%#x - file name\n", file_name_base);
    DPRINTF(Stack, "%#x - env data\n", env_data_base);
    DPRINTF(Stack, "%#x - arg data\n", arg_data_base);
    DPRINTF(Stack, "%#x - auxv array\n", auxv_array_base);
    DPRINTF(Stack, "%#x - envp array\n", envp_array_base);
    DPRINTF(Stack, "%#x - argv array\n", argv_array_base);
    DPRINTF(Stack, "%#x - argc \n", argc_base);
    DPRINTF(Stack, "%#x - window save\n", window_save_base);
    DPRINTF(Stack, "%#x - stack min\n", stack_min);

    assert(window_save_base == stack_min);

    // write contents to stack

    // figure out argc
    IntType argc = argv.size();
    IntType guestArgc = SparcISA::htog(argc);

    // Write out the sentry void *
    uint64_t sentry_NULL = 0;
    initVirtMem.writeBlob(sentry_base,
            (uint8_t*)&sentry_NULL, sentry_size);

    // Write the file name
    initVirtMem.writeString(file_name_base, filename.c_str());

    // Copy the aux stuff
    for (int x = 0; x < auxv.size(); x++) {
        initVirtMem.writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].a_type), intSize);
        initVirtMem.writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].a_val), intSize);
    }

    // Write out the terminating zeroed auxilliary vector
    const IntType zero = 0;
    initVirtMem.writeBlob(auxv_array_base + intSize * 2 * auxv.size(),
            (uint8_t*)&zero, intSize);
    initVirtMem.writeBlob(auxv_array_base + intSize * (2 * auxv.size() + 1),
            (uint8_t*)&zero, intSize);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    initVirtMem.writeBlob(argc_base, (uint8_t*)&guestArgc, intSize);

    // Set up space for the trap handlers into the processes address space.
    // Since the stack grows down and there is reserved address space abov
    // it, we can put stuff above it and stay out of the way.
    fillStart = stack_base;
    spillStart = fillStart + sizeof(MachInst) * numFillInsts;

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    // Set up the thread context to start running the process
    // assert(NumArgumentRegs >= 2);
    // tc->setIntReg(ArgumentReg[0], argc);
    // tc->setIntReg(ArgumentReg[1], argv_array_base);
    tc->setIntReg(StackPointerReg, stack_min - StackBias);

    // %g1 is a pointer to a function that should be run at exit. Since we
    // don't have anything like that, it should be set to 0.
    tc->setIntReg(1, 0);

    tc->pcState(getStartPC());

    // Align the "stack_min" to a page boundary.
    stack_min = roundDown(stack_min, pageSize);

//    num_processes++;
}

void
Sparc64LiveProcess::argsInit(int intSize, int pageSize)
{
    SparcLiveProcess::argsInit<uint64_t>(pageSize);

    // Stuff the trap handlers into the process address space
    initVirtMem.writeBlob(fillStart,
            (uint8_t*)fillHandler64, sizeof(MachInst) * numFillInsts);
    initVirtMem.writeBlob(spillStart,
            (uint8_t*)spillHandler64, sizeof(MachInst) *  numSpillInsts);
}

void
Sparc32LiveProcess::argsInit(int intSize, int pageSize)
{
    SparcLiveProcess::argsInit<uint32_t>(pageSize);

    // Stuff the trap handlers into the process address space
    initVirtMem.writeBlob(fillStart,
            (uint8_t*)fillHandler32, sizeof(MachInst) * numFillInsts);
    initVirtMem.writeBlob(spillStart,
            (uint8_t*)spillHandler32, sizeof(MachInst) *  numSpillInsts);
}

void Sparc32LiveProcess::flushWindows(ThreadContext *tc)
{
    IntReg Cansave = tc->readIntReg(NumIntArchRegs + 3);
    IntReg Canrestore = tc->readIntReg(NumIntArchRegs + 4);
    IntReg Otherwin = tc->readIntReg(NumIntArchRegs + 6);
    MiscReg CWP = tc->readMiscReg(MISCREG_CWP);
    MiscReg origCWP = CWP;
    CWP = (CWP + Cansave + 2) % NWindows;
    while (NWindows - 2 - Cansave != 0) {
        if (Otherwin) {
            panic("Otherwin non-zero.\n");
        } else {
            tc->setMiscReg(MISCREG_CWP, CWP);
            // Do the stores
            IntReg sp = tc->readIntReg(StackPointerReg);
            for (int index = 16; index < 32; index++) {
                uint32_t regVal = tc->readIntReg(index);
                regVal = htog(regVal);
                if (!tc->getMemProxy().tryWriteBlob(
                        sp + (index - 16) * 4, (uint8_t *)&regVal, 4)) {
                    warn("Failed to save register to the stack when "
                            "flushing windows.\n");
                }
            }
            Canrestore--;
            Cansave++;
            CWP = (CWP + 1) % NWindows;
        }
    }
    tc->setIntReg(NumIntArchRegs + 3, Cansave);
    tc->setIntReg(NumIntArchRegs + 4, Canrestore);
    tc->setMiscReg(MISCREG_CWP, origCWP);
}

void
Sparc64LiveProcess::flushWindows(ThreadContext *tc)
{
    IntReg Cansave = tc->readIntReg(NumIntArchRegs + 3);
    IntReg Canrestore = tc->readIntReg(NumIntArchRegs + 4);
    IntReg Otherwin = tc->readIntReg(NumIntArchRegs + 6);
    MiscReg CWP = tc->readMiscReg(MISCREG_CWP);
    MiscReg origCWP = CWP;
    CWP = (CWP + Cansave + 2) % NWindows;
    while (NWindows - 2 - Cansave != 0) {
        if (Otherwin) {
            panic("Otherwin non-zero.\n");
        } else {
            tc->setMiscReg(MISCREG_CWP, CWP);
            // Do the stores
            IntReg sp = tc->readIntReg(StackPointerReg);
            for (int index = 16; index < 32; index++) {
                IntReg regVal = tc->readIntReg(index);
                regVal = htog(regVal);
                if (!tc->getMemProxy().tryWriteBlob(
                        sp + 2047 + (index - 16) * 8, (uint8_t *)&regVal, 8)) {
                    warn("Failed to save register to the stack when "
                            "flushing windows.\n");
                }
            }
            Canrestore--;
            Cansave++;
            CWP = (CWP + 1) % NWindows;
        }
    }
    tc->setIntReg(NumIntArchRegs + 3, Cansave);
    tc->setIntReg(NumIntArchRegs + 4, Canrestore);
    tc->setMiscReg(MISCREG_CWP, origCWP);
}

IntReg
Sparc32LiveProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < 6);
    return bits(tc->readIntReg(FirstArgumentReg + i++), 31, 0);
}

void
Sparc32LiveProcess::setSyscallArg(ThreadContext *tc, int i, IntReg val)
{
    assert(i < 6);
    tc->setIntReg(FirstArgumentReg + i, bits(val, 31, 0));
}

IntReg
Sparc64LiveProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < 6);
    return tc->readIntReg(FirstArgumentReg + i++);
}

void
Sparc64LiveProcess::setSyscallArg(ThreadContext *tc, int i, IntReg val)
{
    assert(i < 6);
    tc->setIntReg(FirstArgumentReg + i, val);
}

void
SparcLiveProcess::setSyscallReturn(ThreadContext *tc, SyscallReturn sysret)
{
    // check for error condition.  SPARC syscall convention is to
    // indicate success/failure in reg the carry bit of the ccr
    // and put the return value itself in the standard return value reg ().
    PSTATE pstate = tc->readMiscRegNoEffect(MISCREG_PSTATE);
    if (sysret.successful()) {
        // no error, clear XCC.C
        tc->setIntReg(NumIntArchRegs + 2,
                      tc->readIntReg(NumIntArchRegs + 2) & 0xEE);
        IntReg val = sysret.returnValue();
        if (pstate.am)
            val = bits(val, 31, 0);
        tc->setIntReg(ReturnValueReg, val);
    } else {
        // got an error, set XCC.C
        tc->setIntReg(NumIntArchRegs + 2,
                      tc->readIntReg(NumIntArchRegs + 2) | 0x11);
        IntReg val = sysret.errnoValue();
        if (pstate.am)
            val = bits(val, 31, 0);
        tc->setIntReg(ReturnValueReg, val);
    }
}
