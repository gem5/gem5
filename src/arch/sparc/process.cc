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
 */

#include "arch/sparc/process.hh"

#include "arch/sparc/asi.hh"
#include "arch/sparc/handlers.hh"
#include "arch/sparc/page_size.hh"
#include "arch/sparc/regs/int.hh"
#include "arch/sparc/regs/misc.hh"
#include "arch/sparc/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/byteswap.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace SparcISA;

SparcProcess::SparcProcess(const ProcessParams &params,
                           loader::ObjectFile *objFile, Addr _StackBias)
    : Process(params,
              new EmulationPageTable(params.name, params.pid, PageBytes),
              objFile),
      StackBias(_StackBias)
{
    fatal_if(params.useArchPT, "Arch page tables not implemented.");
    // Initialize these to 0s
    fillStart = 0;
    spillStart = 0;
}

void
SparcProcess::initState()
{
    Process::initState();

    ThreadContext *tc = system->threads[contextIds[0]];
    // From the SPARC ABI

    // Setup default FP state
    tc->setMiscRegNoEffect(MISCREG_FSR, 0);

    tc->setMiscRegNoEffect(MISCREG_TICK, 0);

    /*
     * Register window management registers
     */

    // No windows contain info from other programs
    tc->setReg(int_reg::Otherwin, (RegVal)0);
    // There are no windows to pop
    tc->setReg(int_reg::Canrestore, (RegVal)0);
    // All windows are available to save into
    tc->setReg(int_reg::Cansave, NWindows - 2);
    // All windows are "clean"
    tc->setReg(int_reg::Cleanwin, NWindows);
    // Start with register window 0
    tc->setMiscReg(MISCREG_CWP, 0);
    // Always use spill and fill traps 0
    tc->setReg(int_reg::Wstate, (RegVal)0);
    // Set the trap level to 0
    tc->setMiscRegNoEffect(MISCREG_TL, 0);
    // Set the ASI register to something fixed
    tc->setMiscReg(MISCREG_ASI, ASI_PRIMARY);

    // Set the MMU Primary Context Register to hold the process' pid
    tc->setMiscReg(MISCREG_MMU_P_CONTEXT, _pid);

    // Enable floating point.
    tc->setMiscReg(MISCREG_FPRS, 0x4);

    /*
     * T1 specific registers
     */
    // Turn on the icache, dcache, dtb translation, and itb translation.
    tc->setMiscRegNoEffect(MISCREG_MMU_LSU_CTRL, 15);
}

void
Sparc32Process::initState()
{
    SparcProcess::initState();

    ThreadContext *tc = system->threads[contextIds[0]];
    // The process runs in user mode with 32 bit addresses
    PSTATE pstate = 0;
    pstate.pef = 1;
    pstate.ie = 1;
    pstate.am = 1;
    tc->setMiscReg(MISCREG_PSTATE, pstate);

    argsInit(32 / 8, PageBytes);
}

void
Sparc64Process::initState()
{
    SparcProcess::initState();

    ThreadContext *tc = system->threads[contextIds[0]];
    // The process runs in user mode
    PSTATE pstate = 0;
    pstate.pef = 1;
    pstate.ie = 1;
    tc->setMiscReg(MISCREG_PSTATE, pstate);

    argsInit(sizeof(RegVal), PageBytes);
}

template <class IntType>
void
SparcProcess::argsInit(int pageSize)
{
    int intSize = sizeof(IntType);

    std::vector<gem5::auxv::AuxVector<IntType>> auxv;

    std::string filename;
    if (argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    // Even for a 32 bit process, the ABI says we still need to
    // maintain double word alignment of the stack pointer.
    uint64_t align = 16;

    enum HardwareCaps
    {
        HwcapSparcFlush = 1,
        HwcapSparcStbar = 2,
        HwcapSparcSwap = 4,
        HwcapSparcMuldiv = 8,
        HwcapSparcV9 = 16,
        // This one should technically only be set
        // if there is a cheetah or cheetah_plus tlb,
        // but we'll use it all the time
        HwcapSparcUltra3 = 32
    };

    const int64_t hwcap = HwcapSparcFlush | HwcapSparcStbar | HwcapSparcSwap |
                          HwcapSparcMuldiv | HwcapSparcV9 | HwcapSparcUltra3;

    // Setup the auxilliary vectors. These will already have endian conversion.
    // Auxilliary vectors are loaded only for elf formatted executables.
    auto *elfObject = dynamic_cast<loader::ElfObject *>(objFile);
    if (elfObject) {
        // Bits which describe the system hardware capabilities
        auxv.emplace_back(gem5::auxv::Hwcap, hwcap);
        // The system page size
        auxv.emplace_back(gem5::auxv::Pagesz, SparcISA::PageBytes);
        // Defined to be 100 in the kernel source.
        // Frequency at which times() increments
        auxv.emplace_back(gem5::auxv::Clktck, 100);
        // For statically linked executables, this is the virtual address of
        // the program header tables if they appear in the executable image
        auxv.emplace_back(gem5::auxv::Phdr, elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.emplace_back(gem5::auxv::Phent, elfObject->programHeaderSize());
        // This is the number of program headers from the original elf file.
        auxv.emplace_back(gem5::auxv::Phnum, elfObject->programHeaderCount());
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.emplace_back(gem5::auxv::Base, getBias());
        // This is hardwired to 0 in the elf loading code in the kernel
        auxv.emplace_back(gem5::auxv::Flags, 0);
        // The entry point to the program
        auxv.emplace_back(gem5::auxv::Entry, objFile->entryPoint());
        // Different user and group IDs
        auxv.emplace_back(gem5::auxv::Uid, uid());
        auxv.emplace_back(gem5::auxv::Euid, euid());
        auxv.emplace_back(gem5::auxv::Gid, gid());
        auxv.emplace_back(gem5::auxv::Egid, egid());
        // Whether to enable "secure mode" in the executable
        auxv.emplace_back(gem5::auxv::Secure, 0);
        // The address of 16 "random" bytes.
        auxv.emplace_back(gem5::auxv::Random, 0);
    }

    // Figure out how big the initial stack needs to be

    // The unaccounted for 8 byte 0 at the top of the stack
    int sentry_size = 8;

    // This is the name of the file which is present on the initial stack
    // It's purpose is to let the user space linker examine the original file.
    int file_name_size = filename.size() + 1;

    const int numRandomBytes = 16;
    int aux_data_size = numRandomBytes;

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
    int frame_size = aux_array_size + envp_array_size + argv_array_size +
                     argc_size + window_save_size;

    // There needs to be padding after the auxiliary vector data so that the
    // very bottom of the stack is aligned properly.
    int aligned_partial_size = roundUp(frame_size, align);
    int aux_padding = aligned_partial_size - frame_size;

    int space_needed =
        info_block_size + aux_data_size + aux_padding + frame_size;

    memState->setStackMin(memState->getStackBase() - space_needed);
    memState->setStackMin(roundDown(memState->getStackMin(), align));
    memState->setStackSize(memState->getStackBase() - memState->getStackMin());

    // Allocate space for the stack
    memState->mapRegion(roundDown(memState->getStackMin(), pageSize),
                        roundUp(memState->getStackSize(), pageSize), "stack");

    // map out initial stack contents
    IntType sentry_base = memState->getStackBase() - sentry_size;
    IntType file_name_base = sentry_base - file_name_size;
    IntType env_data_base = file_name_base - env_data_size;
    IntType arg_data_base = env_data_base - arg_data_size;
    IntType aux_data_base = arg_data_base - info_block_padding - aux_data_size;
    IntType auxv_array_base = aux_data_base - aux_array_size - aux_padding;
    IntType envp_array_base = auxv_array_base - envp_array_size;
    IntType argv_array_base = envp_array_base - argv_array_size;
    IntType argc_base = argv_array_base - argc_size;
    IntType window_save_base = argc_base - window_save_size;

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
    DPRINTF(Stack, "%#x - stack min\n", memState->getStackMin());

    assert(window_save_base == memState->getStackMin());

    // write contents to stack

    // figure out argc
    IntType argc = argv.size();
    IntType guestArgc = htobe(argc);

    // Write out the sentry void *
    uint64_t sentry_NULL = 0;
    initVirtMem->writeBlob(sentry_base, &sentry_NULL, sentry_size);

    // Write the file name
    initVirtMem->writeString(file_name_base, filename.c_str());

    // Fix up the aux vectors which point to data.
    for (auto &aux : auxv) {
        if (aux.type == gem5::auxv::Random)
            aux.val = aux_data_base;
    }

    // Copy the aux stuff
    Addr auxv_array_end = auxv_array_base;
    for (const auto &aux : auxv) {
        initVirtMem->write(auxv_array_end, aux, ByteOrder::big);
        auxv_array_end += sizeof(aux);
    }

    // Write out the terminating zeroed auxilliary vector
    const gem5::auxv::AuxVector<IntType> zero(0, 0);
    initVirtMem->write(auxv_array_end, zero);
    auxv_array_end += sizeof(zero);

    copyStringArray(envp, envp_array_base, env_data_base, ByteOrder::big,
                    *initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, ByteOrder::big,
                    *initVirtMem);

    initVirtMem->writeBlob(argc_base, &guestArgc, intSize);

    // Set up space for the trap handlers into the processes address space.
    // Since the stack grows down and there is reserved address space abov
    // it, we can put stuff above it and stay out of the way.
    fillStart = memState->getStackBase();
    spillStart = fillStart + sizeof(MachInst) * numFillInsts;

    ThreadContext *tc = system->threads[contextIds[0]];
    // Set up the thread context to start running the process
    // assert(NumArgumentRegs >= 2);
    // tc->setReg(ArgumentReg[0], argc);
    // tc->setReg(ArgumentReg[1], argv_array_base);
    tc->setReg(StackPointerReg, memState->getStackMin() - StackBias);

    // %g1 is a pointer to a function that should be run at exit. Since we
    // don't have anything like that, it should be set to 0.
    tc->setReg(int_reg::G1, (RegVal)0);

    tc->pcState(getStartPC());

    // Align the "stack_min" to a page boundary.
    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
}

void
Sparc64Process::argsInit(int intSize, int pageSize)
{
    SparcProcess::argsInit<uint64_t>(pageSize);

    // Stuff the trap handlers into the process address space
    initVirtMem->writeBlob(fillStart, fillHandler64,
                           sizeof(MachInst) * numFillInsts);
    initVirtMem->writeBlob(spillStart, spillHandler64,
                           sizeof(MachInst) * numSpillInsts);
}

void
Sparc32Process::argsInit(int intSize, int pageSize)
{
    SparcProcess::argsInit<uint32_t>(pageSize);

    // Stuff the trap handlers into the process address space
    initVirtMem->writeBlob(fillStart, fillHandler32,
                           sizeof(MachInst) * numFillInsts);
    initVirtMem->writeBlob(spillStart, spillHandler32,
                           sizeof(MachInst) * numSpillInsts);
}

} // namespace gem5
