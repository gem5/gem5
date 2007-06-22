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
#include "arch/sparc/types.hh"
#include "base/loader/object_file.hh"
#include "base/loader/elf_object.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "sim/process_impl.hh"
#include "mem/translating_port.hh"
#include "sim/system.hh"

using namespace std;
using namespace SparcISA;


SparcLiveProcess::SparcLiveProcess(const std::string &nm, ObjectFile *objFile,
        System *_system, int stdin_fd, int stdout_fd, int stderr_fd,
        std::vector<std::string> &argv, std::vector<std::string> &envp,
        const std::string &cwd,
        uint64_t _uid, uint64_t _euid, uint64_t _gid, uint64_t _egid,
        uint64_t _pid, uint64_t _ppid)
    : LiveProcess(nm, objFile, _system, stdin_fd, stdout_fd, stderr_fd,
        argv, envp, cwd, _uid, _euid, _gid, _egid, _pid, _ppid)
{

    // XXX all the below need to be updated for SPARC - Ali
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, VMPageSize);

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    //Initialize these to 0s
    fillStart = 0;
    spillStart = 0;
}

void SparcLiveProcess::handleTrap(int trapNum, ThreadContext *tc)
{
    switch(trapNum)
    {
      case 0x03: //Flush window trap
        warn("Ignoring request to flush register windows.\n");
        break;
      default:
        panic("Unimplemented trap to operating system: trap number %#x.\n", trapNum);
    }
}

void
Sparc32LiveProcess::startup()
{
    argsInit(32 / 8, VMPageSize);

    //From the SPARC ABI

    //The process runs in user mode with 32 bit addresses
    threadContexts[0]->setMiscReg(MISCREG_PSTATE, 0x0a);

    //Setup default FP state
    threadContexts[0]->setMiscRegNoEffect(MISCREG_FSR, 0);

    threadContexts[0]->setMiscRegNoEffect(MISCREG_TICK, 0);
    //
    /*
     * Register window management registers
     */

    //No windows contain info from other programs
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_OTHERWIN, 0);
    threadContexts[0]->setIntReg(NumIntArchRegs + 6, 0);
    //There are no windows to pop
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_CANRESTORE, 0);
    threadContexts[0]->setIntReg(NumIntArchRegs + 4, 0);
    //All windows are available to save into
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_CANSAVE, NWindows - 2);
    threadContexts[0]->setIntReg(NumIntArchRegs + 3, NWindows - 2);
    //All windows are "clean"
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_CLEANWIN, NWindows);
    threadContexts[0]->setIntReg(NumIntArchRegs + 5, NWindows);
    //Start with register window 0
    threadContexts[0]->setMiscRegNoEffect(MISCREG_CWP, 0);
    //Always use spill and fill traps 0
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_WSTATE, 0);
    threadContexts[0]->setIntReg(NumIntArchRegs + 7, 0);
    //Set the trap level to 0
    threadContexts[0]->setMiscRegNoEffect(MISCREG_TL, 0);
    //Set the ASI register to something fixed
    threadContexts[0]->setMiscRegNoEffect(MISCREG_ASI, ASI_PRIMARY);
}

void
Sparc64LiveProcess::startup()
{
    argsInit(sizeof(IntReg), VMPageSize);

    //From the SPARC ABI

    //The process runs in user mode
    threadContexts[0]->setMiscReg(MISCREG_PSTATE, 0x02);

    //Setup default FP state
    threadContexts[0]->setMiscRegNoEffect(MISCREG_FSR, 0);

    threadContexts[0]->setMiscRegNoEffect(MISCREG_TICK, 0);
    //
    /*
     * Register window management registers
     */

    //No windows contain info from other programs
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_OTHERWIN, 0);
    threadContexts[0]->setIntReg(NumIntArchRegs + 6, 0);
    //There are no windows to pop
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_CANRESTORE, 0);
    threadContexts[0]->setIntReg(NumIntArchRegs + 4, 0);
    //All windows are available to save into
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_CANSAVE, NWindows - 2);
    threadContexts[0]->setIntReg(NumIntArchRegs + 3, NWindows - 2);
    //All windows are "clean"
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_CLEANWIN, NWindows);
    threadContexts[0]->setIntReg(NumIntArchRegs + 5, NWindows);
    //Start with register window 0
    threadContexts[0]->setMiscRegNoEffect(MISCREG_CWP, 0);
    //Always use spill and fill traps 0
    //threadContexts[0]->setMiscRegNoEffect(MISCREG_WSTATE, 0);
    threadContexts[0]->setIntReg(NumIntArchRegs + 7, 0);
    //Set the trap level to 0
    threadContexts[0]->setMiscRegNoEffect(MISCREG_TL, 0);
    //Set the ASI register to something fixed
    threadContexts[0]->setMiscRegNoEffect(MISCREG_ASI, ASI_PRIMARY);
}

M5_32_auxv_t::M5_32_auxv_t(int32_t type, int32_t val)
{
    a_type = TheISA::htog(type);
    a_val = TheISA::htog(val);
}

M5_64_auxv_t::M5_64_auxv_t(int64_t type, int64_t val)
{
    a_type = TheISA::htog(type);
    a_val = TheISA::htog(val);
}

void
Sparc64LiveProcess::argsInit(int intSize, int pageSize)
{
    typedef M5_64_auxv_t auxv_t;
    Process::startup();

    string filename;
    if(argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    Addr alignmentMask = ~(intSize - 1);

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    //These are the auxilliary vector types
    enum auxTypes
    {
        SPARC_AT_HWCAP = 16,
        SPARC_AT_PAGESZ = 6,
        SPARC_AT_CLKTCK = 17,
        SPARC_AT_PHDR = 3,
        SPARC_AT_PHENT = 4,
        SPARC_AT_PHNUM = 5,
        SPARC_AT_BASE = 7,
        SPARC_AT_FLAGS = 8,
        SPARC_AT_ENTRY = 9,
        SPARC_AT_UID = 11,
        SPARC_AT_EUID = 12,
        SPARC_AT_GID = 13,
        SPARC_AT_EGID = 14,
        SPARC_AT_SECURE = 23
    };

    enum hardwareCaps
    {
        M5_HWCAP_SPARC_FLUSH = 1,
        M5_HWCAP_SPARC_STBAR = 2,
        M5_HWCAP_SPARC_SWAP = 4,
        M5_HWCAP_SPARC_MULDIV = 8,
        M5_HWCAP_SPARC_V9 = 16,
        //This one should technically only be set
        //if there is a cheetah or cheetah_plus tlb,
        //but we'll use it all the time
        M5_HWCAP_SPARC_ULTRA3 = 32
    };

    const int64_t hwcap =
        M5_HWCAP_SPARC_FLUSH |
        M5_HWCAP_SPARC_STBAR |
        M5_HWCAP_SPARC_SWAP |
        M5_HWCAP_SPARC_MULDIV |
        M5_HWCAP_SPARC_V9 |
        M5_HWCAP_SPARC_ULTRA3;


    //Setup the auxilliary vectors. These will already have endian conversion.
    //Auxilliary vectors are loaded only for elf formatted executables.
    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if(elfObject)
    {
        //Bits which describe the system hardware capabilities
        auxv.push_back(auxv_t(SPARC_AT_HWCAP, hwcap));
        //The system page size
        auxv.push_back(auxv_t(SPARC_AT_PAGESZ, SparcISA::VMPageSize));
        //Defined to be 100 in the kernel source.
        //Frequency at which times() increments
        auxv.push_back(auxv_t(SPARC_AT_CLKTCK, 100));
        // For statically linked executables, this is the virtual address of the
        // program header tables if they appear in the executable image
        auxv.push_back(auxv_t(SPARC_AT_PHDR, elfObject->programHeaderTable()));
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(SPARC_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(SPARC_AT_PHNUM, elfObject->programHeaderCount()));
        //This is the address of the elf "interpreter", It should be set
        //to 0 for regular executables. It should be something else
        //(not sure what) for dynamic libraries.
        auxv.push_back(auxv_t(SPARC_AT_BASE, 0));
        //This is hardwired to 0 in the elf loading code in the kernel
        auxv.push_back(auxv_t(SPARC_AT_FLAGS, 0));
        //The entry point to the program
        auxv.push_back(auxv_t(SPARC_AT_ENTRY, objFile->entryPoint()));
        //Different user and group IDs
        auxv.push_back(auxv_t(SPARC_AT_UID, uid()));
        auxv.push_back(auxv_t(SPARC_AT_EUID, euid()));
        auxv.push_back(auxv_t(SPARC_AT_GID, gid()));
        auxv.push_back(auxv_t(SPARC_AT_EGID, egid()));
        //Whether to enable "secure mode" in the executable
        auxv.push_back(auxv_t(SPARC_AT_SECURE, 0));
    }

    //Figure out how big the initial stack needs to be

    // The unaccounted for 0 at the top of the stack
    int mysterious_size = intSize;

    //This is the name of the file which is present on the initial stack
    //It's purpose is to let the user space linker examine the original file.
    int file_name_size = filename.size() + 1;

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
    int info_block_size =
        (file_name_size +
        env_data_size +
        arg_data_size +
        intSize) & alignmentMask;

    int info_block_padding =
        info_block_size -
        file_name_size -
        env_data_size -
        arg_data_size;

    //Each auxilliary vector is two 8 byte words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;
    int window_save_size = intSize * 16;

    int space_needed =
        mysterious_size +
        info_block_size +
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size +
        window_save_size;

    stack_min = stack_base - space_needed;
    stack_min &= alignmentMask;
    stack_size = stack_base - stack_min;

    // map memory
    pTable->allocate(roundDown(stack_min, pageSize),
                     roundUp(stack_size, pageSize));

    // map out initial stack contents
    Addr mysterious_base = stack_base - mysterious_size;
    Addr file_name_base = mysterious_base - file_name_size;
    Addr env_data_base = file_name_base - env_data_size;
    Addr arg_data_base = env_data_base - arg_data_size;
    Addr auxv_array_base = arg_data_base - aux_array_size - info_block_padding;
    Addr envp_array_base = auxv_array_base - envp_array_size;
    Addr argv_array_base = envp_array_base - argv_array_size;
    Addr argc_base = argv_array_base - argc_size;
#ifndef NDEBUG
    // only used in DPRINTF
    Addr window_save_base = argc_base - window_save_size;
#endif

    DPRINTF(Sparc, "The addresses of items on the initial stack:\n");
    DPRINTF(Sparc, "0x%x - file name\n", file_name_base);
    DPRINTF(Sparc, "0x%x - env data\n", env_data_base);
    DPRINTF(Sparc, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Sparc, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Sparc, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Sparc, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Sparc, "0x%x - argc \n", argc_base);
    DPRINTF(Sparc, "0x%x - window save\n", window_save_base);
    DPRINTF(Sparc, "0x%x - stack min\n", stack_min);

    // write contents to stack

    // figure out argc
    uint64_t argc = argv.size();
    uint64_t guestArgc = TheISA::htog(argc);

    //Write out the mysterious 0
    uint64_t mysterious_zero = 0;
    initVirtMem->writeBlob(mysterious_base,
            (uint8_t*)&mysterious_zero, mysterious_size);

    //Write the file name
    initVirtMem->writeString(file_name_base, filename.c_str());

    //Copy the aux stuff
    for(int x = 0; x < auxv.size(); x++)
    {
        initVirtMem->writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].a_type), intSize);
        initVirtMem->writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].a_val), intSize);
    }
    //Write out the terminating zeroed auxilliary vector
    const uint64_t zero = 0;
    initVirtMem->writeBlob(auxv_array_base + 2 * intSize * auxv.size(),
            (uint8_t*)&zero, 2 * intSize);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    initVirtMem->writeBlob(argc_base, (uint8_t*)&guestArgc, intSize);

    //Stuff the trap handlers into the processes address space.
    //Since the stack grows down and is the highest area in the processes
    //address space, we can put stuff above it and stay out of the way.
    int fillSize = sizeof(MachInst) * numFillInsts;
    int spillSize = sizeof(MachInst) * numSpillInsts;
    fillStart = stack_base;
    spillStart = fillStart + fillSize;
    initVirtMem->writeBlob(fillStart, (uint8_t*)fillHandler64, fillSize);
    initVirtMem->writeBlob(spillStart, (uint8_t*)spillHandler64, spillSize);

    //Set up the thread context to start running the process
    threadContexts[0]->setIntReg(ArgumentReg0, argc);
    threadContexts[0]->setIntReg(ArgumentReg1, argv_array_base);
    threadContexts[0]->setIntReg(StackPointerReg, stack_min - StackBias);

    Addr prog_entry = objFile->entryPoint();
    threadContexts[0]->setPC(prog_entry);
    threadContexts[0]->setNextPC(prog_entry + sizeof(MachInst));
    threadContexts[0]->setNextNPC(prog_entry + (2 * sizeof(MachInst)));

    //Align the "stack_min" to a page boundary.
    stack_min = roundDown(stack_min, pageSize);

//    num_processes++;
}

void
Sparc32LiveProcess::argsInit(int intSize, int pageSize)
{
    typedef M5_32_auxv_t auxv_t;
    Process::startup();

    string filename;
    if(argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    //Even though this is a 32 bit process, the ABI says we still need to
    //maintain double word alignment of the stack pointer.
    Addr alignmentMask = ~(8 - 1);

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    //These are the auxilliary vector types
    enum auxTypes
    {
        SPARC_AT_HWCAP = 16,
        SPARC_AT_PAGESZ = 6,
        SPARC_AT_CLKTCK = 17,
        SPARC_AT_PHDR = 3,
        SPARC_AT_PHENT = 4,
        SPARC_AT_PHNUM = 5,
        SPARC_AT_BASE = 7,
        SPARC_AT_FLAGS = 8,
        SPARC_AT_ENTRY = 9,
        SPARC_AT_UID = 11,
        SPARC_AT_EUID = 12,
        SPARC_AT_GID = 13,
        SPARC_AT_EGID = 14,
        SPARC_AT_SECURE = 23
    };

    enum hardwareCaps
    {
        M5_HWCAP_SPARC_FLUSH = 1,
        M5_HWCAP_SPARC_STBAR = 2,
        M5_HWCAP_SPARC_SWAP = 4,
        M5_HWCAP_SPARC_MULDIV = 8,
        M5_HWCAP_SPARC_V9 = 16,
        //This one should technically only be set
        //if there is a cheetah or cheetah_plus tlb,
        //but we'll use it all the time
        M5_HWCAP_SPARC_ULTRA3 = 32
    };

    const int64_t hwcap =
        M5_HWCAP_SPARC_FLUSH |
        M5_HWCAP_SPARC_STBAR |
        M5_HWCAP_SPARC_SWAP |
        M5_HWCAP_SPARC_MULDIV |
        M5_HWCAP_SPARC_V9 |
        M5_HWCAP_SPARC_ULTRA3;


    //Setup the auxilliary vectors. These will already have endian conversion.
    //Auxilliary vectors are loaded only for elf formatted executables.
    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if(elfObject)
    {
        //Bits which describe the system hardware capabilities
        auxv.push_back(auxv_t(SPARC_AT_HWCAP, hwcap));
        //The system page size
        auxv.push_back(auxv_t(SPARC_AT_PAGESZ, SparcISA::VMPageSize));
        //Defined to be 100 in the kernel source.
        //Frequency at which times() increments
        auxv.push_back(auxv_t(SPARC_AT_CLKTCK, 100));
        // For statically linked executables, this is the virtual address of the
        // program header tables if they appear in the executable image
        auxv.push_back(auxv_t(SPARC_AT_PHDR, elfObject->programHeaderTable()));
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(SPARC_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(SPARC_AT_PHNUM, elfObject->programHeaderCount()));
        //This is the address of the elf "interpreter", It should be set
        //to 0 for regular executables. It should be something else
        //(not sure what) for dynamic libraries.
        auxv.push_back(auxv_t(SPARC_AT_BASE, 0));
        //This is hardwired to 0 in the elf loading code in the kernel
        auxv.push_back(auxv_t(SPARC_AT_FLAGS, 0));
        //The entry point to the program
        auxv.push_back(auxv_t(SPARC_AT_ENTRY, objFile->entryPoint()));
        //Different user and group IDs
        auxv.push_back(auxv_t(SPARC_AT_UID, uid()));
        auxv.push_back(auxv_t(SPARC_AT_EUID, euid()));
        auxv.push_back(auxv_t(SPARC_AT_GID, gid()));
        auxv.push_back(auxv_t(SPARC_AT_EGID, egid()));
        //Whether to enable "secure mode" in the executable
        auxv.push_back(auxv_t(SPARC_AT_SECURE, 0));
    }

    //Figure out how big the initial stack needs to be

    // The unaccounted for 8 byte 0 at the top of the stack
    int mysterious_size = 8;

    //This is the name of the file which is present on the initial stack
    //It's purpose is to let the user space linker examine the original file.
    int file_name_size = filename.size() + 1;

    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    //The info_block - This seems to need an pad for some reason.
    int info_block_size =
        (mysterious_size +
        file_name_size +
        env_data_size +
        arg_data_size + intSize);

    //Each auxilliary vector is two 4 byte words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;
    int window_save_size = intSize * 16;

    int space_needed =
        info_block_size +
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size +
        window_save_size;

    stack_min = stack_base - space_needed;
    stack_min &= alignmentMask;
    stack_size = stack_base - stack_min;

    // map memory
    pTable->allocate(roundDown(stack_min, pageSize),
                     roundUp(stack_size, pageSize));

    // map out initial stack contents
    uint32_t window_save_base = stack_min;
    uint32_t argc_base = window_save_base + window_save_size;
    uint32_t argv_array_base = argc_base + argc_size;
    uint32_t envp_array_base = argv_array_base + argv_array_size;
    uint32_t auxv_array_base = envp_array_base + envp_array_size;
    //The info block is pushed up against the top of the stack, while
    //the rest of the initial stack frame is aligned to an 8 byte boudary.
    uint32_t arg_data_base = stack_base - info_block_size + intSize;
    uint32_t env_data_base = arg_data_base + arg_data_size;
    uint32_t file_name_base = env_data_base + env_data_size;
    uint32_t mysterious_base = file_name_base + file_name_size;

    DPRINTF(Sparc, "The addresses of items on the initial stack:\n");
    DPRINTF(Sparc, "0x%x - file name\n", file_name_base);
    DPRINTF(Sparc, "0x%x - env data\n", env_data_base);
    DPRINTF(Sparc, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Sparc, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Sparc, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Sparc, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Sparc, "0x%x - argc \n", argc_base);
    DPRINTF(Sparc, "0x%x - window save\n", window_save_base);
    DPRINTF(Sparc, "0x%x - stack min\n", stack_min);

    // write contents to stack

    // figure out argc
    uint32_t argc = argv.size();
    uint32_t guestArgc = TheISA::htog(argc);

    //Write out the mysterious 0
    uint64_t mysterious_zero = 0;
    initVirtMem->writeBlob(mysterious_base,
            (uint8_t*)&mysterious_zero, mysterious_size);

    //Write the file name
    initVirtMem->writeString(file_name_base, filename.c_str());

    //Copy the aux stuff
    for(int x = 0; x < auxv.size(); x++)
    {
        initVirtMem->writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].a_type), intSize);
        initVirtMem->writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].a_val), intSize);
    }
    //Write out the terminating zeroed auxilliary vector
    const uint64_t zero = 0;
    initVirtMem->writeBlob(auxv_array_base + 2 * intSize * auxv.size(),
            (uint8_t*)&zero, 2 * intSize);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    initVirtMem->writeBlob(argc_base, (uint8_t*)&guestArgc, intSize);

    //Stuff the trap handlers into the processes address space.
    //Since the stack grows down and is the highest area in the processes
    //address space, we can put stuff above it and stay out of the way.
    int fillSize = sizeof(MachInst) * numFillInsts;
    int spillSize = sizeof(MachInst) * numSpillInsts;
    fillStart = stack_base;
    spillStart = fillStart + fillSize;
    initVirtMem->writeBlob(fillStart, (uint8_t*)fillHandler32, fillSize);
    initVirtMem->writeBlob(spillStart, (uint8_t*)spillHandler32, spillSize);

    //Set up the thread context to start running the process
    //threadContexts[0]->setIntReg(ArgumentReg0, argc);
    //threadContexts[0]->setIntReg(ArgumentReg1, argv_array_base);
    threadContexts[0]->setIntReg(StackPointerReg, stack_min);

    uint32_t prog_entry = objFile->entryPoint();
    threadContexts[0]->setPC(prog_entry);
    threadContexts[0]->setNextPC(prog_entry + sizeof(MachInst));
    threadContexts[0]->setNextNPC(prog_entry + (2 * sizeof(MachInst)));

    //Align the "stack_min" to a page boundary.
    stack_min = roundDown(stack_min, pageSize);

//    num_processes++;
}
