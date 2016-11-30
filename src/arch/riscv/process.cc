/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2016 The University of Virginia
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
 *          Korey Sewell
 *          Alec Roelke
 */
#include "arch/riscv/process.hh"

#include <vector>

#include "arch/riscv/isa_traits.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/process_impl.hh"
#include "sim/system.hh"

using namespace std;
using namespace RiscvISA;

RiscvLiveProcess::RiscvLiveProcess(LiveProcessParams * params,
    ObjectFile *objFile) : LiveProcess(params, objFile)
{
    // Set up stack. On RISC-V, stack starts at the top of kuseg
    // user address space. RISC-V stack grows down from here
    stack_base = 0x7FFFFFFF;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    // Set up break point (Top of Heap)
    brk_point = objFile->bssBase() + objFile->bssSize();

    // Set up region for mmaps.  Start it 1GB above the top of the heap.
    mmap_end = brk_point + 0x40000000L;
}

void
RiscvLiveProcess::initState()
{
    LiveProcess::initState();

    argsInit<uint64_t>(PageBytes);
}

template<class IntType> void
RiscvLiveProcess::argsInit(int pageSize)
{
    updateBias();

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    typedef AuxVector<IntType> auxv_t;
    vector<auxv_t> auxv;
    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if (elfObject) {
        // Set the system page size
        auxv.push_back(auxv_t(M5_AT_PAGESZ, RiscvISA::PageBytes));
        // Set the frequency at which time() increments
        auxv.push_back(auxv_t(M5_AT_CLKTCK, 100));
        // For statically linked executables, this is the virtual
        // address of the program header tables if they appear in the
        // executable image.
        auxv.push_back(auxv_t(M5_AT_PHDR, elfObject->programHeaderTable()));
        DPRINTF(Loader, "auxv at PHDR %08p\n",
            elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(M5_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(M5_AT_PHNUM, elfObject->programHeaderCount()));
        auxv.push_back(auxv_t(M5_AT_BASE, getBias()));
        //The entry point to the program
        auxv.push_back(auxv_t(M5_AT_ENTRY, objFile->entryPoint()));
        //Different user and group IDs
        auxv.push_back(auxv_t(M5_AT_UID, uid()));
        auxv.push_back(auxv_t(M5_AT_EUID, euid()));
        auxv.push_back(auxv_t(M5_AT_GID, gid()));
        auxv.push_back(auxv_t(M5_AT_EGID, egid()));
    }

    const IntType zero = 0;
    IntType argc = htog((IntType)argv.size());
    int argv_array_size = sizeof(Addr) * argv.size();
    int arg_data_size = 0;
    for (string arg: argv)
        arg_data_size += arg.size() + 1;
    int envp_array_size = sizeof(Addr) * envp.size();
    int env_data_size = 0;
    for (string env: envp)
        env_data_size += env.size() + 1;
    int auxv_array_size = 2 * sizeof(IntType)*auxv.size();

    stack_size = sizeof(IntType) + argv_array_size + 2 * sizeof(Addr) +
        arg_data_size + 2 * sizeof(Addr);
    if (!envp.empty()) {
        stack_size += 2 * sizeof(Addr) + envp_array_size + 2 * sizeof(Addr) +
            env_data_size;
    }
    if (!auxv.empty())
        stack_size += 2 * sizeof(Addr) + auxv_array_size;
    stack_min = roundDown(stack_base - stack_size, pageSize);
    allocateMem(stack_min, roundUp(stack_size, pageSize));

    Addr argv_array_base = stack_min + sizeof(IntType);
    Addr arg_data_base = argv_array_base + argv_array_size + 2 * sizeof(Addr);
    Addr envp_array_base = arg_data_base + arg_data_size;
    if (!envp.empty())
        envp_array_base += 2 * sizeof(Addr);
    Addr env_data_base = envp_array_base + envp_array_size;
    if (!envp.empty())
        env_data_base += 2 * sizeof(Addr);

    vector<Addr> arg_pointers;
    if (!argv.empty()) {
        arg_pointers.push_back(arg_data_base);
        for (int i = 0; i < argv.size() - 1; i++) {
            arg_pointers.push_back(arg_pointers[i] + argv[i].size() + 1);
        }
    }

    vector<Addr> env_pointers;
    if (!envp.empty()) {
        env_pointers.push_back(env_data_base);
        for (int i = 0; i < envp.size() - 1; i++) {
            env_pointers.push_back(env_pointers[i] + envp[i].size() + 1);
        }
    }

    Addr sp = stack_min;
    initVirtMem.writeBlob(sp, (uint8_t *)&argc, sizeof(IntType));
    sp += sizeof(IntType);
    for (Addr arg_pointer: arg_pointers) {
        initVirtMem.writeBlob(sp, (uint8_t *)&arg_pointer, sizeof(Addr));
        sp += sizeof(Addr);
    }
    for (int i = 0; i < 2; i++) {
        initVirtMem.writeBlob(sp, (uint8_t *)&zero, sizeof(Addr));
        sp += sizeof(Addr);
    }
    for (int i = 0; i < argv.size(); i++) {
        initVirtMem.writeString(sp, argv[i].c_str());
        sp += argv[i].size() + 1;
    }
    if (!envp.empty()) {
        for (int i = 0; i < 2; i++) {
            initVirtMem.writeBlob(sp, (uint8_t *)&zero, sizeof(Addr));
            sp += sizeof(Addr);
        }
    }
    for (Addr env_pointer: env_pointers)
        initVirtMem.writeBlob(sp, (uint8_t *)&env_pointer, sizeof(Addr));
    if (!envp.empty()) {
        for (int i = 0; i < 2; i++) {
            initVirtMem.writeBlob(sp, (uint8_t *)&zero, sizeof(Addr));
            sp += sizeof(Addr);
        }
    }
    for (int i = 0; i < envp.size(); i++) {
        initVirtMem.writeString(sp, envp[i].c_str());
        sp += envp[i].size() + 1;
    }
    if (!auxv.empty()) {
        for (int i = 0; i < 2; i++) {
            initVirtMem.writeBlob(sp, (uint8_t *)&zero, sizeof(Addr));
            sp += sizeof(Addr);
        }
    }
    for (auxv_t aux: auxv) {
        initVirtMem.writeBlob(sp, (uint8_t *)&aux.a_type, sizeof(IntType));
        initVirtMem.writeBlob(sp + sizeof(IntType), (uint8_t *)&aux.a_val,
            sizeof(IntType));
        sp += 2 * sizeof(IntType);
    }
    for (int i = 0; i < 2; i++) {
        initVirtMem.writeBlob(sp, (uint8_t *)&zero, sizeof(Addr));
        sp += sizeof(Addr);
    }

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    tc->setIntReg(StackPointerReg, stack_min);
    tc->pcState(getStartPC());
}

RiscvISA::IntReg
RiscvLiveProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    return tc->readIntReg(SyscallArgumentRegs[i++]);
}

void
RiscvLiveProcess::setSyscallArg(ThreadContext *tc, int i, RiscvISA::IntReg val)
{
    tc->setIntReg(SyscallArgumentRegs[i], val);
}

void
RiscvLiveProcess::setSyscallReturn(ThreadContext *tc, SyscallReturn sysret)
{
    if (sysret.successful()) {
        // no error
        tc->setIntReg(SyscallPseudoReturnReg, sysret.returnValue());
    } else {
        // got an error, return details
        tc->setIntReg(SyscallPseudoReturnReg, sysret.errnoValue());
    }
}
