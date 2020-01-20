/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "arch/mips/process.hh"

#include "arch/mips/isa_traits.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/process.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

using namespace std;
using namespace MipsISA;

MipsProcess::MipsProcess(ProcessParams *params, ::Loader::ObjectFile *objFile)
    : Process(params,
              new EmulationPageTable(params->name, params->pid, PageBytes),
              objFile)
{
    fatal_if(params->useArchPT, "Arch page tables not implemented.");
    // Set up stack. On MIPS, stack starts at the top of kuseg
    // user address space. MIPS stack grows down from here
    Addr stack_base = 0x7FFFFFFF;

    Addr max_stack_size = 8 * 1024 * 1024;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    Addr next_thread_stack_base = stack_base - max_stack_size;

    // Set up break point (Top of Heap)
    Addr brk_point = image.maxAddr();
    brk_point = roundUp(brk_point, PageBytes);

    // Set up region for mmaps.  Start it 1GB above the top of the heap.
    Addr mmap_end = brk_point + 0x40000000L;

    memState = make_shared<MemState>(this, brk_point, stack_base,
                                     max_stack_size, next_thread_stack_base,
                                     mmap_end);
}

void
MipsProcess::initState()
{
    Process::initState();

    argsInit<uint32_t>(PageBytes);
}

template<class IntType>
void
MipsProcess::argsInit(int pageSize)
{
    int intSize = sizeof(IntType);

    std::vector<AuxVector<IntType>> auxv;

    auto *elfObject = dynamic_cast<::Loader::ElfObject *>(objFile);
    if (elfObject)
    {
        // Set the system page size
        auxv.emplace_back(M5_AT_PAGESZ, MipsISA::PageBytes);
        // Set the frequency at which time() increments
        auxv.emplace_back(M5_AT_CLKTCK, 100);
        // For statically linked executables, this is the virtual
        // address of the program header tables if they appear in the
        // executable image.
        auxv.emplace_back(M5_AT_PHDR, elfObject->programHeaderTable());
        DPRINTF(Loader, "auxv at PHDR %08p\n",
                elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.emplace_back(M5_AT_PHENT, elfObject->programHeaderSize());
        // This is the number of program headers from the original elf file.
        auxv.emplace_back(M5_AT_PHNUM, elfObject->programHeaderCount());
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.emplace_back(M5_AT_BASE, getBias());
        //The entry point to the program
        auxv.emplace_back(M5_AT_ENTRY, objFile->entryPoint());
        //Different user and group IDs
        auxv.emplace_back(M5_AT_UID, uid());
        auxv.emplace_back(M5_AT_EUID, euid());
        auxv.emplace_back(M5_AT_GID, gid());
        auxv.emplace_back(M5_AT_EGID, egid());
        auxv.emplace_back(M5_AT_RANDOM, 0);
    }

    // Calculate how much space we need for arg & env & auxv arrays.
    int argv_array_size = intSize * (argv.size() + 1);
    int envp_array_size = intSize * (envp.size() + 1);
    int auxv_array_size = intSize * 2 * (auxv.size() + 1);

    int arg_data_size = 0;
    for (vector<string>::size_type i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }

    const int numRandomBytes = 16;
    int aux_data_size = numRandomBytes;

    int env_data_size = 0;
    for (vector<string>::size_type i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }

    int space_needed =
        argv_array_size +
        envp_array_size +
        auxv_array_size +
        arg_data_size +
        aux_data_size +
        env_data_size;

    // set bottom of stack
    memState->setStackMin(memState->getStackBase() - space_needed);
    // align it
    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
    memState->setStackSize(memState->getStackBase() - memState->getStackMin());
    // map memory
    memState->mapRegion(memState->getStackMin(),
                        roundUp(memState->getStackSize(), pageSize), "stack");

    // map out initial stack contents; leave room for argc
    IntType argv_array_base = memState->getStackMin() + intSize;
    IntType envp_array_base = argv_array_base + argv_array_size;
    IntType auxv_array_base = envp_array_base + envp_array_size;
    IntType arg_data_base = auxv_array_base + auxv_array_size;
    IntType aux_data_base = arg_data_base - arg_data_size;
    IntType env_data_base = aux_data_base + aux_data_size;

    // write contents to stack
    IntType argc = argv.size();

    argc = htole((IntType)argc);

    initVirtMem->writeBlob(memState->getStackMin(), &argc, intSize);

    copyStringArray(argv, argv_array_base, arg_data_base,
                    LittleEndianByteOrder, *initVirtMem);

    copyStringArray(envp, envp_array_base, env_data_base,
                    LittleEndianByteOrder, *initVirtMem);

    // Fix up the aux vectors which point to data.
    for (auto &aux: auxv) {
        if (aux.type == M5_AT_RANDOM)
            aux.val = aux_data_base;
    }

    // Copy the aux vector
    Addr auxv_array_end = auxv_array_base;
    for (const auto &aux: auxv) {
        initVirtMem->write(auxv_array_end, aux, GuestByteOrder);
        auxv_array_end += sizeof(aux);
    }

    // Write out the terminating zeroed auxilliary vector
    const AuxVector<IntType> zero(0, 0);
    initVirtMem->write(auxv_array_end, zero);
    auxv_array_end += sizeof(zero);

    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    tc->setIntReg(FirstArgumentReg, argc);
    tc->setIntReg(FirstArgumentReg + 1, argv_array_base);
    tc->setIntReg(StackPointerReg, memState->getStackMin());

    tc->pcState(getStartPC());
}

const std::vector<int> MipsProcess::SyscallABI::ArgumentRegs = {
    4, 5, 6, 7, 8, 9
};
