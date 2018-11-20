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
 *
 * Authors: Gabe Black
 *          Ali Saidi
 *          Korey Sewell
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

MipsProcess::MipsProcess(ProcessParams *params, ObjectFile *objFile)
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
    Addr brk_point = objFile->dataBase() + objFile->dataSize() +
                     objFile->bssSize();
    brk_point = roundUp(brk_point, PageBytes);

    // Set up region for mmaps.  Start it 1GB above the top of the heap.
    Addr mmap_end = brk_point + 0x40000000L;

    memState = make_shared<MemState>(brk_point, stack_base, max_stack_size,
                                     next_thread_stack_base, mmap_end);
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

    // Patch the ld_bias for dynamic executables.
    updateBias();

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    typedef AuxVector<IntType> auxv_t;
    std::vector<auxv_t> auxv;

    ElfObject * elfObject = dynamic_cast<ElfObject *>(objFile);
    if (elfObject)
    {
        // Set the system page size
        auxv.push_back(auxv_t(M5_AT_PAGESZ, MipsISA::PageBytes));
        // Set the frequency at which time() increments
        auxv.push_back(auxv_t(M5_AT_CLKTCK, 100));
        // For statically linked executables, this is the virtual
        // address of the program header tables if they appear in the
        // executable image.
        auxv.push_back(auxv_t(M5_AT_PHDR, elfObject->programHeaderTable()));
        DPRINTF(Loader, "auxv at PHDR %08p\n", elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.push_back(auxv_t(M5_AT_PHENT, elfObject->programHeaderSize()));
        // This is the number of program headers from the original elf file.
        auxv.push_back(auxv_t(M5_AT_PHNUM, elfObject->programHeaderCount()));
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.push_back(auxv_t(M5_AT_BASE, getBias()));
        //The entry point to the program
        auxv.push_back(auxv_t(M5_AT_ENTRY, objFile->entryPoint()));
        //Different user and group IDs
        auxv.push_back(auxv_t(M5_AT_UID, uid()));
        auxv.push_back(auxv_t(M5_AT_EUID, euid()));
        auxv.push_back(auxv_t(M5_AT_GID, gid()));
        auxv.push_back(auxv_t(M5_AT_EGID, egid()));
    }

    // Calculate how much space we need for arg & env & auxv arrays.
    int argv_array_size = intSize * (argv.size() + 1);
    int envp_array_size = intSize * (envp.size() + 1);
    int auxv_array_size = intSize * 2 * (auxv.size() + 1);

    int arg_data_size = 0;
    for (vector<string>::size_type i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }
    int env_data_size = 0;
    for (vector<string>::size_type i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }

    int space_needed =
        argv_array_size +
        envp_array_size +
        auxv_array_size +
        arg_data_size +
        env_data_size;

    // set bottom of stack
    memState->setStackMin(memState->getStackBase() - space_needed);
    // align it
    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
    memState->setStackSize(memState->getStackBase() - memState->getStackMin());
    // map memory
    allocateMem(memState->getStackMin(), roundUp(memState->getStackSize(),
                pageSize));

    // map out initial stack contents; leave room for argc
    IntType argv_array_base = memState->getStackMin() + intSize;
    IntType envp_array_base = argv_array_base + argv_array_size;
    IntType auxv_array_base = envp_array_base + envp_array_size;
    IntType arg_data_base = auxv_array_base + auxv_array_size;
    IntType env_data_base = arg_data_base + arg_data_size;

    // write contents to stack
    IntType argc = argv.size();

    argc = htog((IntType)argc);

    initVirtMem.writeBlob(memState->getStackMin(), (uint8_t*)&argc, intSize);

    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);

    // Copy the aux vector
    for (typename vector<auxv_t>::size_type x = 0; x < auxv.size(); x++) {
        initVirtMem.writeBlob(auxv_array_base + x * 2 * intSize,
                (uint8_t*)&(auxv[x].getAuxType()), intSize);
        initVirtMem.writeBlob(auxv_array_base + (x * 2 + 1) * intSize,
                (uint8_t*)&(auxv[x].getAuxVal()), intSize);
    }

    // Write out the terminating zeroed auxilliary vector
    for (unsigned i = 0; i < 2; i++) {
        const IntType zero = 0;
        const Addr addr = auxv_array_base + 2 * intSize * (auxv.size() + i);
        initVirtMem.writeBlob(addr, (uint8_t*)&zero, intSize);
    }

    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    setSyscallArg(tc, 0, argc);
    setSyscallArg(tc, 1, argv_array_base);
    tc->setIntReg(StackPointerReg, memState->getStackMin());

    tc->pcState(getStartPC());
}


RegVal
MipsProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < 6);
    return tc->readIntReg(FirstArgumentReg + i++);
}

void
MipsProcess::setSyscallArg(ThreadContext *tc, int i, RegVal val)
{
    assert(i < 6);
    tc->setIntReg(FirstArgumentReg + i, val);
}

void
MipsProcess::setSyscallReturn(ThreadContext *tc, SyscallReturn sysret)
{
    if (sysret.successful()) {
        // no error
        tc->setIntReg(SyscallSuccessReg, 0);
        tc->setIntReg(ReturnValueReg, sysret.returnValue());
    } else {
        // got an error, return details
        tc->setIntReg(SyscallSuccessReg, (uint32_t)(-1));
        tc->setIntReg(ReturnValueReg, sysret.errnoValue());
    }
}
