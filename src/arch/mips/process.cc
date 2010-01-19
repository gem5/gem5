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

#include "arch/mips/isa_traits.hh"
#include "arch/mips/process.hh"

#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"

#include "mem/page_table.hh"

#include "sim/process.hh"
#include "sim/process_impl.hh"
#include "sim/system.hh"

using namespace std;
using namespace MipsISA;

MipsLiveProcess::MipsLiveProcess(LiveProcessParams * params,
        ObjectFile *objFile)
    : LiveProcess(params, objFile)
{
    // Set up stack. On MIPS, stack starts at the top of kuseg
    // user address space. MIPS stack grows down from here
    stack_base = 0x7FFFFFFF;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    // Set up break point (Top of Heap)
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, VMPageSize);

    // Set up region for mmaps. For now, start at bottom of kuseg space.
    mmap_start = mmap_end = 0x10000;
}

void
MipsLiveProcess::startup()
{
    Process::startup();

    argsInit(MachineBytes, VMPageSize);
}

void
MipsLiveProcess::argsInit(int intSize, int pageSize)
{
    // load object file into target memory
    objFile->loadSections(initVirtMem);

    // Calculate how much space we need for arg & env arrays.
    int argv_array_size = intSize * (argv.size() + 1);
    int envp_array_size = intSize * (envp.size() + 1);
    int arg_data_size = 0;
    for (vector<string>::size_type i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }
    int env_data_size = 0;
    for (vector<string>::size_type i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }

    int space_needed =
         argv_array_size + envp_array_size + arg_data_size + env_data_size;
    if (space_needed < 32*1024)
        space_needed = 32*1024;

    // set bottom of stack
    stack_min = stack_base - space_needed;
    // align it
    stack_min = roundDown(stack_min, pageSize);
    stack_size = stack_base - stack_min;
    // map memory
    pTable->allocate(stack_min, roundUp(stack_size, pageSize));

    // map out initial stack contents
    // ========
    // NOTE: Using uint32_t hardcodes MIPS32 and not MIPS64
    // even if MIPS64 was intended. This is because the
    // copyStringArray function templates on the parameters.
    // Elegant way to check intSize and vary between 32/64?
    // ========
    uint32_t argv_array_base = stack_min + intSize; // room for argc
    uint32_t envp_array_base = argv_array_base + argv_array_size;
    uint32_t arg_data_base = envp_array_base + envp_array_size;
    uint32_t env_data_base = arg_data_base + arg_data_size;

    // write contents to stack
    uint32_t argc = argv.size();

    if (intSize == 8)
        argc = htog((uint64_t)argc);
    else if (intSize == 4)
        argc = htog((uint32_t)argc);
    else
        panic("Unknown int size");


    initVirtMem->writeBlob(stack_min, (uint8_t*)&argc, intSize);

    copyStringArray(argv, argv_array_base, arg_data_base, initVirtMem);

    copyStringArray(envp, envp_array_base, env_data_base, initVirtMem);

    ThreadContext *tc = system->getThreadContext(contextIds[0]);

    setSyscallArg(tc, 0, argc);
    setSyscallArg(tc, 1, argv_array_base);
    tc->setIntReg(StackPointerReg, stack_min);

    Addr prog_entry = objFile->entryPoint();
    tc->setPC(prog_entry);
    tc->setNextPC(prog_entry + sizeof(MachInst));
    tc->setNextNPC(prog_entry + (2 * sizeof(MachInst)));
}


MipsISA::IntReg
MipsLiveProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    assert(i < 6);
    return tc->readIntReg(FirstArgumentReg + i++);
}

void
MipsLiveProcess::setSyscallArg(ThreadContext *tc,
        int i, MipsISA::IntReg val)
{
    assert(i < 6);
    tc->setIntReg(FirstArgumentReg + i, val);
}

void
MipsLiveProcess::setSyscallReturn(ThreadContext *tc,
        SyscallReturn return_value)
{
    if (return_value.successful()) {
        // no error
        tc->setIntReg(SyscallSuccessReg, 0);
        tc->setIntReg(ReturnValueReg, return_value.value());
    } else {
        // got an error, return details
        tc->setIntReg(SyscallSuccessReg, (IntReg) -1);
        tc->setIntReg(ReturnValueReg, -return_value.value());
    }
}
