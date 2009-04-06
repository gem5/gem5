/*
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Stephen Hines
 */

#include "arch/arm/isa_traits.hh"
#include "arch/arm/process.hh"
#include "arch/arm/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "mem/translating_port.hh"
#include "sim/process_impl.hh"
#include "sim/system.hh"

using namespace std;
using namespace ArmISA;

ArmLiveProcess::ArmLiveProcess(LiveProcessParams * params,
        ObjectFile *objFile)
    : LiveProcess(params, objFile)
{
    stack_base = 0xc0000000L;

    // Set pointer for next thread stack.  Reserve 8M for main stack.
    next_thread_stack_base = stack_base - (8 * 1024 * 1024);

    // Set up break point (Top of Heap)
    brk_point = objFile->dataBase() + objFile->dataSize() + objFile->bssSize();
    brk_point = roundUp(brk_point, VMPageSize);

    // Set up region for mmaps. For now, start at bottom of kuseg space.
    mmap_start = mmap_end = 0x70000000L;
}

void
ArmLiveProcess::startup()
{
    argsInit(MachineBytes, VMPageSize);
}

void
ArmLiveProcess::copyStringArray32(std::vector<std::string> &strings,
        Addr array_ptr, Addr data_ptr,
        TranslatingPort* memPort)
{
    Addr data_ptr_swap;
    for (int i = 0; i < strings.size(); ++i) {
        data_ptr_swap = htog(data_ptr);
        memPort->writeBlob(array_ptr, (uint8_t*)&data_ptr_swap,
                sizeof(uint32_t));
        memPort->writeString(data_ptr, strings[i].c_str());
        array_ptr += sizeof(uint32_t);
        data_ptr += strings[i].size() + 1;
    }
    // add NULL terminator
    data_ptr = 0;

    memPort->writeBlob(array_ptr, (uint8_t*)&data_ptr, sizeof(uint32_t));
}

void
ArmLiveProcess::argsInit(int intSize, int pageSize)
{
    // Overloaded argsInit so that we can fine-tune for ARM architecture
    Process::startup();

    // load object file into target memory
    objFile->loadSections(initVirtMem);

    // Calculate how much space we need for arg & env arrays.
    int argv_array_size = intSize * (argv.size() + 1);
    int envp_array_size = intSize * (envp.size() + 1);
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i) {
        arg_data_size += argv[i].size() + 1;
    }
    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i) {
        env_data_size += envp[i].size() + 1;
    }

    int space_needed =
        argv_array_size + envp_array_size + arg_data_size + env_data_size;
    if (space_needed < 16*1024)
        space_needed = 16*1024;

    // set bottom of stack
    stack_min = stack_base - space_needed;
    // align it
    stack_min = roundDown(stack_min, pageSize);
    stack_size = stack_base - stack_min;
    // map memory
    pTable->allocate(stack_min, roundUp(stack_size, pageSize));

    // map out initial stack contents
    Addr argv_array_base = stack_min + intSize; // room for argc
    Addr envp_array_base = argv_array_base + argv_array_size;
    Addr arg_data_base = envp_array_base + envp_array_size;
    Addr env_data_base = arg_data_base + arg_data_size;

    // write contents to stack
    uint64_t argc = argv.size();
    if (intSize == 8)
        argc = htog((uint64_t)argc);
    else if (intSize == 4)
        argc = htog((uint32_t)argc);
    else
        panic("Unknown int size");

    initVirtMem->writeBlob(stack_min, (uint8_t*)&argc, intSize);

    copyStringArray32(argv, argv_array_base, arg_data_base, initVirtMem);
    copyStringArray32(envp, envp_array_base, env_data_base, initVirtMem);

    /*
    //uint8_t insns[] = {0xe5, 0x9f, 0x00, 0x08, 0xe1, 0xa0, 0xf0, 0x0e};
    uint8_t insns[] = {0x08, 0x00, 0x9f, 0xe5, 0x0e, 0xf0, 0xa0, 0xe1};

    initVirtMem->writeBlob(0xffff0fe0, insns, 8);
    */

    threadContexts[0]->setIntReg(ArgumentReg1, argc);
    threadContexts[0]->setIntReg(ArgumentReg2, argv_array_base);
    threadContexts[0]->setIntReg(StackPointerReg, stack_min);

    Addr prog_entry = objFile->entryPoint();
    threadContexts[0]->setPC(prog_entry);
    threadContexts[0]->setNextPC(prog_entry + sizeof(MachInst));
}

