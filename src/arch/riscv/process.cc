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
 */

#include "arch/riscv/process.hh"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "arch/riscv/isa.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/misc.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/process.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace RiscvISA;

RiscvProcess::RiscvProcess(const ProcessParams &params,
        loader::ObjectFile *objFile) :
        Process(params,
                new EmulationPageTable(params.name, params.pid, PageBytes),
                objFile)
{
    fatal_if(params.useArchPT, "Arch page tables not implemented.");
}

RiscvProcess64::RiscvProcess64(const ProcessParams &params,
        loader::ObjectFile *objFile) :
        RiscvProcess(params, objFile)
{
    const Addr stack_base = 0x7FFFFFFFFFFFFFFFL;
    const Addr max_stack_size = 8 * 1024 * 1024;
    const Addr next_thread_stack_base = stack_base - max_stack_size;
    const Addr brk_point = roundUp(image.maxAddr(), PageBytes);
    const Addr mmap_end = 0x4000000000000000L;
    memState = std::make_shared<MemState>(this, brk_point, stack_base,
            max_stack_size, next_thread_stack_base, mmap_end);
}

RiscvProcess32::RiscvProcess32(const ProcessParams &params,
        loader::ObjectFile *objFile) :
        RiscvProcess(params, objFile)
{
    const Addr stack_base = 0x7FFFFFFF;
    const Addr max_stack_size = 8 * 1024 * 1024;
    const Addr next_thread_stack_base = stack_base - max_stack_size;
    const Addr brk_point = roundUp(image.maxAddr(), PageBytes);
    const Addr mmap_end = 0x40000000L;
    memState = std::make_shared<MemState>(this, brk_point, stack_base,
            max_stack_size, next_thread_stack_base, mmap_end);
}

void
RiscvProcess64::initState()
{
    Process::initState();

    argsInit<uint64_t>(PageBytes);
    for (ContextID ctx: contextIds) {
        auto *tc = system->threads[ctx];
        tc->setMiscRegNoEffect(MISCREG_PRV, PRV_U);
        auto *isa = dynamic_cast<ISA*>(tc->getIsaPtr());
        fatal_if(isa->rvType() != RV64, "RISC V CPU should run in 64 bits mode");
        MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
        fatal_if(!(misa.rvu && misa.rvs),
            "RISC V SE mode can't run without supervisor and user "
            "privilege modes.");
    }
}

void
RiscvProcess32::initState()
{
    Process::initState();

    argsInit<uint32_t>(PageBytes);
    for (ContextID ctx: contextIds) {
        auto *tc = system->threads[ctx];
        tc->setMiscRegNoEffect(MISCREG_PRV, PRV_U);
        auto *isa = dynamic_cast<ISA*>(tc->getIsaPtr());
        fatal_if(isa->rvType() != RV32, "RISC V CPU should run in 32 bits mode");
        MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
        fatal_if(!(misa.rvu && misa.rvs),
            "RISC V SE mode can't run without supervisor and user "
            "privilege modes.");
    }
}

template<class IntType> void
RiscvProcess::argsInit(int pageSize)
{
    const int RandomBytes = 16;
    const int addrSize = sizeof(IntType);

    auto *elfObject = dynamic_cast<loader::ElfObject*>(objFile);
    memState->setStackMin(memState->getStackBase());

    // Determine stack size and populate auxv
    Addr stack_top = memState->getStackMin();
    stack_top -= RandomBytes;
    for (const std::string& arg: argv)
        stack_top -= arg.size() + 1;
    for (const std::string& env: envp)
        stack_top -= env.size() + 1;
    stack_top &= -addrSize;

    std::vector<gem5::auxv::AuxVector<IntType>> auxv;
    if (elfObject != nullptr) {
        auxv.emplace_back(gem5::auxv::Entry, objFile->entryPoint());
        auxv.emplace_back(gem5::auxv::Phnum, elfObject->programHeaderCount());
        auxv.emplace_back(gem5::auxv::Phent, elfObject->programHeaderSize());
        auxv.emplace_back(gem5::auxv::Phdr, elfObject->programHeaderTable());
        auxv.emplace_back(gem5::auxv::Pagesz, PageBytes);
        auxv.emplace_back(gem5::auxv::Secure, 0);
        auxv.emplace_back(gem5::auxv::Random, stack_top);
        auxv.emplace_back(gem5::auxv::Null, 0);
    }
    stack_top -= (1 + argv.size()) * addrSize +
                   (1 + envp.size()) * addrSize +
                   addrSize + 2 * sizeof(IntType) * auxv.size();
    stack_top &= -2*addrSize;
    memState->setStackSize(memState->getStackBase() - stack_top);
    memState->mapRegion(roundDown(stack_top, pageSize),
                        roundUp(memState->getStackSize(), pageSize), "stack");

    // Copy random bytes (for AT_RANDOM) to stack
    memState->setStackMin(memState->getStackMin() - RandomBytes);
    uint8_t at_random[RandomBytes];
    std::generate(std::begin(at_random), std::end(at_random),
                  [&]{ return rng->random(0, 0xFF); });
    initVirtMem->writeBlob(memState->getStackMin(), at_random, RandomBytes);

    // Copy argv to stack
    std::vector<Addr> argPointers;
    for (const std::string& arg: argv) {
        memState->setStackMin(memState->getStackMin() - (arg.size() + 1));
        initVirtMem->writeString(memState->getStackMin(), arg.c_str());
        argPointers.push_back(memState->getStackMin());
        if (debug::Stack) {
            std::string wrote;
            initVirtMem->readString(wrote, argPointers.back());
            DPRINTFN("Wrote arg \"%s\" to address %p\n",
                    wrote, (void*)memState->getStackMin());
        }
    }
    argPointers.push_back(0);

    // Copy envp to stack
    std::vector<Addr> envPointers;
    for (const std::string& env: envp) {
        memState->setStackMin(memState->getStackMin() - (env.size() + 1));
        initVirtMem->writeString(memState->getStackMin(), env.c_str());
        envPointers.push_back(memState->getStackMin());
        DPRINTF(Stack, "Wrote env \"%s\" to address %p\n",
                env, (void*)memState->getStackMin());
    }
    envPointers.push_back(0);

    // Align stack
    memState->setStackMin(memState->getStackMin() & -addrSize);

    // Calculate bottom of stack
    memState->setStackMin(memState->getStackMin() -
            ((1 + argv.size()) * addrSize +
             (1 + envp.size()) * addrSize +
             addrSize + 2 * sizeof(IntType) * auxv.size()));
    memState->setStackMin(memState->getStackMin() & (-2 * addrSize));
    Addr sp = memState->getStackMin();
    const auto pushOntoStack =
        [this, &sp](IntType data) {
            initVirtMem->write(sp, data, ByteOrder::little);
            sp += sizeof(data);
        };

    // Push argc and argv pointers onto stack
    IntType argc = argv.size();
    DPRINTF(Stack, "Wrote argc %d to address %#x\n", argc, sp);
    pushOntoStack(argc);

    for (const Addr& argPointer: argPointers) {
        DPRINTF(Stack, "Wrote argv pointer %#x to address %#x\n",
                argPointer, sp);
        pushOntoStack(argPointer);
    }

    // Push env pointers onto stack
    for (const Addr& envPointer: envPointers) {
        DPRINTF(Stack, "Wrote envp pointer %#x to address %#x\n",
                envPointer, sp);
        pushOntoStack(envPointer);
    }

    // Push aux vector onto stack
    std::map<IntType, std::string> aux_keys = {
        {gem5::auxv::Entry, "gem5::auxv::Entry"},
        {gem5::auxv::Phnum, "gem5::auxv::Phnum"},
        {gem5::auxv::Phent, "gem5::auxv::Phent"},
        {gem5::auxv::Phdr, "gem5::auxv::Phdr"},
        {gem5::auxv::Pagesz, "gem5::auxv::Pagesz"},
        {gem5::auxv::Secure, "gem5::auxv::Secure"},
        {gem5::auxv::Random, "gem5::auxv::Random"},
        {gem5::auxv::Null, "gem5::auxv::Null"}
    };
    for (const auto &aux: auxv) {
        DPRINTF(Stack, "Wrote aux key %s to address %#x\n",
                aux_keys[aux.type], sp);
        pushOntoStack(aux.type);
        DPRINTF(Stack, "Wrote aux value %x to address %#x\n", aux.val, sp);
        pushOntoStack(aux.val);
    }

    ThreadContext *tc = system->threads[contextIds[0]];
    tc->setReg(StackPointerReg, memState->getStackMin());
    tc->pcState(getStartPC());

    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
}

} // namespace gem5
