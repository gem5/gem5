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

#ifndef __SPARC_PROCESS_HH__
#define __SPARC_PROCESS_HH__

#include <memory>
#include <string>
#include <vector>

#include "arch/sparc/page_size.hh"
#include "base/loader/object_file.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"

namespace gem5
{

class SparcProcess : public Process
{
  protected:
    const Addr StackBias;

    // The locations of the fill and spill handlers
    Addr fillStart, spillStart;

    SparcProcess(const ProcessParams &params, loader::ObjectFile *objFile,
                 Addr _StackBias);

    void initState() override;

    template <class IntType>
    void argsInit(int pageSize);

  public:
    Addr
    readFillStart()
    {
        return fillStart;
    }

    Addr
    readSpillStart()
    {
        return spillStart;
    }
};

class Sparc32Process : public SparcProcess
{
  protected:
    void initState() override;

  public:
    Sparc32Process(const ProcessParams &params, loader::ObjectFile *objFile)
        : SparcProcess(params, objFile, 0)
    {
        Addr brk_point = image.maxAddr();
        brk_point = roundUp(brk_point, SparcISA::PageBytes);

        // Reserve 8M for main stack.
        Addr max_stack_size = 8 * 1024 * 1024;

        // Set up stack. On SPARC Linux, stack goes from the top of memory
        // downward, less the hole for the kernel address space.
        Addr stack_base = 0xf0000000ULL;

        // Set pointer for next thread stack.
        Addr next_thread_stack_base = stack_base - max_stack_size;

        // Set up region for mmaps.
        Addr mmap_end = 0x70000000;

        memState = std::make_shared<MemState>(
            this, brk_point, stack_base, max_stack_size,
            next_thread_stack_base, mmap_end);
    }

    void argsInit(int intSize, int pageSize);
};

class Sparc64Process : public SparcProcess
{
  protected:
    void initState() override;

  public:
    Sparc64Process(const ProcessParams &params, loader::ObjectFile *objFile)
        : SparcProcess(params, objFile, 2047)
    {
        Addr brk_point = image.maxAddr();
        brk_point = roundUp(brk_point, SparcISA::PageBytes);

        Addr max_stack_size = 8 * 1024 * 1024;

        // Set up stack. On SPARC Linux, stack goes from the top of memory
        // downward, less the hole for the kernel address space.
        Addr stack_base = 0x80000000000ULL;

        // Set pointer for next thread stack.  Reserve 8M for main stack.
        Addr next_thread_stack_base = stack_base - max_stack_size;

        // Set up region for mmaps.
        Addr mmap_end = 0xfffff80000000000ULL;

        memState = std::make_shared<MemState>(
            this, brk_point, stack_base, max_stack_size,
            next_thread_stack_base, mmap_end);
    }

    void argsInit(int intSize, int pageSize);
};

} // namespace gem5

#endif // __SPARC_PROCESS_HH__
