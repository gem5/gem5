/*
 * Copyright 2022 Google Inc.
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

#ifndef __ARCH_X86_BARE_METAL_WORKLOAD_HH__
#define __ARCH_X86_BARE_METAL_WORKLOAD_HH__

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "params/X86BareMetalWorkload.hh"
#include "sim/workload.hh"

namespace gem5
{

namespace X86ISA
{

class BareMetalWorkload : public Workload
{
  public:
    using Params = X86BareMetalWorkloadParams;
    BareMetalWorkload(const Params &p);

  public:
    void initState() override;

    Addr
    getEntry() const override
    {
        return 0;
    }

    ByteOrder
    byteOrder() const override
    {
        return ByteOrder::little;
    }

    loader::Arch
    getArch() const override
    {
        return loader::UnknownArch;
    }

    const loader::SymbolTable &
    symtab(ThreadContext *tc) override
    {
        static loader::SymbolTable sym_tab;
        return sym_tab;
    }

    bool
    insertSymbol(const loader::Symbol &symbol) override
    {
        return false;
    }
};

} // namespace X86ISA

} // namespace gem5

#endif // __ARCH_X86_BARE_METAL_WORKLOAD_HH__
