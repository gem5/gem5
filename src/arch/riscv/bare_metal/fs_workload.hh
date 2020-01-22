/*
 * Copyright (c) 2018 TU Dresden
 * All rights reserved
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

#ifndef __ARCH_RISCV_BARE_METAL_SYSTEM_HH__
#define __ARCH_RISCV_BARE_METAL_SYSTEM_HH__

#include "arch/riscv/fs_workload.hh"
#include "params/RiscvBareMetal.hh"

namespace RiscvISA
{

class BareMetal : public RiscvISA::FsWorkload
{
  protected:
    Loader::ObjectFile *bootloader;
    Loader::SymbolTable bootloaderSymtab;

  public:
    typedef RiscvBareMetalParams Params;
    BareMetal(Params *p);
    ~BareMetal();

    void initState() override;

    Loader::Arch getArch() const override { return bootloader->getArch(); }
    const Loader::SymbolTable &
    symtab(ThreadContext *tc) override
    {
        return bootloaderSymtab;
    }
    bool
    insertSymbol(const Loader::Symbol &symbol) override
    {
        return bootloaderSymtab.insert(symbol);
    }
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_BARE_METAL_FS_WORKLOAD_HH__
