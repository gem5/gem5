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

#include "arch/riscv/remote_gdb.hh"
#include "params/RiscvBareMetal.hh"
#include "sim/workload.hh"

namespace gem5
{

namespace RiscvISA
{

class BareMetal : public Workload
{
  protected:
    // checker for bare metal application
    bool _isBareMetal;
    // entry point for simulation
    Addr _resetVect;
    loader::ObjectFile *bootloader;
    loader::SymbolTable bootloaderSymtab;

  public:
    PARAMS(RiscvBareMetal);
    BareMetal(const Params &p);
    ~BareMetal();

    void initState() override;

    void
    setSystem(System *sys) override
    {
        Workload::setSystem(sys);
        gdb = BaseRemoteGDB::build<RemoteGDB>(
                params().remote_gdb_port, system);
    }

    loader::Arch getArch() const override { return bootloader->getArch(); }
    ByteOrder byteOrder() const override { return ByteOrder::little; }

    const loader::SymbolTable &
    symtab(ThreadContext *tc) override
    {
        return bootloaderSymtab;
    }

    bool
    insertSymbol(const loader::Symbol &symbol) override
    {
        return bootloaderSymtab.insert(symbol);
    }

    // return reset vector
    Addr resetVect() const { return _resetVect; }

    // return bare metal checker
    bool isBareMetal() const { return _isBareMetal; }

    Addr getEntry() const override { return _resetVect; }
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_BARE_METAL_FS_WORKLOAD_HH__
