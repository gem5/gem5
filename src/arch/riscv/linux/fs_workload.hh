/*
 * Copyright (c) 2021 Huawei International
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

#ifndef __ARCH_RISCV_LINUX_SYSTEM_HH__
#define __ARCH_RISCV_LINUX_SYSTEM_HH__

#include <string>

#include "arch/riscv/remote_gdb.hh"
#include "params/RiscvBootloaderKernelWorkload.hh"
#include "params/RiscvLinux.hh"
#include "sim/kernel_workload.hh"

namespace gem5
{

namespace RiscvISA
{

class FsLinux : public KernelWorkload
{
  private:
    /**
     * Event to halt the simulator if the kernel calls panic() or
     * oops_exit()
     **/
    PCEvent *kernelPanicPcEvent = nullptr;
    PCEvent *kernelOopsPcEvent = nullptr;
    void addExitOnKernelPanicEvent();
    void addExitOnKernelOopsEvent();

  public:
    PARAMS(RiscvLinux);

    FsLinux(const Params &p) : KernelWorkload(p) {}

    ~FsLinux()
    {
        if (kernelPanicPcEvent != nullptr) {
            delete kernelPanicPcEvent;
        }
        if (kernelOopsPcEvent != nullptr) {
            delete kernelOopsPcEvent;
        }
    }

    void initState() override;
    void startup() override;

    void
    setSystem(System *sys) override
    {
        KernelWorkload::setSystem(sys);
        gdb =
            BaseRemoteGDB::build<RemoteGDB>(params().remote_gdb_port, system);
    }

    ByteOrder
    byteOrder() const override
    {
        return ByteOrder::little;
    }
};

class BootloaderKernelWorkload : public Workload
{
  private:
    Addr entryPoint = 0;
    loader::ObjectFile *kernel = nullptr;
    loader::ObjectFile *bootloader = nullptr;
    loader::SymbolTable kernelSymbolTable;
    loader::SymbolTable bootloaderSymbolTable;
    const std::string bootArgs;

    /**
     * Event to halt the simulator if the kernel calls panic() or
     * oops_exit()
     **/
    PCEvent *kernelPanicPcEvent = nullptr;
    PCEvent *kernelOopsPcEvent = nullptr;

  private:
    void loadBootloaderSymbolTable();
    void loadKernelSymbolTable();
    void loadBootloader();
    void loadKernel();
    void loadDtb();
    void addExitOnKernelPanicEvent();
    void addExitOnKernelOopsEvent();

  public:
    PARAMS(RiscvBootloaderKernelWorkload);

    BootloaderKernelWorkload(const Params &p)
        : Workload(p), entryPoint(p.entry_point), bootArgs(p.command_line)
    {
        loadBootloaderSymbolTable();
        loadKernelSymbolTable();
    }

    ~BootloaderKernelWorkload()
    {
        if (kernelPanicPcEvent != nullptr) {
            delete kernelPanicPcEvent;
        }
        if (kernelOopsPcEvent != nullptr) {
            delete kernelOopsPcEvent;
        }
    }

    void initState() override;
    void startup() override;

    void
    setSystem(System *sys) override
    {
        Workload::setSystem(sys);
        gdb =
            BaseRemoteGDB::build<RemoteGDB>(params().remote_gdb_port, system);
    }

    Addr
    getEntry() const override
    {
        return entryPoint;
    }

    ByteOrder
    byteOrder() const override
    {
        return ByteOrder::little;
    }

    loader::Arch
    getArch() const override
    {
        return kernel->getArch();
    }

    const loader::SymbolTable &
    symtab(ThreadContext *tc) override
    {
        return kernelSymbolTable;
    }

    bool
    insertSymbol(const loader::Symbol &symbol) override
    {
        return kernelSymbolTable.insert(symbol);
    }

    void serialize(CheckpointOut &checkpoint) const override;
    void unserialize(CheckpointIn &checkpoint) override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_LINUX_FS_WORKLOAD_HH__
