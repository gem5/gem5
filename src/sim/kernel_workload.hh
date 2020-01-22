/*
 * Copyright 2019 Google Inc.
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

#ifndef __SIM_KERNEL_WORKLOAD_HH__
#define __SIM_KERNEL_WORKLOAD_HH__

#include <string>
#include <vector>

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/types.hh"
#include "params/KernelWorkload.hh"
#include "sim/workload.hh"

class System;

class KernelWorkload : public Workload
{
  public:
    using Params = KernelWorkloadParams;

  protected:
    const Params &_params;

    Loader::MemoryImage image;

    /** Mask that should be anded for binary/symbol loading.
     * This allows one two different OS requirements for the same ISA to be
     * handled.  Some OSes are compiled for a virtual address and need to be
     * loaded into physical memory that starts at address 0, while other
     * bare metal tools generate images that start at address 0.
     */
    Addr _loadAddrMask;

    /** Offset that should be used for binary/symbol loading.
     * This further allows more flexibility than the loadAddrMask allows alone
     * in loading kernels and similar. The loadAddrOffset is applied after the
     * loadAddrMask.
     */
    Addr _loadAddrOffset;

    Addr _start, _end;

    std::vector<Loader::ObjectFile *> extras;

    Loader::ObjectFile *kernelObj = nullptr;
    // Keep a separate copy of the kernel's symbol table so we can add things
    // to it.
    Loader::SymbolTable kernelSymtab;

    const std::string commandLine;

  public:
    const Params &params() const { return _params; }

    Addr start() const { return _start; }
    Addr end() const { return _end; }
    Addr loadAddrMask() const { return _loadAddrMask; }
    Addr loadAddrOffset() const { return _loadAddrOffset; }

    KernelWorkload(const Params &p);

    Addr getEntry() const override { return kernelObj->entryPoint(); }
    Loader::Arch
    getArch() const override
    {
        return kernelObj->getArch();
    }

    const Loader::SymbolTable &
    symtab(ThreadContext *tc) override
    {
        return kernelSymtab;
    }

    bool
    insertSymbol(const Loader::Symbol &symbol) override
    {
        return kernelSymtab.insert(symbol);
    }

    void initState() override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** @{ */
    /**
     * Add a function-based event to a kernel symbol.
     *
     * These functions work like their addFuncEvent() and
     * addFuncEventOrPanic() counterparts. The only difference is that
     * they automatically use the kernel symbol table. All arguments
     * are forwarded to the underlying method.
     *
     * @see addFuncEvent()
     * @see addFuncEventOrPanic()
     *
     * @param lbl Function to hook the event to.
     * @param args Arguments to be passed to addFuncEvent
     */
    template <class T, typename... Args>
    T *
    addKernelFuncEvent(const char *lbl, Args... args)
    {
        return addFuncEvent<T>(kernelSymtab, lbl, std::forward<Args>(args)...);
    }

    template <class T, typename... Args>
    T *
    addKernelFuncEventOrPanic(const char *lbl, Args... args)
    {
        T *e = addFuncEvent<T>(kernelSymtab, lbl, std::forward<Args>(args)...);
        panic_if(!e, "Failed to find kernel symbol '%s'", lbl);
        return e;
    }
    /** @} */
};

#endif // __SIM_KERNEL_WORKLOAD_HH__
