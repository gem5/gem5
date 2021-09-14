/*
 * Copyright 2020 Google Inc.
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

#ifndef __SIM_SE_WORKLOAD_HH__
#define __SIM_SE_WORKLOAD_HH__

#include "params/SEWorkload.hh"
#include "sim/mem_pool.hh"
#include "sim/workload.hh"

namespace gem5
{

class SEWorkload : public Workload
{
  protected:
    /** Memory allocation objects for all physical memories in the system. */
    MemPools memPools;

  public:
    using Params = SEWorkloadParams;

    SEWorkload(const Params &p, Addr page_shift=0);

    void setSystem(System *sys) override;

    Addr
    getEntry() const override
    {
        // This object represents the OS, not the individual processes running
        // within it.
        panic("No workload entry point for syscall emulation mode.");
    }

    loader::Arch
    getArch() const override
    {
        // ISA specific subclasses should implement this method.
        // This implemenetation is just to avoid having to implement those for
        // now, and will be removed in the future.
        panic("SEWorkload::getArch() not implemented.");
    }

    const loader::SymbolTable &
    symtab(ThreadContext *) override
    {
        // This object represents the OS, not the individual processes running
        // within it.
        panic("No workload symbol table for syscall emulation mode.");
    }

    bool
    insertSymbol(const loader::Symbol &symbol) override
    {
        // This object represents the OS, not the individual processes running
        // within it.
        panic("No workload symbol table for syscall emulation mode.");
    }

    void syscall(ThreadContext *tc) override;

    // For now, assume the only type of events are system calls.
    void event(ThreadContext *tc) override { syscall(tc); }

    Addr allocPhysPages(int npages, int pool_id=0);
    Addr memSize(int pool_id=0) const;
    Addr freeMemSize(int pool_id=0) const;
};

} // namespace gem5

#endif // __SIM_SE_WORKLOAD_HH__
