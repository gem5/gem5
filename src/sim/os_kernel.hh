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

#ifndef __SIM_OS_KERNEL_HH__
#define __SIM_OS_KERNEL_HH__

#include "base/loader/memory_image.hh"
#include "base/loader/symtab.hh"
#include "params/OsKernel.hh"
#include "sim/sim_object.hh"

class ObjectFile;
class SymbolTable;
class System;

class OsKernel : public SimObject
{
  public:
    using Params = OsKernelParams;

  protected:
    const Params &_params;

    Addr fixFuncEventAddr(Addr);

  public:
    OsKernel(const Params &p);
    ~OsKernel();

    const Params &params() { return _params; }

    void initState() override;

    const std::string commandLine;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    System *system = nullptr;

    ObjectFile *obj = nullptr;
    SymbolTable *symtab = nullptr;

    MemoryImage image;

    Addr start = 0;
    Addr end = MaxAddr;
    Addr entry = 0;

    /** Mask that should be anded for binary/symbol loading.
     * This allows one two different OS requirements for the same ISA to be
     * handled.  Some OSes are compiled for a virtual address and need to be
     * loaded into physical memory that starts at address 0, while other
     * bare metal tools generate images that start at address 0.
     */
    Addr loadAddrMask;

    /** Offset that should be used for binary/symbol loading.
     * This further allows more flexibility than the loadAddrMask allows alone
     * in loading kernels and similar. The loadAddrOffset is applied after the
     * loadAddrMask.
     */
    Addr loadAddrOffset;

    std::vector<ObjectFile *> extras;

    /** @{ */
    /**
     * Add a function-based event to the given function, to be looked
     * up in the specified symbol table.
     *
     * The ...OrPanic flavor of the method causes the simulator to
     * panic if the symbol can't be found.
     *
     * @param symtab Symbol table to use for look up.
     * @param lbl Function to hook the event to.
     * @param desc Description to be passed to the event.
     * @param args Arguments to be forwarded to the event constructor.
     */
    template <class T, typename... Args>
    T *
    addFuncEvent(const SymbolTable *symtab, const char *lbl,
                 const std::string &desc, Args... args)
    {
        Addr addr M5_VAR_USED = 0; // initialize only to avoid compiler warning

        if (symtab->findAddress(lbl, addr)) {
            return new T(system, desc, fixFuncEventAddr(addr),
                          std::forward<Args>(args)...);
        }

        return nullptr;
    }

    template <class T>
    T *
    addFuncEvent(const SymbolTable *symtab, const char *lbl)
    {
        return addFuncEvent<T>(symtab, lbl, lbl);
    }

    template <class T, typename... Args>
    T *
    addFuncEventOrPanic(const SymbolTable *symtab, const char *lbl,
                        Args... args)
    {
        T *e = addFuncEvent<T>(symtab, lbl, std::forward<Args>(args)...);
        panic_if(!e, "Failed to find symbol '%s'", lbl);
        return e;
    }
    /** @} */

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
        return addFuncEvent<T>(symtab, lbl, std::forward<Args>(args)...);
    }

    template <class T, typename... Args>
    T *
    addKernelFuncEventOrPanic(const char *lbl, Args... args)
    {
        T *e(addFuncEvent<T>(symtab, lbl, std::forward<Args>(args)...));
        if (!e)
            panic("Failed to find kernel symbol '%s'", lbl);
        return e;
    }
    /** @} */

  protected:
    /**
     * If needed, serialize additional symbol table entries for a
     * specific subclass of this system.
     *
     * @param os stream to serialize to
     */
    virtual void serializeSymtab(CheckpointOut &os) const {}

    /**
     * If needed, unserialize additional symbol table entries for a
     * specific subclass of this system.
     *
     * @param cp checkpoint to unserialize from
     * @param section relevant section in the checkpoint
     */
    virtual void unserializeSymtab(CheckpointIn &cp) {}
};

#endif // __SIM_OS_KERNEL_HH__
