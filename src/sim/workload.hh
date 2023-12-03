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

#ifndef __SIM_WORKLOAD_HH__
#define __SIM_WORKLOAD_HH__

#include <set>
#include <tuple>

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "enums/ByteOrder.hh"
#include "gdbremote/signals.hh"
#include "params/StubWorkload.hh"
#include "params/Workload.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

namespace gem5
{

class BaseRemoteGDB;
class System;
class ThreadContext;

class Workload : public SimObject
{
  protected:
    virtual Addr
    fixFuncEventAddr(Addr addr) const
    {
        return addr;
    }

    struct WorkloadStats : public statistics::Group
    {
        struct InstStats : public statistics::Group
        {
            statistics::Scalar arm;
            statistics::Scalar quiesce;

            InstStats(statistics::Group *parent)
                : statistics::Group(parent, "inst"),
                  ADD_STAT(arm, statistics::units::Count::get(),
                           "number of arm instructions executed"),
                  ADD_STAT(quiesce, statistics::units::Count::get(),
                           "number of quiesce instructions executed")
            {}

        } instStats;

        WorkloadStats(Workload *workload)
            : statistics::Group(workload), instStats(workload)
        {}
    } stats;

    BaseRemoteGDB *gdb = nullptr;
    bool waitForRemoteGDB = false;
    std::set<ThreadContext *> threads;

    System *system = nullptr;

  public:
    Workload(const WorkloadParams &params)
        : SimObject(params),
          stats(this),
          waitForRemoteGDB(params.wait_for_remote_gdb)
    {}

    virtual void
    setSystem(System *sys)
    {
        system = sys;
    }

    void
    recordQuiesce()
    {
        stats.instStats.quiesce++;
    }

    void
    recordArm()
    {
        stats.instStats.arm++;
    }

    // Once trapping into GDB is no longer a special case routed through the
    // system object, this helper can be removed.
    bool trapToGdb(GDBSignal sig, ContextID ctx_id);
    bool sendToGdb(std::string msg);

    virtual void registerThreadContext(ThreadContext *tc);
    virtual void replaceThreadContext(ThreadContext *tc);

    void startup() override;

    virtual Addr getEntry() const = 0;
    virtual ByteOrder byteOrder() const = 0;
    virtual loader::Arch getArch() const = 0;

    virtual const loader::SymbolTable &symtab(ThreadContext *tc) = 0;
    virtual bool insertSymbol(const loader::Symbol &symbol) = 0;

    virtual void
    syscall(ThreadContext *tc)
    {
        panic("syscall() not implemented.");
    }

    virtual void
    event(ThreadContext *tc)
    {
        warn("Unhandled workload event.");
    }

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
    addFuncEvent(const loader::SymbolTable &symtab, const char *lbl,
                 const std::string &desc, Args... args)
    {
        auto it = symtab.find(lbl);
        if (it == symtab.end())
            return nullptr;

        return new T(system, desc, fixFuncEventAddr(it->address()),
                     std::forward<Args>(args)...);
    }

    template <class T>
    T *
    addFuncEvent(const loader::SymbolTable &symtab, const char *lbl)
    {
        return addFuncEvent<T>(symtab, lbl, lbl);
    }

    template <class T, typename... Args>
    T *
    addFuncEventOrPanic(const loader::SymbolTable &symtab, const char *lbl,
                        Args... args)
    {
        T *e = addFuncEvent<T>(symtab, lbl, std::forward<Args>(args)...);
        panic_if(!e, "Failed to find symbol '%s'", lbl);
        return e;
    }

    /** @} */
};

class StubWorkload : public Workload
{
  private:
    PARAMS(StubWorkload);
    loader::SymbolTable _symtab;

  public:
    StubWorkload(const StubWorkloadParams &params) : Workload(params) {}

    Addr
    getEntry() const override
    {
        return params().entry;
    }

    ByteOrder
    byteOrder() const override
    {
        return params().byte_order;
    }

    loader::Arch
    getArch() const override
    {
        return loader::UnknownArch;
    }

    const loader::SymbolTable &
    symtab(ThreadContext *tc) override
    {
        return _symtab;
    }

    bool
    insertSymbol(const loader::Symbol &symbol) override
    {
        return _symtab.insert(symbol);
    }
};

} // namespace gem5

#endif // __SIM_WORKLOAD_HH__
