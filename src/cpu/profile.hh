/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#ifndef __CPU_PROFILE_HH__
#define __CPU_PROFILE_HH__

#include <map>
#include <string>

#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "debug/Stack.hh"

namespace gem5
{

class ThreadContext;
class FunctionProfile;

namespace loader
{
    class SymbolTable;
} // namespace loader

class BaseStackTrace
{
  private:
    void dump();

  protected:
    ThreadContext *tc = nullptr;
    std::vector<Addr> stack;

    // Subclasses should implement this function so that it collects the
    // the current and return addresses on the stack in the "stack" vector.
    virtual void trace(ThreadContext *tc, bool is_call) = 0;

  public:
    BaseStackTrace() : stack(64) {}
    virtual ~BaseStackTrace() {}

    void
    clear()
    {
        tc = nullptr;
        stack.clear();
    }

    bool valid() const { return tc; }

    bool
    trace(ThreadContext *tc, const StaticInstPtr &inst)
    {
        if (!inst->isCall() && !inst->isReturn())
            return false;

        if (valid())
            clear();

        trace(tc, !inst->isReturn());
        return true;
    }

    const std::vector<Addr> &getstack() const { return stack; }

    void dprintf() { if (debug::Stack) dump(); }

    // This function can be overridden so that special addresses which don't
    // actually refer to PCs can be translated into special names. For
    // instance, the address 1 could translate into "user" for user level
    // code when the symbol table only has kernel symbols.
    //
    // It should return whether addr was recognized and symbol has been set to
    // something.
    virtual bool tryGetSymbol(std::string &symbol, Addr addr,
                              const loader::SymbolTable *symtab);

    void
    getSymbol(std::string &symbol, Addr addr,
              const loader::SymbolTable *symtab)
    {
        panic_if(!tryGetSymbol(symbol, addr, symtab),
                 "Could not find symbol for address %#x\n", addr);
    }
};

class ProfileNode
{
  private:
    friend class FunctionProfile;

    typedef std::map<Addr, ProfileNode *> ChildList;
    ChildList children;

  public:
    Counter count = 0;

  public:
    void dump(const std::string &symbol, uint64_t id,
              const FunctionProfile &prof, std::ostream &os) const;
    void clear();
};

class FunctionProfile
{
  private:
    friend class ProfileNode;

    const loader::SymbolTable &symtab;
    ProfileNode top;
    std::map<Addr, Counter> pc_count;
    std::unique_ptr<BaseStackTrace> trace;

  public:
    FunctionProfile(std::unique_ptr<BaseStackTrace> _trace,
                    const loader::SymbolTable &symtab);

    ProfileNode *consume(ThreadContext *tc, const StaticInstPtr &inst);
    ProfileNode *consume(const std::vector<Addr> &stack);
    void clear();
    void dump(std::ostream &out) const;
    void sample(ProfileNode *node, Addr pc);
};

inline ProfileNode *
FunctionProfile::consume(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (!trace->trace(tc, inst))
        return nullptr;
    trace->dprintf();
    return consume(trace->getstack());
}

} // namespace gem5

#endif // __CPU_PROFILE_HH__
