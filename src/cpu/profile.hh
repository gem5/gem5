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
 *
 * Authors: Nathan Binkert
 */

#ifndef __CPU_PROFILE_HH__
#define __CPU_PROFILE_HH__

#include <map>

#include "arch/stacktrace.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/static_inst.hh"

class ThreadContext;

class ProfileNode
{
  private:
    friend class FunctionProfile;

    typedef std::map<Addr, ProfileNode *> ChildList;
    ChildList children;

  public:
    Counter count;

  public:
    ProfileNode();

    void dump(const std::string &symbol, uint64_t id,
              const SymbolTable *symtab, std::ostream &os) const;
    void clear();
};

class Callback;
class FunctionProfile
{
  private:
    Callback *reset;
    const SymbolTable *symtab;
    ProfileNode top;
    std::map<Addr, Counter> pc_count;
    TheISA::StackTrace trace;

  public:
    FunctionProfile(const SymbolTable *symtab);
    ~FunctionProfile();

    ProfileNode *consume(ThreadContext *tc, const StaticInstPtr &inst);
    ProfileNode *consume(const std::vector<Addr> &stack);
    void clear();
    void dump(ThreadContext *tc, std::ostream &out) const;
    void sample(ProfileNode *node, Addr pc);
};

inline ProfileNode *
FunctionProfile::consume(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (!trace.trace(tc, inst))
        return NULL;
    trace.dprintf();
    return consume(trace.getstack());
}

#endif // __CPU_PROFILE_HH__
