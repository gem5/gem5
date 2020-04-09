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

#include <cstring>

#include "args.hh"
#include "call_type.hh"
#include "dispatch_table.hh"
#include "m5_mmap.h"
#include "usage.hh"

extern "C"
{
#define M5OP(name, func) __typeof__(name) M5OP_MERGE_TOKENS(name, _addr);
M5OP_FOREACH
#undef M5OP
}

namespace
{

DispatchTable addr_dispatch = {
#define M5OP(name, func) .name = &::M5OP_MERGE_TOKENS(name, _addr),
M5OP_FOREACH
#undef M5OP
};

#if defined(M5OP_ADDR)
const bool DefaultAddrDefined = true;
constexpr uint64_t DefaultAddress = M5OP_ADDR;
#else
const bool DefaultAddrDefined = false;
constexpr uint64_t DefaultAddress = 0;
#endif

class AddrCallType : public CallType
{
  public:
    AddrCallType() : CallType("addr") {}

    bool isDefault() const override { return CALL_TYPE_IS_DEFAULT; }
    const DispatchTable &getDispatch() const override { return addr_dispatch; }

    void
    printBrief(std::ostream &os) const override
    {
        os << "--" << name << (DefaultAddrDefined ? " [address override]" :
                                                    " <address override>");
    }

    void
    printDesc(std::ostream &os) const override
    {
        os << "Use the address based invocation method.";
        if (DefaultAddrDefined) {
            os << " The default address is 0x" <<
                std::hex << DefaultAddress << std::dec << ".";
        }
    }

    CheckArgsResult
    checkArgs(Args &args) override
    {
        const std::string prefix = "--" + name;
        uint64_t addr_override;

        // If the first argument doesn't start with --addr...
        if (!args.size() || args[0].substr(0, prefix.size()) != prefix)
            return CheckArgsResult::NoMatch;

        const std::string &arg = args.pop().substr(prefix.size());

        // If there's more text in this argument...
        if (arg.size()) {
            // If it doesn't start with '=', it's malformed.
            if (arg[0] != '=')
                return CheckArgsResult::Usage;
            // Attempt to extract an address after the '='.
            if (!args.stoi(arg.substr(1), addr_override))
                return CheckArgsResult::Usage;
            // If we found an address, use it to override m5op_addr.
            m5op_addr = addr_override;
            return CheckArgsResult::Match;
        }
        // If an address override wasn't part of the first argument, check if
        // it's the second argument. If not, then there's no override.
        if (args.pop(addr_override)) {
            m5op_addr = addr_override;
            return CheckArgsResult::Match;
        }
        // If the default address was not defined, an override is required.
        if (!DefaultAddrDefined)
            return CheckArgsResult::Usage;

        return CheckArgsResult::Match;
    }

    void init() override { map_m5_mem(); }
} addr_call_type;

} // anonymous namespace
