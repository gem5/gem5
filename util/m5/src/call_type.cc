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

#include <cassert>
#include <sstream>

#include "args.hh"
#include "call_type.hh"

std::map<std::string, CallType &> &
CallType::map()
{
    static std::map<std::string, CallType &> all;
    return all;
}

CallType::CheckArgsResult
CallType::checkArgs(Args &args)
{
    if (args.size() && args[0] == "--" + name) {
        args.pop();
        return CheckArgsResult::Match;
    }
    return CheckArgsResult::NoMatch;
}

CallType *
CallType::detect(Args &args)
{
    CallType *def = nullptr;

    for (auto p : map()) {
        auto &ct = p.second;
        if (ct.isDefault())
            def = &ct;
        auto result = ct.checkArgs(args);
        switch (result) {
        case CheckArgsResult::Match:
            ct.init();
            return &ct;
        case CheckArgsResult::NoMatch:
            continue;
        case CheckArgsResult::Usage:
            return nullptr;
        default:
            assert(!"Bad checkArgs result");
        }
    }

    assert(def);
    def->init();
    return def;
}

std::string
CallType::usageSummary()
{
    std::string summary = "";
    for (auto p : map())
        summary += p.second.formattedUsage();
    return summary;
}

std::string
CallType::formattedUsage() const
{
    std::ostringstream os;
    os << "    ";
    printBrief(os);
    if (isDefault())
        os << " (default)";
    os << std::endl;

    os << "        ";
    printDesc(os);
    os << std::endl;
    return os.str();
}
