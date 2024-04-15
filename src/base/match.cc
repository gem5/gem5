/*
 * Copyright (c) 2019 The Regents of the University of California
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#include "base/match.hh"

#include "base/str.hh"

namespace gem5
{

ObjectMatch::ObjectMatch() {}

ObjectMatch::ObjectMatch(const std::string &expr) { setExpression(expr); }

void
ObjectMatch::add(const ObjectMatch &other)
{
    tokens.insert(tokens.end(), other.tokens.begin(), other.tokens.end());
}

void
ObjectMatch::setExpression(const std::string &expr)
{
    tokens.resize(1);
    tokenize(tokens[0], expr, '.');
}

void
ObjectMatch::setExpression(const std::vector<std::string> &expr)
{
    if (expr.empty()) {
        tokens.resize(0);
    } else {
        tokens.resize(expr.size());
        for (std::vector<std::string>::size_type i = 0; i < expr.size(); ++i)
            tokenize(tokens[i], expr[i], '.');
    }
}

/**
 * @todo this should probably be changed to just use regular
 * expression code
 */
bool
ObjectMatch::domatch(const std::string &name) const
{
    std::vector<std::string> name_tokens;
    tokenize(name_tokens, name, '.');
    int ntsize = name_tokens.size();

    int num_expr = tokens.size();
    for (int i = 0; i < num_expr; ++i) {
        const std::vector<std::string> &token = tokens[i];
        int jstop = token.size();

        bool match = true;
        for (int j = 0; j < jstop; ++j) {
            if (j >= ntsize)
                break;

            const std::string &var = token[j];
            if (var != "*" && var != name_tokens[j]) {
                match = false;
                break;
            }
        }

        if (match)
            return true;
    }

    return false;
}

std::vector<std::vector<std::string> >
ObjectMatch::getExpressions()
{
    std::vector<std::vector<std::string> > to_return;
    for (const auto &expression : tokens) {
        std::vector<std::string> to_add;
        to_add.insert(to_add.end(), expression.begin(), expression.end());
        to_return.push_back(to_add);
    }

    return to_return;
}

} // namespace gem5
