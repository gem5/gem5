/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#include "base/str.hh"

#include <string>
#include <vector>

using namespace std;

bool
split_first(const string &s, string &lhs, string &rhs, char c)
{
    string::size_type offset = s.find(c);
    if (offset == string::npos) {
        lhs = s;
        rhs = "";
        return false;
    }

    lhs = s.substr(0, offset);
    rhs = s.substr(offset + 1);
    return true;
}

bool
split_last(const string &s, string &lhs, string &rhs, char c)
{
    string::size_type offset = s.rfind(c);
    if (offset == string::npos) {
        lhs = s;
        rhs = "";
        return false;
    }

    lhs = s.substr(0, offset);
    rhs = s.substr(offset + 1);
    return true;
}

void
tokenize(vector<string>& v, const string &s, char token, bool ignore)
{
    string::size_type first = 0;
    string::size_type last = s.find_first_of(token);

    if (s.empty())
        return;

    if (ignore && last == first) {
        while (last == first)
            last = s.find_first_of(token, ++first);

        if (last == string::npos) {
            if (first != s.size())
                v.push_back(s.substr(first));
            return;
        }
    }

    while (last != string::npos) {
        v.push_back(s.substr(first, last - first));

        if (ignore) {
            first = s.find_first_not_of(token, last + 1);

            if (first == string::npos)
                return;
        } else
            first = last + 1;

        last = s.find_first_of(token, first);
    }

    v.push_back(s.substr(first));
}
