/*
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
 *
 * Authors: Nathan Binkert
 */

#include "base/intmath.hh"
#include "base/range.hh"
#include "base/str.hh"

using namespace std;

template <class T>
bool
__x_parse_range(const std::string &str, T &first, T &last)
{
    std::vector<std::string> values;
    tokenize(values, str, ':');

    T thefirst, thelast;

    if (values.size() != 2)
        return false;

    std::string s = values[0];
    std::string e = values[1];

    if (!to_number(s, thefirst))
        return false;

    bool increment = (e[0] == '+');
    if (increment)
        e = e.substr(1);

    if (!to_number(e, thelast))
        return false;

    if (increment)
        thelast += thefirst - 1;

    first = thefirst;
    last = thelast;

    return true;
}

#define RANGE_PARSE(type) \
template<> bool \
__parse_range(const std::string &s, type &first, type &last) \
{ return __x_parse_range(s, first, last); }

RANGE_PARSE(unsigned long long)
RANGE_PARSE(signed long long)
RANGE_PARSE(unsigned long)
RANGE_PARSE(signed long)
RANGE_PARSE(unsigned int)
RANGE_PARSE(signed int)
RANGE_PARSE(unsigned short)
RANGE_PARSE(signed short)
RANGE_PARSE(unsigned char)
RANGE_PARSE(signed char)
