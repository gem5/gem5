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
 *
 * Authors: Nathan Binkert
 */

#include <cctype>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "base/intmath.hh"
#include "base/str.hh"

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

/**
 * @todo This function will not handle the smallest negative decimal
 * value for a signed type
 */

template <class T>
inline bool
__to_number(string value, T &retval)
{
    static const T maxnum = ((T)-1);
    static const bool sign = numeric_limits<T>::is_signed;
    static const int bits = numeric_limits<T>::digits;
    static const T hexmax = maxnum & (((T)1 << (bits - 4)) - 1);
    static const T octmax = maxnum & (((T)1 << (bits - 3)) - 1);
    static const T signmax = numeric_limits<T>::max();
    static const T decmax = signmax / 10;

#if 0
    cout << "maxnum =  0x" << hex << (unsigned long long)maxnum << "\n"
         << "sign =    0x" << hex << (unsigned long long)sign << "\n"
         << "hexmax =  0x" << hex << (unsigned long long)hexmax << "\n"
         << "octmax =  0x" << hex << (unsigned long long)octmax << "\n"
         << "signmax = 0x" << hex << (unsigned long long)signmax << "\n"
         << "decmax =  0x" << hex << (unsigned long long)decmax << "\n";
#endif

    eat_white(value);

    bool negative = false;
    bool hex = false;
    bool oct = false;
    int last = value.size() - 1;
    retval = 0;
    int i = 0;

    char c = value[i];
    if (!isDec(c)) {
        if (c == '-' && sign)
            negative = true;
        else
            return false;
    }
    else {
        retval += c - '0';
        if (last == 0) return true;
    }

    if (c == '0')
        oct = true;

    c = value[++i];
    if (oct) {
        if (sign && negative)
            return false;

        if (!isOct(c)) {
            if (c == 'X' || c == 'x') {
                hex = true;
                oct = false;
            } else
                return false;
        }
        else
            retval += c - '0';
    } else if (!isDec(c))
        goto multiply;
    else {
        if (sign && negative && c == '0')
            return false;

        retval *= 10;
        retval += c - '0';
        if (last == 1) {
            if (sign && negative) retval = -retval;
            return true;
        }
    }

    if (hex) {
        if (last == 1)
            return false;

        for (i = 2; i <= last ; i++) {
            c = value[i];
            if (!isHex(c))
                return false;

            if (retval > hexmax) return false;
            retval *= 16;
            retval += hex2Int(c);
        }
        return true;
    } else if (oct) {
        for (i = 2; i <= last ; i++) {
            c = value[i];
            if (!isOct(c))
                return false;

            if (retval > octmax) return false;
            retval *= 8;
            retval += (c - '0');
        }
        return true;
    }

    for (i = 2; i < last ; i++) {
        c = value[i];
        if (!isDec(c))
            goto multiply;

        if (retval > decmax) return false;
        bool atmax = retval == decmax;
        retval *= 10;
        retval += c - '0';
        if (atmax && retval < decmax) return false;
        if (sign && (retval & ((T)1 << (sizeof(T) * 8 - 1))))
            return false;
    }

    c = value[last];
    if (isDec(c)) {

        if (retval > decmax) return false;
        bool atmax = retval == decmax;
        retval *= 10;
        retval += c - '0';
        if (atmax && retval < decmax) return false;
        if (sign && negative) {
            if ((retval & ((T)1 << (sizeof(T) * 8 - 1))) &&
                retval >= (T)-signmax)
                return false;
            retval = -retval;
        }
        else
            if (sign && (retval & ((T)1 << ((sizeof(T) * 8) - 1))))
                return false;
        return true;
    }

  multiply:
    signed long long mult = 1;
    T val;
    switch (c) {
      case 'k':
      case 'K':
        if (i != last) return false;
        mult = 1024;
        val = signmax / mult;
        break;
      case 'm':
      case 'M':
        if (i != last) return false;
        mult = 1024 * 1024;
        val = signmax / mult;
        break;
      case 'g':
      case 'G':
        if (i != last) return false;
        mult = 1024 * 1024 * 1024;
        val = signmax / mult;
        break;
      case 'e':
      case 'E':
        if (i >= last) return false;

        mult = 0;
        for (i++; i <= last; i++) {
            c = value[i];
            if (!isDec(c))
                return false;

            mult *= 10;
            mult += c - '0';
        }

        for (i = 0; i < mult; i++) {
            if (retval > signmax / 10)
                return false;
            retval *= 10;
            if (sign && (retval & ((T)1 << (sizeof(T) * 8 - 1))))
                return false;
        }
        if (sign && negative) {
            if ((retval & ((T)1 << (sizeof(T) * 8 - 1))) &&
                retval >= (T)-signmax)
                return false;
            retval = -retval;
        }
        else
            if (sign && (retval & ((T)1 << ((sizeof(T) * 8) - 1))))
                return false;

        return true;

      default:
        return false;
    }

    if (sign && negative)
        return false;

    if (mult > (unsigned long long)signmax)
        return false;

    if (retval > val)
        return false;

    retval *= mult;

    return true;
}

#define STN(type) \
template<> \
bool to_number<type>(const string &value, type &retval) \
{ return __to_number(value, retval); }

STN(unsigned long long)
STN(signed long long)
STN(unsigned long)
STN(signed long)
STN(unsigned int)
STN(signed int)
STN(unsigned short)
STN(signed short)
STN(unsigned char)
STN(signed char)
STN(char)

template<>
bool to_number<bool>(const string &value, bool &retval)
{
    string lowered = to_lower(value);

    if (value == "0") {
        retval = false;
        return true;
    }

    if (value == "1"){
        retval = true;
        return true;
    }

    if (lowered == "false") {
        retval = false;
        return true;
    }

    if (lowered == "true"){
        retval = true;
        return true;
    }

    if (lowered == "no") {
        retval = false;
        return true;
    }

    if (lowered == "yes"){
        retval = true;
        return true;
    }

    return false;
}
