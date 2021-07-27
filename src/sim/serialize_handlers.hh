/*
 * Copyright (c) 2015, 2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

/* @file
 * Serialization Interface Declarations
 */

#ifndef __SERIALIZE_HANDLERS_HH__
#define __SERIALIZE_HANDLERS_HH__


#include <iostream>
#include <type_traits>
#include <utility>

#include "base/str.hh"

namespace gem5
{

/**
 * @ingroup api_serialize
 * @{
 */

// To add support for a new type of field that can be serialized, define
// template specializations of the two classes below, ParseParam and ShowParam,
// as described above each. The way ParseParam is specialized for std::string
// or ShowParam is specialied for bool can be used as examples.

/*
 * A structure which should be specialized to contain a static method with the
 * signature:
 *
 * bool parse(const std::string &s, T &value)
 *
 * which fills in value using the contents of s, and returns if that was
 * successful.
 */
template <class T, class Enable=void>
struct ParseParam;

// Specialization for anything to_number can accept.
template <class T>
struct ParseParam<T, decltype(to_number("", std::declval<T&>()), void())>
{
    static bool
    parse(const std::string &s, T &value)
    {
        return to_number(s, value);
    }
};

template <>
struct ParseParam<bool>
{
    static bool
    parse(const std::string &s, bool &value)
    {
        return to_bool(s, value);
    }
};

template <>
struct ParseParam<std::string>
{
    static bool
    parse(const std::string &s, std::string &value)
    {
        // String requires no processing to speak of
        value = s;
        return true;
    }
};

/*
 * A structure which should be specialized to contain a static method with the
 * signature:
 *
 * void show(std::ostream &os, const T &value)
 *
 * which outputs value to the stream os.
 *
 * This default implementation falls back to the << operator which should work
 * for many types.
 */
template <class T, class Enabled=void>
struct ShowParam
{
    static void show(std::ostream &os, const T &value) { os << value; }
};

// Handle characters specially so that we print their value, not the character
// they encode.
template <class T>
struct ShowParam<T, std::enable_if_t<std::is_same_v<char, T> ||
                                     std::is_same_v<unsigned char, T> ||
                                     std::is_same_v<signed char, T>>>
{
    static void
    show(std::ostream &os, const T &value)
    {
        if (std::is_signed_v<T>)
            os << (int)value;
        else
            os << (unsigned int)value;
    }
};

template <>
struct ShowParam<bool>
{
    static void
    show(std::ostream &os, const bool &value)
    {
        // Display bools as strings
        os << (value ? "true" : "false");
    }
};

/** @} */

} // namespace gem5

#endif // __SERIALIZE_HANDLERS_HH__
