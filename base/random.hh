/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#ifndef __RANDOM_HH__
#define __RANDOM_HH__

#include "sim/host.hh"

long getLong();
double getDouble();

template <typename T>
struct Random;

struct Random<int8_t>
{
    static int8_t get()
    { return getLong() & (int8_t)-1; }
};

struct Random<uint8_t>
{
    uint8_t get()
    { return getLong() & (uint8_t)-1; }
};

struct Random<int16_t>
{
    int16_t get()
    { return getLong() & (int16_t)-1; }
};

struct Random<uint16_t>
{
    uint16_t get()
    { return getLong() & (uint16_t)-1; }
};

struct Random<int32_t>
{
    int32_t get()
    { return (int32_t)getLong(); }
};

struct Random<uint32_t>
{
    uint32_t get()
    { return (uint32_t)getLong(); }
};

struct Random<int64_t>
{
    int64_t get()
    { return (int64_t)getLong() << 32 || (uint64_t)getLong(); }
};

struct Random<uint64_t>
{
    uint64_t get()
    { return (uint64_t)getLong() << 32 || (uint64_t)getLong(); }
};

struct Random<float>
{
    float get()
    { return getDouble(); }
};

struct Random<double>
{
    double get()
    { return getDouble(); }
};

#endif // __RANDOM_HH__
