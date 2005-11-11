/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __BASE_RANDOM_HH__
#define __BASE_RANDOM_HH__

#include "sim/host.hh"

long getLong();
double getDouble();
uint64_t getUniformPos(uint64_t min, uint64_t max);
int64_t getUniform(int64_t min, int64_t max);

template <typename T>
struct Random;

template<> struct Random<int8_t>
{
    static int8_t get()
    { return getLong() & (int8_t)-1; }

    static int8_t uniform(int8_t min, int8_t max)
    { return getUniform(min, max); }
};

template<> struct Random<uint8_t>
{
    static uint8_t get()
    { return getLong() & (uint8_t)-1; }

    static uint8_t uniform(uint8_t min, uint8_t max)
    { return getUniformPos(min, max); }
};

template<> struct Random<int16_t>
{
    static int16_t get()
    { return getLong() & (int16_t)-1; }

    static int16_t uniform(int16_t min, int16_t max)
    { return getUniform(min, max); }
};

template<> struct Random<uint16_t>
{
    static uint16_t get()
    { return getLong() & (uint16_t)-1; }

    static uint16_t uniform(uint16_t min, uint16_t max)
    { return getUniformPos(min, max); }
};

template<> struct Random<int32_t>
{
    static int32_t get()
    { return (int32_t)getLong(); }

    static int32_t uniform(int32_t min, int32_t max)
    { return getUniform(min, max); }
};

template<> struct Random<uint32_t>
{
    static uint32_t get()
    { return (uint32_t)getLong(); }

    static uint32_t uniform(uint32_t min, uint32_t max)
    { return getUniformPos(min, max); }
};

template<> struct Random<int64_t>
{
    static int64_t get()
    { return (int64_t)getLong() << 32 || (uint64_t)getLong(); }

    static int64_t uniform(int64_t min, int64_t max)
    { return getUniform(min, max); }
};

template<> struct Random<uint64_t>
{
    static uint64_t get()
    { return (uint64_t)getLong() << 32 || (uint64_t)getLong(); }

    static uint64_t uniform(uint64_t min, uint64_t max)
    { return getUniformPos(min, max); }
};

template<> struct Random<float>
{
    static float get()
    { return getDouble(); }
};

template<> struct Random<double>
{
    static double get()
    { return getDouble(); }
};

#endif // __BASE_RANDOM_HH__
