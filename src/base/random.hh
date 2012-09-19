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
 *
 * Authors: Nathan Binkert
 *          Ali Saidi
 */

/*
 * Mersenne Twister random number generator has a period of
 * 2^19937-1.
 *
 * The actual math is in its own file to keep the license clear.
 */

#ifndef __BASE_RANDOM_HH__
#define __BASE_RANDOM_HH__

#include <ios>
#include <string>

#include "base/types.hh"

class Checkpoint;

class Random
{
  protected:
    static const int N = 624;
    static const int M = 397;
    static const uint32_t MATRIX_A = (uint32_t)0x9908b0df;
    static const uint32_t UPPER_MASK = (uint32_t)0x80000000;
    static const uint32_t LOWER_MASK = (uint32_t)0x7fffffff;

    uint32_t mt[N];
    int mti;

    uint32_t genrand();
    uint32_t genrand(uint32_t max);
    uint64_t genrand(uint64_t max);

    void
    _random(int8_t &value)
    {
        value = genrand() & (int8_t)-1;
    }

    void
    _random(int16_t &value)
    {
        value = genrand() & (int16_t)-1;
    }

    void
    _random(int32_t &value)
    {
        value = (int32_t)genrand();
    }

    void
    _random(int64_t &value)
    {
        value = (int64_t)genrand() << 32 | (int64_t)genrand();
    }

    void
    _random(uint8_t &value)
    {
        value = genrand() & (uint8_t)-1;
    }

    void
    _random(uint16_t &value)
    {
        value = genrand() & (uint16_t)-1;
    }

    void
    _random(uint32_t &value)
    {
        value = genrand();
    }

    void
    _random(uint64_t &value)
    {
        value = (uint64_t)genrand() << 32 | (uint64_t)genrand();
    }

    // [0,1]
    void
    _random(float &value)
    {
        // ieee floats have 23 bits of mantissa
        value = (genrand() >> 9) / 8388608.0;
    }

    // [0,1]
    void
    _random(double &value)
    {
        double number = genrand() * 2097152.0 + (genrand() >> 11);
        value = number / 9007199254740992.0;
    }


    // Range based versions of the random number generator
    int8_t
    _random(int8_t min, int8_t max)
    {
        uint32_t diff = max - min;
        return static_cast<int8_t>(min + genrand(diff));
    }

    int16_t
    _random(int16_t min, int16_t max)
    {
        uint32_t diff = max - min;
        return static_cast<int16_t>(min + genrand(diff));
    }

    int32_t
    _random(int32_t min, int32_t max)
    {
        uint32_t diff = max - min;
        return static_cast<int32_t>(min + genrand(diff));
    }

    int64_t
    _random(int64_t min, int64_t max)
    {
        uint64_t diff = max - min;
        return static_cast<int64_t>(min + genrand(diff));
    }

    uint8_t
    _random(uint8_t min, uint8_t max)
    {
        uint32_t diff = max - min;
        return static_cast<uint8_t>(min + genrand(diff));
    }

    uint16_t
    _random(uint16_t min, uint16_t max)
    {
        uint32_t diff = max - min;
        return static_cast<uint16_t>(min + genrand(diff));
    }

    uint32_t
    _random(uint32_t min, uint32_t max)
    {
        uint32_t diff = max - min;
        return static_cast<uint32_t>(min + genrand(diff));
    }

    uint64_t
    _random(uint64_t min, uint64_t max)
    {
        uint64_t diff = max - min;
        return static_cast<uint64_t>(min + genrand(diff));
    }

  public:
    Random();
    Random(uint32_t s);
    Random(uint32_t init_key[], int key_length);
    ~Random();

    void init(uint32_t s);
    void init(uint32_t init_key[], int key_length);

    template <typename T>
    T
    random()
    {
        T value;
        _random(value);
        return value;
    }

    template <typename T>
    T
    random(T min, T max)
    {
        return _random(min, max);
    }

    // [0,1]
    double
    gen_real1()
    {
        return genrand() / 4294967296.0;
    }

    // [0,1)
    double
    gen_real2()
    {
        return genrand() / 4294967295.0;
    }

    // (0,1)
    double
    gen_real3()
    {
        return ((double)genrand() + 0.5) / 4294967296.0;
    }

  public:
    void serialize(const std::string &base, std::ostream &os);
    void unserialize(const std::string &base, Checkpoint *cp,
                     const std::string &section);
};

extern Random random_mt;

#endif // __BASE_RANDOM_HH__
