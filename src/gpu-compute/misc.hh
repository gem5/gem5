/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Reinhardt
 */

#ifndef __MISC_HH__
#define __MISC_HH__

#include <bitset>
#include <limits>
#include <memory>

#include "base/logging.hh"

class GPUDynInst;

typedef std::bitset<std::numeric_limits<unsigned long long>::digits> VectorMask;
typedef std::shared_ptr<GPUDynInst> GPUDynInstPtr;

class WaitClass
{
  public:
    WaitClass() : nxtAvail(0), lookAheadAvail(0), tcnt(0) { }
    void init(uint64_t *_tcnt, uint32_t _numStages=0)
    {
        tcnt = _tcnt;
        numStages = _numStages;
    }

    void set(uint32_t i)
    {
        fatal_if(nxtAvail > *tcnt,
                 "Can't allocate resource because it is busy!!!");
        nxtAvail = *tcnt + i;
    }
    void preset(uint32_t delay)
    {
        lookAheadAvail = std::max(lookAheadAvail, delay + (*tcnt) - numStages);
    }
    bool rdy() const { return *tcnt >= nxtAvail; }
    bool prerdy() const { return *tcnt >= lookAheadAvail; }

  private:
    // timestamp indicating when resource will be available
    uint64_t nxtAvail;
    // timestamp indicating when resource will be available including
    // pending uses of the resource (when there is a cycle gap between
    // rdy() and set()
    uint64_t lookAheadAvail;
    // current timestamp
    uint64_t *tcnt;
    // number of stages between checking if a resource is ready and
    // setting the resource's utilization
    uint32_t numStages;
};

class Float16
{
  public:
    uint16_t val;

    Float16() { val = 0; }

    Float16(const Float16 &x) : val(x.val) { }

    Float16(float x)
    {
        uint32_t ai = *(uint32_t *)&x;

        uint32_t s = (ai >> 31) & 0x1;
        uint32_t exp = (ai >> 23) & 0xff;
        uint32_t mant = (ai >> 0) & 0x7fffff;

        if (exp == 0 || exp <= 0x70) {
            exp = 0;
            mant = 0;
        } else if (exp == 0xff) {
            exp = 0x1f;
        } else if (exp >= 0x8f) {
            exp = 0x1f;
            mant = 0;
        } else {
            exp = exp - 0x7f + 0x0f;
        }

        mant = mant >> 13;

        val = 0;
        val |= (s << 15);
        val |= (exp << 10);
        val |= (mant << 0);
    }

    operator float() const
    {
        uint32_t s = (val >> 15) & 0x1;
        uint32_t exp = (val >> 10) & 0x1f;
        uint32_t mant = (val >> 0) & 0x3ff;

        if (!exp) {
            exp = 0;
            mant = 0;
        } else if (exp == 0x1f) {
            exp = 0xff;
        } else {
            exp = exp - 0x0f + 0x7f;
        }

        uint32_t val1 = 0;
        val1 |= (s << 31);
        val1 |= (exp << 23);
        val1 |= (mant << 13);

        return *(float*)&val1;
    }
};

#endif // __MISC_HH__
