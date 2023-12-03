/*
 * Copyright (c) 2023 The Regents of the University of California.
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

#ifndef __PC_COUNT_PAIR_HH__
#define __PC_COUNT_PAIR_HH__

#include "base/types.hh"

namespace gem5
{

class PcCountPair
{
  private:
    /** The Program Counter address */
    Addr pc;
    /** The count of the Program Counter address */
    int count;

  public:
    /** Explicit constructor assigning the pc and count values */
    explicit constexpr PcCountPair(Addr _pc, int _count)
        : pc(_pc), count(_count)
    {}

    /** Default constructor for parameter classes */
    PcCountPair() : pc(0), count(0) {}

    /** Returns the Program Counter address */
    constexpr Addr
    getPC() const
    {
        return pc;
    }

    /** Returns the count of the Program */
    constexpr int
    getCount() const
    {
        return count;
    }

    /** Greater than comparison */
    constexpr bool
    operator>(const PcCountPair &cc) const
    {
        return count > cc.getCount();
    }

    /** Equal comparison */
    constexpr bool
    operator==(const PcCountPair &cc) const
    {
        return (pc == cc.getPC() && count == cc.getCount());
    }

    /** String format */
    std::string
    to_string() const
    {
        std::string s =
            "(" + std::to_string(pc) + "," + std::to_string(count) + ")";
        return s;
    }

    /** Enable hashing for this parameter */
    struct HashFunction
    {
        size_t
        operator()(const PcCountPair &item) const
        {
            size_t xHash = std::hash<int>()(item.pc);
            size_t yHash = std::hash<int>()(item.count);
            return xHash * 2 + yHash;
        }
    };
};

} // namespace gem5

#endif // __PC_COUNT_PAIR_HH__
