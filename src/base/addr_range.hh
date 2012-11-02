/*
 * Copyright (c) 2012 ARM Limited
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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Andreas Hansson
 */

#ifndef __BASE_ADDR_RANGE_HH__
#define __BASE_ADDR_RANGE_HH__

#include <utility> // pair & make_pair

#include "base/types.hh"

class AddrRange
{

  public:

    Addr start;
    Addr end;

    AddrRange()
        : start(1), end(0)
    {}

    AddrRange(Addr _start, Addr _end)
        : start(_start), end(_end)
    {}

    AddrRange(const std::pair<Addr, Addr> &r)
        : start(r.first), end(r.second)
    {}

    Addr size() const { return end - start + 1; }
    bool valid() const { return start < end; }

    /**
     * Determine if another range intersects this one, i.e. if there
     * is an address that is both in this range and the other
     * range. No check is made to ensure either range is valid.
     *
     * @param r Range to intersect with
     * @return true if the intersection of the two ranges is not empty
     */
    bool intersects(const AddrRange& r) const
    {
        return (start <= r.start && end >= r.start) ||
            (start <= r.end && end >= r.end);
    }

    /**
     * Determine if this range is a subset of another range, i.e. if
     * every address in this range is also in the other range. No
     * check is made to ensure either range is valid.
     *
     * @param r Range to compare with
     * @return true if the this range is a subset of the other one
     */
    bool isSubset(const AddrRange& r) const
    {
        return start >= r.start && end <= r.end;
    }
};

/**
 * Keep the operators away from SWIG.
 */
#ifndef SWIG

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 is less than range2 and does not overlap range1.
 */
inline bool
operator<(const AddrRange& range1, const AddrRange& range2)
{
    return range1.start < range2.start;
}

/**
 * @param addr address in the range
 * @param range range compared against.
 * @return indicates that the address is not within the range.
 */
inline bool
operator!=(const Addr& addr, const AddrRange& range)
{
    return addr < range.start || addr > range.end;
}

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * @return indicates that position pos is within the range.
 */
inline bool
operator==(const AddrRange& range, const Addr& addr)
{
    return addr >= range.start && addr <= range.end;
}

inline AddrRange
RangeEx(Addr start, Addr end)
{ return std::make_pair(start, end - 1); }

inline AddrRange
RangeIn(Addr start, Addr end)
{ return std::make_pair(start, end); }

inline AddrRange
RangeSize(Addr start, Addr size)
{ return std::make_pair(start, start + size - 1); }

#endif // SWIG

#endif // __BASE_ADDR_RANGE_HH__
