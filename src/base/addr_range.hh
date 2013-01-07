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

#include "base/cprintf.hh"
#include "base/types.hh"

class AddrRange
{

  private:

    /// Private fields for the start and end of the range. In the
    /// future, these will be extended with interleaving functionality
    /// and hence should never be manipulated directly.
    Addr _start;
    Addr _end;

  public:

    AddrRange()
        : _start(1), _end(0)
    {}

    AddrRange(Addr _start, Addr _end)
        : _start(_start), _end(_end)
    {}

    /**
     * Get the size of the address range. For a case where
     * interleaving is used this should probably cause a panic.
     */
    Addr size() const { return _end - _start + 1; }

    /**
     * Determine if the range is valid.
     */
    bool valid() const { return _start < _end; }

    /**
     * Get the start address of the range.
     */
    Addr start() const { return _start; }

    /**
     * Get a string representation of the range. This could
     * alternatively be implemented as a operator<<, but at the moment
     * that seems like overkill.
     */
    std::string to_string() const
    {
        return csprintf("[%#llx : %#llx]", _start, _end);
    }

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
        return _start <= r._end && _end >= r._start;
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
        return _start >= r._start && _end <= r._end;
    }

    /**
     * Determine if the range contains an address.
     *
     * @param a Address to compare with
     * @return true if the address is in the range
     */
    bool contains(const Addr& a) const
    {
        return a >= _start && a <= _end;
    }

/**
 * Keep the operators away from SWIG.
 */
#ifndef SWIG

    /**
     * Less-than operator used to turn an STL map into a binary search
     * tree of non-overlapping address ranges.
     *
     * @param r Range to compare with
     * @return true if the start address is less than that of the other range
     */
    bool operator<(const AddrRange& r) const
    {
        return _start < r._start;
    }

#endif // SWIG
};

inline AddrRange
RangeEx(Addr start, Addr end)
{ return AddrRange(start, end - 1); }

inline AddrRange
RangeIn(Addr start, Addr end)
{ return AddrRange(start, end); }

inline AddrRange
RangeSize(Addr start, Addr size)
{ return AddrRange(start, start + size - 1); }

#endif // __BASE_ADDR_RANGE_HH__
