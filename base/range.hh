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

#ifndef __RANGE_HH__
#define __RANGE_HH__

#include <cassert>
#include <string>

template <class T>
bool __parse_range(const std::string &s, T &start, T &end);

template <class T>
struct Range
{
  private:
    /**
     * @param s range string
     * Ranges are in the following format:
     *    <range> := {<start_val>}:{<end>}
     *    <end>   := <end_val> | +<delta>
     */
    void
    parse(const std::string &s)
    {
        if (!__parse_range(s, start, end))
            invalidate();
    }

  public:
    T start;
    T end;

  public:
    Range()
    {
        invalidate();
    }

    template <class U>
    Range(const Range<U> &r)
        : start(r.start), end(r.end)
    {}

    template <class U>
    Range(const std::pair<U, U> &r)
        : start(r.first), end(r.second)
    {}

    Range(const std::string &s)
    {
        parse(s);
    }

    template <class U>
    const Range<T> &operator=(const Range<U> &r)
    {
        start = r.start;
        end = r.end;
        return *this;
    }

    template <class U>
    const Range<T> &operator=(const std::pair<U, U> &r)
    {
        start = r.first;
        end = r.second;
        return *this;
    }

    const Range &operator=(const std::string &s)
    {
        parse(s);
        return *this;
    }

    void invalidate() { start = 0; end = 0; }
    bool size() const { return end - start; }
    bool valid() const { return start < end; }
};

template<class T>
inline std::ostream &
operator<<(std::ostream &o, const Range<T> &r)
{
    // don't currently support output of invalid ranges
    assert(r.valid());
    o << r.start << ":" << r.end;
    return o;
}

////////////////////////////////////////////////////////////////////////
//
// Range to Range Comparisons
//

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 and range2 are identical.
 */
template<class T, class U>
inline bool
operator==(const Range<T> &range1, const Range<U> &range2)
{
    assert(range1.valid() && range2.valid());
    return range1.start == range2.start && range1.end == range2.end;
}

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 and range2 are not identical.
 */
template<class T, class U>
inline bool
operator!=(const Range<T> &range1, const Range<U> &range2)
{
    assert(range1.valid() && range2.valid());
    return range1.start != range2.start || range1.end != range2.end;
}

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 is less than range2 and does not overlap range1.
 */
template<class T, class U>
inline bool
operator<(const Range<T> &range1, const Range<U> &range2)
{
    assert(range1.valid() && range2.valid());
    return range1.end <= range2.start;
}

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 is less than range2.  range1 may overlap range2,
 * but not extend beyond the end of range2.
 */
template<class T, class U>
inline bool
operator<=(const Range<T> &range1, const Range<U> &range2)
{
    assert(range1.valid() && range2.valid());
    return range1.start <= range2.start && range1.end <= range2.end;
}

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 is greater than range2 and does not overlap range2.
 */
template<class T, class U>
inline bool
operator>(const Range<T> &range1, const Range<U> &range2)
{
    assert(range1.valid() && range2.valid());
    return range1.start >= range2.end;
}

/**
 * @param range1 is a range.
 * @param range2 is a range.
 * @return if range1 is greater than range2.  range1 may overlap range2,
 * but not extend beyond the beginning of range2.
 */
template<class T, class U>
inline bool
operator>=(const Range<T> &range1, const Range<U> &range2)
{
    assert(range1.valid() && range2.valid());
    return range1.start >= range2.start && range1.end >= range2.end;
}

////////////////////////////////////////////////////////////////////////
//
// Position to Range Comparisons
//

/**
 * @param pos position compared to the range.
 * @param range range compared against.
 * @return indicates that position pos is within the range.
 */
template<class T, class U>
inline bool
operator==(const T &pos, const Range<U> &range)
{
    assert(range.valid());
    return pos >= range.start && pos < range.end;
}

/**
 * @param pos position compared to the range.
 * @param range range compared against.
 * @return indicates that position pos is not within the range.
 */
template<class T, class U>
inline bool
operator!=(const T &pos, const Range<U> &range)
{
    assert(range.valid());
    return pos < range.start || pos >= range.end;
}

/**
 * @param pos position compared to the range.
 * @param range range compared against.
 * @return indicates that position pos is below the range.
 */
template<class T, class U>
inline bool
operator<(const T &pos, const Range<U> &range)
{
    assert(range.valid());
    return pos < range.start;
}

/**
 * @param pos position compared to the range.
 * @param range range compared against.
 * @return indicates that position pos is below or in the range.
 */
template<class T, class U>
inline bool
operator<=(const T &pos, const Range<U> &range)
{
    assert(range.valid());
    return pos < range.end;
}

/**
 * @param pos position compared to the range.
 * @param range range compared against.
 * @return indicates that position pos is above the range.
 */
template<class T, class U>
inline bool
operator>(const T &pos, const Range<U> &range)
{
    assert(range.valid());
    return pos >= range.end;
}

/**
 * @param pos position compared to the range.
 * @param range range compared against.
 * @return indicates that position pos is above or in the range.
 */
template<class T, class U>
inline bool
operator>=(const T &pos, const Range<U> &range)
{
    assert(range.valid());
    return pos >= range.start;
}

////////////////////////////////////////////////////////////////////////
//
// Range to Position Comparisons (for symmetry)
//

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * @return indicates that position pos is within the range.
 */
template<class T, class U>
inline bool
operator==(const Range<T> &range, const U &pos)
{
    assert(range.valid());
    return pos >= range.start && pos < range.end;
}

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * @return indicates that position pos is not within the range.
 */
template<class T, class U>
inline bool
operator!=(const Range<T> &range, const U &pos)
{
    assert(range.valid());
    return pos < range.start || pos >= range.end;
}

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * @return indicates that position pos is above the range.
 */
template<class T, class U>
inline bool
operator<(const Range<T> &range, const U &pos)
{
    assert(range.valid());
    return range.end <= pos;
}

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * @return indicates that position pos is above or in the range.
 */
template<class T, class U>
inline bool
operator<=(const Range<T> &range, const U &pos)
{
    assert(range.valid());
    return range.start <= pos;
}

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * 'range > pos' indicates that position pos is below the range.
 */
template<class T, class U>
inline bool
operator>(const Range<T> &range, const U &pos)
{
    assert(range.valid());
    return range.start > pos;
}

/**
 * @param range range compared against.
 * @param pos position compared to the range.
 * 'range >= pos' indicates that position pos is below or in the range.
 */
template<class T, class U>
inline bool
operator>=(const Range<T> &range, const U &pos)
{
    assert(range.valid());
    return range.end > pos;
}

#endif // __RANGE_HH__
