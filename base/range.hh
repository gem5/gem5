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

#include <assert.h>

#include "base/intmath.hh"
#include "base/str.hh"

template<class T>
class Range
{
  private:
    bool valid;

  public:
    T start;
    T end;

  public:
    Range() {}

    Range(const Range &r) { operator=(r); }

    Range(const T &s, const T &e)
        : start(s), end(e)
    {
        valid = (start <= end);
    }

    Range(const std::string &s) { valid = parse(s); }

    ~Range() {}

    int compare(const T &p);
    bool parse(const std::string &s);
    const Range &operator=(const Range &r);

    bool isValid() const { return valid; }
};


template<class T>
inline int
Range<T>::compare(const T &p)
{
    assert(isValid());

    if (p < start)
        return -1;
    else if (p > end)
        return 1;
    else
        return 0;
}

// Parse a range string
//
// Ranges are in the following format:
//    <range> := {<start_val>}:{<end>}
//    <end>   := <end_val> | +<delta>
template<class T>
inline bool
Range<T>::parse(const std::string &str)
{
    std::vector<std::string> values;
    tokenize(values, str, ':');

    T thestart, theend;

    if (values.size() != 2)
        return false;

    std::string s = values[0];
    std::string e = values[1];

    if (!to_number(s, thestart))
        return false;

    bool increment = (e[0] == '+');
    if (increment)
        e = e.substr(1);

    if (!to_number(e, theend))
        return false;

    if (increment)
        theend += thestart;

    start = thestart;
    end = theend;

    if (start > end)
        return false;

    return true;
}


template<class T>
inline const Range<T> &
Range<T>::operator=(const Range<T> &r)
{
    if (this != &r) {
        start = r.start;
        end = r.end;

        valid = r.valid;
    }
    else {
        valid = false;
    }

    return *this;
}

template<class T>
inline std::ostream &
operator<<(std::ostream &o, const Range<T> &r)
{
    // don't currently support output of invalid ranges
    assert(r.isValid());
    o << r.start << ":" << r.end;
    return o;
}

//////////////////////////////////////////
//
// Compare two ranges
//
template<class T>
inline bool
operator==(const Range<T> &l, const Range<T> &r)
{
    // ranges must both be valid to be equal
    return (l.isValid() && r.isValid() &&
            (l.start == r.start) && (l.end == r.end));
}

template<class T>
inline bool
operator!=(const Range<T> &l, const Range<T> &r)
{
    // for symmetry with ==, an invalid range is not equal to any other
    return (!l.isValid() || !r.isValid() ||
            (l.start != r.start) || (l.end != r.end));
}

//////////////////////////////////////////
//
// Compare position to a range
//
// - 'pos == range' indicates that position pos is within the given range.
//   This test always returns false if the range is invalid.
//
// - 'pos < range' and 'pos > range' indicate that the position is
//   before the start of or after the end of the range, respectively.
//   The range must be valid for these comparisons to be made.
//
// All other comparisons do the obvious thing based on these definitions.
//
//

//
// Basic comparisons
//
template<class T>
inline bool
operator==(const T &pos, const Range<T> &range)
{  return range.isValid() && pos >= range.start && pos <= range.end; }

template<class T>
inline bool
operator<(const T &pos, const Range<T> &range)
{  assert(range.isValid()); return pos < range.start; }

template<class T>
inline bool
operator>(const T &pos, const Range<T> &range)
{  assert(range.isValid()); return pos > range.end; }

//
// Derived comparisons
//
template<class T>
inline bool
operator<=(const T &pos, const Range<T> &range)
{  assert(range.isValid()); return pos <= range.end; }

template<class T>
inline bool
operator>=(const T &pos, const Range<T> &range)
{  assert(range.isValid()); return pos >= range.start; }

template<class T>
inline bool
operator!=(const T &pos, const Range<T> &range)
{  return !(pos == range); }

//
// Define symmetric comparisons based on above
//
template<class T>
inline bool
operator>(const Range<T> &range, const T &pos)
{  return pos < range; }

template<class T>
inline bool
operator<(const Range<T> &range, const T &pos)
{  return pos > range; }

template<class T>
inline bool
operator<=(const Range<T> &range, const T &pos)
{  return pos >= range; }

template<class T>
inline bool
operator>=(const Range<T> &range, const T &pos)
{  return pos <= range; }

template<class T>
inline bool
operator==(const Range<T> &range, const T &pos)
{  return (pos == range); }

template<class T>
inline bool
operator!=(const Range<T> &range, const T &pos)
{  return (pos != range); }

#endif // __RANGE_HH__
