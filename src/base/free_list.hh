/*
 * Copyright (c) 2024 The Board of Trustees of the Leland Stanford
 * Junior University
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

#ifndef __BASE_FREE_LIST_HH__
#define __BASE_FREE_LIST_HH__

#include <algorithm>
#include <cassert>
#include <limits>
#include <list>

#include "base/logging.hh"

namespace gem5
{

template <typename T>
class FreeList
{
  public:
    struct Range
    {
        T base;
        T size;

        Range(T base, T size)
            : base(base), size(size)
        {
        }

        bool
        contains(T x) const
        {
            return base <= x && x < base + size;
        }

        bool
        precedes(T x) const
        {
            return base + size <= x;
        }

        bool
        precedes(const Range &o) const
        {
            return precedes(o.base);
        }

        bool
        overlaps(const Range &o) const
        {
            return !precedes(o) && !o.precedes(*this);
        }
    };

    FreeList() = default;

    FreeList(T base, T size)
    {
        insert(base, size);
    }

  private:
    using RangeList = std::list<Range>;
    RangeList _ranges;
    T _size = 0;

  public:

    /** Mark the range [base, base + size) as free. */
    void
    insert(T base, T size)
    {
        _size += size;

        // Finds first range whose base is greater than
        // or equal to the insertion base.
        auto it = std::lower_bound(
            _ranges.begin(), _ranges.end(), base,
            [] (const Range& range, T base) -> bool {
                return range.base < base;
            });

        // Assert that this isn't a double free.
        panic_if(it != _ranges.end() && it->overlaps(Range(base, size)),
                 "free list: double free!\n");
        panic_if(it != _ranges.begin() &&
                 std::prev(it)->overlaps(Range(base, size)),
                 "free list: double free!\n");

        // Merge left.
        if (it != _ranges.begin()) {
            auto prev = std::prev(it);
            assert(prev->base + prev->size <= base);
            if (prev->base + prev->size == base) {
                base = prev->base;
                size += prev->size;
                _ranges.erase(prev);
            }
        }

        // Merge right.
        if (it != _ranges.end()) {
            assert(base + size <= it->base);
            if (base + size == it->base) {
                size += it->size;
                it = _ranges.erase(it);
            }
        }

        // Insert new range.
        _ranges.emplace(it, base, size);
    }

    /**
     * Allocate a region of size @param size out of free space.
     * @return whether the allocation succeeded.
     */
    std::optional<T>
    allocate(T size)
    {
        assert(size > 0);

        // Find the best-fit free range, i.e.,
        // the smallest range whose size is greater than
        // equal to the requested allocation size.
        auto best_it = _ranges.end();
        T best_size = std::numeric_limits<T>::max();
        for (auto it = _ranges.begin(); it != _ranges.end(); ++it) {
            if (size <= it->size && it->size <= best_size) {
                best_it = it;
                best_size = it->size;
            }
        }

        // Allocation failed.
        if (best_it == _ranges.end())
            return std::nullopt;

        // Allocation succeeded.
        _size -= size;
        assert(best_it != _ranges.end());
        const T base = best_it->base;
        best_it->base += size;
        best_it->size -= size;
        if (best_it->size == 0)
            _ranges.erase(best_it);
        return base;
    }

    /** Return the number of free items. */
    T
    size() const
    {
        return _size;
    }

    /** Return a list of free ranges. */
    const RangeList&
    ranges() const
    {
        return _ranges;
    }
};

}

#endif
