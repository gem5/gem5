/*
 * Copyright (c) 2012, 2014, 2017-2019, 2021 Arm Limited
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
 */

#ifndef __BASE_ADDR_RANGE_HH__
#define __BASE_ADDR_RANGE_HH__

#include <algorithm>
#include <iterator>
#include <list>
#include <vector>

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/types.hh"

namespace gem5
{

class AddrRange;

/**
 * Convenience typedef for a collection of address ranges
 *
 * @ingroup api_addr_range
 */
typedef std::list<AddrRange> AddrRangeList;

/**
 * The AddrRange class encapsulates an address range, and supports a
 * number of tests to check if two ranges intersect, if a range
 * contains a specific address etc. Besides a basic range, the
 * AddrRange also support interleaved ranges, to stripe across cache
 * banks, or memory controllers. The interleaving is implemented by
 * allowing a number of bits of the address, at an arbitrary bit
 * position, to be used as interleaving bits with an associated
 * matching value. In addition, to prevent uniformly strided address
 * patterns from a very biased interleaving, we also allow XOR-based
 * hashing by specifying a set of bits to XOR with before matching.
 *
 * The AddrRange is also able to coalesce a number of interleaved
 * ranges to a contiguous range.
 */
class AddrRange
{

  private:

    /// Private fields for the start and end of the range
    /// _start is the beginning of the range (inclusive).
    /// _end is not part of the range.
    Addr _start;
    Addr _end;

    /**
     * Each mask determines the bits we need to xor to get one bit of
     * sel. The first (0) mask is used to get the LSB and the last for
     * the MSB of sel.
     */
    std::vector<Addr> masks;

    /** The value to compare sel with. */
    uint8_t intlvMatch;

  protected:
    struct Dummy {};

    // The dummy parameter Dummy distinguishes this from the other two argument
    // constructor which takes two Addrs.
    template <class Iterator>
    AddrRange(Dummy, Iterator begin_it, Iterator end_it)
        : _start(1), _end(0), intlvMatch(0)
    {
        if (begin_it != end_it) {
            // get the values from the first one and check the others
            _start = begin_it->_start;
            _end = begin_it->_end;
            masks = begin_it->masks;
            intlvMatch = begin_it->intlvMatch;
        }

        auto count = std::distance(begin_it, end_it);
        // either merge if got all ranges or keep this equal to the single
        // interleaved range
        if (count > 1) {
            fatal_if(count != (1ULL << masks.size()),
                    "Got %d ranges spanning %d interleaving bits.",
                    count, masks.size());

            uint8_t match = 0;
            for (auto it = begin_it; it != end_it; it++) {
                fatal_if(!mergesWith(*it),
                        "Can only merge ranges with the same start, end "
                        "and interleaving bits, %s %s.", to_string(),
                        it->to_string());

                fatal_if(it->intlvMatch != match,
                        "Expected interleave match %d but got %d when "
                        "merging.", match, it->intlvMatch);
                ++match;
            }
            masks.clear();
            intlvMatch = 0;
        }
    }

  public:

    /**
     * @ingroup api_addr_range
     */
    AddrRange()
        : _start(1), _end(0), intlvMatch(0)
    {}

    /**
     * Construct an address range
     *
     * If the user provides a non empty vector of masks then the
     * address range is interleaved. Each mask determines a set of
     * bits that are xored to determine one bit of the sel value,
     * starting from the least significant bit (i.e., masks[0]
     * determines the least significant bit of sel, ...). If sel
     * matches the provided _intlv_match then the address a is in the
     * range.
     *
     * For example if the input mask is
     * _masks = { 1 << 8 | 1 << 11 | 1 << 13,
     *            1 << 15 | 1 << 17 | 1 << 19}
     *
     * Then a belongs to the address range if
     * _start <= a < _end
     * and
     * sel == _intlv_match
     * where
     * sel[0] = a[8] ^ a[11] ^ a[13]
     * sel[1] = a[15] ^ a[17] ^ a[19]
     *
     * @param _start The start address of this range
     * @param _end The end address of this range (not included in  the range)
     * @param _masks The input vector of masks
     * @param intlv_match The matching value of the xor operations
     *
     * @ingroup api_addr_range
     */
    AddrRange(Addr _start, Addr _end, const std::vector<Addr> &_masks,
              uint8_t _intlv_match)
        : _start(_start), _end(_end), masks(_masks),
          intlvMatch(_intlv_match)
    {
        // sanity checks
        fatal_if(!masks.empty() && _intlv_match >= 1ULL << masks.size(),
                 "Match value %d does not fit in %d interleaving bits\n",
                 _intlv_match, masks.size());
    }

    /**
     * Legacy constructor of AddrRange
     *
     * If the user provides a non-zero value in _intlv_high_bit the
     * address range is interleaved.
     *
     * An address a belongs to the address range if
     * _start <= a < _end
     * and
     * sel == _intlv_match
     * where
     * sel = sel1 ^ sel2
     * sel1 = a[_intlv_low_bit:_intlv_high_bit]
     * sel2 = a[_xor_low_bit:_xor_high_bit]
     * _intlv_low_bit = _intlv_high_bit - intv_bits
     * _xor_low_bit = _xor_high_bit - intv_bits
     *
     * @param _start The start address of this range
     * @param _end The end address of this range (not included in  the range)
     * @param _intlv_high_bit The MSB of the intlv bits (disabled if 0)
     * @param _xor_high_bit The MSB of the xor bit (disabled if 0)
     * @param _intlv_bits the size, in bits, of the intlv and xor bits
     * @param intlv_match The matching value of the xor operations
     *
     * @ingroup api_addr_range
     */
    AddrRange(Addr _start, Addr _end, uint8_t _intlv_high_bit,
              uint8_t _xor_high_bit, uint8_t _intlv_bits,
              uint8_t _intlv_match)
        : _start(_start), _end(_end), masks(_intlv_bits),
          intlvMatch(_intlv_match)
    {
        // sanity checks
        fatal_if(_intlv_bits && _intlv_match >= 1ULL << _intlv_bits,
                 "Match value %d does not fit in %d interleaving bits\n",
                 _intlv_match, _intlv_bits);

        // ignore the XOR bits if not interleaving
        if (_intlv_bits && _xor_high_bit) {
            if (_xor_high_bit == _intlv_high_bit) {
                fatal("XOR and interleave high bit must be different\n");
            }  else if (_xor_high_bit > _intlv_high_bit) {
                if ((_xor_high_bit - _intlv_high_bit) < _intlv_bits)
                    fatal("XOR and interleave high bit must be at least "
                          "%d bits apart\n", _intlv_bits);
            } else {
                if ((_intlv_high_bit - _xor_high_bit) < _intlv_bits) {
                    fatal("Interleave and XOR high bit must be at least "
                          "%d bits apart\n", _intlv_bits);
                }
            }
        }

        for (auto i = 0; i < _intlv_bits; i++) {
            uint8_t bit1 = _intlv_high_bit - i;
            Addr mask = (1ULL << bit1);
            if (_xor_high_bit) {
                uint8_t bit2 = _xor_high_bit - i;
                mask |= (1ULL << bit2);
            }
            masks[_intlv_bits - i - 1] = mask;
        }
    }

    AddrRange(Addr _start, Addr _end)
        : _start(_start), _end(_end), intlvMatch(0)
    {}

    /**
     * Create an address range by merging a collection of interleaved
     * ranges.
     *
     * @param ranges Interleaved ranges to be merged
     *
     * @ingroup api_addr_range
     */
    AddrRange(std::vector<AddrRange> ranges)
        : AddrRange(Dummy{}, ranges.begin(), ranges.end())
    {}
    AddrRange(std::list<AddrRange> ranges)
        : AddrRange(Dummy{}, ranges.begin(), ranges.end())
    {}

    /**
     * Determine if the range is interleaved or not.
     *
     * @return true if interleaved
     *
     * @ingroup api_addr_range
     */
    bool interleaved() const { return masks.size() > 0; }

    /**
     * Determing the interleaving granularity of the range.
     *
     * @return The size of the regions created by the interleaving bits
     *
     * @ingroup api_addr_range
     */
    uint64_t
    granularity() const
    {
        if (interleaved()) {
            auto combined_mask = 0;
            for (auto mask: masks) {
                combined_mask |= mask;
            }
            const uint8_t lowest_bit = ctz64(combined_mask);
            return 1ULL << lowest_bit;
        } else {
            return size();
        }
    }

    /**
     * Determine the number of interleaved address stripes this range
     * is part of.
     *
     * @return The number of stripes spanned by the interleaving bits
     *
     * @ingroup api_addr_range
     */
    uint32_t stripes() const { return 1ULL << masks.size(); }

    /**
     * Get the size of the address range. For a case where
     * interleaving is used we make the simplifying assumption that
     * the size is a divisible by the size of the interleaving slice.
     *
     * @ingroup api_addr_range
     */
    Addr
    size() const
    {
        return (_end - _start) >> masks.size();
    }

    /**
     * Determine if the range is valid.
     *
     * @ingroup api_addr_range
     */
    bool valid() const { return _start <= _end; }

    /**
     * Get the start address of the range.
     *
     * @ingroup api_addr_range
     */
    Addr start() const { return _start; }

    /**
     * Get the end address of the range.
     *
     * @ingroup api_addr_range
     */
    Addr end() const { return _end; }

    /**
     * Get a string representation of the range. This could
     * alternatively be implemented as a operator<<, but at the moment
     * that seems like overkill.
     *
     * @ingroup api_addr_range
     */
    std::string
    to_string() const
    {
        if (interleaved()) {
            std::string str;
            for (unsigned int i = 0; i < masks.size(); i++) {
                str += " ";
                Addr mask = masks[i];
                while (mask) {
                    auto bit = ctz64(mask);
                    mask &= ~(1ULL << bit);
                    str += csprintf("a[%d]^", bit);
                }
                str += csprintf("\b=%d", bits(intlvMatch, i));
            }
            return csprintf("[%#llx:%#llx]%s", _start, _end, str);
        } else {
            return csprintf("[%#llx:%#llx]", _start, _end);
        }
    }

    /**
     * Determine if another range merges with the current one, i.e. if
     * they are part of the same contigous range and have the same
     * interleaving bits.
     *
     * @param r Range to evaluate merging with
     * @return true if the two ranges would merge
     *
     * @ingroup api_addr_range
     */
    bool
    mergesWith(const AddrRange& r) const
    {
        return r._start == _start && r._end == _end &&
            r.masks == masks;
    }

    /**
     * Determine if another range intersects this one, i.e. if there
     * is an address that is both in this range and the other
     * range. No check is made to ensure either range is valid.
     *
     * @param r Range to intersect with
     * @return true if the intersection of the two ranges is not empty
     *
     * @ingroup api_addr_range
     */
    bool
    intersects(const AddrRange& r) const
    {
        if (_start >= r._end || _end <= r._start) {
            // start with the simple case of no overlap at all,
            // applicable even if we have interleaved ranges
            return false;
        } else if (!interleaved() && !r.interleaved()) {
            // if neither range is interleaved, we are done
            return true;
        }

        // now it gets complicated, focus on the cases we care about
        if (r.size() == 1) {
            // keep it simple and check if the address is within
            // this range
            return contains(r.start());
        } else if (mergesWith(r)) {
            // restrict the check to ranges that belong to the
            // same chunk
            return intlvMatch == r.intlvMatch;
        } else {
            panic("Cannot test intersection of %s and %s\n",
                  to_string(), r.to_string());
        }
    }

    /**
     * Determine if this range is a subset of another range, i.e. if
     * every address in this range is also in the other range. No
     * check is made to ensure either range is valid.
     *
     * @param r Range to compare with
     * @return true if the this range is a subset of the other one
     *
     * @ingroup api_addr_range
     */
    bool
    isSubset(const AddrRange& r) const
    {
        if (interleaved())
            panic("Cannot test subset of interleaved range %s\n", to_string());

        // This address range is not interleaved and therefore it
        // suffices to check the upper bound, the lower bound and
        // whether it would fit in a continuous segment of the input
        // addr range.
        if (r.interleaved()) {
            return r.contains(_start) && r.contains(_end - 1) &&
                size() <= r.granularity();
        } else {

            if (_end <= _start){
                // Special case: if our range wraps around that is
                // _end is 2^64 so it wraps to 0.
                // In this case r will be a subset only if its _end
                // also wraps around.
                return _start >= r._start && r._end == 0;
            } else {
                // Normal case: Check if our range is completely within 'r'.
                return _start >= r._start && _end <= r._end;
            }

        }
    }

    /**
     * Determine if the range contains an address.
     *
     * @param a Address to compare with
     * @return true if the address is in the range
     *
     * @ingroup api_addr_range
     */
    bool
    contains(const Addr& a) const
    {
        // check if the address is in the range and if there is either
        // no interleaving, or with interleaving also if the selected
        // bits from the address match the interleaving value
        bool in_range = a >= _start && a < _end;
        if (in_range) {
            auto sel = 0;
            for (unsigned int i = 0; i < masks.size(); i++) {
                Addr masked = a & masks[i];
                // The result of an xor operation is 1 if the number
                // of bits set is odd or 0 othersize, thefore it
                // suffices to count the number of bits set to
                // determine the i-th bit of sel.
                sel |= (popCount(masked) % 2) << i;
            }
            return sel == intlvMatch;
        }
        return false;
    }

    /**
     * Remove the interleaving bits from an input address.
     *
     * This function returns a new address in a continous range [
     * start, start + size / intlv_bits). We can achieve this by
     * discarding the LSB in each mask.
     *
     * e.g., if the input address is of the form:
     * ------------------------------------
     * | a_high | x1 | a_mid | x0 | a_low |
     * ------------------------------------
     * where x0 is the LSB set in masks[0]
     * and x1 is the LSB set in masks[1]
     *
     * this function will return:
     * ---------------------------------
     * |    0 | a_high | a_mid | a_low |
     * ---------------------------------
     *
     * @param a the input address
     * @return the new address, or the input address if not interleaved
     *
     * @ingroup api_addr_range
     */
    inline Addr
    removeIntlvBits(Addr a) const
    {
        // Directly return the address if the range is not interleaved
        // to prevent undefined behavior.
        if (!interleaved()) {
            return a;
        }

        // Get the LSB set from each mask
        int masks_lsb[masks.size()];
        for (unsigned int i = 0; i < masks.size(); i++) {
            masks_lsb[i] = ctz64(masks[i]);
        }

        // we need to sort the list of bits we will discard as we
        // discard them one by one starting.
        std::sort(masks_lsb, masks_lsb + masks.size());

        for (unsigned int i = 0; i < masks.size(); i++) {
            const int intlv_bit = masks_lsb[i];
            if (intlv_bit > 0) {
                // on every iteration we remove one bit from the input
                // address, and therefore the lowest invtl_bit has
                // also shifted to the right by i positions.
                a = insertBits(a >> 1, intlv_bit - i - 1, 0, a);
            } else {
                a >>= 1;
            }
        }
        return a;
    }

    /**
     * This method adds the interleaving bits removed by
     * removeIntlvBits.
     *
     * @ingroup api_addr_range
     */
    inline Addr
    addIntlvBits(Addr a) const
    {
        // Directly return the address if the range is not interleaved
        // to prevent undefined behavior.
        if (!interleaved()) {
            return a;
        }

        // Get the LSB set from each mask
        int masks_lsb[masks.size()];
        for (unsigned int i = 0; i < masks.size(); i++) {
            masks_lsb[i] = ctz64(masks[i]);
        }

        // Add bits one-by-one from the LSB side.
        std::sort(masks_lsb, masks_lsb + masks.size());
        for (unsigned int i = 0; i < masks.size(); i++) {
            const int intlv_bit = masks_lsb[i];
            if (intlv_bit > 0) {
                // on every iteration we add one bit from the input
                // address, but the lowest invtl_bit in the iteration is
                // always in the right position because they are sorted
                // increasingly from the LSB
                a = insertBits(a << 1, intlv_bit - 1, 0, a);
            } else {
                a <<= 1;
            }
        }

        for (unsigned int i = 0; i < masks.size(); i++) {
            const int lsb = ctz64(masks[i]);
            const Addr intlv_bit = bits(intlvMatch, i);
            // Calculate the mask ignoring the LSB
            const Addr masked = a & masks[i] & ~(1 << lsb);
            // Set the LSB of the mask to whatever satisfies the selector bit
            a = insertBits(a, lsb, intlv_bit ^ popCount(masked));
        }

        return a;
    }

    /**
     * Determine the offset of an address within the range.
     *
     * This function returns the offset of the given address from the
     * starting address discarding any bits that are used for
     * interleaving. This way we can convert the input address to a
     * new unique address in a continuous range that starts from 0.
     *
     * @param the input address
     * @return the flat offset in the address range
     *
     * @ingroup api_addr_range
     */
    Addr
    getOffset(const Addr& a) const
    {
        bool in_range = a >= _start && a < _end;
        if (!in_range) {
            return MaxAddr;
        }
        if (interleaved()) {
            return removeIntlvBits(a) - removeIntlvBits(_start);
        } else {
            return a - _start;
        }
    }

    /**
     * Subtract a list of intervals from the range and return
     * the resulting collection of ranges, so that the union
     * of the two lists cover the original range
     *
     * The exclusion list can contain overlapping ranges
     * Interleaving ranges are not supported and will fail the
     * assertion.
     *
     * @param the input exclusion list
     * @return the resulting collection of ranges
     *
     * @ingroup api_addr_range
     */
    AddrRangeList
    exclude(const AddrRangeList &exclude_ranges) const
    {
        assert(!interleaved());

        auto sorted_ranges = exclude_ranges;
        sorted_ranges.sort();

        std::list<AddrRange> ranges;

        Addr next_start = start();
        for (const auto &e : sorted_ranges) {
            assert(!e.interleaved());
            if (!intersects(e)) {
                continue;
            }

            if (e.start() <= next_start) {
                if (e.end() < end()) {
                    if (next_start < e.end()) {
                        next_start = e.end();
                    }
                } else {
                    return ranges;
                }
            } else {
                ranges.push_back(AddrRange(next_start, e.start()));
                if (e.end() < end()) {
                    next_start = e.end();
                } else {
                    return ranges;
                }
            }
        }

        if (next_start < end()) {
            ranges.push_back(AddrRange(next_start, end()));
        }

        return ranges;
    }

    AddrRangeList
    exclude(const AddrRange &excluded_range) const
    {
        return exclude(AddrRangeList{excluded_range});
    }

    /**
     * Less-than operator used to turn an STL map into a binary search
     * tree of non-overlapping address ranges.
     *
     * @param r Range to compare with
     * @return true if the start address is less than that of the other range
     *
     * @ingroup api_addr_range
     */
    bool
    operator<(const AddrRange& r) const
    {
        if (_start != r._start) {
            return _start < r._start;
        } else {
            // For now assume that the end is also the same.
            // If both regions are interleaved, assume same interleaving,
            // and compare intlvMatch values.
            // Otherwise, return true if this address range is interleaved.
            if (interleaved() && r.interleaved()) {
                return intlvMatch < r.intlvMatch;
            } else {
                return interleaved();
            }
        }
    }

    /**
     * @ingroup api_addr_range
     */
    bool
    operator==(const AddrRange& r) const
    {
        if (_start != r._start)    return false;
        if (_end != r._end)      return false;
        if (masks != r.masks)         return false;
        if (intlvMatch != r.intlvMatch)   return false;

        return true;
    }

    /**
     * @ingroup api_addr_range
     */
    bool
    operator!=(const AddrRange& r) const
    {
        return !(*this == r);
    }

    /**
     * @ingroup api_addr_range
     */
    AddrRange
    operator&(const AddrRange& r) const
    {
        panic_if(this->interleaved() || r.interleaved(),
                 "Cannot calculate intersection of interleaved ranges.");
        Addr start = std::max(this->_start, r._start);
        Addr end = std::min(this->_end, r._end);
        if (end <= start) {
            return AddrRange(0, 0);
        }
        return AddrRange(start, end);
    }
};

static inline AddrRangeList
operator-(const AddrRange &range, const AddrRangeList &to_exclude)
{
    return range.exclude(to_exclude);
}

static inline AddrRangeList
operator-(const AddrRange &range, const AddrRange &to_exclude)
{
    return range.exclude(to_exclude);
}

static inline AddrRangeList
exclude(const AddrRangeList &base, AddrRangeList to_exclude)
{
    to_exclude.sort();

    AddrRangeList ret;
    for (const auto &range: base)
        ret.splice(ret.end(), range.exclude(to_exclude));

    return ret;
}

static inline AddrRangeList
exclude(const AddrRangeList &base, const AddrRange &to_exclude)
{
    return exclude(base, AddrRangeList{to_exclude});
}

static inline AddrRangeList
operator-(const AddrRangeList &base, const AddrRangeList &to_exclude)
{
    return exclude(base, to_exclude);
}

static inline AddrRangeList
operator-=(AddrRangeList &base, const AddrRangeList &to_exclude)
{
    base = base - to_exclude;
    return base;
}

static inline AddrRangeList
operator-(const AddrRangeList &base, const AddrRange &to_exclude)
{
    return exclude(base, to_exclude);
}

static inline AddrRangeList
operator-=(AddrRangeList &base, const AddrRange &to_exclude)
{
    base = base - to_exclude;
    return base;
}

/**
 * @ingroup api_addr_range
 */
inline AddrRange
RangeEx(Addr start, Addr end)
{
    return AddrRange(start, end);
}

/**
 * @ingroup api_addr_range
 */
inline AddrRange
RangeIn(Addr start, Addr end)
{
    return AddrRange(start, end + 1);
}

/**
 * @ingroup api_addr_range
 */
inline AddrRange
RangeSize(Addr start, Addr size)
{
    return AddrRange(start, start + size);
}

} // namespace gem5

#endif // __BASE_ADDR_RANGE_HH__
