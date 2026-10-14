/**
 * Copyright (c) 2026 Google Inc
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

#ifndef __BASE_ADDR_RANGE_MAP_POLICY_HH__
#define __BASE_ADDR_RANGE_MAP_POLICY_HH__

#include <algorithm>
#include <memory>
#include <typeinfo>
#include <vector>

#include "base/bitfield.hh"
#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/types.hh"

namespace gem5
{

class AddrMapPolicy
{
  public:
    virtual ~AddrMapPolicy() = default;

    /**
     * Checks if the address a is contained within the range defined by
     * rangeStart and rangeEnd, according to this policy.
     * Note: The basic bounds check (a >= start && a < end) should happen
     * BEFORE calling this method.
     */
    virtual bool contains(Addr rangeStart, Addr rangeEnd, Addr a) const = 0;

    /**
     * Calculates the offset of address a within the range.
     */
    virtual Addr getOffset(Addr rangeStart, Addr rangeEnd, Addr a) const = 0;

    /**
     * Inverse of getOffset.
     */
    virtual Addr toInt(Addr rangeStart, Addr rangeEnd, Addr offset) const = 0;

    /**
     * Determing the interleaving granularity of the range.
     */
    virtual uint64_t granularity() const = 0;

    /**
     * Determine the number of interleaved address stripes this range
     * is part of.
     */
    virtual uint32_t stripes() const = 0;

    /**
     * Get the size of the address range.
     */
    virtual Addr size(Addr rangeStart, Addr rangeEnd) const = 0;

    /**
     * Returns true if this policy is "equivalent" to another.
     * Used for equality operators.
     */
    virtual bool
    isEquivalent(const std::shared_ptr<AddrMapPolicy> &other) const = 0;

    /**
     * Determine if another range merges with the current one, i.e. if
     * they are part of the same contigous range and have the same
     * interleaving bits.
     */
    virtual bool
    canMerge(const std::shared_ptr<AddrMapPolicy> &other) const = 0;

    /**
     * Compare policies for sorting.
     *
     * This provides a stable tie-breaker for strict weak ordering in STL
     * containers (e.g., AddrRangeMap) when two ranges have identical start and
     * end addresses but use different interleaving policy implementations.
     */
    virtual bool
    lessThan(const std::shared_ptr<AddrMapPolicy> &other) const
    {
        auto ptr = other.get();
        if (typeid(*this) != typeid(*ptr)) {
            return typeid(*this).before(typeid(*ptr));
        }
        return false;
    }

    /**
     * Check logic intersection with another range.
     */
    virtual bool
    checkIntersection(Addr myStart, Addr myEnd, Addr otherStart, Addr otherEnd,
                      const std::shared_ptr<AddrMapPolicy> &otherPolicy) const
    {
        return true;
    }

    /**
     * Create a new policy by merging this policy with others.
     * Returns nullptr if the result is a non-interleaved (flat) policy.
     */
    virtual std::shared_ptr<AddrMapPolicy>
    createMerged(
        const std::vector<std::shared_ptr<AddrMapPolicy>> &policies) const
    {
        panic("Merging not supported for this policy type");
        return nullptr;
    }

    /**
     * Get a string representation of the policy.
     *
     * This is just the policy information (e.g., interleaving bits). The base,
     * and bounds is printed by the AddrRange class.
     */
    virtual std::string to_string() const = 0;
};

/**
 * Interleaving policy based on bitwise masks and parity matching.
 *
 * If the user provides a non-empty vector of masks, the address range is
 * interleaved. Each mask determines a set of bits that are XORed to determine
 * one bit of the sel value, starting from the least significant bit (i.e.,
 * masks[0] determines the least significant bit of sel, ...). If sel matches
 * the provided intlvMatch, then the address a is in the range.
 *
 * For example, if the input mask is:
 * _masks = { 1 << 8 | 1 << 11 | 1 << 13,
 *            1 << 15 | 1 << 17 | 1 << 19 }
 *
 * Then a belongs to the address range if:
 * rangeStart <= a < rangeEnd
 * and
 * sel == intlvMatch
 * where
 * sel[0] = a[8] ^ a[11] ^ a[13]
 * sel[1] = a[15] ^ a[17] ^ a[19]
 */
class MaskedInterleavingPolicy : public AddrMapPolicy
{
  private:
    std::vector<Addr> masks;
    uint8_t intlvMatch;

  public:
    MaskedInterleavingPolicy(const std::vector<Addr> &_masks,
                             uint8_t _intlvMatch)
        : masks(_masks), intlvMatch(_intlvMatch)
    {}

    bool
    contains(Addr rangeStart, Addr rangeEnd, Addr a) const override
    {
        auto sel = 0;
        for (unsigned int i = 0; i < masks.size(); i++) {
            Addr masked = a & masks[i];
            sel |= (popCount(masked) % 2) << i;
        }
        return sel == intlvMatch;
    }

    Addr
    getOffset(Addr rangeStart, Addr rangeEnd, Addr a) const override
    {
        return removeIntlvBits(a) - removeIntlvBits(rangeStart);
    }

    Addr
    toInt(Addr rangeStart, Addr rangeEnd, Addr offset) const override
    {
        return addIntlvBits(removeIntlvBits(rangeStart) + offset);
    }

    uint64_t
    granularity() const override
    {
        auto combined_mask = Addr(0);
        for (auto mask : masks) {
            combined_mask |= mask;
        }
        const uint8_t lowest_bit = ctz64(combined_mask);
        return 1ULL << lowest_bit;
    }

    uint32_t
    stripes() const override
    {
        return 1ULL << masks.size();
    }

    Addr
    size(Addr rangeStart, Addr rangeEnd) const override
    {
        return (rangeEnd - rangeStart) >> masks.size();
    }

    bool
    isEquivalent(const std::shared_ptr<AddrMapPolicy> &other) const override
    {
        auto casted =
            std::dynamic_pointer_cast<MaskedInterleavingPolicy>(other);
        if (!casted) {
            return false;
        }
        return masks == casted->masks && intlvMatch == casted->intlvMatch;
    }

    bool
    canMerge(const std::shared_ptr<AddrMapPolicy> &other) const override
    {
        auto casted =
            std::dynamic_pointer_cast<MaskedInterleavingPolicy>(other);
        if (!casted) {
            return false;
        }
        return masks == casted->masks;
    }

    /*
     * Helper functions from original AddrRange
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
     **/
    inline Addr
    removeIntlvBits(Addr a) const
    {
        auto masks_lsb = std::make_unique<int[]>(masks.size());
        for (unsigned int i = 0; i < masks.size(); i++) {
            masks_lsb[i] = ctz64(masks[i]);
        }
        std::sort(masks_lsb.get(), masks_lsb.get() + masks.size());

        for (unsigned int i = 0; i < masks.size(); i++) {
            const int intlv_bit = masks_lsb[i];
            if (intlv_bit > 0) {
                a = insertBits(a >> 1, intlv_bit - i - 1, 0, a);
            } else {
                a >>= 1;
            }
        }
        return a;
    }

    inline Addr
    addIntlvBits(Addr a) const
    {
        auto masks_lsb = std::make_unique<int[]>(masks.size());
        for (unsigned int i = 0; i < masks.size(); i++) {
            masks_lsb[i] = ctz64(masks[i]);
        }
        std::sort(masks_lsb.get(), masks_lsb.get() + masks.size());

        for (unsigned int i = 0; i < masks.size(); i++) {
            const int intlv_bit = masks_lsb[i];
            if (intlv_bit > 0) {
                a = insertBits(a << 1, intlv_bit - 1, 0, a);
            } else {
                a <<= 1;
            }
        }

        for (unsigned int i = 0; i < masks.size(); i++) {
            const int lsb = ctz64(masks[i]);
            const Addr intlv_bit = bits(intlvMatch, i);
            const Addr masked = a & masks[i] & ~(1 << lsb);
            a = insertBits(a, lsb, intlv_bit ^ popCount(masked));
        }
        return a;
    }

    const std::vector<Addr> &
    getMasks() const
    {
        return masks;
    }
    uint8_t
    getMatch() const
    {
        return intlvMatch;
    }

    std::string
    to_string() const override
    {
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
        return str;
    }

    bool
    lessThan(const std::shared_ptr<AddrMapPolicy> &other) const override
    {
        auto ptr = other.get();
        if (typeid(*this) != typeid(*ptr)) {
            return AddrMapPolicy::lessThan(other);
        }
        auto casted =
            std::static_pointer_cast<MaskedInterleavingPolicy>(other);
        if (masks != casted->masks) {
            return masks < casted->masks;
        }
        return intlvMatch < casted->intlvMatch;
    }

    bool
    checkIntersection(
        Addr myStart, Addr myEnd, Addr otherStart, Addr otherEnd,
        const std::shared_ptr<AddrMapPolicy> &otherPolicy) const override
    {
        auto casted =
            std::dynamic_pointer_cast<MaskedInterleavingPolicy>(otherPolicy);
        if (casted) {
            if (masks == casted->masks) {
                return intlvMatch == casted->intlvMatch;
            }
        }
        return true;
    }

    std::shared_ptr<AddrMapPolicy>
    createMerged(const std::vector<std::shared_ptr<AddrMapPolicy>> &policies)
        const override
    {
        uint64_t count = policies.size();
        if (count != (1ULL << masks.size())) {
            fatal("Got %d ranges spanning %d interleaving bits.", count,
                  masks.size());
        }

        std::vector<bool> seen(count, false);
        for (const auto &p : policies) {
            auto masked =
                std::dynamic_pointer_cast<MaskedInterleavingPolicy>(p);
            if (!masked) {
                fatal("Cannot merge non-masked policies");
            }
            if (masked->masks != masks) {
                fatal("Masks mismatch during merge");
            }
            if (seen[masked->intlvMatch]) {
                fatal("Duplicate interleave match %d", masked->intlvMatch);
            }
            seen[masked->intlvMatch] = true;
        }
        // This means the ranges merged into a non-interleaved range.
        // This is the only supported outcome of merging.
        return nullptr;
    }
};

/**
 * This policy implements modulo-based interleaving, e.g. for systems
 * with a non-power-of-two number of channels.
 *
 * It assumes a simple interleaving scheme:
 * addr % stripes == match
 *
 * However, since interleaving is usually done at a cache line (or larger)
 * granularity, the check is effectively:
 * (addr >> intlvBit) % stripes == match
 */
class ModuloInterleavingPolicy : public AddrMapPolicy
{
  private:
    const uint32_t nStripes;
    const uint32_t intlvMatch;
    const uint32_t intlvLowBit;

  public:
    ModuloInterleavingPolicy(uint32_t stripes, uint32_t match, uint32_t bit)
        : nStripes(stripes), intlvMatch(match), intlvLowBit(bit)
    {
        fatal_if(stripes == 0, "Modulo stripes must be greater than 0");
        fatal_if(match >= stripes,
                 "Modulo match value %d must be less than stripes %d", match,
                 stripes);
        fatal_if(intlvLowBit >= sizeof(Addr) * 8,
                 "Modulo intlvLowBit %d must be less than %zu", intlvLowBit,
                 sizeof(Addr) * 8);
    }

    bool
    contains(Addr rangeStart, Addr rangeEnd, Addr a) const override
    {
        if (a < rangeStart || a >= rangeEnd) {
            return false;
        }
        return ((a >> intlvLowBit) % nStripes) == intlvMatch;
    }

    uint32_t
    getStripes() const
    {
        return nStripes;
    }
    uint32_t
    getMatch() const
    {
        return intlvMatch;
    }
    uint32_t
    getLowBit() const
    {
        return intlvLowBit;
    }

    Addr
    getOffset(Addr rangeStart, Addr rangeEnd, Addr a) const override
    {
        return toCompact(a) - toCompact(rangeStart);
    }

    Addr
    toInt(Addr rangeStart, Addr rangeEnd, Addr offset) const override
    {
        return fromCompact(offset + toCompact(rangeStart));
    }

    uint64_t
    granularity() const override
    {
        return 1ULL << intlvLowBit;
    }

    uint32_t
    stripes() const override
    {
        return nStripes;
    }

    Addr
    size(Addr rangeStart, Addr rangeEnd) const override
    {
        return toCompact(rangeEnd) - toCompact(rangeStart);
    }

    bool
    isEquivalent(const std::shared_ptr<AddrMapPolicy> &other) const override
    {
        auto casted =
            std::dynamic_pointer_cast<ModuloInterleavingPolicy>(other);
        if (!casted) {
            return false;
        }
        return nStripes == casted->nStripes &&
               intlvMatch == casted->intlvMatch &&
               intlvLowBit == casted->intlvLowBit;
    }

    bool
    canMerge(const std::shared_ptr<AddrMapPolicy> &other) const override
    {
        auto casted =
            std::dynamic_pointer_cast<ModuloInterleavingPolicy>(other);
        if (!casted) {
            return false;
        }
        return nStripes == casted->nStripes &&
               intlvLowBit == casted->intlvLowBit;
    }

    bool
    lessThan(const std::shared_ptr<AddrMapPolicy> &other) const override
    {
        auto ptr = other.get();
        if (typeid(*this) != typeid(*ptr)) {
            return AddrMapPolicy::lessThan(other);
        }
        auto casted =
            std::static_pointer_cast<ModuloInterleavingPolicy>(other);
        if (nStripes != casted->nStripes) {
            return nStripes < casted->nStripes;
        }
        if (intlvLowBit != casted->intlvLowBit) {
            return intlvLowBit < casted->intlvLowBit;
        }
        return intlvMatch < casted->intlvMatch;
    }

    bool
    checkIntersection(
        Addr myStart, Addr myEnd, Addr otherStart, Addr otherEnd,
        const std::shared_ptr<AddrMapPolicy> &otherPolicy) const override
    {
        auto casted =
            std::dynamic_pointer_cast<ModuloInterleavingPolicy>(otherPolicy);
        if (casted) {
            if (nStripes == casted->nStripes &&
                intlvLowBit == casted->intlvLowBit) {
                return intlvMatch == casted->intlvMatch;
            }
        }
        return true;
    }

    std::shared_ptr<AddrMapPolicy>
    createMerged(const std::vector<std::shared_ptr<AddrMapPolicy>> &policies)
        const override
    {
        uint64_t count = policies.size();
        if (count != nStripes) {
            fatal("Got %d ranges for %d stripes.", count, nStripes);
        }

        std::vector<bool> seen(count, false);
        for (const auto &p : policies) {
            auto modulo =
                std::dynamic_pointer_cast<ModuloInterleavingPolicy>(p);
            if (!modulo) {
                fatal("Cannot merge non-modulo policies");
            }
            if (modulo->nStripes != nStripes) {
                fatal("Stripes mismatch during merge");
            }
            if (modulo->intlvLowBit != intlvLowBit) {
                fatal("IntlvLowBit mismatch during merge");
            }
            if (seen[modulo->intlvMatch]) {
                fatal("Duplicate interleave match %d", modulo->intlvMatch);
            }
            seen[modulo->intlvMatch] = true;
        }
        // This means the ranges merged into a non-interleaved range.
        // This is the only supported outcome of merging.
        return nullptr;
    }

    std::string
    to_string() const override
    {
        return csprintf("mod %d @ %d", nStripes, intlvMatch);
    }

  private:
    Addr
    toCompact(Addr a) const
    {
        return ((a >> intlvLowBit) / nStripes) << intlvLowBit |
               (a & ((1ULL << intlvLowBit) - 1));
    }

    Addr
    fromCompact(Addr a) const
    {
        return (((a >> intlvLowBit) * nStripes + intlvMatch) << intlvLowBit) |
               (a & ((1ULL << intlvLowBit) - 1));
    }
};

} // namespace gem5

#endif // __BASE_ADDR_RANGE_MAP_POLICY_HH__
