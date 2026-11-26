/*
 * Copyright (c) 2026 Arm Limited
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
 * Copyright (c) 2020 Daniel R. Carvalho
 * Copyright (c) 2008 The Hewlett-Packard Development Company
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

#ifndef __BASE_FLAGS_HH__
#define __BASE_FLAGS_HH__

#include <bitset>
#include <cassert>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace gem5
{

/**
 * Wrapper that groups a few flag bits under the same undelying container.
 *
 * @tparam T The type of the underlying container. Must be an unsigned integer.
 */
template <typename T>
class Flags
{
  private:
    static_assert(std::is_unsigned_v<T>, "Flag type must be unsigned");

    /** The undelying container of the flags' bits. */
    T _flags;

  public:
    typedef T Type;

    /**
     * @ingroup api_flags
     * @{
     */

    /**
     * Initialize flags with a given value. If no value is provided, the
     * flag bits are initialized cleared.
     *
     * @param flags The value to initialize the flags with.
     */
    Flags(Type flags=0) : _flags(flags) {}

    operator const Type() const { return _flags; }

    const Flags<T> &
    operator=(T flags)
    {
        _flags = flags;
        return *this;
    }

    /**
     * Verifies whether any bit matching the given mask is set.
     *
     * @param mask The mask containing the bits to verify.
     * @return True if any matching bit is set; false otherwise.
     */
    bool isSet(Type mask) const { return (_flags & mask); }

    /**
     * Verifies whether no bits matching the given mask are set.
     *
     * @param mask The mask containing the bits to verify.
     * @return True if matching bits are set; false otherwise.
     */
    bool allSet(Type mask) const { return (_flags & mask) == mask; }

    /**
     * Verifies whether no bits matching the given mask are set.
     *
     * @param mask The mask containing the bits to verify.
     * @return True if matching bits are cleared; false otherwise.
     */
    bool noneSet(Type mask) const { return (_flags & mask) == 0; }

    /** Clear all flag's bits. */
    void clear() { _flags = 0; }

    /**
     * Clear all flag's bits matching the given mask.
     *
     * @param mask The mask containing the bits to be cleared.
     */
    void clear(Type mask) { _flags &= ~mask; }

    /**
     * Set all flag's bits matching the given mask.
     *
     * @param mask The mask containing the bits to be set.
     */
    void set(Type mask) { _flags |= mask; }

    /**
     * Conditionally set or clear some bits of the flag, given a mask.
     *
     * @param mask The mask containing the bits to be modified.
     * @param condition If true, set masked bits; otherwise, clear them.
     */
    void
    set(Type mask, bool condition)
    {
        condition ? set(mask) : clear(mask);
    }

    /**
     * Replace the contents of the bits matching the mask with the
     * corresponding bits in the provided flags.
     *
     * This is equivalent to:
     *     flags.clear(mask); flags.set(flags & mask);
     *
     * @param flags Flags to extract new bits from.
     * @param mask Mask used to determine which bits are replaced.
     */
    void
    replace(Type flags, Type mask)
    {
        _flags = (_flags & ~mask) | (flags & mask);
    }
    /** @} */ // end of api_flags
};

/**
 * Wrapper that groups flag bits under a fixed-size bitset container
 * instead of relying on an unsigned backend storage like the traditional
 * gem5::Flags class above.
 *
 * The intended use-case is for non-pow2 scoped enums flags,
 * example:
 *
 * enum class MyFlag
 * {
 *     FlagA,
 *     FlagB,
 *     NumFlags,
 * };
 *
 * BitsetFlag<typename T, std::size_t NUM> becomes:
 *
 * BitsetFlag<MyFlag, NumFlags>
 *
 * T values are interpreted as flag indices. T must be a scoped enum with an
 * unsigned underlying type, and NUM normally comes from a trailing
 * value which gets usually defined by users to signal the end/max value
 * of the enum.
 *
 * @tparam T The scoped enum type used to identify flags.
 * @tparam NUM The number of flag bits stored by the bitset.
 */
template <typename T, std::size_t NUM> class BitsetFlag
{
  private:
    using IntType = std::underlying_type_t<T>;

    static_assert(std::is_enum_v<T>, "Flag type must be an enum");
    static_assert(!std::is_convertible_v<T, std::underlying_type_t<T>>,
                  "Flag type must be a scoped enum");
    static_assert(std::is_unsigned_v<std::underlying_type_t<T>>,
                  "Flag enum must use an unsigned underlying type");
    static_assert(NUM <= std::numeric_limits<IntType>::digits,
                  "Not enough bits in bitset to store all enum values");
    static_assert(
        NUM > 0, "Negative number of bits in bitset to store all enum values");

    /** The underlying container of the flags' bits. */
    std::bitset<NUM> _flags;

    static constexpr IntType
    index(T flag)
    {
        return static_cast<IntType>(flag);
    }

  public:
    using Type = T;

    /**
     * @ingroup api_flags
     * @{
     */

    /** Initialize flags with all bits cleared. */
    BitsetFlag() = default;

    /** Initialize flags with a single flag set. */
    BitsetFlag(Type flag) { set(flag); }

    const BitsetFlag<T, NUM> &
    operator=(Type flag)
    {
        clear();
        set(flag);
        return *this;
    }

    /** Return the number of flag bits stored by this container. */
    static constexpr std::size_t
    size()
    {
        return NUM;
    }

    /**
     * Verifies whether a flag is set.
     * @param flag The flag to be checked
     * @return True if flag is set; false otherwise.
     */
    bool
    isSet(Type flag) const
    {
        assert(index(flag) < NUM);
        return _flags.test(index(flag));
    }

    /** Verifies whether any flag matching the given set is set. */
    bool
    isSet(const BitsetFlag<T, NUM> &flags) const
    {
        return (_flags & flags._flags).any();
    }

    /** Verifies whether all flags matching the given set are set. */
    bool
    allSet(const BitsetFlag<T, NUM> &flags) const
    {
        return (_flags & flags._flags) == flags._flags;
    }

    /** Verifies whether no flag is set. */
    bool
    noneSet() const
    {
        return _flags.none();
    }

    /** Verifies whether no flag matching the given flag is set. */
    bool
    noneSet(Type flag) const
    {
        return !isSet(flag);
    }

    /** Verifies whether no flag matching the given set is set. */
    bool
    noneSet(const BitsetFlag<T, NUM> &flags) const
    {
        return (_flags & flags._flags).none();
    }

    /** Clear all flag's bits. */
    void
    clear()
    {
        _flags.reset();
    }

    /** Clear the given flag. */
    void
    clear(Type flag)
    {
        assert(index(flag) < NUM);
        _flags.reset(index(flag));
    }

    /** Clear all flags in the given set. */
    void
    clear(const BitsetFlag<T, NUM> &flags)
    {
        _flags &= ~flags._flags;
    }

    /** Clear all flag bits except those in the given set. */
    void
    clearAllExcept(const BitsetFlag<T, NUM> &flags)
    {
        _flags &= flags._flags;
    }

    /** Set the given flag. */
    void
    set(Type flag)
    {
        assert(index(flag) < NUM);
        _flags.set(index(flag));
    }

    /** Set all flags in the given set. */
    void
    set(const BitsetFlag<T, NUM> &flags)
    {
        _flags |= flags._flags;
    }

    /** Conditionally set or clear the given flag. */
    void
    set(Type flag, bool condition)
    {
        condition ? set(flag) : clear(flag);
    }

    /** Conditionally set or clear all flags in the given set. */
    void
    set(const BitsetFlag<T, NUM> &flags, bool condition)
    {
        condition ? set(flags) : clear(flags);
    }

    /**
     * Replace the contents of the bits matching the mask with the
     * corresponding bits in the provided flags.
     */
    void
    replace(const BitsetFlag<T, NUM> &flags, const BitsetFlag<T, NUM> &mask)
    {
        _flags = (_flags & ~mask._flags) | (flags._flags & mask._flags);
    }

    BitsetFlag<T, NUM>
    operator|(Type flag) const
    {
        BitsetFlag<T, NUM> result(*this);
        result.set(flag);
        return result;
    }

    BitsetFlag<T, NUM>
    operator|(const BitsetFlag<T, NUM> &flags) const
    {
        BitsetFlag<T, NUM> result(*this);
        result.set(flags);
        return result;
    }
    /** @} */ // end of api_flags
};

} // namespace gem5

#endif // __BASE_FLAGS_HH__
