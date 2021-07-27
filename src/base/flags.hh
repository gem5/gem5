/*
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

} // namespace gem5

#endif // __BASE_FLAGS_HH__
