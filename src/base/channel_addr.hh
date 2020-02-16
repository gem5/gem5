/*
 * Copyright (c) 2019 ARM Limited
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

#ifndef __BASE_CHANNEL_ADDR_HH__
#define __BASE_CHANNEL_ADDR_HH__

#include <ostream>

#include "base/addr_range.hh"

/**
 * Class holding a guest address in a contiguous channel-local address
 * space.
 */
class ChannelAddr
{
  public:
    using Type = Addr;

    /** Explicit constructor assigning a value. */
    explicit constexpr ChannelAddr(Type _a) : a(_a) { }

    /** Converting back to the value type. */
    explicit constexpr operator Type() const { return a; }

    /** Converting back to the value type. */
    constexpr Type value() const { return a; }

    constexpr ChannelAddr() : a(0) { }

    ChannelAddr(const AddrRange &range, Addr _a)
        : a(range.removeIntlvBits(_a)) {}

    ChannelAddr(const ChannelAddr &) = default;
    ChannelAddr &operator=(const ChannelAddr &) = default;


    Addr getPA(const AddrRange &range) const {
        return range.addIntlvBits(a);
    }

    constexpr ChannelAddr operator|(const Type b) const {
        return ChannelAddr(a | b);
    }
    constexpr ChannelAddr operator&(const Type b) const {
        return ChannelAddr(a & b);
    }

    constexpr ChannelAddr operator>>(const int b) const {
        return ChannelAddr(a >> b);
    }

    constexpr ChannelAddr operator<<(const int b) const {
        return ChannelAddr(a << b);
    }

    constexpr ChannelAddr operator*(const Type &b) const {
        return ChannelAddr(a * b);
    }

    constexpr ChannelAddr operator/(const Type &b) const {
        return ChannelAddr(a / b);
    }

    constexpr ChannelAddr operator+(const Type &b) const {
        return ChannelAddr(a + b);
    }

    constexpr ChannelAddr operator-(const Type &b) const {
        return ChannelAddr(a - b);
    }

    constexpr ChannelAddr operator|(const ChannelAddr &b) const {
        return ChannelAddr(a | b.a);
    }

    constexpr ChannelAddr operator&(const ChannelAddr &b) const {
        return ChannelAddr(a & b.a);
    }

    constexpr ChannelAddr operator^(const ChannelAddr &b) const {
        return ChannelAddr(a ^ b.a);
    }

    constexpr ChannelAddr operator+(const ChannelAddr &b) const {
        return ChannelAddr(a + b.a);
    }

    constexpr ChannelAddr operator-(const ChannelAddr &b) const {
        return ChannelAddr(a - b.a);
    }

    constexpr bool operator>(const ChannelAddr &b) const { return a > b.a; }
    constexpr bool operator>=(const ChannelAddr &b) const { return a >= b.a; }
    constexpr bool operator<(const ChannelAddr &b) const { return a < b.a; }
    constexpr bool operator<=(const ChannelAddr &b) const { return a <= b.a; }
    constexpr bool operator==(const ChannelAddr &b) const { return a == b.a; }
    constexpr bool operator!=(const ChannelAddr &b) const { return a != b.a; }

  private:
    /** Member holding the actual value. */
    Type a;
};

/**
 * The ChanneelAddrRange class describes a contiguous range of
 * addresses in a contiguous channel-local address space.
 */
class ChannelAddrRange
{
  public:
    constexpr ChannelAddrRange()
        : ChannelAddrRange(ChannelAddr(1), ChannelAddr(0)) {}

    constexpr ChannelAddrRange(ChannelAddr start, ChannelAddr end)
        : _start(start), _end(end) {}

    ChannelAddrRange(AddrRange ch_range, Addr start, Addr end);
    ChannelAddrRange(AddrRange ch_range, AddrRange range);

    constexpr ChannelAddrRange(const ChannelAddrRange &) = default;

    constexpr ChannelAddr size() const { return _end - _start + 1; }

    constexpr bool valid() const { return _start <= _end; }

    constexpr ChannelAddr start() const { return _start; }
    constexpr ChannelAddr end() const { return _end; }

    constexpr bool contains(ChannelAddr a) const {
        return a >= _start && a <= _end;
    }

  protected:
    ChannelAddr _start;
    ChannelAddr _end;
};

namespace std
{
    template<>
    struct hash<ChannelAddr>
    {
        typedef ChannelAddr argument_type;
        typedef std::size_t result_type;

        result_type
        operator()(argument_type const &a) const noexcept {
            return std::hash<ChannelAddr::Type>{}(
                static_cast<argument_type::Type>(a));
        }
    };
}

std::ostream &operator<<(std::ostream &out, const ChannelAddr &addr);

#endif // __BASE_CHANNEL_ADDR_HH__
