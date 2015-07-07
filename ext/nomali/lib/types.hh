/*
 * Copyright (c) 2014-2015 ARM Limited
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Andreas Sandberg
 */

#ifndef _LIBNOMALIMODEL_TYPES_HH
#define _LIBNOMALIMODEL_TYPES_HH

#include <cassert>
#include <cstdint>

#include <utility>
#include <vector>

namespace NoMali{

/**
 * @{
 * @name Register handling utilities
 */

/**
 * Register address wrapper
 *
 * This class wraps a register address. Unlike a simple typedef, this
 * provides safety from automatic type conversions from other integer
 * types since the constructor must be called explicitly.
 */
struct RegAddr {
    explicit RegAddr(uint32_t v)
        : value(v) {}

    const uint32_t value;
};

inline bool
operator<(const RegAddr &lhs, const RegAddr &rhs) {
    return lhs.value < rhs.value;
}

inline bool
operator>(const RegAddr &lhs, const RegAddr &rhs) {
    return lhs.value > rhs.value;
}

inline bool
operator<=(const RegAddr &lhs, const RegAddr &rhs) {
    return lhs.value <= rhs.value;
}

inline bool
operator>=(const RegAddr &lhs, const RegAddr &rhs) {
    return lhs.value >= rhs.value;
}

inline bool
operator==(const RegAddr &lhs, const RegAddr &rhs) {
    return lhs.value == rhs.value;
}

inline bool
operator!=(const RegAddr &lhs, const RegAddr &rhs) {
    return lhs.value != rhs.value;
}

inline RegAddr
operator+(const RegAddr &lhs, const RegAddr &rhs) {
    return RegAddr(lhs.value + rhs.value);
}

inline RegAddr
operator-(const RegAddr &lhs, const RegAddr &rhs) {
    assert(lhs >= rhs);
    return RegAddr(lhs.value - rhs.value);
}

/**
 * Class for register storage
 *
 * This class wraps a std::vector and implements a subset of its
 * functionality. Specifically, it is constant size and prevents
 * indexing with anything other than RegAddr instances.
 */
class RegVector
{
  private:
    typedef std::vector<uint32_t> vector_t;

  public:
    typedef vector_t::iterator iterator;
    typedef vector_t::const_iterator const_iterator;
    typedef vector_t::size_type size_type;

  public:
    RegVector(size_type size)
        : vector(size, 0) {}

    /** @{ */
    /**
     * Helper function to get a 64-bit register.
     *
     * @param addr Address to the low part of the register.
     * @return 64-bit value representing the concatenation of the HI
     * and LO parts of the register.
     */
    const uint32_t get64(const RegAddr &addr) const {
        const unsigned idx_lo = index(addr);
        const unsigned idx_hi = idx_lo + 1;
        return (((uint64_t)vector[idx_hi]) << 32) | vector[idx_lo];
    }

    /**
     * Helper function to set a 64-bit register.
     *
     * @param addr Address to the low part of the register.
     * @param value Value to write into the 64-bit register.
     */
    void set64(const RegAddr &addr, uint64_t value) {
        const unsigned idx_lo = index(addr);
        const unsigned idx_hi = idx_lo + 1;
        vector[idx_lo] = value & 0xFFFFFFFF;
        vector[idx_hi] = (value >> 32) & 0xFFFFFFFF;
    }

    const uint32_t &operator[](const RegAddr &addr) const {
        return vector[index(addr)];
    }

    uint32_t &operator[](const RegAddr &addr) {
        return vector[index(addr)];
    }


    iterator begin() noexcept { return vector.begin(); }
    const_iterator begin() const noexcept { return vector.begin(); }
    const_iterator cbegin() const noexcept { return vector.cbegin(); }

    iterator end() noexcept { return vector.end(); }
    const_iterator end() const noexcept { return vector.end(); }
    const_iterator cend() const noexcept { return vector.cend(); }

    const size_type size() const noexcept { return vector.size(); }

  private:
    // Disable default constructor
    RegVector();

    static uint32_t index(const RegAddr &addr) {
        assert((addr.value & 0x3) == 0);
        return addr.value >> 2;
    }


    vector_t vector;
};
/** @} */

/**
 * Class representing the status codes in the Midgard architecture.
 */
struct Status {
    /**
     * Class representing the subsystem a status code originates from.
     */
    enum StatusClass {
        CLASS_NOFAULT = 0,
        CLASS_JOB = 1,
        CLASS_GPU = 2,
        CLASS_MMU = 3,
    };

    typedef uint8_t Code;
    typedef uint8_t SubCode;

    Status(StatusClass cls, Code code, SubCode subcode)
        : value((cls << 6) | (code << 3) | subcode) {
        assert((cls & ~0x3) == 0);
        assert((code & ~0x7) == 0);
        assert((subcode & ~0x7) == 0);
    }

    explicit Status(uint8_t v)
        : value(v) {}

    StatusClass statusClass() const {
        return (StatusClass)((value >> 6) & 0x3);
    }

    Code code() const {
        return (value >> 3) & 0x7;
    }

    SubCode subCode() const {
        return value & 0x7;
    }

    const uint8_t value;
};


}

#endif

