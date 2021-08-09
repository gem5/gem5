/*
 * Copyright (c) 2016-2017 ARM Limited
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

#ifndef __CPU_INST_RES_HH__
#define __CPU_INST_RES_HH__

#include <any>
#include <type_traits>

#include "arch/vecregs.hh"
#include "base/logging.hh"
#include "base/types.hh"

namespace gem5
{

class InstResult
{
  private:
    std::any result;
    std::function<bool(const std::any &a, const std::any &b)> equals;

  public:
    /** Default constructor creates an invalid result. */
    InstResult() :
        // This InstResult is empty, and will only equal other InstResults
        // which are also empty.
        equals([](const std::any &a, const std::any &b) -> bool {
            gem5_assert(!a.has_value());
            return !b.has_value();
        })
    {}
    InstResult(const InstResult &) = default;

    template <typename T>
    explicit InstResult(T val) : result(val),

        // Set equals so it knows how to compare results of type T.
        equals([](const std::any &a, const std::any &b) -> bool {
            // If one has a value but the other doesn't, not equal.
            if (a.has_value() != b.has_value())
                return false;
            // If they are both empty, equal.
            if (!a.has_value())
                return true;
            // At least the local object should be of the right type.
            gem5_assert(a.type() == typeid(T));
            // If these aren't the same type, not equal.
            if (a.type() != b.type())
                return false;
            // We now know these both hold a result of the right type.
            return std::any_cast<const T&>(a) == std::any_cast<const T&>(b);
        })
    {
        static_assert(!std::is_pointer_v<T>,
                "InstResult shouldn't point to external data.");
    }

    // Convert floating point values to integers.
    template <typename T,
             std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
    explicit InstResult(T val) : InstResult(floatToBits(val)) {}

    // Convert all integer types to RegVal.
    template <typename T,
        std::enable_if_t<std::is_integral_v<T> && !std::is_same_v<T, RegVal>,
                         int> = 0>
    explicit InstResult(T val) : InstResult(static_cast<RegVal>(val)) {}

    InstResult &
    operator=(const InstResult& that)
    {
        result = that.result;
        equals = that.equals;
        return *this;
    }

    /**
     * Result comparison
     * Two invalid results always differ.
     */
    bool
    operator==(const InstResult& that) const
    {
        return equals(result, that.result);
    }

    bool
    operator!=(const InstResult& that) const
    {
        return !operator==(that);
    }

    /** Checks */
    /** @{ */
    /** Is this a scalar result?. */
    bool
    isScalar() const
    {
        return result.type() == typeid(RegVal);
    }
    /** Is this a vector result?. */
    bool
    isVector() const
    {
        return result.type() == typeid(TheISA::VecRegContainer);
    }
    /** Is this a predicate result?. */
    bool
    isPred() const
    {
        return result.type() == typeid(TheISA::VecPredRegContainer);
    }

    /** Is this a valid result?. */
    bool isValid() const { return result.has_value(); }
    /** @} */

    /** Explicit cast-like operations. */
    /** @{ */
    RegVal
    asInteger() const
    {
        panic_if(!isScalar(), "Converting non-scalar to scalar!!");
        return std::any_cast<RegVal>(result);
    }

    /** Cast to integer without checking type.
     * This is required to have the o3 cpu checker happy, as it
     * compares results as integers without being fully aware of
     * their nature. */
    RegVal
    asIntegerNoAssert() const
    {
        if (!isScalar())
            return 0;
        return std::any_cast<RegVal>(result);
    }

    TheISA::VecRegContainer
    asVector() const
    {
        panic_if(!isVector(), "Converting scalar (or invalid) to vector!!");
        return std::any_cast<TheISA::VecRegContainer>(result);
    }

    TheISA::VecPredRegContainer
    asPred() const
    {
        panic_if(!isPred(), "Converting scalar (or invalid) to predicate!!");
        return std::any_cast<TheISA::VecPredRegContainer>(result);
    }

    /** @} */
};

} // namespace gem5

#endif // __CPU_INST_RES_HH__
