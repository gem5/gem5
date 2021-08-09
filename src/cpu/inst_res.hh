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

#include <type_traits>
#include <variant>

#include "arch/vecregs.hh"
#include "base/types.hh"

namespace gem5
{

class InstResult
{
  private:
    std::variant<RegVal, TheISA::VecRegContainer,
        TheISA::VecPredRegContainer> result;

  public:
    /** Default constructor creates an invalid result. */
    InstResult() = default;
    InstResult(const InstResult &) = default;

    template <typename T>
    explicit InstResult(T val) : result(val) {}

    template <typename T, typename enable=
        std::enable_if_t<std::is_floating_point_v<T>>>
    explicit InstResult(T val) : result(floatToBits(val)) {}

    InstResult &
    operator=(const InstResult& that)
    {
        result = that.result;
        return *this;
    }

    /**
     * Result comparison
     * Two invalid results always differ.
     */
    bool
    operator==(const InstResult& that) const
    {
        return result == that.result;
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
        return std::holds_alternative<RegVal>(result);
    }
    /** Is this a vector result?. */
    bool
    isVector() const
    {
        return std::holds_alternative<TheISA::VecRegContainer>(result);
    }
    /** Is this a predicate result?. */
    bool
    isPred() const
    {
        return std::holds_alternative<TheISA::VecPredRegContainer>(result);
    }

    /** Is this a valid result?. */
    bool isValid() const { return result.index() != 0; }
    /** @} */

    /** Explicit cast-like operations. */
    /** @{ */
    RegVal
    asInteger() const
    {
        assert(isScalar());
        return std::get<RegVal>(result);
    }

    /** Cast to integer without checking type.
     * This is required to have the o3 cpu checker happy, as it
     * compares results as integers without being fully aware of
     * their nature. */
    RegVal
    asIntegerNoAssert() const
    {
        const RegVal *ptr = std::get_if<RegVal>(&result);
        return ptr ? *ptr : 0;
    }
    const TheISA::VecRegContainer&
    asVector() const
    {
        panic_if(!isVector(), "Converting scalar (or invalid) to vector!!");
        return std::get<TheISA::VecRegContainer>(result);
    }

    const TheISA::VecPredRegContainer&
    asPred() const
    {
        panic_if(!isPred(), "Converting scalar (or invalid) to predicate!!");
        return std::get<TheISA::VecPredRegContainer>(result);
    }

    /** @} */
};

} // namespace gem5

#endif // __CPU_INST_RES_HH__
