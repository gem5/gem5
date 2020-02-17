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

#include "arch/generic/types.hh"
#include "arch/generic/vec_reg.hh"

class InstResult {
    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;
    using VecPredRegContainer = TheISA::VecPredRegContainer;
  public:
    union MultiResult {
        uint64_t integer;
        double dbl;
        VecRegContainer vector;
        VecElem vecElem;
        VecPredRegContainer pred;
        MultiResult() {}
    };

    enum class ResultType {
        Scalar,
        VecElem,
        VecReg,
        VecPredReg,
        NumResultTypes,
        Invalid
    };

  private:
    MultiResult result;
    ResultType type;

  public:
    /** Default constructor creates an invalid result. */
    InstResult() : type(ResultType::Invalid) { }
    InstResult(const InstResult &) = default;
    /** Scalar result from scalar. */
    template<typename T>
    explicit InstResult(T i, const ResultType& t) : type(t) {
        static_assert(std::is_integral<T>::value ^
                        std::is_floating_point<T>::value,
                "Parameter type is neither integral nor fp, or it is both");
        if (std::is_integral<T>::value) {
            result.integer = i;
        } else if (std::is_floating_point<T>::value) {
            result.dbl = i;
        }
    }
    /** Vector result. */
    explicit InstResult(const VecRegContainer& v, const ResultType& t)
        : type(t) { result.vector = v; }
    /** Predicate result. */
    explicit InstResult(const VecPredRegContainer& v, const ResultType& t)
        : type(t) { result.pred = v; }

    InstResult& operator=(const InstResult& that) {
        type = that.type;
        switch (type) {
        /* Given that misc regs are not written to, there may be invalids in
         * the result stack. */
        case ResultType::Invalid:
            break;
        case ResultType::Scalar:
            result.integer = that.result.integer;
            break;
        case ResultType::VecElem:
            result.vecElem = that.result.vecElem;
            break;
        case ResultType::VecReg:
            result.vector = that.result.vector;
            break;
        case ResultType::VecPredReg:
            result.pred = that.result.pred;
            break;

        default:
            panic("Assigning result from unknown result type");
            break;
        }
        return *this;
    }
    /**
     * Result comparison
     * Two invalid results always differ.
     */
    bool operator==(const InstResult& that) const {
        if (this->type != that.type)
            return false;
        switch (type) {
        case ResultType::Scalar:
            return result.integer == that.result.integer;
        case ResultType::VecElem:
            return result.vecElem == that.result.vecElem;
        case ResultType::VecReg:
            return result.vector == that.result.vector;
        case ResultType::VecPredReg:
            return result.pred == that.result.pred;
        case ResultType::Invalid:
            return false;
        default:
            panic("Unknown type of result: %d\n", (int)type);
        }
    }

    bool operator!=(const InstResult& that) const {
        return !operator==(that);
    }

    /** Checks */
    /** @{ */
    /** Is this a scalar result?. */
    bool isScalar() const { return type == ResultType::Scalar; }
    /** Is this a vector result?. */
    bool isVector() const { return type == ResultType::VecReg; }
    /** Is this a vector element result?. */
    bool isVecElem() const { return type == ResultType::VecElem; }
    /** Is this a predicate result?. */
    bool isPred() const { return type == ResultType::VecPredReg; }
    /** Is this a valid result?. */
    bool isValid() const { return type != ResultType::Invalid; }
    /** @} */

    /** Explicit cast-like operations. */
    /** @{ */
    const uint64_t&
    asInteger() const
    {
        assert(isScalar());
        return result.integer;
    }

    /** Cast to integer without checking type.
     * This is required to have the o3 cpu checker happy, as it
     * compares results as integers without being fully aware of
     * their nature. */
    const uint64_t&
    asIntegerNoAssert() const
    {
        return result.integer;
    }
    const VecRegContainer&
    asVector() const
    {
        panic_if(!isVector(), "Converting scalar (or invalid) to vector!!");
        return result.vector;
    }
    const VecElem&
    asVectorElem() const
    {
        panic_if(!isVecElem(), "Converting scalar (or invalid) to vector!!");
        return result.vecElem;
    }

    const VecPredRegContainer&
    asPred() const
    {
        panic_if(!isPred(), "Converting scalar (or invalid) to predicate!!");
        return result.pred;
    }

    /** @} */
};

#endif // __CPU_INST_RES_HH__
