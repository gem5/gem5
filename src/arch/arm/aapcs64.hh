/*
 * Copyright 2019 Google Inc.
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

#ifndef __ARCH_ARM_AAPCS64_HH__
#define __ARCH_ARM_AAPCS64_HH__

#include <algorithm>
#include <array>
#include <type_traits>
#include <utility>

#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/arm/utility.hh"
#include "base/intmath.hh"
#include "cpu/thread_context.hh"
#include "sim/guest_abi.hh"
#include "sim/proxy_ptr.hh"

namespace gem5
{

class ThreadContext;

struct Aapcs64
{
    using UintPtr = uint64_t;

    struct State
    {
        int ngrn=0; // Next general purpose register number.
        int nsrn=0; // Next SIMD and floating point register number.
        Addr nsaa; // Next stacked argument address.

        // The maximum allowed general purpose register number.
        static const int MAX_GRN = 7;
        // The maximum allowed SIMD and floating point register number.
        static const int MAX_SRN = 7;

        explicit State(const ThreadContext *tc) :
            nsaa(tc->getReg(ArmISA::int_reg::Spx))
        {}
    };
};

namespace guest_abi
{

/*
 * Short Vectors
 */

// A short vector is a machine type that is composed of repeated instances of
// one fundamental integral or floating- point type. It may be 8 or 16 bytes
// in total size.

template <typename T, typename Enabled=void>
struct IsAapcs64ShortVector : public std::false_type {};

template <typename E, size_t N>
struct IsAapcs64ShortVector<E[N],
    typename std::enable_if_t<
        (std::is_integral_v<E> || std::is_floating_point_v<E>) &&
        (sizeof(E) * N == 8 || sizeof(E) * N == 16)>> :
        public std::true_type
{};

template <typename T>
constexpr bool IsAapcs64ShortVectorV = IsAapcs64ShortVector<T>::value;

/*
 * Composite Types
 */

template <typename T, typename Enabled=void>
struct IsAapcs64Composite : public std::false_type {};

template <typename T>
struct IsAapcs64Composite<T, typename std::enable_if_t<
    (std::is_array_v<T> || std::is_class_v<T> || std::is_union_v<T>) &&
    // VarArgs is technically a composite type, but it's not a normal argument.
    !IsVarArgsV<T> &&
    // Short vectors are also composite types, but don't treat them as one.
    !IsAapcs64ShortVectorV<T>
    >> : public std::true_type
{};

template <typename T>
constexpr bool IsAapcs64CompositeV = IsAapcs64Composite<T>::value;

// Homogeneous Aggregates
// These *should* be any aggregate type which has only one type of member, but
// we can't actually detect that or manipulate that with templates. Instead,
// we approximate that by detecting only arrays with that property.

// An Homogeneous Floating-Point Aggregate (HFA) is an Homogeneous Aggregate
// with a Fundemental Data Type that is a Floating-Point type and at most four
// uniquely addressable members.

template <typename T, typename Enabled=void>
struct IsAapcs64Hfa : public std::false_type {};

template <typename E, size_t N>
struct IsAapcs64Hfa<E[N],
    typename std::enable_if_t<std::is_floating_point_v<E> && N <= 4>> :
    public std::true_type
{};

template <typename T>
constexpr bool IsAapcs64HfaV = IsAapcs64Hfa<T>::value;

// An Homogeneous Short-Vector Aggregate (HVA) is an Homogeneous Aggregate with
// a Fundamental Data Type that is a Short-Vector type and at most four
// uniquely addressable members.

template <typename T, typename Enabled=void>
struct IsAapcs64Hva : public std::false_type {};

template <typename E, size_t N>
struct IsAapcs64Hva<E[N],
    typename std::enable_if_t<IsAapcs64ShortVectorV<E> && N <= 4>> :
    public std::true_type
{};

template <typename T>
constexpr bool IsAapcs64HvaV = IsAapcs64Hva<T>::value;

// A shorthand to test if a type is an HVA or an HFA.
template <typename T, typename Enabled=void>
struct IsAapcs64Hxa : public std::false_type {};

template <typename T>
struct IsAapcs64Hxa<T, typename std::enable_if_t<
    IsAapcs64HfaV<T> || IsAapcs64HvaV<T>>> :
    public std::true_type
{};

template <typename T>
constexpr bool IsAapcs64HxaV = IsAapcs64Hxa<T>::value;

struct Aapcs64ArgumentBase
{
    template <typename T>
    static T
    loadFromStack(ThreadContext *tc, Aapcs64::State &state)
    {
        // The alignment is the larger of 8 or the natural alignment of T.
        size_t align = std::max<size_t>(8, alignof(T));
        // Increase the size to the next multiple of 8.
        size_t size = roundUp(sizeof(T), 8);

        // Align the stack.
        state.nsaa = roundUp(state.nsaa, align);

        // Extract the value from it.
        ConstVPtr<T> val(state.nsaa, tc);

        // Move the nsaa past this argument.
        state.nsaa += size;

        // Return the value we extracted.
        return gtoh(*val, ArmISA::byteOrder(tc));
    }
};


/*
 * Floating point and Short-Vector arguments and return values.
 */

template <typename Float>
struct Argument<Aapcs64, Float, typename std::enable_if_t<
    std::is_floating_point_v<Float> || IsAapcs64ShortVectorV<Float>>> :
    public Aapcs64ArgumentBase
{
    static Float
    get(ThreadContext *tc, Aapcs64::State &state)
    {
        if (state.nsrn <= state.MAX_SRN) {
            RegId id = ArmISA::vecRegClass[state.nsrn++];
            ArmISA::VecRegContainer vc;
            tc->getReg(id, &vc);
            return vc.as<Float>()[0];
        }

        return loadFromStack<Float>(tc, state);
    }
};

template <typename Float>
struct Result<Aapcs64, Float, typename std::enable_if_t<
    std::is_floating_point_v<Float> || IsAapcs64ShortVectorV<Float>>>
{
    static void
    store(ThreadContext *tc, const Float &f)
    {
        RegId id = ArmISA::vecRegClass[0];
        ArmISA::VecRegContainer reg;
        tc->getReg(id, &reg);
        reg.as<Float>()[0] = f;
        tc->setReg(id, &reg);
    }
};


/*
 * Integer arguments and return values.
 */

// This will pick up Addr as well, which should be used for guest pointers.
template <typename Integer>
struct Argument<Aapcs64, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) <= 8)>> :
    public Aapcs64ArgumentBase
{
    static Integer
    get(ThreadContext *tc, Aapcs64::State &state)
    {
        if (state.ngrn <= state.MAX_GRN)
            return tc->getReg(ArmISA::intRegClass[state.ngrn++]);

        // Max out ngrn since we've effectively saturated it.
        state.ngrn = state.MAX_GRN + 1;

        return loadFromStack<Integer>(tc, state);
    }
};

template <typename Integer>
struct Argument<Aapcs64, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) > 8)>> :
    public Aapcs64ArgumentBase
{
    static Integer
    get(ThreadContext *tc, Aapcs64::State &state)
    {
        if (alignof(Integer) == 16 && (state.ngrn % 2))
            state.ngrn++;

        if (sizeof(Integer) == 16 && state.ngrn + 1 <= state.MAX_GRN) {
            Integer low = tc->getReg(ArmISA::intRegClass[state.ngrn++]);
            Integer high = tc->getReg(ArmISA::intRegClass[state.ngrn++]);
            high = high << 64;
            return high | low;
        }

        // Max out ngrn since we've effectively saturated it.
        state.ngrn = state.MAX_GRN + 1;

        return loadFromStack<Integer>(tc, state);
    }
};

template <typename Integer>
struct Result<Aapcs64, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) <= 8)>>
{
    static void
    store(ThreadContext *tc, const Integer &i)
    {
        tc->setReg(ArmISA::int_reg::X0, i);
    }
};

template <typename Integer>
struct Result<Aapcs64, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) > 8)>>
{
    static void
    store(ThreadContext *tc, const Integer &i)
    {
        tc->setReg(ArmISA::int_reg::X0, (uint64_t)i);
        tc->setReg(ArmISA::int_reg::X1, (uint64_t)(i >> 64));
    }
};


/*
 * Homogeneous Floating-Point and Short-Vector Aggregates (HFAs and HVAs)
 * argument and return values.
 */

template <typename T>
struct Aapcs64ArrayType { using Type = void; };

template <typename E, size_t N>
struct Aapcs64ArrayType<E[N]> { using Type = E; };

template <typename HA>
struct Argument<Aapcs64, HA, typename std::enable_if_t<
    IsAapcs64HxaV<HA>>> : public Aapcs64ArgumentBase
{
    static HA
    get(ThreadContext *tc, Aapcs64::State &state)
    {
        using Elem = typename Aapcs64ArrayType<HA>::Type;
        constexpr size_t Count = sizeof(HA) / sizeof(Elem);

        if (state.nsrn + Count - 1 <= state.MAX_SRN) {
            HA ha;
            for (int i = 0; i < Count; i++)
                ha[i] = Argument<Aapcs64, Elem>::get(tc, state);
            return ha;
        }

        // Max out the nsrn since we effectively exhausted it.
        state.nsrn = state.MAX_SRN + 1;

        return loadFromStack<HA>(tc, state);
    }
};

template <typename HA>
struct Result<Aapcs64, HA, typename std::enable_if_t<IsAapcs64HxaV<HA>>>
{
    static HA
    store(ThreadContext *tc, const HA &ha)
    {
        using Elem = typename Aapcs64ArrayType<HA>::Type;
        constexpr size_t Count = sizeof(HA) / sizeof(Elem);

        for (int i = 0; i < Count; i++)
            Result<Aapcs64, Elem>::store(tc, ha[i]);
    }
};


/*
 * Composite arguments and return values which are not HVAs or HFAs.
 */

template <typename Composite>
struct Argument<Aapcs64, Composite, typename std::enable_if_t<
    IsAapcs64CompositeV<Composite> && !IsAapcs64HxaV<Composite>>> :
    public Aapcs64ArgumentBase
{
    static Composite
    get(ThreadContext *tc, Aapcs64::State &state)
    {
        if (sizeof(Composite) > 16) {
            // Composite values larger than 16 which aren't HFAs or HVAs are
            // kept in a buffer, and the argument is actually a pointer to that
            // buffer.
            Addr addr = Argument<Aapcs64, Addr>::get(tc, state);
            ConstVPtr<Composite> composite(addr, tc);
            return gtoh(*composite, ArmISA::byteOrder(tc));
        }

        // The size of Composite must be 16 bytes or less after this point.

        size_t bytes = sizeof(Composite);
        using Chunk = uint64_t;

        const size_t chunk_size = sizeof(Chunk);
        const size_t regs = (bytes + chunk_size - 1) / chunk_size;

        // Can it fit in GPRs?
        if (state.ngrn + regs - 1 <= state.MAX_GRN) {
            std::align_val_t align {alignof(Composite)};
            auto buf = std::unique_ptr<uint8_t[]>(new (align) uint8_t[bytes]);
            for (int i = 0; i < regs; i++) {
                Chunk val = tc->getReg(ArmISA::intRegClass[state.ngrn++]);
                val = htog(val, ArmISA::byteOrder(tc));
                size_t to_copy = std::min(bytes, chunk_size);
                memcpy(buf.get() + i * chunk_size, &val, to_copy);
                bytes -= to_copy;
            }
            return gtoh(*(Composite *)buf.get(), ArmISA::byteOrder(tc));
        }

        // Max out the ngrn since we effectively exhausted it.
        state.ngrn = state.MAX_GRN;

        return loadFromStack<Composite>(tc, state);
    }
};

template <typename Composite>
struct Result<Aapcs64, Composite, typename std::enable_if_t<
    IsAapcs64CompositeV<Composite> && !IsAapcs64HxaV<Composite>>>
{
    static void
    store(ThreadContext *tc, const Composite &c)
    {
        if (sizeof(Composite) > 16) {
            Addr addr = tc->getReg(ArmISA::int_reg::X8);
            VPtr<Composite> composite(addr, tc);
            *composite = htog(c, ArmISA::byteOrder(tc));
            return;
        }

        // The size of Composite must be 16 bytes or less after this point.

        size_t bytes = sizeof(Composite);
        using Chunk = uint64_t;

        const int chunk_size = sizeof(Chunk);
        const int regs = (bytes + chunk_size - 1) / chunk_size;

        Composite cp = htog(c, ArmISA::byteOrder(tc));
        uint8_t *buf = (uint8_t *)&cp;
        for (int i = 0; i < regs; i++) {
            size_t to_copy = std::min<size_t>(bytes, chunk_size);

            Chunk val;
            memcpy(&val, buf, to_copy);
            val = gtoh(val, ArmISA::byteOrder(tc));

            tc->setReg(ArmISA::int_reg::x(i), val);

            bytes -= to_copy;
            buf += to_copy;
        }
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_ARM_AAPCS64_HH__
