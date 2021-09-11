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

#ifndef __ARCH_ARM_AAPCS32_HH__
#define __ARCH_ARM_AAPCS32_HH__

#include <algorithm>
#include <array>
#include <type_traits>
#include <utility>

#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/vec.hh"
#include "arch/arm/utility.hh"
#include "base/intmath.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "sim/full_system.hh"
#include "sim/guest_abi.hh"
#include "sim/proxy_ptr.hh"

namespace gem5
{

class ThreadContext;

struct Aapcs32
{
    struct State
    {
        bool stackUsed=false; // Whether anything has been put on the stack.

        int ncrn=0; // Next general purpose register number.
        Addr nsaa; // Next stacked argument address.

        // The maximum allowed general purpose register number.
        static const int MAX_CRN = 3;

        Addr retAddr=0;

        explicit State(const ThreadContext *tc) :
            nsaa(tc->getReg(ArmISA::int_reg::Spx))
        {}
    };
};

GEM5_DEPRECATED_NAMESPACE(GuestABI, guest_abi);
namespace guest_abi
{

/*
 * Composite Types
 */

template <typename T, typename Enabled=void>
struct IsAapcs32Composite : public std::false_type {};

template <typename T>
struct IsAapcs32Composite<T, typename std::enable_if_t<
    (std::is_array_v<T> || std::is_class_v<T> || std::is_union_v<T>) &&
    // VarArgs is technically a composite type, but it's not a normal argument.
    !IsVarArgsV<T>
    >> : public std::true_type
{};

template <typename T>
constexpr bool IsAapcs32CompositeV = IsAapcs32Composite<T>::value;

// Homogeneous Aggregates
// These *should* be any aggregate type which has only one type of member, but
// we can't actually detect that or manipulate that with templates. Instead,
// we approximate that by detecting only arrays with that property.

template <typename T, std::size_t count, typename Enabled=void>
using Aapcs32HomogeneousAggregate = T[count];

template <typename T>
struct IsAapcs32HomogeneousAggregate : public std::false_type {};

template <typename E, size_t N>
struct IsAapcs32HomogeneousAggregate<E[N]> : public std::true_type {};

template <typename T>
constexpr bool IsAapcs32HomogeneousAggregateV =
    IsAapcs32HomogeneousAggregate<T>::value;

struct Aapcs32ArgumentBase
{
    template <typename T>
    static T
    loadFromStack(ThreadContext *tc, Aapcs32::State &state)
    {
        state.stackUsed = true;

        // The alignment is the larger of 4 or the natural alignment of T.
        size_t align = std::max<size_t>(4, alignof(T));
        // Increase the size to the next multiple of 4.
        size_t size = roundUp(sizeof(T), 4);

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
 * Integer arguments and return values.
 */

template <typename Integer>
struct Result<Aapcs32, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) < sizeof(uint32_t))>>
{
    static void
    store(ThreadContext *tc, const Integer &i)
    {
        uint32_t val = std::is_signed_v<Integer> ?
                sext<sizeof(Integer) * 8>(i) : i;
        tc->setReg(ArmISA::int_reg::R0, val);
    }
};

template <typename Integer>
struct Result<Aapcs32, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) == sizeof(uint32_t))>>
{
    static void
    store(ThreadContext *tc, const Integer &i)
    {
        tc->setReg(ArmISA::int_reg::R0, (uint32_t)i);
    }
};

template <typename Integer>
struct Result<Aapcs32, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) == sizeof(uint64_t))>>
{
    static void
    store(ThreadContext *tc, const Integer &i)
    {
        if (ArmISA::byteOrder(tc) == ByteOrder::little) {
            tc->setReg(ArmISA::int_reg::R0, (uint32_t)(i >> 0));
            tc->setReg(ArmISA::int_reg::R1, (uint32_t)(i >> 32));
        } else {
            tc->setReg(ArmISA::int_reg::R0, (uint32_t)(i >> 32));
            tc->setReg(ArmISA::int_reg::R1, (uint32_t)(i >> 0));
        }
    }
};

template <typename Integer>
struct Argument<Aapcs32, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) <= sizeof(uint32_t))
    >> : public Aapcs32ArgumentBase
{
    static Integer
    get(ThreadContext *tc, Aapcs32::State &state)
    {
        if (state.ncrn <= state.MAX_CRN) {
            return tc->getReg(ArmISA::intRegClass[state.ncrn++]);
        }

        // Max out the ncrn since we effectively exhausted it.
        state.ncrn = state.MAX_CRN + 1;

        return loadFromStack<Integer>(tc, state);
    }
};

template <typename Integer>
struct Argument<Aapcs32, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer> && (sizeof(Integer) > sizeof(uint32_t))
    >> : public Aapcs32ArgumentBase
{
    static Integer
    get(ThreadContext *tc, Aapcs32::State &state)
    {
        if (alignof(Integer) == 8 && (state.ncrn % 2))
            state.ncrn++;

        if (sizeof(Integer) == sizeof(uint64_t) &&
                state.ncrn + 1 <= state.MAX_CRN) {
            Integer low, high;
            if (ArmISA::byteOrder(tc) == ByteOrder::little) {
                low = tc->getReg(ArmISA::intRegClass[state.ncrn++]) & mask(32);
                high = tc->getReg(ArmISA::intRegClass[state.ncrn++]) &
                    mask(32);
            } else {
                high = tc->getReg(ArmISA::intRegClass[state.ncrn++]) &
                    mask(32);
                low = tc->getReg(ArmISA::intRegClass[state.ncrn++]) & mask(32);
            }
            return low | (high << 32);
        }

        // Max out the ncrn since we effectively exhausted it.
        state.ncrn = state.MAX_CRN + 1;

        return loadFromStack<Integer>(tc, state);
    }
};


/*
 * Floating point and Short-Vector arguments and return values.
 */

template <typename Float>
struct Result<Aapcs32, Float, typename std::enable_if_t<
    std::is_floating_point_v<Float>>>
{
    static void
    store(ThreadContext *tc, const Float &f, Aapcs32::State &state)
    {
        auto i = floatToBits(f);
        storeResult<Aapcs32, decltype(i)>(tc, i, state);
    };
};

template <typename Float>
struct Argument<Aapcs32, Float, typename std::enable_if_t<
    std::is_floating_point_v<Float>>> : public Aapcs32ArgumentBase
{
    static Float
    get(ThreadContext *tc, Aapcs32::State &state)
    {
        if (sizeof(Float) == sizeof(uint32_t)) {
            return bitsToFloat32(
                    getArgument<Aapcs32, uint32_t>(tc, state));
        } else {
            return bitsToFloat64(
                    getArgument<Aapcs32, uint64_t>(tc, state));
        }
    }
};


/*
 * Composite arguments and return values.
 */

template <typename Composite>
struct Result<Aapcs32, Composite, typename std::enable_if_t<
    IsAapcs32CompositeV<Composite>>>
{
    static void
    store(ThreadContext *tc, const Composite &composite,
          Aapcs32::State &state)
    {
        if (sizeof(Composite) <= sizeof(uint32_t)) {
            Composite cp = htog(composite, ArmISA::byteOrder(tc));
            uint32_t val;
            memcpy((void *)&val, (void *)&cp, sizeof(Composite));
            val = gtoh(val, ArmISA::byteOrder(tc));
            tc->setReg(ArmISA::int_reg::R0, val);
        } else {
            VPtr<Composite> cp(state.retAddr, tc);
            *cp = htog(composite, ArmISA::byteOrder(tc));
        }
    }

    static void
    prepare(ThreadContext *tc, Aapcs32::State &state)
    {
        if (sizeof(Composite) > sizeof(uint32_t))
            state.retAddr = tc->getReg(ArmISA::intRegClass[state.ncrn++]);
    }
};

template <typename Composite>
struct Argument<Aapcs32, Composite, typename std::enable_if_t<
    IsAapcs32CompositeV<Composite>>> :
    public Aapcs32ArgumentBase
{
    static Composite
    get(ThreadContext *tc, Aapcs32::State &state)
    {
        size_t bytes = sizeof(Composite);
        using Chunk = uint32_t;

        const int chunk_size = sizeof(Chunk);
        const int regs = (bytes + chunk_size - 1) / chunk_size;

        if (bytes <= chunk_size) {
            if (state.ncrn++ <= state.MAX_CRN) {
                alignas(alignof(Composite)) uint32_t val =
                    tc->getReg(ArmISA::intRegClass[state.ncrn++]);
                val = htog(val, ArmISA::byteOrder(tc));
                return gtoh(*(Composite *)&val, ArmISA::byteOrder(tc));
            }
        }

        if (alignof(Composite) == 8 && (state.ncrn % 2))
            state.ncrn++;

        if (state.ncrn + regs - 1 <= state.MAX_CRN) {
            alignas(alignof(Composite)) uint8_t buf[bytes];
            for (int i = 0; i < regs; i++) {
                Chunk val = tc->getReg(ArmISA::intRegClass[state.ncrn++]);
                val = htog(val, ArmISA::byteOrder(tc));
                size_t to_copy = std::min<size_t>(bytes, chunk_size);
                memcpy(buf + i * chunk_size, &val, to_copy);
                bytes -= to_copy;
            }
            return gtoh(*(Composite *)buf, ArmISA::byteOrder(tc));
        }

        if (!state.stackUsed && state.ncrn <= state.MAX_CRN) {
            alignas(alignof(Composite)) uint8_t buf[bytes];

            int offset = 0;
            while (state.ncrn <= state.MAX_CRN) {
                Chunk val = tc->getReg(ArmISA::intRegClass[state.ncrn++]);
                val = htog(val, ArmISA::byteOrder(tc));
                size_t to_copy = std::min<size_t>(bytes, chunk_size);
                memcpy(buf + offset, &val, to_copy);
                offset += to_copy;
                bytes -= to_copy;
            }

            if (bytes) {
                (FullSystem ? TranslatingPortProxy(tc) :
                    SETranslatingPortProxy(tc)).readBlob(
                    state.nsaa, buf, bytes);

                state.stackUsed = true;
                state.nsaa += roundUp(bytes, 4);
                state.ncrn = state.MAX_CRN + 1;
            }

            return gtoh(*(Composite *)buf, ArmISA::byteOrder(tc));
        }

        state.ncrn = state.MAX_CRN + 1;

        return loadFromStack<Composite>(tc, state);
    }
};

} // namespace guest_abi


/*
 * VFP ABI variant.
 */

struct Aapcs32Vfp : public Aapcs32
{
    struct State : public Aapcs32::State
    {
        bool variadic=false; // Whether this function is variadic.

        // Whether the various single and double precision registers have
        // been allocated.
        std::array<bool, 16> s;
        std::array<bool, 8> d;

        explicit State(const ThreadContext *tc) : Aapcs32::State(tc)
        {
            s.fill(false);
            d.fill(false);
        }

        int
        allocate(float, int count)
        {
            int last = 0;
            for (int i = 0; i <= s.size() - count; i++) {
                if (s[i]) {
                    last = i + 1;
                    continue;
                }
                if (i - last + 1 == count) {
                    for (int j = 0; j < count; j++) {
                        s[last + j] = true;
                        d[(last + j) / 2] = true;
                    }
                    return last;
                }
            }
            s.fill(true);
            d.fill(true);
            return -1;
        }

        int
        allocate(double, int count)
        {
            int last = 0;
            for (int i = 0; i <= d.size() - count; i++) {
                if (d[i]) {
                    last = i + 1;
                    continue;
                }
                if (i - last + 1 == count) {
                    for (int j = 0; j < count; j++) {
                        d[last + j] = true;
                        s[(last + j) * 2] = true;
                        s[(last + j) * 2 + 1] = true;
                    }
                    return last;
                }
            }
            s.fill(true);
            d.fill(true);
            return -1;
        }
    };
};

GEM5_DEPRECATED_NAMESPACE(GuestABI, guest_abi);
namespace guest_abi
{

/*
 * Integer arguments and return values.
 */

template <typename Integer>
struct Result<Aapcs32Vfp, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer>>> : public Result<Aapcs32, Integer>
{};

template <typename Integer>
struct Argument<Aapcs32Vfp, Integer, typename std::enable_if_t<
    std::is_integral_v<Integer>>> : public Argument<Aapcs32, Integer>
{};


/*
 * Floating point arguments and return values.
 */

template <typename Float>
struct Result<Aapcs32Vfp, Float, typename std::enable_if_t<
    std::is_floating_point_v<Float>>>
{
    static void
    store(ThreadContext *tc, const Float &f, Aapcs32Vfp::State &state)
    {
        if (state.variadic) {
            storeResult<Aapcs32, Float>(tc, f, state);
            return;
        }

        auto bytes = floatToBits(f);
        auto *vec_elems = static_cast<ArmISA::VecElem *>(&bytes);
        constexpr int chunks = sizeof(Float) / sizeof(ArmISA::VecElem);
        for (int chunk = 0; chunk < chunks; chunk++)
            tc->setReg(ArmISA::vecElemClass[chunk], vec_elems[chunk]);
    };
};

template <typename Float>
struct Argument<Aapcs32Vfp, Float, typename std::enable_if_t<
    std::is_floating_point_v<Float>>> : public Aapcs32ArgumentBase
{
    static Float
    get(ThreadContext *tc, Aapcs32Vfp::State &state)
    {
        if (state.variadic)
            return getArgument<Aapcs32, Float>(tc, state);

        const int index = state.allocate(Float{}, 1);

        if (index < 0)
            return loadFromStack<Float>(tc, state);

        decltype(floatToBits(Float{})) result;
        auto *vec_elems = static_cast<ArmISA::VecElem *>(&result);

        constexpr int chunks = sizeof(Float) / sizeof(ArmISA::VecElem);
        for (int chunk = 0; chunk < chunks; chunk++)
            vec_elems[chunk] = tc->getReg(ArmISA::vecElemClass[chunk]);

        return bitsToFloat(result);
    }
};


/*
 * Composite arguments and return values which are not Homogeneous Aggregates.
 */

template <typename Composite>
struct Result<Aapcs32Vfp, Composite, typename std::enable_if_t<
    IsAapcs32CompositeV<Composite> &&
    !IsAapcs32HomogeneousAggregateV<Composite>>> :
    public Result<Aapcs32, Composite>
{};

template <typename Composite>
struct Argument<Aapcs32Vfp, Composite, typename std::enable_if_t<
    IsAapcs32CompositeV<Composite> &&
    !IsAapcs32HomogeneousAggregateV<Composite>>> :
    public Argument<Aapcs32, Composite>
{};


/*
 * Homogeneous Aggregate argument and return values.
 */

template <typename T>
struct Aapcs32ArrayType { using Type = void; };

template <typename E, size_t N>
struct Aapcs32ArrayType<E[N]> { using Type = E; };

template <typename HA>
struct Argument<Aapcs32Vfp, HA, typename std::enable_if_t<
    IsAapcs32HomogeneousAggregateV<HA>>> :
    public Aapcs32ArgumentBase
{
    static bool
    useBaseABI(Aapcs32Vfp::State &state)
    {
        using Elem = typename Aapcs32ArrayType<HA>::Type;
        constexpr size_t Count = sizeof(HA) / sizeof(Elem);
        return state.variadic || !std::is_floating_point_v<Elem> ||
            Count > 4;
    }

    static HA
    get(ThreadContext *tc, Aapcs32Vfp::State &state)
    {
        using Elem = typename Aapcs32ArrayType<HA>::Type;
        constexpr size_t Count = sizeof(HA) / sizeof(Elem);

        if (useBaseABI(state))
            return getArgument<Aapcs32, HA>(tc, state);

        const int base = state.allocate(Elem{}, Count);
        if (base >= 0) {
            constexpr int lane_per_reg = 16 / sizeof(Elem);
            HA ha;
            for (int i = 0; i < Count; i++) {
                const int index = base + i;
                const int reg = index / lane_per_reg;
                const int lane = index % lane_per_reg;

                RegId id = ArmISA::vecRegClass[reg];
                ArmISA::VecRegContainer val;
                tc->getReg(id, &val);
                ha[i] = val.as<Elem>()[lane];
            }
            return ha;
        }

        return loadFromStack<HA>(tc, state);
    }

    static void
    prepare(ThreadContext *tc, Aapcs32Vfp::State &state)
    {
        if (useBaseABI(state))
            return Argument<Aapcs32, HA>::prepare(tc, state);
    }
};

template <typename HA>
struct Result<Aapcs32Vfp, HA,
    typename std::enable_if_t<IsAapcs32HomogeneousAggregateV<HA>>>
{
    static bool
    useBaseABI(Aapcs32Vfp::State &state)
    {
        using Elem = typename Aapcs32ArrayType<HA>::Type;
        constexpr size_t Count = sizeof(HA) / sizeof(Elem);
        return state.variadic || !std::is_floating_point_v<Elem> ||
            Count > 4;
    }

    static HA
    store(ThreadContext *tc, const HA &ha, Aapcs32Vfp::State &state)
    {
        using Elem = typename Aapcs32ArrayType<HA>::Type;
        constexpr size_t Count = sizeof(HA) / sizeof(Elem);

        if (useBaseABI(state)) {
             storeResult<Aapcs32, HA>(tc, ha, state);
             return;
        }

        constexpr int lane_per_reg = 16 / sizeof(Elem);
        for (int i = 0; i < Count; i++) {
            const int reg = i / lane_per_reg;
            const int lane = i % lane_per_reg;

            RegId id = ArmISA::vecRegClass[reg];
            ArmISA::VecRegContainer val;
            tc->getReg(id, &val);
            val.as<Elem>()[lane] = ha[i];
            tc->setReg(id, &val);
        }
    }

    static void
    prepare(ThreadContext *tc, Aapcs32Vfp::State &state)
    {
        if (useBaseABI(state))
            return Result<Aapcs32, HA>::prepare(tc, state);
    }
};


/*
 * Varargs
 */

template <typename ...Types>
struct Argument<Aapcs32Vfp, VarArgs<Types...>>
{
    static VarArgs<Types...>
    get(ThreadContext *tc, typename Aapcs32Vfp::State &state)
    {
        state.variadic = true;
        return getArgument<Aapcs32, VarArgs<Types...>>(tc, state);
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_ARM_AAPCS32_HH__
