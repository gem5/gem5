// Copyright (c) 2017, 2021 Arm Limited
// All rights reserved
//
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __ARCH_GENERIC_VEC_PRED_REG_HH__
#define __ARCH_GENERIC_VEC_PRED_REG_HH__

#include <array>
#include <cassert>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "base/cprintf.hh"
#include "base/types.hh"
#include "sim/serialize_handlers.hh"

namespace gem5
{

template <size_t NumBits, bool Packed>
class VecPredRegContainer;

/// Predicate register view.
///
/// This generic class implements the View in an MVC pattern, similarly to
/// @see VecRegT. Since predicates are mainly used in conjunction with vectors
/// to specify which lanes are active in a vector operation, the class is
/// templated on the vector element type to simplify ISA definitions.
/// @tparam VecElem Type of the vector elements.
/// @tparam NumElems Number of vector elements making up the view.
/// @tparam Packed True if the predicate register relies on a packed
/// representation, i.e. adjacent bits refer to different vector elements
/// irrespective of the vector element size (e.g. this is the case for
/// AVX-512). If false, the predicate register relies on an unpacked
/// representation, where each bit refers to the corresponding byte in a vector
/// register (e.g. this is the case for ARM SVE).
/// @tparam Const True if the underlying container can be modified through
/// the view.
template <typename VecElem, size_t NumElems, bool Packed, bool Const>
class VecPredRegT
{
  protected:
    /// Size of the register in bits.
    static constexpr size_t NUM_BITS =
        Packed ? NumElems : sizeof(VecElem) * NumElems;

  public:
    /// Container type alias.
    using Container = typename std::conditional_t<
        Const, const VecPredRegContainer<NUM_BITS, Packed>,
        VecPredRegContainer<NUM_BITS, Packed>>;

  protected:
    // Alias for this type
    using MyClass = VecPredRegT<VecElem, NumElems, Packed, Const>;
    /// Container corresponding to this view.
    Container &container;

  public:
    VecPredRegT(Container &c) : container(c) {}

    /// Reset the register to an all-false value.
    template <bool Condition = !Const>
    std::enable_if_t<Condition>
    reset()
    {
        container.reset();
    }

    /// Reset the register to an all-true value.
    template <bool Condition = !Const>
    std::enable_if_t<Condition>
    set()
    {
        container.set();
    }

    template <bool Condition = !Const>
    std::enable_if_t<Condition, MyClass &>
    operator=(const MyClass &that)
    {
        container = that.container;
        return *this;
    }

    const bool &
    operator[](size_t idx) const
    {
        return container[idx * (Packed ? 1 : sizeof(VecElem))];
    }

    template <bool Condition = !Const>
    std::enable_if_t<Condition, bool &>
    operator[](size_t idx)
    {
        return container[idx * (Packed ? 1 : sizeof(VecElem))];
    }

    /// Return an element of the predicate register as it appears
    /// in the raw (untyped) internal representation
    uint8_t
    getRaw(size_t idx) const
    {
        return container.getBits(idx * (Packed ? 1 : sizeof(VecElem)),
                                 (Packed ? 1 : sizeof(VecElem)));
    }

    /// Write a raw value in an element of the predicate register
    template <bool Condition = !Const>
    std::enable_if_t<Condition>
    setRaw(size_t idx, uint8_t val)
    {
        container.setBits(idx * (Packed ? 1 : sizeof(VecElem)),
                          (Packed ? 1 : sizeof(VecElem)), val);
    }

    /// Equality operator, required to compare thread contexts.
    template <typename VE2, size_t NE2, bool P2, bool C2>
    bool
    operator==(const VecPredRegT<VE2, NE2, P2, C2> &that) const
    {
        return container == that.container;
    }

    /// Inequality operator, required to compare thread contexts.
    template <typename VE2, size_t NE2, bool P2, bool C2>
    bool
    operator!=(const VecPredRegT<VE2, NE2, P2, C2> &that) const
    {
        return !operator==(that);
    }

    friend std::ostream &
    operator<<(std::ostream &os, const MyClass &p)
    {
        // Size must be greater than 0.
        for (int i = 0; i < NUM_BITS; i++)
            ccprintf(os, "%s%d", i ? " " : "[", (int)p.container[i]);
        ccprintf(os, "]");
        return os;
    }

    /// Returns true if the first active element of the register is true.
    /// @param mask Input mask used to filter the predicates to be tested.
    /// @param actual_num_elems Actual number of vector elements considered for
    /// the test (corresponding to the current vector length).
    template <bool MC>
    bool
    firstActive(const VecPredRegT<VecElem, NumElems, Packed, MC> &mask,
                size_t actual_num_elems) const
    {
        assert(actual_num_elems <= NumElems);
        for (int i = 0; i < actual_num_elems; ++i) {
            if (mask[i]) {
                return (*this)[i];
            }
        }
        return false;
    }

    /// Returns true if there are no active elements in the register.
    /// @param mask Input mask used to filter the predicates to be tested.
    /// @param actual_num_elems Actual number of vector elements considered for
    /// the test (corresponding to the current vector length).
    template <bool MC>
    bool
    noneActive(const VecPredRegT<VecElem, NumElems, Packed, MC> &mask,
               size_t actual_num_elems) const
    {
        assert(actual_num_elems <= NumElems);
        for (int i = 0; i < actual_num_elems; ++i) {
            if (mask[i] && operator[](i)) {
                return false;
            }
        }
        return true;
    }

    /// Returns true if the last active element of the register is true.
    /// @param mask Input mask used to filter the predicates to be tested.
    /// @param actual_num_elems Actual number of vector elements considered for
    /// the test (corresponding to the current vector length).
    template <bool MC>
    bool
    lastActive(const VecPredRegT<VecElem, NumElems, Packed, MC> &mask,
               size_t actual_num_elems) const
    {
        assert(actual_num_elems <= NumElems);
        for (int i = actual_num_elems - 1; i >= 0; --i) {
            if (mask[i]) {
                return operator[](i);
            }
        }
        return false;
    }
};

/// Generic predicate register container.
///
/// This generic class implements the Model in an MVC pattern, similarly to
/// @see VecRegContainer.
/// @tparam NumBits Size of the container in bits.
/// @tparam Packed See @VecRegT.
template <size_t NumBits, bool Packed>
class VecPredRegContainer
{
    static_assert(NumBits > 0, "Size of a predicate register must be > 0");

  public:
    static constexpr size_t NUM_BITS = NumBits;
    using Container = std::array<bool, NumBits>;

  private:
    Container container;
    // Alias for this type
    using MyClass = VecPredRegContainer<NumBits, Packed>;

  public:
    VecPredRegContainer() {}

    VecPredRegContainer(const VecPredRegContainer &) = default;

    MyClass &
    operator=(const MyClass &that)
    {
        if (&that == this)
            return *this;
        container = that.container;
        return *this;
    }

    /// Required for de-serialization.
    MyClass &
    operator=(const std::vector<uint8_t> &that)
    {
        assert(that.size() == NUM_BITS);
        std::copy(that.begin(), that.end(), container.begin());
        return *this;
    }

    /// Resets the predicate register to an all-false register.
    void
    reset()
    {
        container.fill(false);
    }

    /// Sets the predicate register to an all-true value.
    void
    set()
    {
        container.fill(true);
    }

    /// Equality operator, required to compare thread contexts.
    template <size_t N2, bool P2>
    inline bool
    operator==(const VecPredRegContainer<N2, P2> &that) const
    {
        return NumBits == N2 && Packed == P2 && container == that.container;
    }

    /// Inequality operator, required to compare thread contexts.
    template <size_t N2, bool P2>
    bool
    operator!=(const VecPredRegContainer<N2, P2> &that) const
    {
        return !operator==(that);
    }

    /// Returns a reference to a specific element of the internal container.
    bool &
    operator[](size_t idx)
    {
        return container[idx];
    }

    /// Returns a const reference to a specific element of the internal
    /// container.
    const bool &
    operator[](size_t idx) const
    {
        return container[idx];
    }

    /// Returns a subset of bits starting from a specific element in the
    /// container.
    uint8_t
    getBits(size_t idx, uint8_t nbits) const
    {
        assert(nbits > 0 && nbits <= 8 && (idx + nbits - 1) < NumBits);
        uint8_t v = 0;
        idx = idx + nbits - 1;
        for (int i = 0; i < nbits; ++i, --idx) {
            v <<= 1;
            v |= container[idx];
        }
        return v;
    }

    /// Set a subset of bits starting from a specific element in the
    /// container.
    void
    setBits(size_t idx, uint8_t nbits, uint8_t bval)
    {
        assert(nbits > 0 && nbits <= 8 && (idx + nbits - 1) < NumBits);
        for (int i = 0; i < nbits; ++i, ++idx) {
            container[idx] = bval & 1;
            bval >>= 1;
        }
    }

    friend std::ostream &
    operator<<(std::ostream &os, const MyClass &p)
    {
        // Size must be greater than 0.
        for (int i = 0; i < NumBits; i++)
            ccprintf(os, "%s%d", i ? " " : "[", (int)p.container[i]);
        ccprintf(os, "]");
        return os;
    }

    friend ShowParam<VecPredRegContainer<NumBits, Packed>>;

    /// Create a view of this container.
    ///
    /// @tparam VecElem Type of the vector elements.
    /// @{
    template <typename VecElem>
    auto
    as() const
    {
        static_assert(NumBits % sizeof(VecElem) == 0,
                      "Container size incompatible with view size.");
        return VecPredRegT < VecElem,
               Packed ? NumBits : (NumBits / sizeof(VecElem)), Packed,
               true > (*this);
    }

    template <typename VecElem>
    auto
    as()
    {
        static_assert(NumBits % sizeof(VecElem) == 0,
                      "Container size incompatible with view size.");
        return VecPredRegT < VecElem,
               Packed ? NumBits : (NumBits / sizeof(VecElem)), Packed,
               false > (*this);
    }

    /// @}
};

template <size_t NumBits, bool Packed>
struct ParseParam<VecPredRegContainer<NumBits, Packed>>
{
    static bool
    parse(const std::string &s, VecPredRegContainer<NumBits, Packed> &value)
    {
        int i = 0;
        for (const auto &c : s)
            value[i++] = (c == '1');
        return true;
    }
};

template <size_t NumBits, bool Packed>
struct ShowParam<VecPredRegContainer<NumBits, Packed>>
{
    static void
    show(std::ostream &os, const VecPredRegContainer<NumBits, Packed> &value)
    {
        for (auto b : value.container)
            ccprintf(os, "%d", b);
    }
};

/// Dummy type aliases and constants for architectures that do not implement
/// vector predicate registers.
/// @{
struct DummyVecPredRegContainer
{
    RegVal filler = 0;

    bool
    operator==(const DummyVecPredRegContainer &d) const
    {
        return true;
    }

    bool
    operator!=(const DummyVecPredRegContainer &d) const
    {
        return true;
    }

    template <typename VecElem>
    VecElem *
    as()
    {
        return nullptr;
    }
};

template <>
struct ParseParam<DummyVecPredRegContainer>
{
    static bool
    parse(const std::string &s, DummyVecPredRegContainer &value)
    {
        return false;
    }
};

static_assert(sizeof(DummyVecPredRegContainer) == sizeof(RegVal));

static inline std::ostream &
operator<<(std::ostream &os, const DummyVecPredRegContainer &d)
{
    return os;
}

/// @}

} // namespace gem5

#endif // __ARCH_GENERIC_VEC_PRED_REG_HH__
