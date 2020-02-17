// Copyright (c) 2017 ARM Limited
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
#include <vector>

#include "arch/generic/vec_reg.hh"
#include "base/cprintf.hh"

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
    static constexpr size_t NUM_BITS = Packed ? NumElems :
                                                sizeof(VecElem) * NumElems;

  public:
    /// Container type alias.
    using Container = typename std::conditional<
        Const,
        const VecPredRegContainer<NUM_BITS, Packed>,
        VecPredRegContainer<NUM_BITS, Packed>>::type;

  protected:
    // Alias for this type
    using MyClass = VecPredRegT<VecElem, NumElems, Packed, Const>;
    /// Container corresponding to this view.
    Container& container;

  public:
    VecPredRegT(Container& c) : container(c) {}

    /// Reset the register to an all-false value.
    template<bool Condition = !Const>
    typename std::enable_if<Condition, void>::type
    reset() { container.reset(); }

    /// Reset the register to an all-true value.
    template<bool Condition = !Const>
    typename std::enable_if<Condition, void>::type
    set() { container.set(); }

    template<bool Condition = !Const>
    typename std::enable_if<Condition, MyClass&>::type
    operator=(const MyClass& that)
    {
        container = that.container;
        return *this;
    }

    const bool&
    operator[](size_t idx) const
    {
        return container[idx * (Packed ? 1 : sizeof(VecElem))];
    }

    template<bool Condition = !Const>
    typename std::enable_if<Condition, bool&>::type
    operator[](size_t idx)
    {
        return container[idx * (Packed ? 1 : sizeof(VecElem))];
    }

    /// Return an element of the predicate register as it appears
    /// in the raw (untyped) internal representation
    uint8_t
    get_raw(size_t idx) const
    {
        return container.get_bits(idx * (Packed ? 1 : sizeof(VecElem)),
                (Packed ? 1 : sizeof(VecElem)));
    }

    /// Write a raw value in an element of the predicate register
    template<bool Condition = !Const>
    typename std::enable_if<Condition, void>::type
    set_raw(size_t idx, uint8_t val)
    {
        container.set_bits(idx * (Packed ? 1 : sizeof(VecElem)),
                (Packed ? 1 : sizeof(VecElem)), val);
    }

    /// Equality operator, required to compare thread contexts.
    template<typename VE2, size_t NE2, bool P2, bool C2>
    bool
    operator==(const VecPredRegT<VE2, NE2, P2, C2>& that) const
    {
        return container == that.container;
    }

    /// Inequality operator, required to compare thread contexts.
    template<typename VE2, size_t NE2, bool P2, bool C2>
    bool
    operator!=(const VecPredRegT<VE2, NE2, P2, C2>& that) const
    {
        return !operator==(that);
    }

    friend std::ostream&
    operator<<(std::ostream& os, const MyClass& p)
    {
        // 0-sized is not allowed
        os << '[' << p.container[0];
        for (int i = 0; i < p.NUM_BITS; ++i) {
            os << " " << (p.container[i] ? 1 : 0);
        }
        os << ']';
        return os;
    }

    /// Returns a string representation of the register content.
    const std::string print() const { return csprintf("%s", *this); }

    /// Returns true if the first active element of the register is true.
    /// @param mask Input mask used to filter the predicates to be tested.
    /// @param actual_num_elems Actual number of vector elements considered for
    /// the test (corresponding to the current vector length).
    template <bool MC>
    bool
    firstActive(const VecPredRegT<VecElem, NumElems, Packed, MC>& mask,
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
    noneActive(const VecPredRegT<VecElem, NumElems, Packed, MC>& mask,
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
    lastActive(const VecPredRegT<VecElem, NumElems, Packed, MC>& mask,
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
    static_assert(NumBits > 0,
                  "Size of a predicate register must be > 0");

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

    MyClass&
    operator=(const MyClass& that)
    {
        if (&that == this)
            return *this;
        container = that.container;
        return *this;
    }

    /// Required for de-serialization.
    MyClass&
    operator=(const std::vector<uint8_t>& that)
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
    template<size_t N2, bool P2>
    inline bool
    operator==(const VecPredRegContainer<N2, P2>& that) const
    {
        return NumBits == N2 && Packed == P2 && container == that.container;
    }

    /// Inequality operator, required to compare thread contexts.
    template<size_t N2, bool P2>
    bool
    operator!=(const VecPredRegContainer<N2, P2>& that) const
    {
        return !operator==(that);
    }

    /// Returns a reference to a specific element of the internal container.
    bool& operator[](size_t idx) { return container[idx]; }

    /// Returns a const reference to a specific element of the internal
    /// container.
    const bool& operator[](size_t idx) const { return container[idx]; }

    /// Returns a subset of bits starting from a specific element in the
    /// container.
    uint8_t
    get_bits(size_t idx, uint8_t nbits) const
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
    set_bits(size_t idx, uint8_t nbits, uint8_t bval)
    {
        assert(nbits > 0 && nbits <= 8 && (idx + nbits - 1) < NumBits);
        for (int i = 0; i < nbits; ++i, ++idx) {
            container[idx] = bval & 1;
            bval >>= 1;
        }
    }

    /// Returns a string representation of the register content.
    const std::string print() const { return csprintf("%s", *this); }

    friend std::ostream&
    operator<<(std::ostream& os, const MyClass& v)
    {
        for (auto b: v.container) {
            os << csprintf("%d", b);
        }
        return os;
    }

    /// Create a view of this container.
    ///
    /// If NumElems is provided, the size of the container is bounds-checked,
    /// otherwise the size is inferred from the container size.
    /// @tparam VecElem Type of the vector elements.
    /// @tparam NumElems Number of vector elements making up the view.
    /// @{
    template <typename VecElem,
              size_t NumElems = (Packed ? NumBits : NumBits / sizeof(VecElem))>
    VecPredRegT<VecElem, NumElems, Packed, true> as() const
    {
        static_assert((Packed && NumElems <= NumBits) ||
                      (!Packed &&
                       NumBits % sizeof(VecElem) == 0 &&
                       sizeof(VecElem) * NumElems <= NumBits),
                      "Container size incompatible with view size");
        return VecPredRegT<VecElem, NumElems, Packed, true>(*this);
    }

    template <typename VecElem,
              size_t NumElems = (Packed ? NumBits : NumBits / sizeof(VecElem))>
    VecPredRegT<VecElem, NumElems, Packed, false> as()
    {
        static_assert((Packed && NumElems <= NumBits) ||
                      (!Packed &&
                       NumBits % sizeof(VecElem) == 0 &&
                       sizeof(VecElem) * NumElems <= NumBits),
                      "Container size incompatible with view size");
        return VecPredRegT<VecElem, NumElems, Packed, false>(*this);
    }
    /// @}
};

/// Helper functions used for serialization/de-serialization
template <size_t NumBits, bool Packed>
inline bool
to_number(const std::string& value, VecPredRegContainer<NumBits, Packed>& p)
{
    int i = 0;
    for (const auto& c: value) {
        p[i] = (c == '1');
    }
    return true;
}

/// Dummy type aliases and constants for architectures that do not implement
/// vector predicate registers.
/// @{
constexpr bool DummyVecPredRegHasPackedRepr = false;
using DummyVecPredReg = VecPredRegT<DummyVecElem, DummyNumVecElemPerVecReg,
                                    DummyVecPredRegHasPackedRepr, false>;
using DummyConstVecPredReg = VecPredRegT<DummyVecElem,
                                         DummyNumVecElemPerVecReg,
                                         DummyVecPredRegHasPackedRepr, true>;
using DummyVecPredRegContainer = DummyVecPredReg::Container;
constexpr size_t DummyVecPredRegSizeBits = 8;
/// @}

#endif  // __ARCH_GENERIC_VEC_PRED_REG_HH__
