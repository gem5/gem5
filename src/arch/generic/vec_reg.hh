/*
 * Copyright (c) 2015-2018 ARM Limited
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

/** \file arch/generic/vec_reg.hh
 * Vector Registers layout specification.
 *
 * This register type is to be used to model the SIMD registers.
 * It takes into account the possibility that different architectural names
 * may overlap (like for ARMv8 AArch32 for example).
 *
 * The design is having a basic vector register container that holds the
 * bytes, unaware of anything else. This is implemented by VecRegContainer.
 * As the (maximum) length of the physical vector register is a compile-time
 * constant, it is defined as a template parameter.
 *
 * This file also describes two views of the container that have semantic
 * information about the bytes. The first of this views is VecRegT.
 *    A VecRegT is a view of a VecRegContainer (by reference). The VecRegT has
 *    a type (VecElem) to which bytes are casted, and the amount of such
 *    elements that the vector contains (NumElems). The size of a view,
 *    calculated as sizeof(VecElem) * NumElems must match the size of the
 *    underlying container. As VecRegT has some degree of type information it
 *    has vector semantics, and defines the index operator ([]) to get
 *    references to particular bytes understood as a VecElem.
 * The second view of a container implemented in this file is VecLaneT, which
 * is a view of a subset of the container.
 *    A VecLaneT is a view of a lane of a vector register, where a lane is
 *    identified by a type (VecElem) and an index (although the view is
 *    unaware of its index). Operations on the lane are directly applied to
 *    the corresponding bytes of the underlying VecRegContainer through a
 *    reference.
 *
 * The intended usage is requesting views to the VecRegContainer via the
 * member 'as' for VecRegT and the member 'laneView' for VecLaneT. Kindly
 * find an example of usage in the following.
 *
 *
 * // We declare 512 bits vectors
 * using Vec512 = VecRegContainer<64>;
 * ...
 * // We implement the physical vector register file
 * Vec512 physicalVecRegFile[NUM_VREGS];
 * ...
 * // Usage example, for a macro op:
 * VecFloat8Add(ExecContext* xd) {
 *    // Request source vector register to the execution context (const as it
 *    // is read only).
 *    const Vec512& vsrc1raw = xc->readVecRegOperand(this, 0);
 *    // View it as a vector of floats (we could just specify the first
 *    // template parametre, the second has a default value that works, and the
 *    // last one is derived by the constness of vsrc1raw).
 *    VecRegT<float, 8, true>& vsrc1 = vsrc1raw->as<float, 8>();
 *
 *    // Second source and view
 *    const Vec512& vsrc2raw = xc->readVecRegOperand(this, 1);
 *    VecRegT<float, 8, true>& vsrc2 = vsrc2raw->as<float, 8>();
 *
 *    // Destination and view
 *    Vec512 vdstraw;
 *    VecRegT<float, 8, false>& vdst = vdstraw->as<float, 8>();
 *
 *    for (auto i = 0; i < 8; i++) {
 *        // This asignment sets the bits in the underlying Vec512: vdstraw
 *        vdst[i] = vsrc1[i] + vsrc2[i];
 *    }
 *    xc->setWriteRegOperand(this, 0, vdstraw);
 * }
 *
 * // Usage example, for a micro op that operates over lane number _lidx:
 * VecFloatLaneAdd(ExecContext* xd) {
 *    // Request source vector register to the execution context (const as it
 *    // is read only).
 *    const Vec512& vsrc1raw = xc->readVecRegOperand(this, 0);
 *    // View it as a lane of a vector of floats (we could just specify the
 *    // first template parametre, the second is derived by the constness of
 *    // vsrc1raw).
 *    VecLaneT<float, true>& src1 = vsrc1raw->laneView<float>(this->_lidx);
 *
 *    // Second source and view
 *    const Vec512& vsrc2raw = xc->readVecRegOperand(this, 1);
 *    VecLaneT<float, true>& src2 = vsrc2raw->laneView<float>(this->_lidx);
 *
 *    // (Writable) destination and view
 *    // As this is a partial write, we need the exec context to support that
 *    // through, e.g., 'readVecRegOperandToWrite' returning a writable
 *    // reference to the register
 *    Vec512 vdstraw = xc->readVecRegOperandToWrite(this, 3);
 *    VecLaneT<float, false>& dst = vdstraw->laneView<float>(this->_lidx);
 *
 *    dst = src1 + src2;
 *    // There is no need to copy the value back into the exec context, as
 *    // the assignment to dst modifies the appropriate bytes in vdstraw which
 *    // is in turn, a reference to the register in the cpu model.
 *    // For operations that do conditional writeback, we can decouple the
 *    // write by doing:
 *    //   auto tmp = src1 + src2;
 *    //   if (test) {
 *    //       dst = tmp; // do writeback
 *    //   } else {
 *    //      // do not do writeback
 *    //   }
 * }
 *
 */

#ifndef __ARCH_GENERIC_VEC_REG_HH__
#define __ARCH_GENERIC_VEC_REG_HH__

#include <array>
#include <cassert>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

#include "base/cprintf.hh"
#include "base/logging.hh"

constexpr unsigned MaxVecRegLenInBytes = 4096;

template <size_t Sz>
class VecRegContainer;

/** Vector Register Abstraction
 * This generic class is a view in a particularization of MVC, to vector
 * registers. There is a VecRegContainer that implements the model, and
 * contains the data. To that model we can interpose different instantiations
 * of VecRegT to view the container as a vector of NumElems elems of type
 * VecElem.
 * @tparam VecElem Type of each element of the vector.
 * @tparam NumElems Amount of components of the vector.
 * @tparam Const Indicate if the underlying container can be modified through
 * the view.
 */
template <typename VecElem, size_t NumElems, bool Const>
class VecRegT
{
    /** Size of the register in bytes. */
    static constexpr inline size_t
    size()
    {
        return sizeof(VecElem) * NumElems;
    }
  public:
    /** Container type alias. */
    using Container = typename std::conditional<Const,
                                              const VecRegContainer<size()>,
                                              VecRegContainer<size()>>::type;
  private:
    /** My type alias. */
    using MyClass = VecRegT<VecElem, NumElems, Const>;
    /** Reference to container. */
    Container& container;

  public:
    /** Constructor. */
    VecRegT(Container& cnt) : container(cnt) {};

    /** Zero the container. */
    template<bool Condition = !Const>
    typename std::enable_if<Condition, void>::type
    zero() { container.zero(); }

    template<bool Condition = !Const>
    typename std::enable_if<Condition, MyClass&>::type
    operator=(const MyClass& that)
    {
        container = that.container;
        return *this;
    }

    /** Index operator. */
    const VecElem& operator[](size_t idx) const
    {
        return container.template raw_ptr<VecElem>()[idx];
    }

    /** Index operator. */
    template<bool Condition = !Const>
    typename std::enable_if<Condition, VecElem&>::type
    operator[](size_t idx)
    {
        return container.template raw_ptr<VecElem>()[idx];
    }

    /** Equality operator.
     * Required to compare thread contexts.
     */
    template<typename VE2, size_t NE2, bool C2>
    bool
    operator==(const VecRegT<VE2, NE2, C2>& that) const
    {
        return container == that.container;
    }
    /** Inequality operator.
     * Required to compare thread contexts.
     */
    template<typename VE2, size_t NE2, bool C2>
    bool
    operator!=(const VecRegT<VE2, NE2, C2>& that) const
    {
        return !operator==(that);
    }

    /** Output stream operator. */
    friend std::ostream&
    operator<<(std::ostream& os, const MyClass& vr)
    {
        /* 0-sized is not allowed */
        os << "[" << std::hex << (uint32_t)vr[0];
        for (uint32_t e = 1; e < vr.size(); e++)
            os << " " << std::hex << (uint32_t)vr[e];
        os << ']';
        return os;
    }

    const std::string print() const { return csprintf("%s", *this); }
    /**
     * Cast to VecRegContainer&
     * It is useful to get the reference to the container for ISA tricks,
     * because casting to reference prevents unnecessary copies.
     */
    operator Container&() { return container; }
};

/* Forward declaration. */
template <typename VecElem, bool Const>
class VecLaneT;

/**
 * Vector Register Abstraction
 * This generic class is the model in a particularization of MVC, to vector
 * registers. The model has functionality to create views of itself, or a
 * portion through the method 'as
 * @tparam Sz Size of the container in bytes.
 */
template <size_t SIZE>
class VecRegContainer
{
  static_assert(SIZE > 0,
          "Cannot create Vector Register Container of zero size");
  static_assert(SIZE <= MaxVecRegLenInBytes,
          "Vector Register size limit exceeded");
  public:
    static constexpr inline size_t size() { return SIZE; };
    using Container = std::array<uint8_t, SIZE>;
  private:
    // 16-byte aligned to support 128bit element view
    alignas(16) Container container;
    using MyClass = VecRegContainer<SIZE>;

  public:
    VecRegContainer() {}
    VecRegContainer(const VecRegContainer &) = default;
    /* This is required for de-serialisation. */
    VecRegContainer(const std::vector<uint8_t>& that)
    {
        assert(that.size() >= SIZE);
        std::memcpy(container.data(), &that[0], SIZE);
    }

    /** Zero the container. */
    void zero() { memset(container.data(), 0, SIZE); }

    /** Assignment operators. */
    /** @{ */
    /** From VecRegContainer */
    MyClass& operator=(const MyClass& that)
    {
        if (&that == this)
            return *this;
        memcpy(container.data(), that.container.data(), SIZE);
        return *this;
    }

    /** From appropriately sized uint8_t[]. */
    MyClass& operator=(const Container& that)
    {
        std::memcpy(container.data(), that.data(), SIZE);
        return *this;
    }

    /** From vector<uint8_t>.
     * This is required for de-serialisation.
     * */
    MyClass& operator=(const std::vector<uint8_t>& that)
    {
        assert(that.size() >= SIZE);
        std::memcpy(container.data(), that.data(), SIZE);
        return *this;
    }
    /** @} */

    /** Copy the contents into the input buffer. */
    /** @{ */
    /** To appropriately sized uint8_t[] */
    void copyTo(Container& dst) const
    {
        std::memcpy(dst.data(), container.data(), SIZE);
    }

    /** To vector<uint8_t>
     * This is required for serialisation.
     * */
    void copyTo(std::vector<uint8_t>& dst) const
    {
        dst.resize(SIZE);
        std::memcpy(dst.data(), container.data(), SIZE);
    }
    /** @} */

    /** Equality operator.
     * Required to compare thread contexts.
     */
    template<size_t S2>
    inline bool
    operator==(const VecRegContainer<S2>& that) const
    {
        return SIZE == S2 &&
               !memcmp(container.data(), that.container.data(), SIZE);
    }
    /** Inequality operator.
     * Required to compare thread contexts.
     */
    template<size_t S2>
    bool
    operator!=(const VecRegContainer<S2>& that) const
    {
        return !operator==(that);
    }

    const std::string print() const { return csprintf("%s", *this); }
    /** Get pointer to bytes. */
    template <typename Ret>
    const Ret* raw_ptr() const { return (const Ret*)container.data(); }

    template <typename Ret>
    Ret* raw_ptr() { return (Ret*)container.data(); }

    /**
     * View interposers.
     * Create a view of this container as a vector of VecElems with an
     * optional amount of elements. If the amount of elements is provided,
     * the size of the container is checked, to test bounds. If it is not
     * provided, the length is inferred from the container size and the
     * element size.
     * @tparam VecElem Type of each element of the vector for the view.
     * @tparam NumElem Amount of elements in the view.
     */
    /** @{ */
    template <typename VecElem, size_t NumElems=(SIZE / sizeof(VecElem))>
    VecRegT<VecElem, NumElems, true> as() const
    {
        static_assert(SIZE % sizeof(VecElem) == 0,
                "VecElem does not evenly divide the register size");
        static_assert(sizeof(VecElem) * NumElems <= SIZE,
                "Viewing VecReg as something bigger than it is");
        return VecRegT<VecElem, NumElems, true>(*this);
    }

    template <typename VecElem, size_t NumElems=(SIZE / sizeof(VecElem))>
    VecRegT<VecElem, NumElems, false> as()
    {
        static_assert(SIZE % sizeof(VecElem) == 0,
                "VecElem does not evenly divide the register size");
        static_assert(sizeof(VecElem) * NumElems <= SIZE,
                "Viewing VecReg as something bigger than it is");
        return VecRegT<VecElem, NumElems, false>(*this);
    }

    template <typename VecElem, int LaneIdx>
    VecLaneT<VecElem, false> laneView();
    template <typename VecElem, int LaneIdx>
    VecLaneT<VecElem, true> laneView() const;
    template <typename VecElem>
    VecLaneT<VecElem, false> laneView(int laneIdx);
    template <typename VecElem>
    VecLaneT<VecElem, true> laneView(int laneIdx) const;
    /** @} */
    /**
     * Output operator.
     * Used for serialization.
     */
    friend std::ostream& operator<<(std::ostream& os, const MyClass& v)
    {
        for (auto& b: v.container) {
            os << csprintf("%02x", b);
        }
        return os;
    }
};

/** We define an auxiliary abstraction for LaneData. The ISA should care
 * about the semantics of a, e.g., 32bit element, treating it as a signed or
 * unsigned int, or a float depending on the semantics of a particular
 * instruction. On the other hand, the cpu model should only care about it
 * being a 32-bit value. */
enum class LaneSize
{
    Empty = 0,
    Byte,
    TwoByte,
    FourByte,
    EightByte,
};

/** LaneSize is an abstraction of a LS byte value for the execution and thread
 * contexts to handle values just depending on its width. That way, the ISA
 * can request, for example, the second 4 byte lane of register 5 to the model.
 * The model serves that value, agnostic of the semantics of those bits. Then,
 * it is up to the ISA to interpret those bits as a float, or as an uint.
 * To maximize the utility, this class implements the assignment operator and
 * the casting to equal-size types.
 * As opposed to a RegLaneT, LaneData is not 'backed' by a VecRegContainer.
 * The idea is:
 *  When data is passed and is susceptible to being copied, use LaneData, as
 *     copying the primitive type is build on is cheap.
 *  When data is passed as references (const or not), use RegLaneT, as all
 *     operations happen 'in place', avoiding any copies (no copies is always
 *     cheaper than cheap copies), especially when things are inlined, and
 *     references are not explicitly passed.
 */
template <LaneSize LS>
class LaneData
{
  public:
    /** Alias to the native type of the appropriate size. */
    using UnderlyingType =
        typename std::conditional<LS == LaneSize::EightByte, uint64_t,
            typename std::conditional<LS == LaneSize::FourByte, uint32_t,
                typename std::conditional<LS == LaneSize::TwoByte, uint16_t,
                    typename std::conditional<LS == LaneSize::Byte, uint8_t,
                    void>::type
                >::type
            >::type
        >::type;
  private:
    static constexpr auto ByteSz = sizeof(UnderlyingType);
    UnderlyingType _val;
    using MyClass = LaneData<LS>;

  public:
    template <typename T> explicit
    LaneData(typename std::enable_if<sizeof(T) == ByteSz, const T&>::type t)
                : _val(t) {}

    template <typename T>
    typename std::enable_if<sizeof(T) == ByteSz, MyClass&>::type
    operator=(const T& that)
    {
        _val = that;
        return *this;
    }
    template<typename T,
             typename std::enable_if<sizeof(T) == ByteSz, int>::type I = 0>
    operator T() const {
        return *static_cast<const T*>(&_val);
    }
};

/** Output operator overload for LaneData<Size>. */
template <LaneSize LS>
inline std::ostream&
operator<<(std::ostream& os, const LaneData<LS>& d)
{
    return os << static_cast<typename LaneData<LS>::UnderlyingType>(d);
}

/** Vector Lane abstraction
 * Another view of a container. This time only a partial part of it is exposed.
 * @tparam VecElem Type of each element of the vector.
 * @tparam Const Indicate if the underlying container can be modified through
 * the view.
 */
/** @{ */
/* General */
template <typename VecElem, bool Const>
class VecLaneT
{
  public:
    /** VecRegContainer friendship to access private VecLaneT constructors.
     * Only VecRegContainers can build VecLanes.
     */
    /** @{ */
    friend VecLaneT<VecElem, !Const>;

    /*template <size_t Sz>
    friend class VecRegContainer;*/
    friend class VecRegContainer<8>;
    friend class VecRegContainer<16>;
    friend class VecRegContainer<32>;
    friend class VecRegContainer<64>;
    friend class VecRegContainer<128>;
    friend class VecRegContainer<256>;
    friend class VecRegContainer<MaxVecRegLenInBytes>;

    /** My type alias. */
    using MyClass = VecLaneT<VecElem, Const>;

  private:
    using Cont = typename std::conditional<Const,
                                              const VecElem,
                                              VecElem>::type;
    static_assert(!std::is_const<VecElem>::value || Const,
            "Asked for non-const lane of const type!");
    static_assert(std::is_integral<VecElem>::value,
            "VecElem type is not integral!");
    /** Reference to data. */
    Cont& container;

    /** Constructor */
    VecLaneT(Cont& cont) : container(cont) { }

  public:
    /** Assignment operators.
     * Assignment operators are only enabled if the underlying container is
     * non-constant.
     */
    /** @{ */
    template <bool Assignable = !Const>
    typename std::enable_if<Assignable, MyClass&>::type
    operator=(const VecElem& that) {
        container = that;
        return *this;
    }
    /**
     * Generic.
     * Generic bitwise assignment. Narrowing and widening assignemnts are
     * not allowed, pre-treatment of the rhs is required to conform.
     */
    template <bool Assignable = !Const, typename T>
    typename std::enable_if<Assignable, MyClass&>::type
    operator=(const T& that) {
        static_assert(sizeof(T) >= sizeof(VecElem),
                "Attempt to perform widening bitwise copy.");
        static_assert(sizeof(T) <= sizeof(VecElem),
                "Attempt to perform narrowing bitwise copy.");
        container = static_cast<VecElem>(that);
        return *this;
    }
    /** @} */
    /** Cast to vecElem. */
    operator VecElem() const { return container; }

    /** Constification. */
    template <bool Cond = !Const, typename std::enable_if<Cond, int>::type = 0>
    operator VecLaneT<typename std::enable_if<Cond, VecElem>::type, true>()
    {
        return VecLaneT<VecElem, true>(container);
    }
};

namespace std {
    template<typename T, bool Const>
    struct add_const<VecLaneT<T, Const>> { typedef VecLaneT<T, true> type; };
}

/** View as the Nth lane of type VecElem. */
template <size_t Sz>
template <typename VecElem, int LaneIdx>
VecLaneT<VecElem, false>
VecRegContainer<Sz>::laneView()
{
    return VecLaneT<VecElem, false>(as<VecElem>()[LaneIdx]);
}

/** View as the const Nth lane of type VecElem. */
template <size_t Sz>
template <typename VecElem, int LaneIdx>
VecLaneT<VecElem, true>
VecRegContainer<Sz>::laneView() const
{
    return VecLaneT<VecElem, true>(as<VecElem>()[LaneIdx]);
}

/** View as the Nth lane of type VecElem. */
template <size_t Sz>
template <typename VecElem>
VecLaneT<VecElem, false>
VecRegContainer<Sz>::laneView(int laneIdx)
{
    return VecLaneT<VecElem, false>(as<VecElem>()[laneIdx]);
}

/** View as the const Nth lane of type VecElem. */
template <size_t Sz>
template <typename VecElem>
VecLaneT<VecElem, true>
VecRegContainer<Sz>::laneView(int laneIdx) const
{
    return VecLaneT<VecElem, true>(as<VecElem>()[laneIdx]);
}

using VecLane8 = VecLaneT<uint8_t, false>;
using VecLane16 = VecLaneT<uint16_t, false>;
using VecLane32 = VecLaneT<uint32_t, false>;
using VecLane64 = VecLaneT<uint64_t, false>;

using ConstVecLane8 = VecLaneT<uint8_t, true>;
using ConstVecLane16 = VecLaneT<uint16_t, true>;
using ConstVecLane32 = VecLaneT<uint32_t, true>;
using ConstVecLane64 = VecLaneT<uint64_t, true>;

/**
 * Calls required for serialization/deserialization
 */
/** @{ */
template <size_t Sz>
inline bool
to_number(const std::string& value, VecRegContainer<Sz>& v)
{
    fatal_if(value.size() > 2 * VecRegContainer<Sz>::size(),
             "Vector register value overflow at unserialize");

    for (int i = 0; i < VecRegContainer<Sz>::size(); i++) {
        uint8_t b = 0;
        if (2 * i < value.size())
            b = stoul(value.substr(i * 2, 2), nullptr, 16);
        v.template raw_ptr<uint8_t>()[i] = b;
    }
    return true;
}
/** @} */

/**
 * Dummy type aliases and constants for architectures that do not implement
 * vector registers.
 */
/** @{ */
using DummyVecElem = uint32_t;
constexpr unsigned DummyNumVecElemPerVecReg = 2;
using DummyVecReg = VecRegT<DummyVecElem, DummyNumVecElemPerVecReg, false>;
using DummyConstVecReg = VecRegT<DummyVecElem, DummyNumVecElemPerVecReg, true>;
using DummyVecRegContainer = DummyVecReg::Container;
constexpr size_t DummyVecRegSizeBytes = DummyNumVecElemPerVecReg *
    sizeof(DummyVecElem);
/** @} */

#endif /* __ARCH_GENERIC_VEC_REG_HH__ */
