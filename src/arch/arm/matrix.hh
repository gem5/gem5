/*
 * Copyright (c) 2022 Arm Limited
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

/** \file arch/arm/matrix.hh
 * Matrix Register Specification.
 *
 * In this file we add three new classes which are used to provide both
 * the backing storage for matrix registers (MatStore) and for accessing
 * them using a set of views onto the backing store (Tile, TileSlice).
 *
 * The MatStore provides the backing store for the matrix, handles the
 * serialisation/unserialisation, and provides interfaces to obtain
 * views of the matrix. The underlying element for the MatStore is a
 * byte, and it uses two templated parameters, X and Y, to set the
 * overall size of the matrix. The common use case will be that X and Y
 * are the same size, yielding a square matrix, but this is not a
 * requirement - it is possible to create non-square matricies too if
 * such a thing is desired.
 *
 * The Tile provides a view on top of the MatStore which is intended to
 * preserve the original aspect ratio of the underlying MatStore as the
 * element size scales. It does so by row-wise interleaving one or more
 * sub-matrices on top of the MatStore, where the number of sub-matrices
 * is governed by the element size (in bytes) itself. As an example, if
 * the elements are half-words, i.e. 2 bytes wide, then there are two
 * interleaved matrices with even rows belonging to sub-matrix 0 and odd
 * rows belonging to sub-matrix 1. However, each of these sub-matricies
 * maintains the original aspect ratio of the MatStore - the element
 * size has doubled (bytes => half words), hence each row contains half
 * the original number of elements, and each sub-matrix contains half of
 * the number of rows themselves.
 *
 * The TileSlice class provides a view of either a row or a column of a
 * matrix, and can be generated from either the MatStore directly, or
 * from the Tile. In the former case this allows a matrix to be viewed
 * as a set of rows or columns, and in the latter this same approach is
 * applied to the Tile. In both cases this is achieved by adjusting the
 * striding through the backing store accordingly.
 *
 * The intended usage of the views is as follows:
 *
 * // declare an 8x8 matrix of bytes
 * using Mat8x8 = MatStore<8, 8>;
 *
 * // Create a matrix and make sure that it is zeroed
 * Mat8x8 mat;
 * mat.zero();
 *
 * // Interleave four tiles of int32_t onto the 8x8 matrix, and get
 * // tile 0. (Each of these tiles will be a 2x2 matrix)
 * auto mat0 = mat.asTile<int32_t>(0);
 *
 * // Set both elements of row 0 to 10
 * for (auto i = 0; i < 2; ++i) {
 *     mat0[0][i] = 10;
 * }
 *
 * // Sum both elements of row 1
 * int32_t sum = 0;
 * auto row = mat0.asHSlice(1);
 * for (auto i = 0; i < 2; ++i) {
 *     sum += row[i];
 * }
 *
 * // print column 1 of the whole MatStore when viewed as uint16_t
 * col = mat.asVSlice<uint16_t>(1);
 * for (auto i = 0; i < 4; ++i) {
 *     std::cout << col[i] << std::endl;
 * }
 *
 */

#ifndef __ARCH_ARM_MATRIX_HH__
#define __ARCH_ARM_MATRIX_HH__

#include <array>
#include <cassert>
#include <cstring>
#include <iostream>
#include <type_traits>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "sim/serialize_handlers.hh"

namespace gem5
{

constexpr unsigned MaxMatRegRowLenInBytes = 256;
constexpr unsigned MaxMatRegRows = 256;

// Forward declarations
template <size_t X, size_t Y>
class MatStore;
template <typename ElemType, typename Container>
class Tile;

template <size_t X, size_t Y>
struct ParseParam<MatStore<X, Y>>;

/**
 * @brief Provides a view of a horizontal slice of either a
 *        MatStore or a Tile.
 *
 * Based on whether this view it is being used from the MatStore
 * directly or from the Tile different parameters are
 * used. Behind the scenes the parameters are used to stride through the
 * (linear) backing store in order to return or maniplate the desired
 * elements of the row/column.
 *
 * @tparam ElemType The type of element to use for the view.
 * @tparam Container The type of container being used as the backing store.
 * @tparam FromTile Set true if operating on an interleaved tile.
 */
template <typename ElemType, typename Container, bool FromTile>
class HorizontalSlice
{
    template <size_t, size_t> friend class MatStore;
    template <typename, typename> friend class Tile;

  private:
    Container * container;
    size_t index;
    size_t xElems;
    size_t yElems;
    size_t startElts;
    size_t strideElts;

  private:
    HorizontalSlice(Container& cnt, size_t _startBytes, size_t _strideBytes,
                    size_t idx)
      : container(&cnt), index(idx),
        xElems(container->xSize() / sizeof(ElemType)),
        yElems(container->ySize() / (FromTile ? sizeof(ElemType): 1)),
        startElts(_startBytes / sizeof(ElemType)),
        strideElts(_strideBytes / sizeof(ElemType))
    {
        gem5_assert(xElems > 0, "The number of xElems cannot be 0");
        gem5_assert(yElems > 0, "The number of yElems cannot be 0");

        // Make sure that we have a whole multiple of an element size
        assert (_startBytes % sizeof(ElemType) == 0);
        assert (_strideBytes % sizeof(ElemType) == 0);

        if constexpr (!FromTile) {
            // If we are not operating on a tile, the stride must be the
            // same as the row length, X.
            assert(_strideBytes == container->xSize());
        } else {
            // If we are operating on a tile, then the stride must be
            // sizeof(ElemSize) greater than X.
            assert(_strideBytes / container->xSize() == sizeof(ElemType));
        }
    };

  public:
    ElemType&
    operator[](size_t elem_idx)
    {
        assert(elem_idx < xElems);
        size_t linear_index = startElts + index * strideElts + elem_idx;
        return container->template rawPtr<ElemType>()[linear_index];
    };

    void
    zero()
    {
        for (int i = 0; i < xElems; ++i) {
            (*this)[i] = (ElemType)0;
        }
    };
};

/**
 * @brief Provides a view of a vertical slice of either a
 *        MatStore or a Tile.
 *
 * Based on whether this view it is being used from the MatStore
 * directly or from the Tile different parameters are used. Behind the
 * scenes the parameters are used to stride through the (linear) backing
 * store in order to return or maniplate the desired elements of the
 * row/column.
 *
 * @tparam ElemType The type of element to use for the view.
 * @tparam Container The type of container being used as the backing store.
 * @tparam FromTile Set true if operating on an interleaved tile.
 */
template <typename ElemType, typename Container, bool FromTile>
class VerticalSlice
{
    template <size_t, size_t> friend class MatStore;
    template <typename, typename> friend class Tile;

  private:
    Container * container;
    size_t index;
    size_t xElems;
    size_t yElems;
    size_t startElts;
    size_t strideElts;

  private:
    VerticalSlice(Container& cnt, size_t _startBytes, size_t _strideBytes, size_t idx)
      : container(&cnt), index(idx),
        xElems(container->xSize() / sizeof(ElemType)),
        yElems(container->ySize() / (FromTile ? sizeof(ElemType): 1)),
        startElts(_startBytes / sizeof(ElemType)),
        strideElts(_strideBytes / sizeof(ElemType))
    {
        gem5_assert(xElems > 0, "The number of xElems cannot be 0");
        gem5_assert(yElems > 0, "The number of yElems cannot be 0");

        // Make sure that we have a whole multiple of an element size
        assert (_startBytes % sizeof(ElemType) == 0);
        assert (_strideBytes % sizeof(ElemType) == 0);

        if constexpr (!FromTile) {
            // If we are not operating on a tile, the stride must be the
            // same as the row length, X.
            assert(_strideBytes == container->xSize());
        } else {
            // If we are operating on a tile, then the stride must be
            // sizeof(ElemSize) greater than X.
            assert(_strideBytes / container->xSize() == sizeof(ElemType));
        }
    };

  public:
    ElemType&
    operator[](size_t elem_idx)
    {
        assert(elem_idx < yElems);
        size_t linear_index = startElts + elem_idx * strideElts + index;
        return container->template rawPtr<ElemType>()[linear_index];
    };

    void
    zero()
    {
        for (int i = 0; i < yElems; ++i) {
            (*this)[i] = (ElemType)0;
        }
    };
};

/**
 * @brief Provides a view of a matrix that is row-interleaved onto a
 *        MatStore.
 *
 * This class largely acts as a shim between the MatStore and the
 * TileSlice view. The size of the ElemType and the index passed to the
 * constructor are used to calculate the stride and start which are
 * passed to the TileSlice view to control how it strides through the
 * backing store.
 *
 * @tparam ElemType The type of element to use for the view.
 * @tparam Container The type of container being used as the backing store.
 */
template <typename ElemType, typename Container>
class Tile
{
    template <size_t, size_t> friend class MatStore;

    // We "calculate" the number of possible tiles based on the element size
    static constexpr size_t NUM_TILES = sizeof(ElemType);

  private:
    Container * container;
    size_t index;
    size_t startBytes;
    size_t strideBytes;

  private:
    Tile(Container& cnt, size_t idx)
      : container(&cnt), index(idx)
    {
        assert(index < NUM_TILES);
        startBytes = container->xSize() * index;
        strideBytes = NUM_TILES * container->xSize();
    };

  public:
    auto
    operator[](size_t idx)
    {
        assert(idx < (container->ySize() / NUM_TILES));
        return asHSlice(idx);
    };

    Container*
    getContainer()
    {
        return container;
    };

    auto
    asHSlice(size_t row_idx)
    {
        assert(row_idx < container->ySize() / NUM_TILES);
        return HorizontalSlice<ElemType, Container, true>(*container,
                                                          startBytes,
                                                          strideBytes,
                                                          row_idx);
    };

    auto
    asVSlice(size_t col_idx)
    {
        assert(col_idx < container->xSize());
        return VerticalSlice<ElemType, Container, true>(*container, startBytes,
                                                        strideBytes, col_idx);
    };

    void
    zero()
    {
        for (int i = 0; i < container->ySize() / NUM_TILES; ++i) {
            // We zero the tile by rows. We need to do it this way due
            // to the interleaving.
            auto row = this->asHSlice(i);
            row.zero();
        }
    };
};

// Base container class for a matrix. Allows for non-square matricies.
/**
 * @brief Backing store for matrices.
 *
 * This class provides the backing store for matricies, and is largely a
 * wrapper around an std::array of bytes. This class provides some basic
 * interfaces for assignment (copy the backing store) and comparison,
 * and provides the interface for generating views onto the backing
 * store. It is these views that are intended to be used by the end-user
 * of the matrix in most cases.
 *
 * This class is also responsible for handling the
 * serialisation/unserialisation of matrix registers (see ShowParam and
 * ParseParam).
 *
 * @tparam X X size in bytes (number of columns).
 * @tparam Y Y size in bytes (number of rows).
 */
template <size_t X, size_t Y>
class MatStore
{
    static_assert(X > 0, "X size cannot be 0");
    static_assert(Y > 0, "Y size cannot be 0");

    static constexpr size_t LINEAR_SIZE = X * Y;

    template <typename, typename, bool> friend class HorizontalSlice;
    template <typename, typename, bool> friend class VerticalSlice;

  public:
    static constexpr inline size_t xSize() { return X; };
    static constexpr inline size_t ySize() { return Y; };
    static constexpr inline size_t linearSize() { return LINEAR_SIZE; };

    using Container = std::array<uint8_t, LINEAR_SIZE>;
    using MyClass = MatStore<X, Y>;
  private:
    // We need to be able to handle 128-bit types; align accordingly
    alignas(16) Container container;

  public:
    /** Constructor */
    MatStore() {};

    MatStore(const MatStore&) = default;

    void
    zero()
    {
        memset(container.data(), 0 , LINEAR_SIZE);
    }

    /** Assignment operators. */
    /** @{ */
    /** From MatStore */
    MyClass&
    operator=(const MyClass& that)
    {
        if (&that == this)
            return *this;
        memcpy(container.data(), that.container.data(), LINEAR_SIZE);
        return *this;
    }
    /** @} */

    /** Equality operator.
     * Required to compare thread contexts.
     */
    template<size_t X2, size_t Y2>
    inline bool
    operator==(const MatStore<X2, Y2>& that) const
    {
        return X == X2 && Y == Y2 &&
               !memcmp(container.data(), that.container.data(), LINEAR_SIZE);
    }

    /** Inequality operator.
     * Required to compare thread contexts.
     */
    template<size_t X2, size_t Y2>
    bool
    operator!=(const MatStore<X2, Y2>& that) const
    {
        return !operator==(that);
    }

  private:
    /** Get pointer to the raw data. */
    template <typename ElemType>
    const ElemType* rawPtr() const
    {
        return reinterpret_cast<const ElemType*>(container.data());
    }

    template <typename ElemType>
    ElemType* rawPtr() { return reinterpret_cast<ElemType*>(container.data()); }

  public:
    template <typename ElemType>
    auto
    asTile(size_t index)
    {
        return Tile<ElemType, MyClass>(*this, index);
    }

    template <typename ElemType>
    auto
    asHSlice(size_t row_idx)
    {
        return HorizontalSlice<ElemType, MyClass, false>(*this, 0, X, row_idx);
    }

    template <typename ElemType>
    auto
    asVSlice(size_t col_idx)
    {
        return VerticalSlice<ElemType, MyClass, false>(*this, 0, X, col_idx);
    }

    friend std::ostream&
    operator<<(std::ostream& os, const MatStore<X, Y>& v)
    {
        // When printing for human consumption, break into 4 byte chunks.
        ccprintf(os, "[");
        size_t count = 0;
        for (auto& b: v.container) {
            if (count && (count % 4) == 0)
                os << "_";
            ccprintf(os, "%02x", b);
            count++;
        }
        ccprintf(os, "]");
        return os;
    }

    /** @} */
    /**
     * Used for serialization/unserialisation.
     */
    friend ParseParam<MatStore<X, Y>>;
    friend ShowParam<MatStore<X, Y>>;

};

/**
 * Calls required for serialization/deserialization
 */
/** @{ */
template <size_t X, size_t Y>
struct ParseParam<MatStore<X, Y>>
{
    static bool
    parse(const std::string &str, MatStore<X, Y> &value)
    {
        fatal_if(str.size() > 2 * X * Y,
                 "Matrix register value overflow at unserialize");
        fatal_if(str.size() < 2 * X * Y,
                 "Matrix register value underflow at unserialize");

        for (int i = 0; i < X * Y; i++) {
            uint8_t b = 0;
            if (2 * i < str.size())
                b = stoul(str.substr(i * 2, 2), nullptr, 16);
            value.template rawPtr<uint8_t>()[i] = b;
        }
        return true;
    }
};

template <size_t X, size_t Y>
struct ShowParam<MatStore<X, Y>>
{
    static void
    show(std::ostream &os, const MatStore<X, Y> &value)
    {
        for (auto& b: value.container)
            ccprintf(os, "%02x", b);
    }
};
/** @} */

/**
 * Dummy type aliases and constants for architectures that do not
 * implement matrix registers.
 */
/** @{ */
struct DummyMatRegContainer
{
    RegVal filler = 0;
    bool operator == (const DummyMatRegContainer &d) const { return true; }
    bool operator != (const DummyMatRegContainer &d) const { return true; }
};
template <>
struct ParseParam<DummyMatRegContainer>
{
    static bool
    parse(const std::string &s, DummyMatRegContainer &value)
    {
        return false;
    }
};
static_assert(sizeof(DummyMatRegContainer) == sizeof(RegVal));
static inline std::ostream &
operator<<(std::ostream &os, const DummyMatRegContainer &d)
{
    return os;
}
/** @} */

} // namespace gem5

#endif // __ARCH_ARM_MATRIX_HH__
