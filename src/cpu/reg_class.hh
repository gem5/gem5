/*
 * Copyright (c) 2016-2019 ARM Limited
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
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *.
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

#ifndef __CPU__REG_CLASS_HH__
#define __CPU__REG_CLASS_HH__

#include <cassert>
#include <cstddef>

#include "arch/vecregs.hh"
#include "base/types.hh"
#include "config/the_isa.hh"

namespace gem5
{

/** Enumerate the classes of registers. */
enum RegClass
{
    IntRegClass,        ///< Integer register
    FloatRegClass,      ///< Floating-point register
    /** Vector Register. */
    VecRegClass,
    /** Vector Register Native Elem lane. */
    VecElemClass,
    VecPredRegClass,
    CCRegClass,         ///< Condition-code register
    MiscRegClass        ///< Control (misc) register
};

class RegClassInfo
{
  private:
    size_t _size;
    const RegIndex _zeroReg;

  public:
    RegClassInfo(size_t new_size, RegIndex new_zero = -1) :
        _size(new_size), _zeroReg(new_zero)
    {}

    size_t size() const { return _size; }
    RegIndex zeroReg() const { return _zeroReg; }
};

/** Register ID: describe an architectural register with its class and index.
 * This structure is used instead of just the register index to disambiguate
 * between different classes of registers. For example, a integer register with
 * index 3 is represented by Regid(IntRegClass, 3).
 */
class RegId
{
  protected:
    static const char* regClassStrings[];
    RegClass regClass;
    RegIndex regIdx;
    ElemIndex elemIdx;
    static constexpr size_t Scale = TheISA::NumVecElemPerVecReg;
    int numPinnedWrites;

    friend struct std::hash<RegId>;

  public:
    RegId() : RegId(IntRegClass, 0) {}

    RegId(RegClass reg_class, RegIndex reg_idx)
        : RegId(reg_class, reg_idx, IllegalElemIndex) {}

    explicit RegId(RegClass reg_class, RegIndex reg_idx, ElemIndex elem_idx)
        : regClass(reg_class), regIdx(reg_idx), elemIdx(elem_idx),
          numPinnedWrites(0)
    {
        if (elemIdx == IllegalElemIndex) {
            panic_if(regClass == VecElemClass,
                    "Creating vector physical index w/o element index");
        } else {
            panic_if(regClass != VecElemClass,
                    "Creating non-vector physical index w/ element index");
        }
    }

    bool
    operator==(const RegId& that) const
    {
        return regClass == that.classValue() && regIdx == that.index() &&
            elemIdx == that.elemIndex();
    }

    bool operator!=(const RegId& that) const { return !(*this==that); }

    /** Order operator.
     * The order is required to implement maps with key type RegId
     */
    bool
    operator<(const RegId& that) const
    {
        return regClass < that.classValue() ||
            (regClass == that.classValue() && (
                   regIdx < that.index() ||
                   (regIdx == that.index() && elemIdx < that.elemIndex())));
    }

    /**
     * Return true if this register can be renamed
     */
    bool
    isRenameable() const
    {
        return regClass != MiscRegClass;
    }

    /** @return true if it is of the specified class. */
    bool is(RegClass reg_class) const { return regClass == reg_class; }

    /** Index accessors */
    /** @{ */
    RegIndex index() const { return regIdx; }

    /** Index flattening.
     * Required to be able to use a vector for the register mapping.
     */
    RegIndex
    flatIndex() const
    {
        switch (regClass) {
          case IntRegClass:
          case FloatRegClass:
          case VecRegClass:
          case VecPredRegClass:
          case CCRegClass:
          case MiscRegClass:
            return regIdx;
          case VecElemClass:
            return Scale * regIdx + elemIdx;
        }
        panic("Trying to flatten a register without class!");
    }
    /** @} */

    /** Elem accessor */
    RegIndex elemIndex() const { return elemIdx; }
    /** Class accessor */
    RegClass classValue() const { return regClass; }
    /** Return a const char* with the register class name. */
    const char* className() const { return regClassStrings[regClass]; }

    int getNumPinnedWrites() const { return numPinnedWrites; }
    void setNumPinnedWrites(int num_writes) { numPinnedWrites = num_writes; }

    friend std::ostream&
    operator<<(std::ostream& os, const RegId& rid)
    {
        return os << rid.className() << "{" << rid.index() << "}";
    }
};

/** Physical register ID.
 * Like a register ID but physical. The inheritance is private because the
 * only relationship between this types is functional, and it is done to
 * prevent code replication. */
class PhysRegId : private RegId
{
  private:
    RegIndex flatIdx;
    int numPinnedWritesToComplete;
    bool pinned;

  public:
    explicit PhysRegId() : RegId(IntRegClass, -1), flatIdx(-1),
                           numPinnedWritesToComplete(0)
    {}

    /** Scalar PhysRegId constructor. */
    explicit PhysRegId(RegClass _regClass, RegIndex _regIdx,
              RegIndex _flatIdx)
        : RegId(_regClass, _regIdx), flatIdx(_flatIdx),
          numPinnedWritesToComplete(0), pinned(false)
    {}

    /** Vector PhysRegId constructor (w/ elemIndex). */
    explicit PhysRegId(RegClass _regClass, RegIndex _regIdx,
              ElemIndex elem_idx, RegIndex flat_idx)
        : RegId(_regClass, _regIdx, elem_idx), flatIdx(flat_idx),
          numPinnedWritesToComplete(0), pinned(false)
    {}

    /** Visible RegId methods */
    /** @{ */
    using RegId::index;
    using RegId::classValue;
    using RegId::className;
    using RegId::elemIndex;
    using RegId::is;
     /** @} */
    /**
     * Explicit forward methods, to prevent comparisons of PhysRegId with
     * RegIds.
     */
    /** @{ */
    bool
    operator<(const PhysRegId& that) const
    {
        return RegId::operator<(that);
    }

    bool
    operator==(const PhysRegId& that) const
    {
        return RegId::operator==(that);
    }

    bool
    operator!=(const PhysRegId& that) const
    {
        return RegId::operator!=(that);
    }
    /** @} */

    /**
     * Returns true if this register is always associated to the same
     * architectural register.
     */
    bool isFixedMapping() const { return !isRenameable(); }

    /** Flat index accessor */
    const RegIndex& flatIndex() const { return flatIdx; }

    static PhysRegId
    elemId(PhysRegId* vid, ElemIndex elem)
    {
        assert(vid->is(VecRegClass));
        return PhysRegId(VecElemClass, vid->index(), elem);
    }

    int getNumPinnedWrites() const { return numPinnedWrites; }

    void
    setNumPinnedWrites(int numWrites)
    {
        // An instruction with a pinned destination reg can get
        // squashed. The numPinnedWrites counter may be zero when
        // the squash happens but we need to know if the dest reg
        // was pinned originally in order to reset counters properly
        // for a possible re-rename using the same physical reg (which
        // may be required in case of a mem access order violation).
        pinned = (numWrites != 0);
        numPinnedWrites = numWrites;
    }

    void decrNumPinnedWrites() { --numPinnedWrites; }
    void incrNumPinnedWrites() { ++numPinnedWrites; }

    bool isPinned() const { return pinned; }

    int
    getNumPinnedWritesToComplete() const
    {
        return numPinnedWritesToComplete;
    }

    void
    setNumPinnedWritesToComplete(int numWrites)
    {
        numPinnedWritesToComplete = numWrites;
    }

    void decrNumPinnedWritesToComplete() { --numPinnedWritesToComplete; }
    void incrNumPinnedWritesToComplete() { ++numPinnedWritesToComplete; }
};

using PhysRegIdPtr = PhysRegId*;

} // namespace gem5

namespace std
{
template<>
struct hash<gem5::RegId>
{
    size_t
    operator()(const gem5::RegId& reg_id) const
    {
        // Extract unique integral values for the effective fields of a RegId.
        const size_t flat_index = static_cast<size_t>(reg_id.flatIndex());
        const size_t class_num = static_cast<size_t>(reg_id.regClass);

        const size_t shifted_class_num =
            class_num << (sizeof(gem5::RegIndex) << 3);

        // Concatenate the class_num to the end of the flat_index, in order to
        // maximize information retained.
        const size_t concatenated_hash = flat_index | shifted_class_num;

        // If RegIndex is larger than size_t, then class_num will not be
        // considered by this hash function, so we may wish to perform a
        // different operation to include that information in the hash.
        static_assert(sizeof(gem5::RegIndex) < sizeof(size_t),
            "sizeof(RegIndex) should be less than sizeof(size_t)");

        return concatenated_hash;
    }
};
} // namespace std

#endif // __CPU__REG_CLASS_HH__
