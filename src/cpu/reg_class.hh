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
 *
 * Authors: Steve Reinhardt
 *          Nathanael Premillieu
 *          Rekai Gonzalez
 */

#ifndef __CPU__REG_CLASS_HH__
#define __CPU__REG_CLASS_HH__

#include <cassert>
#include <cstddef>

#include "arch/generic/types.hh"
#include "arch/registers.hh"
#include "config/the_isa.hh"

/** Enumerate the classes of registers. */
enum RegClass {
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

/** Number of register classes.
 * This value is not part of the enum, because putting it there makes the
 * compiler complain about unhandled cases in some switch statements.
 */
const int NumRegClasses = MiscRegClass + 1;

/** Register ID: describe an architectural register with its class and index.
 * This structure is used instead of just the register index to disambiguate
 * between different classes of registers. For example, a integer register with
 * index 3 is represented by Regid(IntRegClass, 3).
 */
class RegId {
  private:
    static const char* regClassStrings[];
    RegClass regClass;
    RegIndex regIdx;
    ElemIndex elemIdx;
    static constexpr size_t Scale = TheISA::NumVecElemPerVecReg;
    friend struct std::hash<RegId>;
  public:
    RegId() : regClass(IntRegClass), regIdx(0), elemIdx(-1) {}
    RegId(RegClass reg_class, RegIndex reg_idx)
        : regClass(reg_class), regIdx(reg_idx), elemIdx(-1)
    {
        panic_if(regClass == VecElemClass,
                "Creating vector physical index w/o element index");
    }

    explicit RegId(RegClass reg_class, RegIndex reg_idx, ElemIndex elem_idx)
        : regClass(reg_class), regIdx(reg_idx), elemIdx(elem_idx)
    {
        panic_if(regClass != VecElemClass,
                "Creating non-vector physical index w/ element index");
    }

    bool operator==(const RegId& that) const {
        return regClass == that.classValue() && regIdx == that.index()
                                             && elemIdx == that.elemIndex();
    }

    bool operator!=(const RegId& that) const {
        return !(*this==that);
    }

    /** Order operator.
     * The order is required to implement maps with key type RegId
     */
    bool operator<(const RegId& that) const {
        return regClass < that.classValue() ||
            (regClass == that.classValue() && (
                   regIdx < that.index() ||
                   (regIdx == that.index() && elemIdx < that.elemIndex())));
    }

    /**
     * Return true if this register can be renamed
     */
    bool isRenameable() const
    {
        return regClass != MiscRegClass;
    }

    /**
     * Check if this is the zero register.
     * Returns true if this register is a zero register (needs to have a
     * constant zero value throughout the execution).
     */

    inline bool isZeroReg() const
    {
        return ((regClass == IntRegClass && regIdx == TheISA::ZeroReg) ||
               (THE_ISA == ALPHA_ISA && regClass == FloatRegClass &&
                regIdx == TheISA::ZeroReg));
    }

    /** @return true if it is an integer physical register. */
    bool isIntReg() const { return regClass == IntRegClass; }

    /** @return true if it is a floating-point physical register. */
    bool isFloatReg() const { return regClass == FloatRegClass; }

    /** @Return true if it is a  condition-code physical register. */
    bool isVecReg() const { return regClass == VecRegClass; }

    /** @Return true if it is a  condition-code physical register. */
    bool isVecElem() const { return regClass == VecElemClass; }

    /** @Return true if it is a predicate physical register. */
    bool isVecPredReg() const { return regClass == VecPredRegClass; }

    /** @Return true if it is a  condition-code physical register. */
    bool isCCReg() const { return regClass == CCRegClass; }

    /** @Return true if it is a  condition-code physical register. */
    bool isMiscReg() const { return regClass == MiscRegClass; }

    /**
     * Return true if this register can be renamed
     */
    bool isRenameable()
    {
        return regClass != MiscRegClass;
    }

    /** Index accessors */
    /** @{ */
    const RegIndex& index() const { return regIdx; }
    RegIndex& index() { return regIdx; }

    /** Index flattening.
     * Required to be able to use a vector for the register mapping.
     */
    inline RegIndex flatIndex() const
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
            return Scale*regIdx + elemIdx;
        }
        panic("Trying to flatten a register without class!");
        return -1;
    }
    /** @} */

    /** Elem accessor */
    const RegIndex& elemIndex() const { return elemIdx; }
    /** Class accessor */
    const RegClass& classValue() const { return regClass; }
    /** Return a const char* with the register class name. */
    const char* className() const { return regClassStrings[regClass]; }

    friend std::ostream&
    operator<<(std::ostream& os, const RegId& rid) {
        return os << rid.className() << "{" << rid.index() << "}";
    }
};

/** Physical register index type.
 * Although the Impl might be a better for this, but there are a few classes
 * that need this typedef yet are not templated on the Impl.
 */
using PhysRegIndex = short int;

/** Physical register ID.
 * Like a register ID but physical. The inheritance is private because the
 * only relationship between this types is functional, and it is done to
 * prevent code replication. */
class PhysRegId : private RegId {
  private:
    PhysRegIndex flatIdx;

  public:
    explicit PhysRegId() : RegId(IntRegClass, -1), flatIdx(-1) {}

    /** Scalar PhysRegId constructor. */
    explicit PhysRegId(RegClass _regClass, PhysRegIndex _regIdx,
              PhysRegIndex _flatIdx)
        : RegId(_regClass, _regIdx), flatIdx(_flatIdx)
    {}

    /** Vector PhysRegId constructor (w/ elemIndex). */
    explicit PhysRegId(RegClass _regClass, PhysRegIndex _regIdx,
              ElemIndex elem_idx, PhysRegIndex flat_idx)
        : RegId(_regClass, _regIdx, elem_idx), flatIdx(flat_idx) { }

    /** Visible RegId methods */
    /** @{ */
    using RegId::index;
    using RegId::classValue;
    using RegId::isZeroReg;
    using RegId::className;
    using RegId::elemIndex;
     /** @} */
    /**
     * Explicit forward methods, to prevent comparisons of PhysRegId with
     * RegIds.
     */
    /** @{ */
    bool operator<(const PhysRegId& that) const {
        return RegId::operator<(that);
    }

    bool operator==(const PhysRegId& that) const {
        return RegId::operator==(that);
    }

    bool operator!=(const PhysRegId& that) const {
        return RegId::operator!=(that);
    }
    /** @} */

    /** @return true if it is an integer physical register. */
    bool isIntPhysReg() const { return isIntReg(); }

    /** @return true if it is a floating-point physical register. */
    bool isFloatPhysReg() const { return isFloatReg(); }

    /** @Return true if it is a  condition-code physical register. */
    bool isCCPhysReg() const { return isCCReg(); }

    /** @Return true if it is a vector physical register. */
    bool isVectorPhysReg() const { return isVecReg(); }

    /** @Return true if it is a vector element physical register. */
    bool isVectorPhysElem() const { return isVecElem(); }

    /** @return true if it is a vector predicate physical register. */
    bool isVecPredPhysReg() const { return isVecPredReg(); }

    /** @Return true if it is a  condition-code physical register. */
    bool isMiscPhysReg() const { return isMiscReg(); }

    /**
     * Returns true if this register is always associated to the same
     * architectural register.
     */
    bool isFixedMapping() const
    {
        return !isRenameable();
    }

    /** Flat index accessor */
    const PhysRegIndex& flatIndex() const { return flatIdx; }

    static PhysRegId elemId(const PhysRegId* vid, ElemIndex elem)
    {
        assert(vid->isVectorPhysReg());
        return PhysRegId(VecElemClass, vid->index(), elem);
    }
};

/** Constant pointer definition.
 * PhysRegIds only need to be created once and then we can just share
 * pointers */
using PhysRegIdPtr = const PhysRegId*;

namespace std
{
template<>
struct hash<RegId>
{
    size_t operator()(const RegId& reg_id) const
    {
        // Extract unique integral values for the effective fields of a RegId.
        const size_t flat_index = static_cast<size_t>(reg_id.flatIndex());
        const size_t class_num = static_cast<size_t>(reg_id.regClass);

        const size_t shifted_class_num = class_num << (sizeof(RegIndex) << 3);

        // Concatenate the class_num to the end of the flat_index, in order to
        // maximize information retained.
        const size_t concatenated_hash = flat_index | shifted_class_num;

        // If RegIndex is larger than size_t, then class_num will not be
        // considered by this hash function, so we may wish to perform a
        // different operation to include that information in the hash.
        static_assert(sizeof(RegIndex) < sizeof(size_t),
            "sizeof(RegIndex) should be less than sizeof(size_t)");

        return concatenated_hash;
    }
};
}

#endif // __CPU__REG_CLASS_HH__
