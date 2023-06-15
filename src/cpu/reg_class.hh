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

#include <cstddef>
#include <iterator>
#include <string>

#include "base/cprintf.hh"
#include "base/debug.hh"
#include "base/intmath.hh"
#include "base/types.hh"
#include "debug/InvalidReg.hh"

namespace gem5
{

/** Enumerate the classes of registers. */
enum RegClassType
{
    IntRegClass,        ///< Integer register
    FloatRegClass,      ///< Floating-point register
    /** Vector Register. */
    VecRegClass,
    /** Vector Register Native Elem lane. */
    VecElemClass,
    VecPredRegClass,
    MatRegClass,        ///< Matrix Register
    CCRegClass,         ///< Condition-code register
    MiscRegClass,       ///< Control (misc) register
    InvalidRegClass = -1
};

// "Standard" register class names. Using these is encouraged but optional.
inline constexpr char IntRegClassName[] = "integer";
inline constexpr char FloatRegClassName[] = "floating_point";
inline constexpr char VecRegClassName[] = "vector";
inline constexpr char VecElemClassName[] = "vector_element";
inline constexpr char VecPredRegClassName[] = "vector_predicate";
inline constexpr char MatRegClassName[] = "matrix";
inline constexpr char CCRegClassName[] = "condition_code";
inline constexpr char MiscRegClassName[] = "miscellaneous";

class RegClass;
class RegClassIterator;
class BaseISA;

/** Register ID: describe an architectural register with its class and index.
 * This structure is used instead of just the register index to disambiguate
 * between different classes of registers. For example, a integer register with
 * index 3 is represented by Regid(IntRegClass, 3).
 */
class RegId
{
  protected:
    const RegClass *_regClass = nullptr;
    RegIndex regIdx;
    int numPinnedWrites;

    friend struct std::hash<RegId>;
    friend class RegClassIterator;

  public:
    inline constexpr RegId();

    constexpr RegId(const RegClass &reg_class, RegIndex reg_idx)
        : _regClass(&reg_class), regIdx(reg_idx), numPinnedWrites(0)
    {}

    constexpr operator RegIndex() const
    {
        return index();
    }

    constexpr bool
    operator==(const RegId& that) const
    {
        return classValue() == that.classValue() && regIdx == that.index();
    }

    constexpr bool
    operator!=(const RegId& that) const
    {
        return !(*this==that);
    }

    /** Order operator.
     * The order is required to implement maps with key type RegId
     */
    constexpr bool
    operator<(const RegId& that) const
    {
        return classValue() < that.classValue() ||
            (classValue() == that.classValue() && (regIdx < that.index()));
    }

    /**
     * Return true if this register can be renamed
     */
    constexpr bool
    isRenameable() const
    {
        return classValue() != MiscRegClass && classValue() != InvalidRegClass;
    }

    /** @return true if it is of the specified class. */
    inline constexpr bool is(RegClassType reg_class) const;

    /** Index accessors */
    /** @{ */
    constexpr RegIndex index() const { return regIdx; }

    /** Class accessor */
    constexpr const RegClass &regClass() const { return *_regClass; }
    inline constexpr RegClassType classValue() const;
    /** Return a const char* with the register class name. */
    inline constexpr const char* className() const;

    inline constexpr bool isFlat() const;
    inline RegId flatten(const BaseISA &isa) const;

    int getNumPinnedWrites() const { return numPinnedWrites; }
    void setNumPinnedWrites(int num_writes) { numPinnedWrites = num_writes; }

    friend inline std::ostream& operator<<(std::ostream& os, const RegId& rid);
};

class RegClassOps
{
  public:
    /** Print the name of the register specified in id. */
    virtual std::string regName(const RegId &id) const;
    /** Print the value of a register pointed to by val of size size. */
    virtual std::string valString(const void *val, size_t size) const;
    /** Flatten register id id using information in the ISA object isa. */
    virtual RegId
    flatten(const BaseISA &isa, const RegId &id) const
    {
        return id;
    }
};

class RegClassIterator;

class RegClass
{
  private:
    RegClassType _type;
    const char *_name;

    size_t _numRegs;
    size_t _regBytes = sizeof(RegVal);
    // This is how much to shift an index by to get an offset of a register in
    // a register file from the register index, which would otherwise need to
    // be calculated with a multiply.
    size_t _regShift = ceilLog2(sizeof(RegVal));

    static inline RegClassOps defaultOps;
    const RegClassOps *_ops = &defaultOps;
    const debug::Flag &debugFlag;

    bool _flat = true;

  public:
    constexpr RegClass(RegClassType type, const char *new_name,
            size_t num_regs, const debug::Flag &debug_flag) :
        _type(type), _name(new_name), _numRegs(num_regs), debugFlag(debug_flag)
    {}

    constexpr RegClass
    needsFlattening() const
    {
        RegClass reg_class = *this;
        reg_class._flat = false;
        return reg_class;
    }

    constexpr RegClass
    ops(const RegClassOps &new_ops) const
    {
        RegClass reg_class = *this;
        reg_class._ops = &new_ops;
        return reg_class;
    }

    template <class RegType>
    constexpr RegClass
    regType() const
    {
        RegClass reg_class = *this;
        reg_class._regBytes = sizeof(RegType);
        reg_class._regShift = ceilLog2(reg_class._regBytes);
        return reg_class;
    }

    constexpr RegClassType type() const { return _type; }
    constexpr const char *name() const { return _name; }
    constexpr size_t numRegs() const { return _numRegs; }
    constexpr size_t regBytes() const { return _regBytes; }
    constexpr size_t regShift() const { return _regShift; }
    constexpr const debug::Flag &debug() const { return debugFlag; }
    constexpr bool isFlat() const { return _flat; }

    std::string regName(const RegId &id) const { return _ops->regName(id); }
    std::string
    valString(const void *val) const
    {
        return _ops->valString(val, regBytes());
    }
    RegId
    flatten(const BaseISA &isa, const RegId &id) const
    {
        return isFlat() ? id : _ops->flatten(isa, id);
    }

    using iterator = RegClassIterator;

    inline iterator begin() const;
    inline iterator end() const;

    inline constexpr RegId operator[](RegIndex idx) const;
};

inline constexpr RegClass
    invalidRegClass(InvalidRegClass, "invalid", 0, debug::InvalidReg);

constexpr RegId::RegId() : RegId(invalidRegClass, 0) {}

constexpr bool
RegId::is(RegClassType reg_class) const
{
    return _regClass->type() == reg_class;
}

constexpr RegClassType RegId::classValue() const { return _regClass->type(); }
constexpr const char* RegId::className() const { return _regClass->name(); }

constexpr bool RegId::isFlat() const { return _regClass->isFlat(); }
RegId
RegId::flatten(const BaseISA &isa) const
{
    return _regClass->flatten(isa, *this);
}

std::ostream&
operator<<(std::ostream& os, const RegId& rid)
{
    return os << rid.regClass().regName(rid);
}

class RegClassIterator
{
  private:
    RegId id;

    RegClassIterator(const RegClass &reg_class, RegIndex idx) :
        id(reg_class, idx)
    {}

    friend class RegClass;

  public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::size_t;
    using value_type = const RegId;
    using pointer = value_type *;
    using reference = value_type &;

    reference operator*() const { return id; }
    pointer operator->() { return &id; }

    RegClassIterator &
    operator++()
    {
        id.regIdx++;
        return *this;
    }

    RegClassIterator
    operator++(int)
    {
        auto tmp = *this;
        ++(*this);
        return tmp;
    }

    bool
    operator==(const RegClassIterator &other) const
    {
        return id == other.id;
    }

    bool
    operator!=(const RegClassIterator &other) const
    {
        return id != other.id;
    }
};

RegClassIterator
RegClass::begin() const
{
    return RegClassIterator(*this, 0);
}

RegClassIterator
RegClass::end() const
{
    return RegClassIterator(*this, numRegs());
}

constexpr RegId
RegClass::operator[](RegIndex idx) const
{
    return RegId(*this, idx);
}

template <typename ValueType>
class TypedRegClassOps : public RegClassOps
{
  public:
    std::string
    valString(const void *val, size_t size) const override
    {
        assert(size == sizeof(ValueType));
        return csprintf("%s", *(const ValueType *)val);
    }
};

template <typename ValueType>
class VecElemRegClassOps : public TypedRegClassOps<ValueType>
{
  protected:
    size_t elemsPerVec;

  public:
    explicit VecElemRegClassOps(size_t elems_per_vec) :
        elemsPerVec(elems_per_vec)
    {}

    std::string
    regName(const RegId &id) const override
    {
        RegIndex reg_idx = id.index() / elemsPerVec;
        RegIndex elem_idx = id.index() % elemsPerVec;
        return csprintf("v%d[%d]", reg_idx, elem_idx);
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
    explicit PhysRegId() : RegId(invalidRegClass, -1), flatIdx(-1),
                           numPinnedWritesToComplete(0)
    {}

    /** Scalar PhysRegId constructor. */
    explicit PhysRegId(const RegClass &reg_class, RegIndex _regIdx,
              RegIndex _flatIdx)
        : RegId(reg_class, _regIdx), flatIdx(_flatIdx),
          numPinnedWritesToComplete(0), pinned(false)
    {}

    /** Visible RegId methods */
    /** @{ */
    using RegId::index;
    using RegId::regClass;
    using RegId::classValue;
    using RegId::className;
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
        const size_t index = static_cast<size_t>(reg_id.index());
        const size_t class_num = static_cast<size_t>(reg_id.classValue());

        const size_t shifted_class_num =
            class_num << (sizeof(gem5::RegIndex) << 3);

        // Concatenate the class_num to the end of the flat_index, in order to
        // maximize information retained.
        const size_t concatenated_hash = index | shifted_class_num;

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
