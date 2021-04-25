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
#include <string>

#include "base/intmath.hh"
#include "base/types.hh"

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
    CCRegClass,         ///< Condition-code register
    MiscRegClass        ///< Control (misc) register
};

class RegId;

class RegClassOps
{
  public:
    virtual std::string regName(const RegId &id) const = 0;
};

class DefaultRegClassOps : public RegClassOps
{
  public:
    std::string regName(const RegId &id) const override;
};

class VecElemRegClassOps : public RegClassOps
{
  protected:
    size_t elemsPerVec;

  public:
    explicit VecElemRegClassOps(size_t elems_per_vec) :
        elemsPerVec(elems_per_vec)
    {}

    std::string regName(const RegId &id) const override;
};

class RegClass
{
  private:
    size_t _numRegs;
    const RegIndex _zeroReg;
    size_t _regBytes;
    // This is how much to shift an index by to get an offset of a register in
    // a register file from the register index, which would otherwise need to
    // be calculated with a multiply.
    size_t _regShift;

    static inline DefaultRegClassOps defaultOps;
    RegClassOps *_ops = &defaultOps;

  public:
    RegClass(size_t num_regs, RegIndex new_zero=-1,
            size_t reg_bytes=sizeof(RegVal)) :
        _numRegs(num_regs), _zeroReg(new_zero), _regBytes(reg_bytes),
        _regShift(ceilLog2(reg_bytes))
    {}
    RegClass(size_t num_regs, RegClassOps &new_ops, RegIndex new_zero=-1,
            size_t reg_bytes=sizeof(RegVal)) :
        RegClass(num_regs, new_zero, reg_bytes)
    {
        _ops = &new_ops;
    }

    size_t numRegs() const { return _numRegs; }
    RegIndex zeroReg() const { return _zeroReg; }
    size_t regBytes() const { return _regBytes; }
    size_t regShift() const { return _regShift; }

    std::string regName(const RegId &id) const { return _ops->regName(id); }
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
    RegClassType regClass;
    RegIndex regIdx;
    int numPinnedWrites;

    friend struct std::hash<RegId>;

  public:
    RegId() : RegId(IntRegClass, 0) {}

    explicit RegId(RegClassType reg_class, RegIndex reg_idx)
        : regClass(reg_class), regIdx(reg_idx), numPinnedWrites(0)
    {}

    bool
    operator==(const RegId& that) const
    {
        return regClass == that.classValue() && regIdx == that.index();
    }

    bool operator!=(const RegId& that) const { return !(*this==that); }

    /** Order operator.
     * The order is required to implement maps with key type RegId
     */
    bool
    operator<(const RegId& that) const
    {
        return regClass < that.classValue() ||
            (regClass == that.classValue() && (regIdx < that.index()));
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
    bool is(RegClassType reg_class) const { return regClass == reg_class; }

    /** Index accessors */
    /** @{ */
    RegIndex index() const { return regIdx; }

    /** Class accessor */
    RegClassType classValue() const { return regClass; }
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
    explicit PhysRegId(RegClassType _regClass, RegIndex _regIdx,
              RegIndex _flatIdx)
        : RegId(_regClass, _regIdx), flatIdx(_flatIdx),
          numPinnedWritesToComplete(0), pinned(false)
    {}

    /** Visible RegId methods */
    /** @{ */
    using RegId::index;
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
        const size_t class_num = static_cast<size_t>(reg_id.regClass);

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
