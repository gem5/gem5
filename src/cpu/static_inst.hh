/*
 * Copyright (c) 2017, 2020 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#ifndef __CPU_STATIC_INST_HH__
#define __CPU_STATIC_INST_HH__

#include <bitset>
#include <memory>
#include <string>

#include "arch/registers.hh"
#include "arch/types.hh"
#include "base/logging.hh"
#include "base/refcnt.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/op_class.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/thread_context.hh"
#include "enums/StaticInstFlags.hh"
#include "sim/byteswap.hh"

// forward declarations
class Packet;

class ExecContext;

namespace Loader
{
class SymbolTable;
} // namespace Loader

namespace Trace
{
class InstRecord;
} // namespace Trace

/**
 * Base, ISA-independent static instruction class.
 *
 * The main component of this class is the vector of flags and the
 * associated methods for reading them.  Any object that can rely
 * solely on these flags can process instructions without being
 * recompiled for multiple ISAs.
 */
class StaticInst : public RefCounted, public StaticInstFlags
{
  public:
    /// Binary extended machine instruction type.
    typedef TheISA::ExtMachInst ExtMachInst;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,        //< Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs       //< Max dest regs
    };

  protected:

    /// Flag values for this instruction.
    std::bitset<Num_Flags> flags;

    /// See opClass().
    OpClass _opClass;

    /// See numSrcRegs().
    int8_t _numSrcRegs;

    /// See numDestRegs().
    int8_t _numDestRegs;

    /// The following are used to track physical register usage
    /// for machines with separate int & FP reg files.
    //@{
    int8_t _numFPDestRegs;
    int8_t _numIntDestRegs;
    int8_t _numCCDestRegs;
    //@}

    /** To use in architectures with vector register file. */
    /** @{ */
    int8_t _numVecDestRegs;
    int8_t _numVecElemDestRegs;
    int8_t _numVecPredDestRegs;
    /** @} */

  public:

    /// @name Register information.
    /// The sum of numFPDestRegs(), numIntDestRegs(), numVecDestRegs(),
    /// numVecElemDestRegs() and numVecPredDestRegs() equals numDestRegs().
    /// The former two functions are used to track physical register usage for
    /// machines with separate int & FP reg files, the next three are for
    /// machines with vector and predicate register files.
    //@{
    /// Number of source registers.
    int8_t numSrcRegs()  const { return _numSrcRegs; }
    /// Number of destination registers.
    int8_t numDestRegs() const { return _numDestRegs; }
    /// Number of floating-point destination regs.
    int8_t numFPDestRegs()  const { return _numFPDestRegs; }
    /// Number of integer destination regs.
    int8_t numIntDestRegs() const { return _numIntDestRegs; }
    /// Number of vector destination regs.
    int8_t numVecDestRegs() const { return _numVecDestRegs; }
    /// Number of vector element destination regs.
    int8_t numVecElemDestRegs() const { return _numVecElemDestRegs; }
    /// Number of predicate destination regs.
    int8_t numVecPredDestRegs() const { return _numVecPredDestRegs; }
    /// Number of coprocesor destination regs.
    int8_t numCCDestRegs() const { return _numCCDestRegs; }
    //@}

    /// @name Flag accessors.
    /// These functions are used to access the values of the various
    /// instruction property flags.  See StaticInst::Flags for descriptions
    /// of the individual flags.
    //@{

    bool isNop()          const { return flags[IsNop]; }

    bool isMemRef()       const { return flags[IsMemRef]; }
    bool isLoad()         const { return flags[IsLoad]; }
    bool isStore()        const { return flags[IsStore]; }
    bool isAtomic()       const { return flags[IsAtomic]; }
    bool isStoreConditional()     const { return flags[IsStoreConditional]; }
    bool isInstPrefetch() const { return flags[IsInstPrefetch]; }
    bool isDataPrefetch() const { return flags[IsDataPrefetch]; }
    bool isPrefetch()     const { return isInstPrefetch() ||
                                         isDataPrefetch(); }

    bool isInteger()      const { return flags[IsInteger]; }
    bool isFloating()     const { return flags[IsFloating]; }
    bool isVector()       const { return flags[IsVector]; }
    bool isCC()           const { return flags[IsCC]; }

    bool isControl()      const { return flags[IsControl]; }
    bool isCall()         const { return flags[IsCall]; }
    bool isReturn()       const { return flags[IsReturn]; }
    bool isDirectCtrl()   const { return flags[IsDirectControl]; }
    bool isIndirectCtrl() const { return flags[IsIndirectControl]; }
    bool isCondCtrl()     const { return flags[IsCondControl]; }
    bool isUncondCtrl()   const { return flags[IsUncondControl]; }
    bool isCondDelaySlot() const { return flags[IsCondDelaySlot]; }

    bool isThreadSync()   const { return flags[IsThreadSync]; }
    bool isSerializing()  const { return flags[IsSerializing] ||
                                      flags[IsSerializeBefore] ||
                                      flags[IsSerializeAfter]; }
    bool isSerializeBefore() const { return flags[IsSerializeBefore]; }
    bool isSerializeAfter() const { return flags[IsSerializeAfter]; }
    bool isSquashAfter() const { return flags[IsSquashAfter]; }
    bool isMemBarrier()   const { return flags[IsMemBarrier]; }
    bool isWriteBarrier() const { return flags[IsWriteBarrier]; }
    bool isNonSpeculative() const { return flags[IsNonSpeculative]; }
    bool isQuiesce() const { return flags[IsQuiesce]; }
    bool isIprAccess() const { return flags[IsIprAccess]; }
    bool isUnverifiable() const { return flags[IsUnverifiable]; }
    bool isSyscall() const { return flags[IsSyscall]; }
    bool isMacroop() const { return flags[IsMacroop]; }
    bool isMicroop() const { return flags[IsMicroop]; }
    bool isDelayedCommit() const { return flags[IsDelayedCommit]; }
    bool isLastMicroop() const { return flags[IsLastMicroop]; }
    bool isFirstMicroop() const { return flags[IsFirstMicroop]; }
    //This flag doesn't do anything yet
    bool isMicroBranch() const { return flags[IsMicroBranch]; }
    // hardware transactional memory
    // HtmCmds must be identified as such in order
    // to provide them with necessary memory ordering semantics.
    bool isHtmStart() const { return flags[IsHtmStart]; }
    bool isHtmStop() const { return flags[IsHtmStop]; }
    bool isHtmCancel() const { return flags[IsHtmCancel]; }

    bool
    isHtmCmd() const
    {
        return isHtmStart() || isHtmStop() || isHtmCancel();
    }
    //@}

    void setFirstMicroop() { flags[IsFirstMicroop] = true; }
    void setLastMicroop() { flags[IsLastMicroop] = true; }
    void setDelayedCommit() { flags[IsDelayedCommit] = true; }
    void setFlag(Flags f) { flags[f] = true; }

    /// Operation class.  Used to select appropriate function unit in issue.
    OpClass opClass()     const { return _opClass; }


    /// Return logical index (architectural reg num) of i'th destination reg.
    /// Only the entries from 0 through numDestRegs()-1 are valid.
    const RegId& destRegIdx(int i) const { return _destRegIdx[i]; }

    /// Return logical index (architectural reg num) of i'th source reg.
    /// Only the entries from 0 through numSrcRegs()-1 are valid.
    const RegId& srcRegIdx(int i)  const { return _srcRegIdx[i]; }

    /// Pointer to a statically allocated "null" instruction object.
    static StaticInstPtr nullStaticInstPtr;

    /// Pointer to a statically allocated generic "nop" instruction object.
    static StaticInstPtr nopStaticInstPtr;

    /// The binary machine instruction.
    const ExtMachInst machInst;

  protected:

    /// See destRegIdx().
    RegId _destRegIdx[MaxInstDestRegs];
    /// See srcRegIdx().
    RegId _srcRegIdx[MaxInstSrcRegs];

    /**
     * Base mnemonic (e.g., "add").  Used by generateDisassembly()
     * methods.  Also useful to readily identify instructions from
     * within the debugger when #cachedDisassembly has not been
     * initialized.
     */
    const char *mnemonic;

    /**
     * String representation of disassembly (lazily evaluated via
     * disassemble()).
     */
    mutable std::string *cachedDisassembly;

    /**
     * Internal function to generate disassembly string.
     */
    virtual std::string
    generateDisassembly(Addr pc, const Loader::SymbolTable *symtab) const = 0;

    /// Constructor.
    /// It's important to initialize everything here to a sane
    /// default, since the decoder generally only overrides
    /// the fields that are meaningful for the particular
    /// instruction.
    StaticInst(const char *_mnemonic, ExtMachInst _machInst, OpClass __opClass)
        : _opClass(__opClass), _numSrcRegs(0), _numDestRegs(0),
          _numFPDestRegs(0), _numIntDestRegs(0), _numCCDestRegs(0),
          _numVecDestRegs(0), _numVecElemDestRegs(0), _numVecPredDestRegs(0),
          machInst(_machInst), mnemonic(_mnemonic), cachedDisassembly(0)
    { }

  public:
    virtual ~StaticInst();

    virtual Fault execute(ExecContext *xc,
                          Trace::InstRecord *traceData) const = 0;

    virtual Fault initiateAcc(ExecContext *xc,
                              Trace::InstRecord *traceData) const
    {
        panic("initiateAcc not defined!");
    }

    virtual Fault completeAcc(Packet *pkt, ExecContext *xc,
                              Trace::InstRecord *traceData) const
    {
        panic("completeAcc not defined!");
    }

    virtual void advancePC(TheISA::PCState &pcState) const = 0;

    /**
     * Return the microop that goes with a particular micropc. This should
     * only be defined/used in macroops which will contain microops
     */
    virtual StaticInstPtr fetchMicroop(MicroPC upc) const;

    /**
     * Return the target address for a PC-relative branch.
     * Invalid if not a PC-relative branch (i.e. isDirectCtrl()
     * should be true).
     */
    virtual TheISA::PCState branchTarget(const TheISA::PCState &pc) const;

    /**
     * Return the target address for an indirect branch (jump).  The
     * register value is read from the supplied thread context, so
     * the result is valid only if the thread context is about to
     * execute the branch in question.  Invalid if not an indirect
     * branch (i.e. isIndirectCtrl() should be true).
     */
    virtual TheISA::PCState branchTarget(ThreadContext *tc) const;

    /**
     * Return true if the instruction is a control transfer, and if so,
     * return the target address as well.
     */
    bool hasBranchTarget(const TheISA::PCState &pc, ThreadContext *tc,
                         TheISA::PCState &tgt) const;

    /**
     * Return string representation of disassembled instruction.
     * The default version of this function will call the internal
     * virtual generateDisassembly() function to get the string,
     * then cache it in #cachedDisassembly.  If the disassembly
     * should not be cached, this function should be overridden directly.
     */
    virtual const std::string &disassemble(Addr pc,
        const Loader::SymbolTable *symtab=nullptr) const;

    /**
     * Print a separator separated list of this instruction's set flag
     * names on the given stream.
     */
    void printFlags(std::ostream &outs, const std::string &separator) const;

    /// Return name of machine instruction
    std::string getName() { return mnemonic; }

  protected:
    template<typename T>
    size_t
    simpleAsBytes(void *buf, size_t max_size, const T &t)
    {
        size_t size = sizeof(T);
        if (size <= max_size)
            *reinterpret_cast<T *>(buf) = htole<T>(t);
        return size;
    }

  public:
    /**
     * Instruction classes can override this function to return a
     * a representation of themselves as a blob of bytes, generally assumed to
     * be that instructions ExtMachInst.
     *
     * buf is a buffer to hold the bytes.
     * max_size is the size allocated for that buffer by the caller.
     * The return value is how much data was actually put into the buffer,
     * zero if no data was put in the buffer, or the necessary size of the
     * buffer if there wasn't enough space.
     */
    virtual size_t asBytes(void *buf, size_t max_size) { return 0; }
};

#endif // __CPU_STATIC_INST_HH__
