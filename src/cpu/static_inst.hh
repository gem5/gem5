/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Steve Reinhardt
 */

#ifndef __CPU_STATIC_INST_HH__
#define __CPU_STATIC_INST_HH__

#include <bitset>
#include <string>

#include "arch/registers.hh"
#include "arch/types.hh"
#include "base/misc.hh"
#include "base/refcnt.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/op_class.hh"
#include "cpu/static_inst_fwd.hh"
#include "sim/fault_fwd.hh"

// forward declarations
struct AlphaSimpleImpl;
struct OzoneImpl;
struct SimpleImpl;
class ThreadContext;
class DynInst;
class Packet;

struct O3CPUImpl;
template <class Impl> class BaseO3DynInst;
typedef BaseO3DynInst<O3CPUImpl> O3DynInst;
template <class Impl> class OzoneDynInst;
class InOrderDynInst;

class CheckerCPU;
class FastCPU;
class AtomicSimpleCPU;
class TimingSimpleCPU;
class InorderCPU;
class SymbolTable;

namespace Trace {
    class InstRecord;
}

/**
 * Base, ISA-independent static instruction class.
 *
 * The main component of this class is the vector of flags and the
 * associated methods for reading them.  Any object that can rely
 * solely on these flags can process instructions without being
 * recompiled for multiple ISAs.
 */
class StaticInst : public RefCounted
{
  public:
    /// Binary extended machine instruction type.
    typedef TheISA::ExtMachInst ExtMachInst;
    /// Logical register index type.
    typedef TheISA::RegIndex RegIndex;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,        //< Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs       //< Max dest regs
    };

    /// Set of boolean static instruction properties.
    ///
    /// Notes:
    /// - The IsInteger and IsFloating flags are based on the class of
    /// registers accessed by the instruction.  Although most
    /// instructions will have exactly one of these two flags set, it
    /// is possible for an instruction to have neither (e.g., direct
    /// unconditional branches, memory barriers) or both (e.g., an
    /// FP/int conversion).
    /// - If IsMemRef is set, then exactly one of IsLoad or IsStore
    /// will be set.
    /// - If IsControl is set, then exactly one of IsDirectControl or
    /// IsIndirect Control will be set, and exactly one of
    /// IsCondControl or IsUncondControl will be set.
    /// - IsSerializing, IsMemBarrier, and IsWriteBarrier are
    /// implemented as flags since in the current model there's no
    /// other way for instructions to inject behavior into the
    /// pipeline outside of fetch.  Once we go to an exec-in-exec CPU
    /// model we should be able to get rid of these flags and
    /// implement this behavior via the execute() methods.
    ///
    enum Flags {
        IsNop,          ///< Is a no-op (no effect at all).

        IsInteger,      ///< References integer regs.
        IsFloating,     ///< References FP regs.

        IsMemRef,       ///< References memory (load, store, or prefetch).
        IsLoad,         ///< Reads from memory (load or prefetch).
        IsStore,        ///< Writes to memory.
        IsStoreConditional,    ///< Store conditional instruction.
        IsIndexed,      ///< Accesses memory with an indexed address computation
        IsInstPrefetch, ///< Instruction-cache prefetch.
        IsDataPrefetch, ///< Data-cache prefetch.

        IsControl,              ///< Control transfer instruction.
        IsDirectControl,        ///< PC relative control transfer.
        IsIndirectControl,      ///< Register indirect control transfer.
        IsCondControl,          ///< Conditional control transfer.
        IsUncondControl,        ///< Unconditional control transfer.
        IsCall,                 ///< Subroutine call.
        IsReturn,               ///< Subroutine return.

        IsCondDelaySlot,///< Conditional Delay-Slot Instruction

        IsThreadSync,   ///< Thread synchronization operation.

        IsSerializing,  ///< Serializes pipeline: won't execute until all
                        /// older instructions have committed.
        IsSerializeBefore,
        IsSerializeAfter,
        IsMemBarrier,   ///< Is a memory barrier
        IsWriteBarrier, ///< Is a write barrier
        IsReadBarrier,  ///< Is a read barrier
        IsERET, /// <- Causes the IFU to stall (MIPS ISA)

        IsNonSpeculative, ///< Should not be executed speculatively
        IsQuiesce,      ///< Is a quiesce instruction

        IsIprAccess,    ///< Accesses IPRs
        IsUnverifiable, ///< Can't be verified by a checker

        IsSyscall,      ///< Causes a system call to be emulated in syscall
                        /// emulation mode.

        //Flags for microcode
        IsMacroop,      ///< Is a macroop containing microops
        IsMicroop,      ///< Is a microop
        IsDelayedCommit,        ///< This microop doesn't commit right away
        IsLastMicroop,  ///< This microop ends a microop sequence
        IsFirstMicroop, ///< This microop begins a microop sequence
        //This flag doesn't do anything yet
        IsMicroBranch,  ///< This microop branches within the microcode for a macroop
        IsDspOp,
        IsSquashAfter, ///< Squash all uncommitted state after executed
        NumFlags
    };

  protected:

    /// Flag values for this instruction.
    std::bitset<NumFlags> flags;

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
    //@}

  public:

    /// @name Register information.
    /// The sum of numFPDestRegs() and numIntDestRegs() equals
    /// numDestRegs().  The former two functions are used to track
    /// physical register usage for machines with separate int & FP
    /// reg files.
    //@{
    /// Number of source registers.
    int8_t numSrcRegs()  const { return _numSrcRegs; }
    /// Number of destination registers.
    int8_t numDestRegs() const { return _numDestRegs; }
    /// Number of floating-point destination regs.
    int8_t numFPDestRegs()  const { return _numFPDestRegs; }
    /// Number of integer destination regs.
    int8_t numIntDestRegs() const { return _numIntDestRegs; }
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
    bool isStoreConditional()     const { return flags[IsStoreConditional]; }
    bool isInstPrefetch() const { return flags[IsInstPrefetch]; }
    bool isDataPrefetch() const { return flags[IsDataPrefetch]; }
    bool isPrefetch()     const { return isInstPrefetch() ||
                                         isDataPrefetch(); }

    bool isInteger()      const { return flags[IsInteger]; }
    bool isFloating()     const { return flags[IsFloating]; }

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
    //@}

    void setLastMicroop() { flags[IsLastMicroop] = true; }
    void setDelayedCommit() { flags[IsDelayedCommit] = true; }
    void setFlag(Flags f) { flags[f] = true; }

    /// Operation class.  Used to select appropriate function unit in issue.
    OpClass opClass()     const { return _opClass; }


    /// Return logical index (architectural reg num) of i'th destination reg.
    /// Only the entries from 0 through numDestRegs()-1 are valid.
    RegIndex destRegIdx(int i) const { return _destRegIdx[i]; }

    /// Return logical index (architectural reg num) of i'th source reg.
    /// Only the entries from 0 through numSrcRegs()-1 are valid.
    RegIndex srcRegIdx(int i)  const { return _srcRegIdx[i]; }

    /// Pointer to a statically allocated "null" instruction object.
    /// Used to give eaCompInst() and memAccInst() something to return
    /// when called on non-memory instructions.
    static StaticInstPtr nullStaticInstPtr;

    /**
     * Memory references only: returns "fake" instruction representing
     * the effective address part of the memory operation.  Used to
     * obtain the dependence info (numSrcRegs and srcRegIdx[]) for
     * just the EA computation.
     */
    virtual const
    StaticInstPtr &eaCompInst() const { return nullStaticInstPtr; }

    /**
     * Memory references only: returns "fake" instruction representing
     * the memory access part of the memory operation.  Used to
     * obtain the dependence info (numSrcRegs and srcRegIdx[]) for
     * just the memory access (not the EA computation).
     */
    virtual const
    StaticInstPtr &memAccInst() const { return nullStaticInstPtr; }

    /// The binary machine instruction.
    const ExtMachInst machInst;

  protected:

    /// See destRegIdx().
    RegIndex _destRegIdx[MaxInstDestRegs];
    /// See srcRegIdx().
    RegIndex _srcRegIdx[MaxInstSrcRegs];

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
    generateDisassembly(Addr pc, const SymbolTable *symtab) const = 0;

    /// Constructor.
    /// It's important to initialize everything here to a sane
    /// default, since the decoder generally only overrides
    /// the fields that are meaningful for the particular
    /// instruction.
    StaticInst(const char *_mnemonic, ExtMachInst _machInst, OpClass __opClass)
        : _opClass(__opClass), _numSrcRegs(0), _numDestRegs(0),
          _numFPDestRegs(0), _numIntDestRegs(0),
          machInst(_machInst), mnemonic(_mnemonic), cachedDisassembly(0)
    { }

  public:
    virtual ~StaticInst();

/**
 * The execute() signatures are auto-generated by scons based on the
 * set of CPU models we are compiling in today.
 */
#include "cpu/static_inst_exec_sigs.hh"

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
        const SymbolTable *symtab = 0) const;

    /// Return name of machine instruction
    std::string getName() { return mnemonic; }
};

#endif // __CPU_STATIC_INST_HH__
