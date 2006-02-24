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
 */

#ifndef __CPU_STATIC_INST_HH__
#define __CPU_STATIC_INST_HH__

#include <bitset>
#include <string>

#include "base/hashmap.hh"
#include "base/refcnt.hh"
#include "encumbered/cpu/full/op_class.hh"
#include "sim/host.hh"
#include "arch/isa_traits.hh"

// forward declarations
struct AlphaSimpleImpl;
class ExecContext;
class DynInst;

template <class Impl>
class AlphaDynInst;

class FastCPU;
class SimpleCPU;
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
class StaticInstBase : public RefCounted
{
  protected:

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
        IsNop,		///< Is a no-op (no effect at all).

        IsInteger,	///< References integer regs.
        IsFloating,	///< References FP regs.

        IsMemRef,	///< References memory (load, store, or prefetch).
        IsLoad,		///< Reads from memory (load or prefetch).
        IsStore,	///< Writes to memory.
        IsInstPrefetch,	///< Instruction-cache prefetch.
        IsDataPrefetch,	///< Data-cache prefetch.
        IsCopy,         ///< Fast Cache block copy

        IsControl,		///< Control transfer instruction.
        IsDirectControl,	///< PC relative control transfer.
        IsIndirectControl,	///< Register indirect control transfer.
        IsCondControl,		///< Conditional control transfer.
        IsUncondControl,	///< Unconditional control transfer.
        IsCall,			///< Subroutine call.
        IsReturn,		///< Subroutine return.

        IsCondDelaySlot,///< Conditional Delay-Slot Instruction

        IsThreadSync,	///< Thread synchronization operation.

        IsSerializing,	///< Serializes pipeline: won't execute until all
                        /// older instructions have committed.
        IsSerializeBefore,
        IsSerializeAfter,
        IsMemBarrier,	///< Is a memory barrier
        IsWriteBarrier,	///< Is a write barrier

        IsNonSpeculative, ///< Should not be executed speculatively

        NumFlags
    };

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

    /// Constructor.
    /// It's important to initialize everything here to a sane
    /// default, since the decoder generally only overrides
    /// the fields that are meaningful for the particular
    /// instruction.
    StaticInstBase(OpClass __opClass)
        : _opClass(__opClass), _numSrcRegs(0), _numDestRegs(0),
          _numFPDestRegs(0), _numIntDestRegs(0)
    {
    }

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
    /// instruction property flags.  See StaticInstBase::Flags for descriptions
    /// of the individual flags.
    //@{

    bool isNop() 	  const { return flags[IsNop]; }

    bool isMemRef()    	  const { return flags[IsMemRef]; }
    bool isLoad()	  const { return flags[IsLoad]; }
    bool isStore()	  const { return flags[IsStore]; }
    bool isInstPrefetch() const { return flags[IsInstPrefetch]; }
    bool isDataPrefetch() const { return flags[IsDataPrefetch]; }
    bool isCopy()         const { return flags[IsCopy];}

    bool isInteger()	  const { return flags[IsInteger]; }
    bool isFloating()	  const { return flags[IsFloating]; }

    bool isControl()	  const { return flags[IsControl]; }
    bool isCall()	  const { return flags[IsCall]; }
    bool isReturn()	  const { return flags[IsReturn]; }
    bool isDirectCtrl()	  const { return flags[IsDirectControl]; }
    bool isIndirectCtrl() const { return flags[IsIndirectControl]; }
    bool isCondCtrl()	  const { return flags[IsCondControl]; }
    bool isUncondCtrl()	  const { return flags[IsUncondControl]; }

    bool isThreadSync()   const { return flags[IsThreadSync]; }
    bool isSerializing()  const { return flags[IsSerializing] ||
                                      flags[IsSerializeBefore] ||
                                      flags[IsSerializeAfter]; }
    bool isSerializeBefore() const { return flags[IsSerializeBefore]; }
    bool isSerializeAfter() const { return flags[IsSerializeAfter]; }
    bool isMemBarrier()   const { return flags[IsMemBarrier]; }
    bool isWriteBarrier() const { return flags[IsWriteBarrier]; }
    bool isNonSpeculative() const { return flags[IsNonSpeculative]; }
    //@}

    /// Operation class.  Used to select appropriate function unit in issue.
    OpClass opClass()     const { return _opClass; }
};


// forward declaration
class StaticInstPtr;

/**
 * Generic yet ISA-dependent static instruction class.
 *
 * This class builds on StaticInstBase, defining fields and interfaces
 * that are generic across all ISAs but that differ in details
 * according to the specific ISA being used.
 */
class StaticInst : public StaticInstBase
{
  public:

    /// Binary machine instruction type.
    typedef TheISA::MachInst MachInst;
    /// Logical register index type.
    typedef TheISA::RegIndex RegIndex;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,	//< Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs,	//< Max dest regs
    };


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
    const MachInst machInst;

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
    StaticInst(const char *_mnemonic, MachInst _machInst, OpClass __opClass)
        : StaticInstBase(__opClass),
          machInst(_machInst), mnemonic(_mnemonic), cachedDisassembly(0)
    {
    }

  public:

    virtual ~StaticInst()
    {
        if (cachedDisassembly)
            delete cachedDisassembly;
    }

/**
 * The execute() signatures are auto-generated by scons based on the
 * set of CPU models we are compiling in today.
 */
#include "cpu/static_inst_exec_sigs.hh"

    /**
     * Return the target address for a PC-relative branch.
     * Invalid if not a PC-relative branch (i.e. isDirectCtrl()
     * should be true).
     */
    virtual Addr branchTarget(Addr branchPC) const
    {
        panic("StaticInst::branchTarget() called on instruction "
              "that is not a PC-relative branch.");
    }

    /**
     * Return the target address for an indirect branch (jump).  The
     * register value is read from the supplied execution context, so
     * the result is valid only if the execution context is about to
     * execute the branch in question.  Invalid if not an indirect
     * branch (i.e. isIndirectCtrl() should be true).
     */
    virtual Addr branchTarget(ExecContext *xc) const
    {
        panic("StaticInst::branchTarget() called on instruction "
              "that is not an indirect branch.");
    }

    /**
     * Return true if the instruction is a control transfer, and if so,
     * return the target address as well.
     */
    bool hasBranchTarget(Addr pc, ExecContext *xc, Addr &tgt) const;

    /**
     * Return string representation of disassembled instruction.
     * The default version of this function will call the internal
     * virtual generateDisassembly() function to get the string,
     * then cache it in #cachedDisassembly.  If the disassembly
     * should not be cached, this function should be overridden directly.
     */
    virtual const std::string &disassemble(Addr pc,
                                           const SymbolTable *symtab = 0) const
    {
        if (!cachedDisassembly)
            cachedDisassembly =
                new std::string(generateDisassembly(pc, symtab));

        return *cachedDisassembly;
    }

    /// Decoded instruction cache type.
    /// For now we're using a generic hash_map; this seems to work
    /// pretty well.
    typedef m5::hash_map<MachInst, StaticInstPtr> DecodeCache;

    /// A cache of decoded instruction objects.
    static DecodeCache decodeCache;

    /**
     * Dump some basic stats on the decode cache hash map.
     * Only gets called if DECODE_CACHE_HASH_STATS is defined.
     */
    static void dumpDecodeCacheStats();

    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    //This is defined as inline below.
    static StaticInstPtr decode(MachInst mach_inst);
};

typedef RefCountingPtr<StaticInstBase> StaticInstBasePtr;

/// Reference-counted pointer to a StaticInst object.
/// This type should be used instead of "StaticInst *" so that
/// StaticInst objects can be properly reference-counted.
class StaticInstPtr : public RefCountingPtr<StaticInst>
{
  public:
    /// Constructor.
    StaticInstPtr()
        : RefCountingPtr<StaticInst>()
    {
    }

    /// Conversion from "StaticInst *".
    StaticInstPtr(StaticInst *p)
        : RefCountingPtr<StaticInst>(p)
    {
    }

    /// Copy constructor.
    StaticInstPtr(const StaticInstPtr &r)
        : RefCountingPtr<StaticInst>(r)
    {
    }

    /// Construct directly from machine instruction.
    /// Calls StaticInst::decode().
    StaticInstPtr(TheISA::MachInst mach_inst)
        : RefCountingPtr<StaticInst>(StaticInst::decode(mach_inst))
    {
    }

    /// Convert to pointer to StaticInstBase class.
    operator const StaticInstBasePtr()
    {
        return this->get();
    }
};

inline StaticInstPtr
StaticInst::decode(StaticInst::MachInst mach_inst)
{
#ifdef DECODE_CACHE_HASH_STATS
    // Simple stats on decode hash_map.  Turns out the default
    // hash function is as good as anything I could come up with.
    const int dump_every_n = 10000000;
    static int decodes_til_dump = dump_every_n;

    if (--decodes_til_dump == 0) {
        dumpDecodeCacheStats();
        decodes_til_dump = dump_every_n;
    }
#endif

    DecodeCache::iterator iter = decodeCache.find(mach_inst);
    if (iter != decodeCache.end()) {
        return iter->second;
    }

    StaticInstPtr si = TheISA::decodeInst(mach_inst);
    decodeCache[mach_inst] = si;
    return si;
}

#endif // __CPU_STATIC_INST_HH__
