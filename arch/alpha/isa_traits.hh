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

#ifndef __ARCH_ALPHA_ISA_TRAITS_HH__
#define __ARCH_ALPHA_ISA_TRAITS_HH__

namespace LittleEndianGuest {}
using namespace LittleEndianGuest;

//#include "arch/alpha/faults.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
#include "sim/host.hh"
#include "sim/faults.hh"

class ExecContext;
class FastCPU;
class FullCPU;
class Checkpoint;

#define TARGET_ALPHA

class StaticInst;
class StaticInstPtr;

namespace EV5 {
int DTB_ASN_ASN(uint64_t reg);
int ITB_ASN_ASN(uint64_t reg);
}

namespace AlphaISA
{

    typedef uint32_t MachInst;
    typedef uint64_t ExtMachInst;
    typedef uint8_t  RegIndex;

    const int NumIntArchRegs = 32;
    const int NumPALShadowRegs = 8;
    const int NumFloatArchRegs = 32;
    // @todo: Figure out what this number really should be.
    const int NumMiscArchRegs = 32;

    // Static instruction parameters
    const int MaxInstSrcRegs = 3;
    const int MaxInstDestRegs = 2;

    // semantically meaningful register indices
    const int ZeroReg = 31;	// architecturally meaningful
    // the rest of these depend on the ABI
    const int StackPointerReg = 30;
    const int GlobalPointerReg = 29;
    const int ProcedureValueReg = 27;
    const int ReturnAddressReg = 26;
    const int ReturnValueReg = 0;
    const int FramePointerReg = 15;
    const int ArgumentReg0 = 16;
    const int ArgumentReg1 = 17;
    const int ArgumentReg2 = 18;
    const int ArgumentReg3 = 19;
    const int ArgumentReg4 = 20;
    const int ArgumentReg5 = 21;

    const int LogVMPageSize = 13;	// 8K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;


    const int NumIntRegs = NumIntArchRegs + NumPALShadowRegs;
    const int NumFloatRegs = NumFloatArchRegs;
    const int NumMiscRegs = NumMiscArchRegs;

    // These enumerate all the registers for dependence tracking.
    enum DependenceTags {
        // 0..31 are the integer regs 0..31
        // 32..63 are the FP regs 0..31, i.e. use (reg + FP_Base_DepTag)
        FP_Base_DepTag = 40,
        Ctrl_Base_DepTag = 72,
        Fpcr_DepTag = 72,		// floating point control register
        Uniq_DepTag = 73,
        Lock_Flag_DepTag = 74,
        Lock_Addr_DepTag = 75,
        IPR_Base_DepTag = 76
    };

    typedef uint64_t IntReg;
    typedef IntReg IntRegFile[NumIntRegs];

    // floating point register file entry type
    typedef union {
        uint64_t q;
        double d;
    } FloatReg;

    typedef union {
        uint64_t q[NumFloatRegs];	// integer qword view
        double d[NumFloatRegs];		// double-precision floating point view
    } FloatRegFile;

extern const Addr PageShift;
extern const Addr PageBytes;
extern const Addr PageMask;
extern const Addr PageOffset;

// redirected register map, really only used for the full system case.
extern const int reg_redir[NumIntRegs];

#if FULL_SYSTEM

    typedef uint64_t InternalProcReg;

#include "arch/alpha/isa_fullsys_traits.hh"

#else
    const int NumInternalProcRegs = 0;
#endif

    // control register file contents
    typedef uint64_t MiscReg;
    class MiscRegFile {
      protected:
        uint64_t	fpcr;		// floating point condition codes
        uint64_t	uniq;		// process-unique register
        bool		lock_flag;	// lock flag for LL/SC
        Addr		lock_addr;	// lock address for LL/SC

      public:
        MiscReg readReg(int misc_reg);

        //These functions should be removed once the simplescalar cpu model
        //has been replaced.
        int getInstAsid();
        int getDataAsid();

        MiscReg readRegWithEffect(int misc_reg, Fault &fault, ExecContext *xc);

        Fault setReg(int misc_reg, const MiscReg &val);

        Fault setRegWithEffect(int misc_reg, const MiscReg &val,
                               ExecContext *xc);

#if FULL_SYSTEM
      protected:
        InternalProcReg ipr[NumInternalProcRegs]; // Internal processor regs

      private:
        MiscReg readIpr(int idx, Fault &fault, ExecContext *xc);

        Fault setIpr(int idx, uint64_t val, ExecContext *xc);
#endif
        friend class RegFile;
    };

    const int TotalNumRegs = NumIntRegs + NumFloatRegs +
        NumMiscRegs + NumInternalProcRegs;

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    typedef union {
        IntReg  intreg;
        FloatReg   fpreg;
        MiscReg ctrlreg;
    } AnyReg;

    struct RegFile {
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegs;		// control register file
        Addr pc;			// program counter
        Addr npc;			// next-cycle program counter
#if FULL_SYSTEM
        int intrflag;			// interrupt flag
        inline int instAsid()
        { return EV5::ITB_ASN_ASN(miscRegs.ipr[IPR_ITB_ASN]); }
        inline int dataAsid()
        { return EV5::DTB_ASN_ASN(miscRegs.ipr[IPR_DTB_ASN]); }
#endif // FULL_SYSTEM

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

    static inline ExtMachInst makeExtMI(MachInst inst, const uint64_t &pc);

    StaticInstPtr decodeInst(ExtMachInst);

    // return a no-op instruction... used for instruction fetch faults
    extern const ExtMachInst NoopMachInst;

    enum annotes {
        ANNOTE_NONE = 0,
        // An impossible number for instruction annotations
        ITOUCH_ANNOTE = 0xffffffff,
    };

    static inline bool isCallerSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return (reg >= 1 && reg <= 8 || reg >= 22 && reg <= 25 || reg == 27);
    }

    static inline bool isCalleeSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return (reg >= 9 && reg <= 15);
    }

    static inline bool isCallerSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    static inline bool isCalleeSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    static inline Addr alignAddress(const Addr &addr,
                                         unsigned int nbytes) {
        return (addr & ~(nbytes - 1));
    }

    // Instruction address compression hooks
    static inline Addr realPCToFetchPC(const Addr &addr) {
        return addr;
    }

    static inline Addr fetchPCToRealPC(const Addr &addr) {
        return addr;
    }

    // the size of "fetched" instructions (not necessarily the size
    // of real instructions for PISA)
    static inline size_t fetchInstSize() {
        return sizeof(MachInst);
    }

    static inline MachInst makeRegisterCopy(int dest, int src) {
        panic("makeRegisterCopy not implemented");
        return 0;
    }

    // Machine operations

    void saveMachineReg(AnyReg &savereg, const RegFile &reg_file,
                               int regnum);

    void restoreMachineReg(RegFile &regs, const AnyReg &reg,
                                  int regnum);

#if 0
    static void serializeSpecialRegs(const Serializable::Proxy &proxy,
                                     const RegFile &regs);

    static void unserializeSpecialRegs(const IniFile *db,
                                       const std::string &category,
                                       ConfigNode *node,
                                       RegFile &regs);
#endif

    /**
     * Function to insure ISA semantics about 0 registers.
     * @param xc The execution context.
     */
    template <class XC>
    void zeroRegisters(XC *xc);

    const Addr MaxAddr = (Addr)-1;
};

#if !FULL_SYSTEM
class SyscallReturn {
        public:
           template <class T>
           SyscallReturn(T v, bool s)
           {
               retval = (uint64_t)v;
               success = s;
           }

           template <class T>
           SyscallReturn(T v)
           {
               success = (v >= 0);
               retval = (uint64_t)v;
           }

           ~SyscallReturn() {}

           SyscallReturn& operator=(const SyscallReturn& s) {
               retval = s.retval;
               success = s.success;
               return *this;
           }

           bool successful() { return success; }
           uint64_t value() { return retval; }


       private:
           uint64_t retval;
           bool success;
};

#endif

static inline AlphaISA::ExtMachInst
AlphaISA::makeExtMI(AlphaISA::MachInst inst, const uint64_t &pc) {
#if FULL_SYSTEM
    AlphaISA::ExtMachInst ext_inst = inst;
    if (pc && 0x1)
        return ext_inst|=(static_cast<AlphaISA::ExtMachInst>(pc & 0x1) << 32);
    else
        return ext_inst;
#else
    return AlphaISA::ExtMachInst(inst);
#endif
}

#if FULL_SYSTEM

#include "arch/alpha/ev5.hh"
#endif

#endif // __ARCH_ALPHA_ISA_TRAITS_HH__
