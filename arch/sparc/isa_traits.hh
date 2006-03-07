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

#ifndef __ARCH_SPARC_ISA_TRAITS_HH__
#define __ARCH_SPARC_ISA_TRAITS_HH__

#include "arch/sparc/faults.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
#include "sim/host.hh"

//This makes sure the big endian versions of certain functions are used.
namespace BigEndianGuest {}
using namespace BigEndianGuest;

class ExecContext;
class FastCPU;
//class FullCPU;
class Checkpoint;

#define TARGET_SPARC

class StaticInst;
class StaticInstPtr;

//namespace EV5
//{
//	int DTB_ASN_ASN(uint64_t reg);
//	int ITB_ASN_ASN(uint64_t reg);
//}

namespace SparcISA
{
    typedef uint32_t MachInst;
    typedef uint64_t Addr;
    typedef uint8_t  RegIndex;

    enum
    {
        MemoryEnd = 0xffffffffffffffffULL,

        NumFloatRegs = 32,
        NumMiscRegs = 32,

        MaxRegsOfAnyType = 32,
        // Static instruction parameters
        MaxInstSrcRegs = 3,
        MaxInstDestRegs = 2,

        // Maximum trap level
        MaxTL = 4,

        // semantically meaningful register indices
        ZeroReg = 0	// architecturally meaningful
        // the rest of these depend on the ABI
    };
    typedef uint64_t IntReg;

    class IntRegFile
    {
        private:
          //For right now, let's pretend the register file is static
          IntReg regs[32];
        public:
          IntReg & operator [] (RegIndex index)
          {
              //Don't allow indexes outside of the 32 registers
              index &= 0x1F;
              return regs[index];
          }
    };

    void serialize(std::ostream & os);

    void unserialize(Checkpoint *cp, const std::string &section);

    class FloatRegFile
    {
      private:
        //By using the largest data type, we ensure everything
        //is aligned correctly in memory
        union
        {
            long double rawRegs[16];
            uint64_t regDump[32];
        };
        class QuadRegs
        {
          private:
            FloatRegFile * parent;
          public:
            QuadRegs(FloatRegFile * p) : parent(p) {;}
            long double & operator [] (RegIndex index)
            {
                //Quad floats are index by the single
                //precision register the start on,
                //and only 16 should be accessed
                index = (index >> 2) & 0xF;
                return parent->rawRegs[index];
            }
        };
        class DoubleRegs
        {
          private:
            FloatRegFile * parent;
          public:
            DoubleRegs(FloatRegFile * p) : parent(p) {;}
            double & operator [] (RegIndex index)
            {
                //Double floats are index by the single
                //precision register the start on,
                //and only 32 should be accessed
                index = (index >> 1) & 0x1F;
                return ((double *)parent->rawRegs)[index];
            }
        };
        class SingleRegs
        {
          private:
            FloatRegFile * parent;
          public:
            SingleRegs(FloatRegFile * p) : parent(p) {;}
            float & operator [] (RegIndex index)
            {
                //Only 32 single floats should be accessed
                index &= 0x1F;
                return ((float *)parent->rawRegs)[index];
            }
        };
      public:
        void serialize(std::ostream & os);

        void unserialize(Checkpoint * cp, std::string & section);

        QuadRegs quadRegs;
        DoubleRegs doubleRegs;
        SingleRegs singleRegs;
        FloatRegFile() : quadRegs(this), doubleRegs(this), singleRegs(this)
        {;}
    };

    // control register file contents
    typedef uint64_t MiscReg;
    // The control registers, broken out into fields
    class MiscRegFile
    {
      public:
        union
        {
            uint16_t pstate;		// Process State Register
            struct
            {
                uint16_t ag:1;		// Alternate Globals
                uint16_t ie:1;		// Interrupt enable
                uint16_t priv:1;	// Privelege mode
                uint16_t am:1;		// Address mask
                uint16_t pef:1;		// PSTATE enable floating-point
                uint16_t red:1;		// RED (reset, error, debug) state
                uint16_t mm:2;		// Memory Model
                uint16_t tle:1;		// Trap little-endian
                uint16_t cle:1;		// Current little-endian
            } pstateFields;
        };
        uint64_t tba;		// Trap Base Address
        union
        {
            uint64_t y;		// Y (used in obsolete multiplication)
            struct
            {
                uint64_t value:32;	// The actual value stored in y
                const uint64_t :32;	// reserved bits
            } yFields;
        };
        uint8_t pil;		// Process Interrupt Register
        uint8_t cwp;		// Current Window Pointer
        uint16_t tt[MaxTL];	// Trap Type (Type of trap which occured
                                // on the previous level)
        union
        {
            uint8_t	ccr;		// Condition Code Register
            struct
            {
                union
                {
                    uint8_t icc:4;	// 32-bit condition codes
                    struct
                    {
                        uint8_t c:1;	// Carry
                        uint8_t v:1;	// Overflow
                        uint8_t z:1;	// Zero
                        uint8_t n:1;	// Negative
                    } iccFields:4;
                } :4;
                union
                {
                    uint8_t xcc:4;	// 64-bit condition codes
                    struct
                    {
                        uint8_t c:1;	// Carry
                        uint8_t v:1;	// Overflow
                        uint8_t z:1;	// Zero
                        uint8_t n:1;	// Negative
                    } xccFields:4;
                } :4;
            } ccrFields;
        };
        uint8_t asi;		// Address Space Identifier
        uint8_t tl;		// Trap Level
        uint64_t tpc[MaxTL];	// Trap Program Counter (value from
                                // previous trap level)
        uint64_t tnpc[MaxTL];	// Trap Next Program Counter (value from
                                // previous trap level)
        union
        {
            uint64_t tstate[MaxTL];	// Trap State
            struct
            {
                //Values are from previous trap level
                uint64_t cwp:5;		// Current Window Pointer
                const uint64_t :2;	// Reserved bits
                uint64_t pstate:10;	// Process State
                const uint64_t :6;	// Reserved bits
                uint64_t asi:8;		// Address Space Identifier
                uint64_t ccr:8;		// Condition Code Register
            } tstateFields[MaxTL];
        };
        union
        {
            uint64_t tick;		// Hardware clock-tick counter
            struct
            {
                uint64_t counter:63;	// Clock-tick count
                uint64_t npt:1;		// Non-priveleged trap
            } tickFields;
        }
        uint8_t cansave;	// Savable windows
        uint8_t canrestore;	// Restorable windows
        uint8_t otherwin;	// Other windows
        uint8_t cleanwin;	// Clean windows
        union
        {
            uint8_t wstate;		// Window State
            struct
            {
                uint8_t normal:3;	// Bits TT<4:2> are set to on a normal
                                        // register window trap
                uint8_t other:3;	// Bits TT<4:2> are set to on an "otherwin"
                                        // register window trap
            } wstateFields;
        };
        union
        {
            uint64_t ver;		// Version
            struct
            {
                uint64_t maxwin:5;	// Max CWP value
                const uint64_t :2;	// Reserved bits
                uint64_t maxtl:8;	// Maximum trap level
                const uint64_t :8;	// Reserved bits
                uint64_t mask:8;	// Processor mask set revision number
                uint64_t impl:16;	// Implementation identification number
                uint64_t manuf:16;	// Manufacturer code
            } verFields;
        };
        union
        {
            uint64_t	fsr;	// Floating-Point State Register
            struct
            {
                union
                {
                    uint64_t cexc:5;	// Current excpetion
                    struct
                    {
                        uint64_t nxc:1;		// Inexact
                        uint64_t dzc:1;		// Divide by zero
                        uint64_t ufc:1;		// Underflow
                        uint64_t ofc:1;		// Overflow
                        uint64_t nvc:1;		// Invalid operand
                    } cexecFields:5;
                } :5;
                union
                {
                    uint64_t aexc:5;		// Accrued exception
                    struct
                    {
                        uint64_t nxc:1;		// Inexact
                        uint64_t dzc:1;		// Divide by zero
                        uint64_t ufc:1;		// Underflow
                        uint64_t ofc:1;		// Overflow
                        uint64_t nvc:1;		// Invalid operand
                    } aexecFields:5;
                } :5;
                uint64_t fcc0:2;		// Floating-Point condtion codes
                const uint64_t :1;		// Reserved bits
                uint64_t qne:1;			// Deferred trap queue not empty
                                                // with no queue, it should read 0
                uint64_t ftt:3;			// Floating-Point trap type
                uint64_t ver:3;			// Version (of the FPU)
                const uint64_t :2;		// Reserved bits
                uint64_t ns:1;			// Nonstandard floating point
                union
                {
                    uint64_t tem:5;			// Trap Enable Mask
                    struct
                    {
                        uint64_t nxm:1;		// Inexact
                        uint64_t dzm:1;		// Divide by zero
                        uint64_t ufm:1;		// Underflow
                        uint64_t ofm:1;		// Overflow
                        uint64_t nvm:1;		// Invalid operand
                    } temFields:5;
                } :5;
                const uint64_t :2;		// Reserved bits
                uint64_t rd:2;			// Rounding direction
                uint64_t fcc1:2;		// Floating-Point condition codes
                uint64_t fcc2:2;		// Floating-Point condition codes
                uint64_t fcc3:2;		// Floating-Point condition codes
                const uint64_t :26;		// Reserved bits
            } fsrFields;
        }
        union
        {
            uint8_t		fprs;	// Floating-Point Register State
            struct
            {
                uint8_t dl:1;		// Dirty lower
                uint8_t du:1;		// Dirty upper
                fef:1;		// FPRS enable floating-Point
            } fprsFields;
        };

        void serialize(std::ostream & os)
        {
            SERIALIZE_SCALAR(pstate);
            SERIAlIZE_SCALAR(tba);
            SERIALIZE_SCALAR(y);
            SERIALIZE_SCALAR(pil);
            SERIALIZE_SCALAR(cwp);
            SERIALIZE_ARRAY(tt, MaxTL);
            SERIALIZE_SCALAR(ccr);
            SERIALIZE_SCALAR(asi);
            SERIALIZE_SCALAR(tl);
            SERIALIZE_SCALAR(tpc);
            SERIALIZE_SCALAR(tnpc);
            SERIALIZE_ARRAY(tstate, MaxTL);
            SERIALIZE_SCALAR(tick);
            SERIALIZE_SCALAR(cansave);
            SERIALIZE_SCALAR(canrestore);
            SERIALIZE_SCALAR(otherwin);
            SERIALIZE_SCALAR(cleanwin);
            SERIALIZE_SCALAR(wstate);
            SERIALIZE_SCALAR(ver);
            SERIALIZE_SCALAR(fsr);
            SERIALIZE_SCALAR(fprs);
        }

        void unserialize(Checkpoint &* cp, std::string & section)
        {
            UNSERIALIZE_SCALAR(pstate);
            UNSERIAlIZE_SCALAR(tba);
            UNSERIALIZE_SCALAR(y);
            UNSERIALIZE_SCALAR(pil);
            UNSERIALIZE_SCALAR(cwp);
            UNSERIALIZE_ARRAY(tt, MaxTL);
            UNSERIALIZE_SCALAR(ccr);
            UNSERIALIZE_SCALAR(asi);
            UNSERIALIZE_SCALAR(tl);
            UNSERIALIZE_SCALAR(tpc);
            UNSERIALIZE_SCALAR(tnpc);
            UNSERIALIZE_ARRAY(tstate, MaxTL);
            UNSERIALIZE_SCALAR(tick);
            UNSERIALIZE_SCALAR(cansave);
            UNSERIALIZE_SCALAR(canrestore);
            UNSERIALIZE_SCALAR(otherwin);
            UNSERIALIZE_SCALAR(cleanwin);
            UNSERIALIZE_SCALAR(wstate);
            UNSERIALIZE_SCALAR(ver);
            UNSERIALIZE_SCALAR(fsr);
            UNSERIALIZE_SCALAR(fprs);
        }
    };

    typedef union
    {
        IntReg  intreg;
        FloatReg   fpreg;
        MiscReg ctrlreg;
    } AnyReg;

    struct RegFile
    {
        IntRegFile intRegFile;		// (signed) integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegFile;	// control register file

        Addr pc;		// Program Counter
        Addr npc;		// Next Program Counter

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

    static StaticInstPtr decodeInst(MachInst);

    // return a no-op instruction... used for instruction fetch faults
    static const MachInst NoopMachInst;

    // Instruction address compression hooks
    static inline Addr realPCToFetchPC(const Addr &addr)
    {
        return addr;
    }

    static inline Addr fetchPCToRealPC(const Addr &addr)
    {
        return addr;
    }

    // the size of "fetched" instructions (not necessarily the size
    // of real instructions for PISA)
    static inline size_t fetchInstSize()
    {
        return sizeof(MachInst);
    }

    /**
     * Function to insure ISA semantics about 0 registers.
     * @param xc The execution context.
     */
    template <class XC>
    static void zeroRegisters(XC *xc);
};

const int VMPageSize   = TheISA::VMPageSize;
const int LogVMPageSize   = TheISA::LogVMPageSize;
const int ZeroReg = TheISA::ZeroReg;
const int BranchPredAddrShiftAmt = TheISA::BranchPredAddrShiftAmt;
const int MaxAddr = (Addr)-1;

#if !FULL_SYSTEM
class SyscallReturn
{
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

    SyscallReturn& operator=(const SyscallReturn& s)
    {
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


#if FULL_SYSTEM

#include "arch/alpha/ev5.hh"
#endif

#endif // __ARCH_SPARC_ISA_TRAITS_HH__
