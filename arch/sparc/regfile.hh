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

#ifndef __ARCH_SPARC_REGFILE_HH__
#define __ARCH_SPARC_REGFILE_HH__

#include "arch/sparc/faults.hh"
#include "base/trace.hh"
#include "sim/byteswap.hh"
#include "sim/host.hh"

class Checkpoint;

namespace SparcISA
{

    typedef uint8_t  RegIndex;

    // MAXTL - maximum trap level
    const int MaxTL = 4;

    // NWINDOWS - number of register windows, can be 3 to 32
    const int NWindows = 6;

    class IntRegFile
    {
      protected:
        static const int FrameOffsetBits = 3;
        static const int FrameNumBits = 2;

        static const int RegsPerFrame = 1 << FrameOffsetBits;
        static const int FrameNumMask =
                (FrameNumBits == sizeof(int)) ?
                (unsigned int)(-1) :
                (1 << FrameNumBits) - 1;
        static const int FrameOffsetMask =
                (FrameOffsetBits == sizeof(int)) ?
                (unsigned int)(-1) :
                (1 << FrameOffsetBits) - 1;

        IntReg regGlobals[RegsPerFrame];
        IntReg altGlobals[RegsPerFrame];
        IntReg regSegments[2 * NWindows][RegsPerFrame];

        enum regFrame {Globals, Outputs, Locals, Inputs, NumFrames};

        IntReg * regView[NumFrames];

        static const int RegGlobalOffset = 0;
        static const int AltGlobalOffset = 8;
        static const int FrameOffset = 16;
        int offset[NumFrames];

      public:

        int flattenIndex(int reg)
        {
            int flatIndex = offset[reg >> FrameOffsetBits]
                | (reg & FrameOffsetMask);
            DPRINTF(Sparc, "Flattened index %d into %d.\n", reg, flatIndex);
            return flatIndex;
        }

        void clear()
        {
            bzero(regGlobals, sizeof(regGlobals));
            bzero(altGlobals, sizeof(altGlobals));
            for(int x = 0; x < 2 * NWindows; x++)
                bzero(regSegments[x], sizeof(regSegments[x]));
        }

        IntRegFile()
        {
            offset[Globals] = 0;
            regView[Globals] = regGlobals;
            setCWP(0);
            clear();
        }

        IntReg readReg(int intReg)
        {
            IntReg val =
                regView[intReg >> FrameOffsetBits][intReg & FrameOffsetMask];
            DPRINTF(Sparc, "Read register %d = 0x%x\n", intReg, val);
            return val;
        }

        Fault setReg(int intReg, const IntReg &val)
        {
            if(intReg)
                DPRINTF(Sparc, "Wrote register %d = 0x%x\n", intReg, val);
            regView[intReg >> FrameOffsetBits][intReg & FrameOffsetMask] = val;
            return NoFault;
        }

        //This doesn't effect the actual CWP register.
        //It's purpose is to adjust the view of the register file
        //to what it would be if CWP = cwp.
        void setCWP(int cwp)
        {
            int index = ((NWindows - cwp) % NWindows) * 2;
            offset[Outputs] = FrameOffset + (index * RegsPerFrame);
            offset[Locals] = FrameOffset + ((index+1) * RegsPerFrame);
            offset[Inputs] = FrameOffset +
                (((index+2) % (NWindows * 2)) * RegsPerFrame);
            regView[Outputs] = regSegments[index];
            regView[Locals] = regSegments[index+1];
            regView[Inputs] = regSegments[(index+2) % (NWindows * 2)];

            DPRINTF(Sparc, "Changed the CWP value to %d\n", cwp);
        }

        void setAltGlobals(bool useAlt)
        {
            DPRINTF(Sparc, "Now using %s globals",
                    useAlt ? "alternate" : "regular");
            regView[Globals] = useAlt ? altGlobals : regGlobals;
            offset[Globals] = useAlt ? AltGlobalOffset : RegGlobalOffset;
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);
    };

    typedef float float32_t;
    typedef double float64_t;
    //FIXME long double refers to a 10 byte float, rather than a
    //16 byte float as required. This data type may have to be emulated.
    typedef double float128_t;

    class FloatRegFile
    {
      public:
        static const int SingleWidth = 32;
        static const int DoubleWidth = 64;
        static const int QuadWidth = 128;

      protected:

        //Since the floating point registers overlap each other,
        //A generic storage space is used. The float to be returned is
        //pulled from the appropriate section of this region.
        char regSpace[SingleWidth / 8 * NumFloatRegs];

      public:

        void clear()
        {
            bzero(regSpace, sizeof(regSpace));
        }

        FloatReg readReg(int floatReg, int width)
        {
            //In each of these cases, we have to copy the value into a temporary
            //variable. This is because we may otherwise try to access an
            //unaligned portion of memory.
            switch(width)
            {
              case SingleWidth:
                float32_t result32;
                memcpy(&result32, regSpace + 4 * floatReg, width);
                return htog(result32);
              case DoubleWidth:
                float64_t result64;
                memcpy(&result64, regSpace + 4 * floatReg, width);
                return htog(result64);
              case QuadWidth:
                float128_t result128;
                memcpy(&result128, regSpace + 4 * floatReg, width);
                return htog(result128);
              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
        }

        FloatRegBits readRegBits(int floatReg, int width)
        {
            //In each of these cases, we have to copy the value into a temporary
            //variable. This is because we may otherwise try to access an
            //unaligned portion of memory.
            switch(width)
            {
              case SingleWidth:
                uint32_t result32;
                memcpy(&result32, regSpace + 4 * floatReg, width);
                return htog(result32);
              case DoubleWidth:
                uint64_t result64;
                memcpy(&result64, regSpace + 4 * floatReg, width);
                return htog(result64);
              case QuadWidth:
                uint64_t result128;
                memcpy(&result128, regSpace + 4 * floatReg, width);
                return htog(result128);
              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
        }

        Fault setReg(int floatReg, const FloatReg &val, int width)
        {
            //In each of these cases, we have to copy the value into a temporary
            //variable. This is because we may otherwise try to access an
            //unaligned portion of memory.
            switch(width)
            {
              case SingleWidth:
                uint32_t result32 = gtoh((uint32_t)val);
                memcpy(regSpace + 4 * floatReg, &result32, width);
              case DoubleWidth:
                uint64_t result64 = gtoh((uint64_t)val);
                memcpy(regSpace + 4 * floatReg, &result64, width);
              case QuadWidth:
                uint64_t result128 = gtoh((uint64_t)val);
                memcpy(regSpace + 4 * floatReg, &result128, width);
              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
            return NoFault;
        }

        Fault setRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            //In each of these cases, we have to copy the value into a temporary
            //variable. This is because we may otherwise try to access an
            //unaligned portion of memory.
            switch(width)
            {
              case SingleWidth:
                uint32_t result32 = gtoh((uint32_t)val);
                memcpy(regSpace + 4 * floatReg, &result32, width);
              case DoubleWidth:
                uint64_t result64 = gtoh((uint64_t)val);
                memcpy(regSpace + 4 * floatReg, &result64, width);
              case QuadWidth:
                uint64_t result128 = gtoh((uint64_t)val);
                memcpy(regSpace + 4 * floatReg, &result128, width);
              default:
                panic("Attempted to read a %d bit floating point register!", width);
            }
            return NoFault;
        }

        void serialize(std::ostream &os);

        void unserialize(Checkpoint *cp, const std::string &section);
    };

    enum MiscRegIndex
    {
        MISCREG_PSTATE,
        MISCREG_PSTATE_AG,
        MISCREG_PSTATE_IE,
        MISCREG_PSTATE_PRIV,
        MISCREG_PSTATE_AM,
        MISCREG_PSTATE_PEF,
        MISCREG_PSTATE_RED,
        MISCREG_PSTATE_MM,
        MISCREG_PSTATE_TLE,
        MISCREG_PSTATE_CLE,
        MISCREG_TBA,
        MISCREG_Y,
        MISCREG_Y_VALUE,
        MISCREG_PIL,
        MISCREG_CWP,
        MISCREG_TT_BASE,
        MISCREG_TT_END = MISCREG_TT_BASE + MaxTL,
        MISCREG_CCR,
        MISCREG_CCR_ICC,
        MISCREG_CCR_ICC_C,
        MISCREG_CCR_ICC_V,
        MISCREG_CCR_ICC_Z,
        MISCREG_CCR_ICC_N,
        MISCREG_CCR_XCC,
        MISCREG_CCR_XCC_C,
        MISCREG_CCR_XCC_V,
        MISCREG_CCR_XCC_Z,
        MISCREG_CCR_XCC_N,
        MISCREG_ASI,
        MISCREG_TL,
        MISCREG_TPC_BASE,
        MISCREG_TPC_END = MISCREG_TPC_BASE + MaxTL,
        MISCREG_TNPC_BASE,
        MISCREG_TNPC_END = MISCREG_TNPC_BASE + MaxTL,
        MISCREG_TSTATE_BASE,
        MISCREG_TSTATE_END = MISCREG_TSTATE_BASE + MaxTL,
        MISCREG_TSTATE_CWP_BASE,
        MISCREG_TSTATE_CWP_END = MISCREG_TSTATE_CWP_BASE + MaxTL,
        MISCREG_TSTATE_PSTATE_BASE,
        MISCREG_TSTATE_PSTATE_END = MISCREG_TSTATE_PSTATE_BASE + MaxTL,
        MISCREG_TSTATE_ASI_BASE,
        MISCREG_TSTATE_ASI_END = MISCREG_TSTATE_ASI_BASE + MaxTL,
        MISCREG_TSTATE_CCR_BASE,
        MISCREG_TSTATE_CCR_END = MISCREG_TSTATE_CCR_BASE + MaxTL,
        MISCREG_TICK,
        MISCREG_TICK_COUNTER,
        MISCREG_TICK_NPT,
        MISCREG_CANSAVE,
        MISCREG_CANRESTORE,
        MISCREG_OTHERWIN,
        MISCREG_CLEANWIN,
        MISCREG_WSTATE,
        MISCREG_WSTATE_NORMAL,
        MISCREG_WSTATE_OTHER,
        MISCREG_VER,
        MISCREG_VER_MAXWIN,
        MISCREG_VER_MAXTL,
        MISCREG_VER_MASK,
        MISCREG_VER_IMPL,
        MISCREG_VER_MANUF,
        MISCREG_FSR,
        MISCREG_FSR_CEXC,
        MISCREG_FSR_CEXC_NXC,
        MISCREG_FSR_CEXC_DZC,
        MISCREG_FSR_CEXC_UFC,
        MISCREG_FSR_CEXC_OFC,
        MISCREG_FSR_CEXC_NVC,
        MISCREG_FSR_AEXC,
        MISCREG_FSR_AEXC_NXC,
        MISCREG_FSR_AEXC_DZC,
        MISCREG_FSR_AEXC_UFC,
        MISCREG_FSR_AEXC_OFC,
        MISCREG_FSR_AEXC_NVC,
        MISCREG_FSR_FCC0,
        MISCREG_FSR_QNE,
        MISCREG_FSR_FTT,
        MISCREG_FSR_VER,
        MISCREG_FSR_NS,
        MISCREG_FSR_TEM,
        MISCREG_FSR_TEM_NXM,
        MISCREG_FSR_TEM_DZM,
        MISCREG_FSR_TEM_UFM,
        MISCREG_FSR_TEM_OFM,
        MISCREG_FSR_TEM_NVM,
        MISCREG_FSR_RD,
        MISCREG_FSR_FCC1,
        MISCREG_FSR_FCC2,
        MISCREG_FSR_FCC3,
        MISCREG_FPRS,
        MISCREG_FPRS_DL,
        MISCREG_FPRS_DU,
        MISCREG_FPRS_FEF,
        numMiscRegs
    };

    // The control registers, broken out into fields
    class MiscRegFile
    {
      private:
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
                uint64_t :32;	// reserved bits
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
                    } iccFields;
                };
                union
                {
                    uint8_t xcc:4;	// 64-bit condition codes
                    struct
                    {
                        uint8_t c:1;	// Carry
                        uint8_t v:1;	// Overflow
                        uint8_t z:1;	// Zero
                        uint8_t n:1;	// Negative
                    } xccFields;
                };
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
                uint64_t :2;	// Reserved bits
                uint64_t pstate:10;	// Process State
                uint64_t :6;	// Reserved bits
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
        };
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
                uint64_t :2;	// Reserved bits
                uint64_t maxtl:8;	// Maximum trap level
                uint64_t :8;	// Reserved bits
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
                    } cexcFields;
                };
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
                    } aexcFields;
                };
                uint64_t fcc0:2;		// Floating-Point condtion codes
                uint64_t :1;		// Reserved bits
                uint64_t qne:1;			// Deferred trap queue not empty
                                                // with no queue, it should read 0
                uint64_t ftt:3;			// Floating-Point trap type
                uint64_t ver:3;			// Version (of the FPU)
                uint64_t :2;		// Reserved bits
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
                    } temFields;
                };
                uint64_t :2;		// Reserved bits
                uint64_t rd:2;			// Rounding direction
                uint64_t fcc1:2;		// Floating-Point condition codes
                uint64_t fcc2:2;		// Floating-Point condition codes
                uint64_t fcc3:2;		// Floating-Point condition codes
                uint64_t :26;		// Reserved bits
            } fsrFields;
        };
        union
        {
            uint8_t		fprs;	// Floating-Point Register State
            struct
            {
                uint8_t dl:1;		// Dirty lower
                uint8_t du:1;		// Dirty upper
                uint8_t fef:1;		// FPRS enable floating-Point
            } fprsFields;
        };

      public:

        void reset()
        {
            pstateFields.pef = 0; //No FPU
            //pstateFields.pef = 1; //FPU
#if FULL_SYSTEM
            //For SPARC, when a system is first started, there is a power
            //on reset Trap which sets the processor into the following state.
            //Bits that aren't set aren't defined on startup.
            tl = MaxTL;
            tt[tl] = PowerOnReset.trapType();
            pstateFields.mm = 0; //Total Store Order
            pstateFields.red = 1; //Enter RED_State
            pstateFields.am = 0; //Address Masking is turned off
            pstateFields.priv = 1; //Processor enters privileged mode
            pstateFields.ie = 0; //Interrupts are disabled
            pstateFields.ag = 1; //Globals are replaced with alternate globals
            pstateFields.tle = 0; //Big Endian mode for traps
            pstateFields.cle = 0; //Big Endian mode for non-traps
            tickFields.npt = 1; //The TICK register is unreadable by
                                //non-priveleged software
#else
            //This sets up the initial state of the processor for usermode processes
            pstateFields.priv = 0; //Process runs in user mode
            pstateFields.ie = 1; //Interrupts are enabled
            fsrFields.rd = 0; //Round to nearest
            fsrFields.tem = 0; //Floating point traps not enabled
            fsrFields.ns = 0; //Non standard mode off
            fsrFields.qne = 0; //Floating point queue is empty
            fsrFields.aexc = 0; //No accrued exceptions
            fsrFields.cexc = 0; //No current exceptions

            //Register window management registers
            otherwin = 0; //No windows contain info from other programs
            canrestore = 0; //There are no windows to pop
            cansave = MaxTL - 2; //All windows are available to save into
            cleanwin = MaxTL;
#endif
        }

        MiscRegFile()
        {
            reset();
        }

        MiscReg readReg(int miscReg);

        MiscReg readRegWithEffect(int miscReg, Fault &fault, ExecContext *xc);

        Fault setReg(int miscReg, const MiscReg &val);

        Fault setRegWithEffect(int miscReg,
                const MiscReg &val, ExecContext * xc);

        void serialize(std::ostream & os);

        void unserialize(Checkpoint * cp, const std::string & section);

        void copyMiscRegs(ExecContext * xc);
    };

    typedef union
    {
        IntReg  intreg;
        FloatReg   fpreg;
        MiscReg ctrlreg;
    } AnyReg;

    class RegFile
    {
      protected:
        Addr pc;		// Program Counter
        Addr npc;		// Next Program Counter
        Addr nnpc;

      public:
        Addr readPC()
        {
            return pc;
        }

        void setPC(Addr val)
        {
            pc = val;
        }

        Addr readNextPC()
        {
            return npc;
        }

        void setNextPC(Addr val)
        {
            npc = val;
        }

        Addr readNextNPC()
        {
            return nnpc;
        }

        void setNextNPC(Addr val)
        {
            nnpc = val;
        }

      protected:
        IntRegFile intRegFile;		// integer register file
        FloatRegFile floatRegFile;	// floating point register file
        MiscRegFile miscRegFile;	// control register file

      public:

        void clear()
        {
            intRegFile.clear();
            floatRegFile.clear();
        }

        int flattenIntIndex(int reg)
        {
            return intRegFile.flattenIndex(reg);
        }

        MiscReg readMiscReg(int miscReg)
        {
            return miscRegFile.readReg(miscReg);
        }

        MiscReg readMiscRegWithEffect(int miscReg,
                Fault &fault, ExecContext *xc)
        {
            return miscRegFile.readRegWithEffect(miscReg, fault, xc);
        }

        Fault setMiscReg(int miscReg, const MiscReg &val)
        {
            return miscRegFile.setReg(miscReg, val);
        }

        Fault setMiscRegWithEffect(int miscReg, const MiscReg &val,
                ExecContext * xc)
        {
            return miscRegFile.setRegWithEffect(miscReg, val, xc);
        }

        FloatReg readFloatReg(int floatReg, int width)
        {
            return floatRegFile.readReg(floatReg, width);
        }

        FloatReg readFloatReg(int floatReg)
        {
            //Use the "natural" width of a single float
            return floatRegFile.readReg(floatReg, FloatRegFile::SingleWidth);
        }

        FloatRegBits readFloatRegBits(int floatReg, int width)
        {
            return floatRegFile.readRegBits(floatReg, width);
        }

        FloatRegBits readFloatRegBits(int floatReg)
        {
            //Use the "natural" width of a single float
            return floatRegFile.readRegBits(floatReg,
                    FloatRegFile::SingleWidth);
        }

        Fault setFloatReg(int floatReg, const FloatReg &val, int width)
        {
            return floatRegFile.setReg(floatReg, val, width);
        }

        Fault setFloatReg(int floatReg, const FloatReg &val)
        {
            //Use the "natural" width of a single float
            return setFloatReg(floatReg, val, FloatRegFile::SingleWidth);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val, int width)
        {
            return floatRegFile.setRegBits(floatReg, val, width);
        }

        Fault setFloatRegBits(int floatReg, const FloatRegBits &val)
        {
            //Use the "natural" width of a single float
            return floatRegFile.setRegBits(floatReg, val,
                    FloatRegFile::SingleWidth);
        }

        IntReg readIntReg(int intReg)
        {
            return intRegFile.readReg(intReg);
        }

        Fault setIntReg(int intReg, const IntReg &val)
        {
            return intRegFile.setReg(intReg, val);
        }

        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);

      public:

        enum ContextParam
        {
            CONTEXT_CWP,
            CONTEXT_GLOBALS
        };

        union ContextVal
        {
            MiscReg reg;
            bool altGlobals;
        };

        void changeContext(ContextParam param, ContextVal val)
        {
            switch(param)
            {
              case CONTEXT_CWP:
                intRegFile.setCWP(val.reg);
                break;
              case CONTEXT_GLOBALS:
                intRegFile.setAltGlobals(val.altGlobals);
                break;
              default:
                panic("Tried to set illegal context parameter in the SPARC regfile.\n");
            }
        }
    };

    void copyRegs(ExecContext *src, ExecContext *dest);

    void copyMiscRegs(ExecContext *src, ExecContext *dest);

} // namespace SparcISA

#endif
