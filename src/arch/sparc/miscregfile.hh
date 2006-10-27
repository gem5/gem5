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
 * Authors: Gabe Black
 *          Ali Saidi
 */

#ifndef __ARCH_SPARC_MISCREGFILE_HH__
#define __ARCH_SPARC_MISCREGFILE_HH__

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/types.hh"

#include <string>

namespace SparcISA
{

    //These functions map register indices to names
    std::string getMiscRegName(RegIndex);

    const int AsrStart = 0;
    const int PrStart = 32;
    const int HprStart = 64;
    const int MiscStart = 96;

    enum MiscRegIndex
    {
        /** Ancillary State Registers */
        MISCREG_Y  = AsrStart + 0,
        MISCREG_CCR = AsrStart + 2,
        MISCREG_ASI = AsrStart + 3,
        MISCREG_TICK = AsrStart + 4,
        MISCREG_FPRS = AsrStart + 6,
        MISCREG_PCR = AsrStart + 16,
        MISCREG_PIC = AsrStart + 17,
        MISCREG_GSR = AsrStart + 19,
        MISCREG_SOFTINT_SET = AsrStart + 20,
        MISCREG_SOFTINT_CLR = AsrStart + 21,
        MISCREG_SOFTINT = AsrStart + 22,
        MISCREG_TICK_CMPR = AsrStart + 23,
        MISCREG_STICK = AsrStart + 24,
        MISCREG_STICK_CMPR = AsrStart + 25,

        /** Privilged Registers */
        MISCREG_TPC = PrStart + 0,
        MISCREG_TNPC = PrStart + 1,
        MISCREG_TSTATE = PrStart + 2,
        MISCREG_TT = PrStart + 3,
        MISCREG_PRIVTICK = PrStart + 4,
        MISCREG_TBA = PrStart + 5,
        MISCREG_PSTATE = PrStart + 6,
        MISCREG_TL = PrStart + 7,
        MISCREG_PIL = PrStart + 8,
        MISCREG_CWP = PrStart + 9,
        MISCREG_CANSAVE = PrStart + 10,
        MISCREG_CANRESTORE = PrStart + 11,
        MISCREG_CLEANWIN = PrStart + 12,
        MISCREG_OTHERWIN = PrStart + 13,
        MISCREG_WSTATE = PrStart + 14,
        MISCREG_GL = PrStart + 16,

        /** Hyper privileged registers */
        MISCREG_HPSTATE = HprStart + 0,
        MISCREG_HTSTATE = HprStart + 1,
        MISCREG_HINTP = HprStart + 3,
        MISCREG_HTBA = HprStart + 5,
        MISCREG_HVER = HprStart + 6,
        MISCREG_STRAND_STS_REG = HprStart + 16,
        MISCREG_HSTICK_CMPR = HprStart + 31,

        /** Floating Point Status Register */
        MISCREG_FSR = MiscStart + 0

    };

    // The control registers, broken out into fields
    class MiscRegFile
    {
      private:

        /* ASR Registers */
        union {
            uint64_t y;		// Y (used in obsolete multiplication)
            struct {
                uint64_t value:32;	// The actual value stored in y
                uint64_t :32;	// reserved bits
            } yFields;
        };
        union {
            uint8_t	ccr;		// Condition Code Register
            struct {
                union {
                    uint8_t icc:4;	// 32-bit condition codes
                    struct {
                        uint8_t c:1;	// Carry
                        uint8_t v:1;	// Overflow
                        uint8_t z:1;	// Zero
                        uint8_t n:1;	// Negative
                    } iccFields;
                };
                union {
                    uint8_t xcc:4;	// 64-bit condition codes
                    struct {
                        uint8_t c:1;	// Carry
                        uint8_t v:1;	// Overflow
                        uint8_t z:1;	// Zero
                        uint8_t n:1;	// Negative
                    } xccFields;
                };
            } ccrFields;
        };
        uint8_t asi;		// Address Space Identifier
        union {
            uint64_t tick;		// Hardware clock-tick counter
            struct {
                int64_t counter:63;	// Clock-tick count
                uint64_t npt:1;		// Non-priveleged trap
            } tickFields;
        };
        union {
            uint8_t		fprs;	// Floating-Point Register State
            struct {
                uint8_t dl:1;		// Dirty lower
                uint8_t du:1;		// Dirty upper
                uint8_t fef:1;		// FPRS enable floating-Point
            } fprsFields;
        };
        union {
            uint64_t gsr;		//General Status Register
            struct {
                uint64_t mask:32;
                uint64_t :4;
                uint64_t im:1;
                uint64_t irnd:2;
                uint64_t :17;
                uint64_t scale:5;
                uint64_t align:3;
            } gsrFields;
        };
        union {
            uint64_t softint;
            struct {
                uint64_t tm:1;
                uint64_t int_level:14;
                uint64_t sm:1;
            } softintFields;
        };
        union {
            uint64_t tick_cmpr;		// Hardware tick compare registers
            struct {
                uint64_t tick_cmpr:63;	// Clock-tick count
                uint64_t int_dis:1;		// Non-priveleged trap
            } tick_cmprFields;
        };
        union {
            uint64_t stick;		// Hardware clock-tick counter
            struct {
                int64_t :63;	// Not used, storage in SparcSystem
                uint64_t npt:1;		// Non-priveleged trap
            } stickFields;
        };
        union {
            uint64_t stick_cmpr;		// Hardware tick compare registers
            struct {
                uint64_t tick_cmpr:63;	// Clock-tick count
                uint64_t int_dis:1;		// Non-priveleged trap
            } stick_cmprFields;
        };


        /* Privileged Registers */
        uint64_t tpc[MaxTL];	// Trap Program Counter (value from
                                // previous trap level)
        uint64_t tnpc[MaxTL];	// Trap Next Program Counter (value from
                                // previous trap level)
        union {
            uint64_t tstate[MaxTL];	// Trap State
            struct {
                //Values are from previous trap level
                uint64_t cwp:5;		// Current Window Pointer
                uint64_t :3;	// Reserved bits
                uint64_t pstate:13;	// Process State
                uint64_t :3;	// Reserved bits
                uint64_t asi:8;		// Address Space Identifier
                uint64_t ccr:8;		// Condition Code Register
                uint64_t gl:8;		// Global level
            } tstateFields[MaxTL];
        };
        uint16_t tt[MaxTL];	// Trap Type (Type of trap which occured
                                // on the previous level)
        uint64_t tba;		// Trap Base Address

        union {
            uint16_t pstate;		// Process State Register
            struct {
                uint16_t :1;		// reserved
                uint16_t ie:1;		// Interrupt enable
                uint16_t priv:1;	// Privelege mode
                uint16_t am:1;		// Address mask
                uint16_t pef:1;		// PSTATE enable floating-point
                uint16_t :1;		// reserved2
                uint16_t mm:2;		// Memory Model
                uint16_t tle:1;		// Trap little-endian
                uint16_t cle:1;		// Current little-endian
            } pstateFields;
        };
        uint8_t tl;		// Trap Level
        uint8_t pil;		// Process Interrupt Register
        uint8_t cwp;		// Current Window Pointer
        uint8_t cansave;	// Savable windows
        uint8_t canrestore;	// Restorable windows
        uint8_t cleanwin;	// Clean windows
        uint8_t otherwin;	// Other windows
        union {
            uint8_t wstate;		// Window State
            struct {
                uint8_t normal:3;	// Bits TT<4:2> are set to on a normal
                                        // register window trap
                uint8_t other:3;	// Bits TT<4:2> are set to on an "otherwin"
                                        // register window trap
            } wstateFields;
        };
        uint8_t gl;             // Global level register


        /** Hyperprivileged Registers */
        union {
            uint64_t hpstate; // Hyperprivileged State Register
            struct {
                uint8_t tlz: 1;
                uint8_t :1;
                uint8_t hpriv:1;
                uint8_t :2;
                uint8_t red:1;
                uint8_t :4;
                uint8_t ibe:1;
                uint8_t id:1;
            } hpstateFields;
        };

        uint64_t htstate[MaxTL]; // Hyperprivileged Trap State Register
        uint64_t hintp;
        uint64_t htba; // Hyperprivileged Trap Base Address register
        union {
            uint64_t hstick_cmpr;		// Hardware tick compare registers
            struct {
                uint64_t tick_cmpr:63;	// Clock-tick count
                uint64_t int_dis:1;		// Non-priveleged trap
            } hstick_cmprFields;
        };

        uint64_t strandStatusReg; // Per strand status register


        /** Floating point misc registers. */
        union {
            uint64_t	fsr;	// Floating-Point State Register
            struct {
                union {
                    uint64_t cexc:5;	// Current excpetion
                    struct {
                        uint64_t nxc:1;		// Inexact
                        uint64_t dzc:1;		// Divide by zero
                        uint64_t ufc:1;		// Underflow
                        uint64_t ofc:1;		// Overflow
                        uint64_t nvc:1;		// Invalid operand
                    } cexcFields;
                };
                union {
                    uint64_t aexc:5;		// Accrued exception
                    struct {
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
                union {
                    uint64_t tem:5;			// Trap Enable Mask
                    struct {
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

        // These need to check the int_dis field and if 0 then
        // set appropriate bit in softint and checkinterrutps on the cpu
#if FULL_SYSTEM
        /** Process a tick compare event and generate an interrupt on the cpu if
         * appropriate. */
        void processTickCompare(ThreadContext *tc);
        void processSTickCompare(ThreadContext *tc);
        void processHSTickCompare(ThreadContext *tc);

        typedef CpuEventWrapper<MiscRegFile,
                &MiscRegFile::processTickCompare> TickCompareEvent;
        TickCompareEvent *tickCompare;

        typedef CpuEventWrapper<MiscRegFile,
                &MiscRegFile::processSTickCompare> STickCompareEvent;
        STickCompareEvent *sTickCompare;

        typedef CpuEventWrapper<MiscRegFile,
                &MiscRegFile::processHSTickCompare> HSTickCompareEvent;
        HSTickCompareEvent *hSTickCompare;

        /** Fullsystem only register version of ReadRegWithEffect() */
        MiscReg readFSRegWithEffect(int miscReg, Fault &fault, ThreadContext *tc);
        /** Fullsystem only register version of SetRegWithEffect() */
        Fault setFSRegWithEffect(int miscReg, const MiscReg &val,
                ThreadContext * tc);
#endif
      public:

        void reset();

        MiscRegFile()
        {
            reset();
        }

        /** read a value out of an either an SE or FS IPR. No checking is done
         * about SE vs. FS as this is mostly used to copy the regfile. Thus more
         * register are copied that are necessary for FS. However this prevents
         * a bunch of ifdefs and is rarely called so is not performance
         * criticial. */
        MiscReg readReg(int miscReg);

        /** Read a value from an IPR. Only the SE iprs are here and the rest
         * are are readFSRegWithEffect (which is called by readRegWithEffect()).
         * Checking is done for permission based on state bits in the miscreg
         * file. */
        MiscReg readRegWithEffect(int miscReg, Fault &fault, ThreadContext *tc);

        /** write a value into an either an SE or FS IPR. No checking is done
         * about SE vs. FS as this is mostly used to copy the regfile. Thus more
         * register are copied that are necessary for FS. However this prevents
         * a bunch of ifdefs and is rarely called so is not performance
         * criticial.*/
        Fault setReg(int miscReg, const MiscReg &val);

        /** Write a value into an IPR. Only the SE iprs are here and the rest
         * are are setFSRegWithEffect (which is called by setRegWithEffect()).
         * Checking is done for permission based on state bits in the miscreg
         * file. */
        Fault setRegWithEffect(int miscReg,
                const MiscReg &val, ThreadContext * tc);

        void serialize(std::ostream & os);

        void unserialize(Checkpoint * cp, const std::string & section);

        void copyMiscRegs(ThreadContext * tc);

      protected:

        bool isHyperPriv() { return hpstateFields.hpriv; }
        bool isPriv() { return hpstateFields.hpriv || pstateFields.priv; }
        bool isNonPriv() { return !isPriv(); }
    };
}

#endif
