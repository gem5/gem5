/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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
 */

#ifndef __ARCH_X86_MISCREGS_HH__
#define __ARCH_X86_MISCREGS_HH__

#include "base/bitunion.hh"

namespace X86ISA
{
    enum CondFlagBit {
        CFBit = 1 << 0,
        PFBit = 1 << 2,
        ECFBit = 1 << 3,
        AFBit = 1 << 4,
        EZFBit = 1 << 5,
        ZFBit = 1 << 6,
        SFBit = 1 << 7,
        DFBit = 1 << 10,
        OFBit = 1 << 11
    };

    enum MiscRegIndex
    {
        // Control registers
        // Most of these are invalid.
        MISCREG_CR_BASE,
        MISCREG_CR0 = MISCREG_CR_BASE,
        MISCREG_CR1,
        MISCREG_CR2,
        MISCREG_CR3,
        MISCREG_CR4,
        MISCREG_CR5,
        MISCREG_CR6,
        MISCREG_CR7,
        MISCREG_CR8,
        MISCREG_CR9,
        MISCREG_CR10,
        MISCREG_CR11,
        MISCREG_CR12,
        MISCREG_CR13,
        MISCREG_CR14,
        MISCREG_CR15,

        // Debug registers
        MISCREG_DR_BASE,
        MISCREG_DR0 = MISCREG_DR_BASE,
        MISCREG_DR1,
        MISCREG_DR2,
        MISCREG_DR3,
        MISCREG_DR4,
        MISCREG_DR5,
        MISCREG_DR6,
        MISCREG_DR7,

        // Flags register
        MISCREG_RFLAGS,

        // Segment selectors
        MISCREG_SEG_SEL_BASE,
        MISCREG_ES = MISCREG_SEG_SEL_BASE,
        MISCREG_CS,
        MISCREG_SS,
        MISCREG_DS,
        MISCREG_FS,
        MISCREG_GS,

        // Hidden segment base field
        MISCREG_SEG_BASE_BASE,
        MISCREG_ES_BASE = MISCREG_SEG_BASE_BASE,
        MISCREG_CS_BASE,
        MISCREG_SS_BASE,
        MISCREG_DS_BASE,
        MISCREG_FS_BASE,
        MISCREG_GS_BASE,

        // Hidden segment limit field
        MISCREG_SEG_LIMIT_BASE,
        MISCREG_ES_LIMIT = MISCREG_SEG_LIMIT_BASE,
        MISCREG_CS_LIMIT,
        MISCREG_SS_LIMIT,
        MISCREG_DS_LIMIT,
        MISCREG_FS_LIMIT,
        MISCREG_GS_LIMIT,

        // Hidden segment limit attributes
        MISCREG_SEG_ATTR_BASE,
        MISCREG_ES_ATTR = MISCREG_SEG_ATTR_BASE,
        MISCREG_CS_ATTR,
        MISCREG_SS_ATTR,
        MISCREG_DS_ATTR,
        MISCREG_FS_ATTR,
        MISCREG_GS_ATTR,

        // System segment selectors
        MISCREG_SYSSEG_SEL_BASE,
        MISCREG_LDTR = MISCREG_SYSSEG_SEL_BASE,
        MISCREG_TR,

        // Hidden system segment base field
        MISCREG_SYSSEG_BASE_BASE,
        MISCREG_LDTR_BASE = MISCREG_SYSSEG_BASE_BASE,
        MISCREG_TR_BASE,
        MISCREG_GDTR_BASE,
        MISCREG_IDTR_BASE,

        // Hidden system segment limit field
        MISCREG_SYSSEG_LIMIT_BASE,
        MISCREG_LDTR_LIMIT = MISCREG_SYSSEG_LIMIT_BASE,
        MISCREG_TR_LIMIT,
        MISCREG_GDTR_LIMIT,
        MISCREG_IDTR_LIMIT,

        // Hidden system segment attribute field
        MISCREG_SYSSEG_ATTR_BASE,
        MISCREG_LDTR_ATTR = MISCREG_SYSSEG_ATTR_BASE,
        MISCREG_TR_ATTR,

        //XXX Add "Model-Specific Registers"

        NUM_MISCREGS
    };

    /**
     * A type to describe the condition code bits of the RFLAGS register,
     * plus two flags, EZF and ECF, which are only visible to microcode.
     */
    BitUnion64(CCFlagBits)
        Bitfield<11> OF;
        Bitfield<7> SF;
        Bitfield<6> ZF;
        Bitfield<5> EZF;
        Bitfield<4> AF;
        Bitfield<3> ECF;
        Bitfield<2> PF;
        Bitfield<0> CF;
    EndBitUnion(CCFlagBits)

    /**
     * RFLAGS
     */
    BitUnion64(RFLAGS)
        Bitfield<21> ID; // ID Flag
        Bitfield<20> VIP; // Virtual Interrupt Pending
        Bitfield<19> VIF; // Virtual Interrupt Flag
        Bitfield<18> AC; // Alignment Check
        Bitfield<17> VM; // Virtual-8086 Mode
        Bitfield<16> RF; // Resume Flag
        Bitfield<14> NT; // Nested Task
        Bitfield<13, 12> IOPL; // I/O Privilege Level
        Bitfield<11> OF; // Overflow Flag
        Bitfield<10> DF; // Direction Flag
        Bitfield<9> IF; // Interrupt Flag
        Bitfield<8> TF; // Trap Flag
        Bitfield<7> SF; // Sign Flag
        Bitfield<6> ZF; // Zero Flag
        Bitfield<4> AF; // Auxiliary Flag
        Bitfield<2> PF; // Parity Flag
        Bitfield<0> CF; // Carry Flag
    EndBitUnion(RFLAGS)

    /**
     * Control registers
     */
    BitUnion64(CR0)
        Bitfield<31> PG; // Paging
        Bitfield<30> CD; // Cache Disable
        Bitfield<29> NW; // Not Writethrough
        Bitfield<18> AM; // Alignment Mask
        Bitfield<16> WP; // Write Protect
        Bitfield<5> NE; // Numeric Error
        Bitfield<4> ET; // Extension Type
        Bitfield<3> TS; // Task Switched
        Bitfield<2> EM; // Emulation
        Bitfield<1> MP; // Monitor Coprocessor
        Bitfield<0> PE; // Protection Enabled
    EndBitUnion(CR0)

    // Page Fault Virtual Address
    BitUnion64(CR2)
        Bitfield<31, 0> legacy;
    EndBitUnion(CR2)

    BitUnion64(CR3)
        Bitfield<51, 12> longPDTB; // Long Mode Page-Directory-Table
                                   // Base Address
        Bitfield<31, 12> PDTB; // Non-PAE Addressing Page-Directory-Table
                               // Base Address
        Bitfield<31, 5> PAEPDTB; // PAE Addressing Page-Directory-Table
                                 // Base Address
        Bitfield<4> PCD; // Page-Level Cache Disable
        Bitfield<3> PWT; // Page-Level Writethrough
    EndBitUnion(CR3)

    BitUnion64(CR4)
        Bitfield<10> OSXMMEXCPT; // Operating System Unmasked
                                 // Exception Support
        Bitfield<9> OSFXSR; // Operating System FXSave/FSRSTOR Support
        Bitfield<8> PCE; // Performance-Monitoring Counter Enable
        Bitfield<7> PGE; // Page-Global Enable
        Bitfield<6> MCE; // Machine Check Enable
        Bitfield<5> PAE; // Physical-Address Extension
        Bitfield<4> PSE; // Page Size Extensions
        Bitfield<3> DE; // Debugging Extensions
        Bitfield<2> TSD; // Time Stamp Disable
        Bitfield<1> PVI; // Protected-Mode Virtual Interrupts
        Bitfield<0> VME; // Virtual-8086 Mode Extensions
    EndBitUnion(CR4)

    BitUnion64(CR8)
        Bitfield<3, 0> TPR; // Task Priority Register
    EndBitUnion(CR4)

    /**
     * Segment Selector
     */
    BitUnion64(SegSelector)
        Bitfield<15, 3> SI; // Selector Index
        Bitfield<2> TI; // Table Indicator
        Bitfield<1, 0> RPL; // Requestor Privilege Level
    EndBitUnion(SegSelector)

    /**
     * Segment Descriptors
     */

    BitUnion64(SegDescriptor)
        Bitfield<63, 56> baseHigh;
        Bitfield<39, 16> baseLow;
        Bitfield<55> G; // Granularity
        Bitfield<54> D; // Default Operand Size
        Bitfield<54> B; // Default Operand Size
        Bitfield<53> L; // Long Attribute Bit
        Bitfield<52> AVL; // Available To Software
        Bitfield<51, 48> limitHigh;
        Bitfield<15, 0> limitLow;
        Bitfield<47> P; // Present
        Bitfield<46, 45> DPL; // Descriptor Privilege-Level
        Bitfield<44> S; // System
        SubBitUnion(type, 43, 40)
            // Specifies whether this descriptor is for code or data.
            Bitfield<43> codeOrData;

            // These bit fields are for code segments
            Bitfield<42> C; // Conforming
            Bitfield<41> R; // Readable

            // These bit fields are for data segments
            Bitfield<42> E; // Expand-Down
            Bitfield<41> W; // Writable

            // This is used for both code and data segments.
            Bitfield<40> A; // Accessed
        EndSubBitUnion(type)
    EndBitUnion(SegDescriptor)

    BitUnion64(GateDescriptor)
        Bitfield<63, 48> offsetHigh; // Target Code-Segment Offset
        Bitfield<15, 0> offsetLow; // Target Code-Segment Offset
        Bitfield<31, 16> selector; // Target Code-Segment Selector
        Bitfield<47> P; // Present
        Bitfield<46, 45> DPL; // Descriptor Privilege-Level
        Bitfield<43, 40> type;
        Bitfield<36, 32> count; // Parameter Count
    EndBitUnion(GateDescriptor)

    /**
     * Descriptor-Table Registers
     */
    BitUnion64(GDTR)
    EndBitUnion(GDTR)

    BitUnion64(IDTR)
    EndBitUnion(IDTR)

    BitUnion64(LDTR)
    EndBitUnion(LDTR)

    /**
     * Task Register
     */
    BitUnion64(TR)
    EndBitUnion(TR)
};

#endif // __ARCH_X86_INTREGS_HH__
