/*
 * Copyright N) 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright N) <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. ($(B!H(BMIPS$(B!I(B) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED $(B!H(BAS IS.$(B!I(B  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Jaidev Patwardhan
 *
 */

#ifndef __ARCH_MIPS_PAGETABLE_H__
#define __ARCH_MIPS_PAGETABLE_H__

#include "arch/mips/isa_traits.hh"
#include "arch/mips/utility.hh"
#include "config/full_system.hh"

namespace MipsISA {

    struct VAddr
    {
        static const int ImplBits = 43;
        static const Addr ImplMask = (ULL(1) << ImplBits) - 1;
        static const Addr UnImplMask = ~ImplMask;

        VAddr(Addr a) : addr(a) {}
        Addr addr;
        operator Addr() const { return addr; }
        const VAddr &operator=(Addr a) { addr = a; return *this; }

        Addr vpn() const { return (addr & ImplMask) >> PageShift; }
        Addr page() const { return addr & Page_Mask; }
        Addr offset() const { return addr & PageOffset; }

        Addr level3() const
        { return MipsISA::PteAddr(addr >> PageShift); }
        Addr level2() const
        { return MipsISA::PteAddr(addr >> NPtePageShift + PageShift); }
        Addr level1() const
        { return MipsISA::PteAddr(addr >> 2 * NPtePageShift + PageShift); }
    };

    // ITB/DTB page table entry
    struct PTE
    {
      Addr Mask; // What parts of the VAddr (from bits 28..11) should be used in translation (includes Mask and MaskX from PageMask)
      Addr VPN; // Virtual Page Number (/2) (Includes VPN2 + VPN2X .. bits 31..11 from EntryHi)
      uint8_t asid; // Address Space ID (8 bits) // Lower 8 bits of EntryHi

      bool G;    // Global Bit - Obtained by an *AND* of EntryLo0 and EntryLo1 G bit

      /* Contents of Entry Lo0 */
      Addr PFN0; // Physical Frame Number - Even
      bool D0;   // Even entry Dirty Bit
      bool V0;   // Even entry Valid Bit
      uint8_t C0; // Cache Coherency Bits - Even

      /* Contents of Entry Lo1 */
      Addr PFN1; // Physical Frame Number - Odd
      bool D1;   // Odd entry Dirty Bit
      bool V1;   // Odd entry Valid Bit
      uint8_t C1; // Cache Coherency Bits (3 bits)

      /* The next few variables are put in as optimizations to reduce TLB lookup overheads */
      /* For a given Mask, what is the address shift amount, and what is the OffsetMask */
      int AddrShiftAmount;
      int OffsetMask;

      bool Valid() { return (V0 | V1);};
        void serialize(std::ostream &os);
        void unserialize(Checkpoint *cp, const std::string &section);
    };

};
#endif // __ARCH_MIPS_PAGETABLE_H__

