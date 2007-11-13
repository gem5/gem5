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

#include "arch/mips/pagetable.hh"
#include "sim/serialize.hh"

namespace MipsISA
{


    void
    PTE::serialize(std::ostream &os)
    {
        SERIALIZE_SCALAR(Mask);
        SERIALIZE_SCALAR(VPN);
        SERIALIZE_SCALAR(asid);
        SERIALIZE_SCALAR(G);
        SERIALIZE_SCALAR(PFN0);
        SERIALIZE_SCALAR(D0);
        SERIALIZE_SCALAR(V0);
        SERIALIZE_SCALAR(C0);
        SERIALIZE_SCALAR(PFN1);
        SERIALIZE_SCALAR(D1);
        SERIALIZE_SCALAR(V1);
        SERIALIZE_SCALAR(C1);
        SERIALIZE_SCALAR(AddrShiftAmount);
        SERIALIZE_SCALAR(OffsetMask);
    }

    void
    PTE::unserialize(Checkpoint *cp, const std::string &section)
    {
        UNSERIALIZE_SCALAR(Mask);
        UNSERIALIZE_SCALAR(VPN);
        UNSERIALIZE_SCALAR(asid);
        UNSERIALIZE_SCALAR(G);
        UNSERIALIZE_SCALAR(PFN0);
        UNSERIALIZE_SCALAR(D0);
        UNSERIALIZE_SCALAR(V0);
        UNSERIALIZE_SCALAR(C0);
        UNSERIALIZE_SCALAR(PFN1);
        UNSERIALIZE_SCALAR(D1);
        UNSERIALIZE_SCALAR(V1);
        UNSERIALIZE_SCALAR(C1);
        UNSERIALIZE_SCALAR(AddrShiftAmount);
        UNSERIALIZE_SCALAR(OffsetMask);
    }
}
