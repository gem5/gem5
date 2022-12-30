// -*- mode:c++ -*-

// Copyright (c) 2020 ARM Limited
// Copyright (c) 2020 Metempsy Technology Consulting
// All rights reserved
//
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "arch/arm/pauth_helpers.hh"

#include "arch/arm/faults.hh"
#include "base/bitfield.hh"

namespace gem5
{

using namespace ArmISA;

bool
ArmISA::calculateTBI(ThreadContext* tc, ExceptionLevel el,
                     uint64_t ptr, bool data)
{
    bool tbi = false;
    if (upperAndLowerRange(tc, el)) {

        ExceptionLevel s1_el = s1TranslationRegime(tc, el);
        assert (s1_el == EL1 || s1_el == EL2);
        TCR tcr = (s1_el == EL1) ? tc->readMiscReg(MISCREG_TCR_EL1):
                                   tc->readMiscReg(MISCREG_TCR_EL2);
        bool b55 = bits(ptr, 55) == 1;
        if (data)
            tbi = b55 ? tcr.tbi1 == 1 : tcr.tbi0 == 1;
        else
            tbi = b55 ? (tcr.tbi1 == 1 && tcr.tbid1 == 0) :
                        (tcr.tbi0 == 1 && tcr.tbid0 == 0);

    }
    else if (el == EL2) {
        TCR tcr = tc->readMiscReg(MISCREG_TCR_EL2);
        tbi = data ? tcr.tbi == 1 : (tcr.tbi == 1 && tcr.tbid == 0);
    }
    else if (el == EL3) {
        TCR tcr = tc->readMiscReg(MISCREG_TCR_EL3);
        tbi = data ? tcr.tbi == 1 : (tcr.tbi == 1 && tcr.tbid == 0);
    }
    return tbi;
}

int
ArmISA::calculateBottomPACBit(ThreadContext* tc, ExceptionLevel el,
                              bool top_bit)
{
    uint32_t tsz_field;
    bool using64k;
    if (upperAndLowerRange(tc, el)) {
        ExceptionLevel s1_el = s1TranslationRegime(tc, el);
        assert (s1_el == EL1 || s1_el == EL2);
        if (s1_el == EL1) {
            // EL1 translation regime registers
            TCR tcr = tc->readMiscReg(MISCREG_TCR_EL1);
            tsz_field = top_bit ? (uint32_t)tcr.t1sz : (uint32_t)tcr.t0sz;
            using64k = top_bit ? tcr.tg1 == 0x3 : tcr.tg0 == 0x1;
        } else {
            // EL2 translation regime registers
            TCR tcr = tc->readMiscReg(MISCREG_TCR_EL2);
            assert (ArmSystem::haveEL(tc, EL2));
            tsz_field = top_bit? (uint32_t)tcr.t1sz : (uint32_t)tcr.t0sz;
            using64k = top_bit ? tcr.tg1 == 0x3 : tcr.tg0 == 0x1;
        }
    } else {
        TCR tcr2 = tc->readMiscReg(MISCREG_TCR_EL2);
        TCR tcr3 = tc->readMiscReg(MISCREG_TCR_EL3);
        tsz_field = el == EL2 ? (uint32_t)tcr2.t0sz: (uint32_t)tcr3.t0sz;
        using64k  = el == EL2 ? tcr2.tg0 == 0x1 : tcr3.tg0 == 0x1 ;
    }
    uint32_t max_limit_tsz_field = using64k ? 47 : 48;
    tsz_field = std::min(tsz_field, max_limit_tsz_field);
    const AA64MMFR2 mm_fr2 = tc->readMiscReg(MISCREG_ID_AA64MMFR2_EL1);

    uint32_t tszmin = (using64k && (bool)mm_fr2.varange) ? 12 : 16;
    tsz_field = std::max(tsz_field, tszmin);

    return (64-tsz_field);
}

Fault
ArmISA::trapPACUse(ThreadContext *tc, ExceptionLevel target_el)
{
    assert(ArmSystem::haveEL(tc, target_el) &&
           target_el != EL0 && (target_el >= currEL(tc)));

    switch (target_el) {
       case EL2:
            return std::make_shared<HypervisorTrap>(
                0x0, 0, ExceptionClass::TRAPPED_PAC);
       case EL3:
            return std::make_shared<SecureMonitorTrap>(
                0x0, 0, ExceptionClass::TRAPPED_PAC);
       default:
            return NoFault;
    }
}

uint64_t
ArmISA::addPAC (ThreadContext* tc, ExceptionLevel el, uint64_t  ptr,
        uint64_t modifier, uint64_t k1, uint64_t k0, bool data)
{
    uint64_t PAC;
    uint64_t result;
    uint64_t ext_ptr;
    bool selbit;

    bool tbi = calculateTBI(tc, el, ptr, data);
    int top_bit = tbi ? 55 : 63;
    bool b55 = bits(ptr, 55);
    bool b63 = bits(ptr, 63);
    // If tagged pointers are in use for a regime with two TTBRs,use bit<55> of
    // the pointer to select between upper and lower ranges, and preserve this.
    // This handles the awkward case where there is apparently no correct
    // choice between the upper and lower address range - ie an addr of
    // 1xxxxxxx0... with TBI0=0 and TBI1=1 and 0xxxxxxx1 with TBI1=0 and TBI0=1

    if (upperAndLowerRange(tc, el)) {
        ExceptionLevel s1_el = s1TranslationRegime(tc, el);
        assert (s1_el == EL1 || s1_el == EL2);
        if (s1_el == EL1) {
            // EL1 translation regime registers
            TCR tcr = tc->readMiscReg(MISCREG_TCR_EL1);
            if (data) {
                selbit = (tcr.tbi1 == 1 || tcr.tbi0 == 1) ? b55: b63;
            } else {
                selbit = ((tcr.tbi1 == 1 && tcr.tbid1 == 0)
                          || (tcr.tbi0 == 1 && tcr.tbid0 == 0)) ? b55 : b63;
            }
        } else {
            // EL2 translation regime registers
            TCR tcr = tc->readMiscReg(MISCREG_TCR_EL2);
            bool have_el2 = ArmSystem::haveEL(tc, EL2);
            if (data) {
                selbit = (have_el2 &&
                          (tcr.tbi0 == 1 || tcr.tbi1 == 1))?  b55: b63;
            }
            else
            {
                selbit = (have_el2 &&
                          ((tcr.tbi1 == 1 && tcr.tbid1 == 0) ||
                           (tcr.tbi0 == 1 && tcr.tbid0 == 0)))?  b55: b63;
            }
        }
    } else {
        selbit = tbi ? b55: b63;
    }

    int bottom_PAC_bit = calculateBottomPACBit(tc, el, selbit);
    // The pointer authentication code field takes all the available
    //   bits in between

    uint32_t nbits = (top_bit+1) - bottom_PAC_bit;
    uint64_t pacbits = ((uint64_t)0x1 << nbits) -1; // 2^n -1;
    uint64_t mask = pacbits << bottom_PAC_bit; // creates mask

    if (selbit) {
        ext_ptr = ptr | mask;
    } else {
        ext_ptr = ptr & ~mask;
    }

    PAC = QARMA::computePAC(ext_ptr, modifier, k1, k0);
    // Check if the ptr has good extension bits and corrupt the
    //    pointer authentication code if not;
    uint64_t t = bits(ptr, top_bit, bottom_PAC_bit);
    if (t != 0x0 && t != pacbits) {
        PAC ^= ((uint64_t)0x1 << (top_bit-1));
    }
    // Preserve the determination between upper and lower address
    //   at bit<55> and insert PAC
    if (tbi) {
        // ptr<63:56>:selbit:PAC<54:bottom_PAC_bit>:ptr<bottom_PAC_bit-1:0>;
        result = ptr & 0xFF00000000000000;
    } else {
        // PAC<63:56>:selbit:PAC<54:bottom_PAC_bit>:ptr<bottom_PAC_bit-1:0>;
        result = PAC & 0xFF00000000000000;
    }

    uint64_t masked_PAC = PAC & 0x007FFFFFFFFFFFFF;
    uint64_t pacbit_mask = ((uint64_t)0x1 << bottom_PAC_bit) -1;
    uint64_t masked_ptr = ptr & pacbit_mask;

    masked_PAC &= ~pacbit_mask;
    result |= ((uint64_t)selbit << 55) | masked_PAC | masked_ptr;

    return result;
}


uint64_t
ArmISA::auth(ThreadContext *tc, ExceptionLevel el, uint64_t ptr,
        uint64_t modifier, uint64_t k1, uint64_t k0, bool data,
        uint8_t errorcode)
{
    uint64_t PAC;
    uint64_t result;
    uint64_t original_ptr;
    // Reconstruct the extension field used of adding the PAC to the pointer
    bool tbi = calculateTBI(tc, el, ptr, data);
    bool selbit = (bool) bits(ptr, 55);

    int bottom_PAC_bit = calculateBottomPACBit(tc, el, selbit);

    uint32_t top_tbi = tbi? 56: 64;
    uint32_t nbits = top_tbi - bottom_PAC_bit;
    uint64_t pacbits = ((uint64_t)0x1 << nbits) -1; // 2^n -1;
    uint64_t mask = (pacbits << bottom_PAC_bit); // creates mask

    if (selbit) {
        original_ptr = ptr | mask;
    } else {
        original_ptr = ptr & ~mask;
    }


    PAC = QARMA::computePAC(original_ptr,  modifier, k1, k0);
    // Check pointer authentication code

    // <bottom_PAC_bit:0>
    uint64_t low_mask = ((uint64_t)0x1 << bottom_PAC_bit) -1;
    // <54:bottom_PAC_bit>
    uint64_t pac_mask = 0x007FFFFFFFFFFFFF & ~low_mask;

    uint64_t masked_pac = PAC & pac_mask;
    uint64_t masked_ptr = ptr & pac_mask;

    if (tbi) {
        if (masked_pac == masked_ptr) {
            result = original_ptr;
        } else {
            uint64_t mask2= ~((uint64_t)0x3 << 53);
            result = original_ptr & mask2;
            result |= (uint64_t)errorcode << 53;
        }
    } else {
        if ((masked_pac == masked_ptr) && ((PAC >>56)==(ptr >> 56))) {
            result = original_ptr;
        } else {
            uint64_t mask2 = ~((uint64_t)0x3 << 61);
            result = original_ptr & mask2;
            result |= (uint64_t)errorcode << 61;
        }
    }
    return result;
}

Fault
ArmISA::authDA(ThreadContext * tc, uint64_t X, uint64_t Y, uint64_t* out)
{
/*
  Returns a 64-bit value containing X, but replacing the pointer
  authentication code field bits with the extension of the address bits.
  The instruction checks a pointer
  authentication code in the pointer authentication code field bits of X,
  using the same algorithm and key as AddPACDA().
*/

    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APDAKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APDAKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.enda : (bool)sc2.enda;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                    (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            enable = sc1.enda;
            trapEL2 = EL2Enabled(tc) && hcr.api == 0;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL2:
            enable = sc2.enda;
            trapEL2 = false;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL3:
            enable = sc3.enda;
            trapEL2 = false;
            trapEL3 = false;
            break;
        default:
            // Unreachable
            break;
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else {
        *out =  auth(tc, el, X, Y, hi_key, lo_key, true, 0x1);
    }
    return NoFault;
}

Fault
ArmISA::authDB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out)
{
/*
  Returns a 64-bit value containing X, but replacing the pointer
  authentication code field bits with the extension of the address bits.
  The instruction checks a pointer
  authentication code in the pointer authentication code field bits of X,
  using the same algorithm and key as AddPACDA().
*/

    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APDBKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APDBKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);

    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.endb : (bool)sc2.endb;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                    (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            enable = sc1.endb;
            trapEL2 = EL2Enabled(tc) && hcr.api == 0;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL2:
            enable = sc2.endb;
            trapEL2 = false;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL3:
            enable = sc3.endb;
            trapEL2 = false;
            trapEL3 = false;
            break;
        default:
            //unreachable
            break;
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out =  auth(tc, el, X, Y, hi_key, lo_key, true, 0x2);

    return NoFault;
}


Fault
ArmISA::authIA(ThreadContext * tc, uint64_t X, uint64_t Y, uint64_t* out)
{
/*
  Returns a 64-bit value containing X, but replacing the pointer
  authentication code field bits with the extension of the address bits.
  The instruction checks a pointer
  authentication code in the pointer authentication code field bits of X,
  using the same algorithm and key as AddPACDA().
*/

    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APIAKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APIAKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.enia : (bool)sc2.enia;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                           (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            {
                enable = sc1.enia;
                trapEL2 = EL2Enabled(tc) && hcr.api == 0;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL2:
            {
                enable = sc2.enia;
                trapEL2 = false;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL3:
            {
                enable = sc3.enia;
                trapEL2 = false;
                trapEL3 = false;
                break;
            }
        default:
            //unreachable
            break;
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = auth(tc, el, X, Y, hi_key, lo_key, false, 0x1);

    return NoFault;
}

Fault
ArmISA::authIB(ThreadContext *tc, uint64_t X, uint64_t Y, uint64_t* out)
{
/*
  Returns a 64-bit value containing X, but replacing the pointer
  authentication code field bits with the extension of the address bits.
  The instruction checks a pointer
  authentication code in the pointer authentication code field bits of X,
  using the same algorithm and key as AddPACDA().
*/

    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APIBKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APIBKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.enib : (bool)sc2.enib;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                           (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            {
                enable = sc1.enib;
                trapEL2 = EL2Enabled(tc) && hcr.api == 0;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL2:
            {
                enable = sc2.enib;
                trapEL2 = false;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL3:
            {
                enable = sc3.enib;
                trapEL2 = false;
                trapEL3 = false;
                break;
            }
        default:
            //unreachable
            break;
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = auth(tc, el, X, Y, hi_key, lo_key, false, 0x2);

    return NoFault;
}



Fault
ArmISA::addPACDA(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out)
{
    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APDAKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APDAKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.enda : (bool)sc2.enda;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            {
                enable = sc1.enda;
                trapEL2 = EL2Enabled(tc) && hcr.api == 0;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL2:
            {
                enable = sc2.enda;
                trapEL2 = false;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL3:
            {
                enable = sc3.enda;
                trapEL2 = false;
                trapEL3 = false;
                break;
            }
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = addPAC(tc, el, X, Y, hi_key, lo_key, true);

    return NoFault;
}


Fault
ArmISA::addPACDB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out)
{
    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APDBKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APDBKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.endb : (bool)sc2.endb;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                           (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
                enable = sc1.endb;
                trapEL2 = EL2Enabled(tc) && hcr.api == 0;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
        case EL2:
                enable = sc2.endb;
                trapEL2 = false;
                trapEL3 = have_el3 && scr3.api == 0;
                break;
        case EL3:
                enable = sc3.endb;
                trapEL2 = false;
                trapEL3 = false;
                break;
        default:
            // unreachable
            break;
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = addPAC(tc, el, X, Y, hi_key, lo_key, true);

    return NoFault;
}


Fault
ArmISA::addPACGA(ThreadContext * tc, uint64_t X, uint64_t Y, uint64_t* out)
{
    bool trapEL2 = false;
    bool trapEL3 = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APGAKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APGAKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCR sc3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                      (hcr.tge == 0 || hcr.e2h == 0));
            trapEL3 = have_el3 && sc3.api == 0;
            break;
        case EL1:
            trapEL2 = EL2Enabled(tc) && hcr.api == 0;
            trapEL3 = have_el3 && sc3.api == 0;
            break;
        case EL2:
            trapEL2 = false;
            trapEL3 = have_el3 && sc3.api == 0;
            break;
        case EL3:
            trapEL2 = false;
            trapEL3 = false;
            break;
        default:
            //unreachable
            break;
    }
    if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = QARMA::computePAC(X, Y, hi_key, lo_key) & 0xFFFFFFFF00000000;

    return NoFault;
}


Fault
ArmISA::addPACIA(ThreadContext * tc, uint64_t X, uint64_t Y, uint64_t* out){
    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APIAKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APIAKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.enia : (bool)sc2.enia;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                           (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            enable = sc1.enia;
            trapEL2 = EL2Enabled(tc) && hcr.api == 0;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL2:
            enable = sc2.enia;
            trapEL2 = false;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL3:
            enable = sc3.enia;
            trapEL2 = false;
            trapEL3 = false;
            break;
        default:
            //unreachable
            break;
    }
    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = addPAC(tc, el, X, Y, hi_key, lo_key, false);

    return NoFault;
}

Fault
ArmISA::addPACIB(ThreadContext* tc, uint64_t X, uint64_t Y, uint64_t* out){
    bool trapEL2 = false;
    bool trapEL3 = false;
    bool enable = false;

    uint64_t hi_key= tc->readMiscReg(MISCREG_APIBKeyHi_EL1);
    uint64_t lo_key= tc->readMiscReg(MISCREG_APIBKeyLo_EL1);

    ExceptionLevel el = currEL(tc);
    SCTLR sc1 = tc->readMiscReg(MISCREG_SCTLR_EL1);
    SCTLR sc2 = tc->readMiscReg(MISCREG_SCTLR_EL2);
    SCTLR sc3 = tc->readMiscReg(MISCREG_SCTLR_EL3);
    SCR scr3 = tc->readMiscReg(MISCREG_SCR_EL3);
    HCR   hcr = tc->readMiscReg(MISCREG_HCR_EL2);
    bool have_el3 = ArmSystem::haveEL(tc, EL3);

    switch (el)
    {
        case EL0:
            {
                bool IsEL1Regime = s1TranslationRegime(tc, el) == EL1;
                enable = IsEL1Regime ? (bool)sc1.enib : (bool)sc2.enib;
                trapEL2 = (EL2Enabled(tc) && hcr.api == 0 &&
                           (hcr.tge == 0 || hcr.e2h == 0));
                trapEL3 = have_el3 && scr3.api == 0;
                break;
            }
        case EL1:
            enable = sc1.enib;
            trapEL2 = EL2Enabled(tc) && hcr.api == 0;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL2:
            enable = sc2.enib;
            trapEL2 = false;
            trapEL3 = have_el3 && scr3.api == 0;
            break;
        case EL3:
            enable = sc3.enib;
            trapEL2 = false;
            trapEL3 = false;
            break;
        default:
            // Unnaccessible
            break;
    }

    if (!enable)
        *out = X;
    else if (trapEL2)
        return trapPACUse(tc, EL2);
    else if (trapEL3)
        return trapPACUse(tc, EL3);
    else
        *out = addPAC(tc, el, X, Y, hi_key, lo_key, false);

    return NoFault;
}



void
ArmISA::stripPAC(ThreadContext* tc, uint64_t A, bool data, uint64_t* out)
{
    ExceptionLevel el = currEL(tc);

    bool tbi = calculateTBI(tc, el, A, data);
    bool selbit = (bool) bits(A, 55);
    int bottom_PAC_bit = calculateBottomPACBit(tc, el, selbit);

    int top_bit = tbi ? 55 : 63;
    uint32_t nbits = (top_bit + 1) - bottom_PAC_bit;
    uint64_t pacbits = ((uint64_t)0x1 << nbits) -1; // 2^n -1;
    uint64_t mask = pacbits << bottom_PAC_bit; // creates mask

    if (selbit) {
        *out = A | mask;
    } else {
        *out = A & ~mask;
    }
}

} // namespace gem5
